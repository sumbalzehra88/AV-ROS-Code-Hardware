#!/usr/bin/env python3
"""
gnss_checkpoint_monitor.py

ROS2 node that:
- Loads checkpoints from YAML or CSV
- Subscribes to /fix (NavSatFix), /jmoab_compass (Float32MultiArray), /vel (TwistStamped)
- Publishes:
    - /point          (std_msgs/Bool)       -> True when inside checkpoint tolerance zone
    - /point_idx      (std_msgs/Int32)      -> index of active checkpoint (or -1)
    - /aligned        (std_msgs/Float32)    -> target heading when a checkpoint requests alignment
    - /alignment_req  (std_msgs/Bool)       -> True while node is actively asking aligner to align
    - /point_info     (std_msgs/String)     -> JSON debug/info for the current point
- Subscribes to:
    - /alignment_done (std_msgs/Bool)       -> external aligner tells us alignment is finished
Behavior summary:
- Choose next checkpoint (closest-unvisited by default)
- When distance <= tolerance_m -> set /point True and (if desired_heading provided)
  repeatedly publish /aligned until /alignment_done==True.
- When /alignment_done received -> publish /point False, mark visited, move on.
- Detect "passed" checkpoint and skip (disable publishes) if necessary.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, Float32, String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
import math
import time
import yaml
import csv
import json
import os

# ---------------------------
# Utility math (Haversine & bearing)
# ---------------------------
def haversine_m(lat1, lon1, lat2, lon2):
    # Earth radius in meters
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2.0)**2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2.0)**2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c

def bearing_deg(lat1, lon1, lat2, lon2):
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    dlonr = math.radians(lon2 - lon1)
    y = math.sin(dlonr) * math.cos(lat2r)
    x = math.cos(lat1r)*math.sin(lat2r) - math.sin(lat1r)*math.cos(lat2r)*math.cos(dlonr)
    brng = math.degrees(math.atan2(y, x))
    return brng % 360.0

def angle_diff_short(target_deg, current_deg):
    # returns signed shortest difference in degrees (-180, 180]
    d = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return d

# ---------------------------
# Node
# ---------------------------
class GNSSCheckpointMonitor(Node):
    def __init__(self):
        super().__init__('gnss_checkpoint_monitor')

        # ------- PARAMETERS (tune as needed) -------
        self.declare_parameter('checkpoints_file', 'gnss_cheakpoint.yaml')  # YAML preferred
        self.declare_parameter('file_type', 'yaml')  # 'yaml' or 'csv'
        self.declare_parameter('mode', 'sequential')   # 'closest' or 'sequential'
        self.declare_parameter('tolerance_m_default', 2.0)
        self.declare_parameter('heading_tolerance_deg', 1.0)  # used by aligner but informative
        self.declare_parameter('revisit_timeout', 30.0)  # seconds before allowing re-trigger
        self.declare_parameter('publish_aligned_rate', 5.0)  # Hz when actively asking alignment
        self.declare_parameter('min_gps_age', 0.5)  # seconds - optional gating (if you publish age)
        self.declare_parameter('max_pass_distance', 3.0)  # distance beyond which considered passed

        self.file = self.get_parameter('checkpoints_file').value
        self.file_type = self.get_parameter('file_type').value.lower()
        self.mode = self.get_parameter('mode').value.lower()
        self.tol_default = float(self.get_parameter('tolerance_m_default').value)
        self.heading_tolerance = float(self.get_parameter('heading_tolerance_deg').value)
        self.revisit_timeout = float(self.get_parameter('revisit_timeout').value)
        self.publish_aligned_rate = float(self.get_parameter('publish_aligned_rate').value)
        self.max_pass_distance = float(self.get_parameter('max_pass_distance').value)

        # --------- state ----------
        self.checkpoints = []   # list of dicts: {id, name, lat, lon, tolerance_m, desired_heading_deg, behavior, visited, last_visited_time}
        self.active_idx = -1    # index of currently active checkpoint
        self.gps_lat = None
        self.gps_lon = None
        self.hdg = None         # current heading from /jmoab_compass (degrees)
        self.speed = 0.0        # ground speed magnitude (m/s)
        self.last_dist = None   # last distance to active point (for pass detection)
        self.alignment_requested = False
        self.alignment_publish_last = 0.0
        self.point_flag = False  # published /point state (so we can publish transitions)
        self.alignment_done = False

        # ---------- publishers ----------
        self.pub_point = self.create_publisher(Bool, '/point', 10)
        self.pub_point_idx = self.create_publisher(Int32, '/point_idx', 10)
        self.pub_aligned = self.create_publisher(Float32, '/aligned', 10)
        self.pub_alignment_req = self.create_publisher(Bool, '/alignment_req', 10)
        self.pub_point_info = self.create_publisher(String, '/point_info', 10)

        # ---------- subscribers ----------
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.create_subscription(Float32MultiArray, '/jmoab_compass', self.compass_callback, 10)
        self.create_subscription(TwistStamped, '/vel', self.gps_vel_callback, 10)
        self.create_subscription(Bool, '/alignment_done', self.alignment_done_cb, 10)

        # ---------- load checkpoints ----------
        self.load_checkpoints(self.file, self.file_type)

        # ---------- main timer ----------
        self.timer = self.create_timer(0.05, self.main_loop)  # 20 Hz loop

        self.get_logger().info(f"GNSS Checkpoint Monitor started with {len(self.checkpoints)} checkpoints, mode={self.mode}")

    # ---------- file loaders ----------
    def load_checkpoints(self, path, ftype='yaml'):
        if not os.path.isabs(path):
            # try relative to ROS package path? allow relative; user can give absolute
            path = os.path.join(os.getcwd(), path)
        try:
            if ftype == 'yaml':
                with open(path, 'r') as fh:
                    data = yaml.safe_load(fh)
                    items = data.get('checkpoints', [])
            else:
                # CSV loader: expect header id,name,lat,lon,tolerance_m,desired_heading_deg,behavior
                items = []
                with open(path, newline='') as csvfile:
                    reader = csv.DictReader(csvfile)
                    for r in reader:
                        items.append(r)
            # normalize and store
            for i, it in enumerate(items):
                lat = float(it.get('lat') or it.get('latitude') or 0.0)
                lon = float(it.get('lon') or it.get('longitude') or 0.0)
                tid = int(it.get('id', i))
                name = it.get('name', f'pt_{tid}')
                tol = float(it.get('tolerance_m', self.tol_default) or self.tol_default)
                desired = it.get('desired_heading_deg', None)
                desired = None if desired in (None, '', 'null') else float(desired)
                behavior = it.get('behavior', 'align_then_continue')
                self.checkpoints.append({
                    'id': tid,
                    'name': name,
                    'lat': lat,
                    'lon': lon,
                    'tolerance_m': tol,
                    'desired_heading_deg': desired,
                    'behavior': behavior,
                    'visited': False,
                    'last_visited_time': 0.0
                })
            self.get_logger().info(f"Loaded {len(self.checkpoints)} checkpoints from {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load checkpoints from {path}: {e}")

    # ---------- callbacks ----------
    def gps_callback(self, msg: NavSatFix):
        # store latest GNSS fix
        try:
            self.gps_lat = float(msg.latitude)
            self.gps_lon = float(msg.longitude)
        except Exception as e:
            self.get_logger().warn(f"Bad GPS msg: {e}")

    def compass_callback(self, msg: Float32MultiArray):
        # expects [roll, pitch, heading]
        try:
            self.hdg = float(msg.data[2])
        except Exception as e:
            self.get_logger().warn(f"compass_callback error: {e}")

    def gps_vel_callback(self, msg: TwistStamped):
        try:
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            self.speed = float(math.sqrt(vx*vx + vy*vy))
        except Exception as e:
            self.get_logger().warn(f"gps_vel_callback error: {e}")

    def alignment_done_cb(self, msg: Bool):
        if msg.data:
            # aligner signals alignment complete
            self.get_logger().info("Received /alignment_done = True")
            self.alignment_done = True

    # ---------- main loop ----------
    def main_loop(self):
        # must have GPS
        if self.gps_lat is None or self.gps_lon is None:
            return

        # choose next target if none active
        if self.active_idx == -1:
            idx = self.select_next_checkpoint()
            if idx is None:
                return
            self.set_active(idx)

        # compute distance to active point
        active = self.checkpoints[self.active_idx]
        d = haversine_m(self.gps_lat, self.gps_lon, active['lat'], active['lon'])

        # publish debug info
        info = {
            'active_idx': self.active_idx,
            'active_name': active['name'],
            'distance_m': d,
            'tolerance_m': active['tolerance_m'],
            'gps': [self.gps_lat, self.gps_lon],
            'heading': self.hdg,
            'speed_m_s': self.speed
        }
        self.pub_point_info.publish(String(data=json.dumps(info)))

        # Check entering tolerance zone
        if d <= active['tolerance_m']:
            # we've reached the checkpoint
            if not self.point_flag:
                # first time: set /point True
                self.point_flag = True
                self.publish_point(True)
                self.get_logger().info(f"Checkpoint {self.active_idx} ({active['name']}) reached (d={d:.3f} m). publishing /point True")
            # If checkpoint requires desired heading: request alignment
            if active['desired_heading_deg'] is not None and not self.alignment_done:
                # publish aligned repeatedly at configured rate until alignment_done
                now = self.get_clock().now().nanoseconds / 1e9
                if now - self.alignment_publish_last >= (1.0 / max(1.0, self.publish_aligned_rate)):
                    self.pub_aligned.publish(Float32(data=float(active['desired_heading_deg'])))
                    self.pub_alignment_req.publish(Bool(data=True))
                    self.alignment_publish_last = now
            else:
                # if no desired heading or already aligned - do nothing
                pass

        else:
            # we're outside tolerance
            # if we previously were inside and now outside, maybe we've passed it -> handle pass logic
            if self.point_flag:
                # we left tolerance after being inside: treat as passed or alignment completed
                # if alignment was done, this exit is normal (we turned and left), otherwise it's a pass
                if self.alignment_done:
                    self.get_logger().info("Left tolerance after alignment done. Clearing point and advancing.")
                    self.clear_and_advance()
                else:
                    # Not aligned but left zone -> consider it passed; disable publishing and advance
                    self.get_logger().warn("Left tolerance zone without alignment -> marking passed and advancing")
                    self.clear_and_advance()
            else:
                # not inside, normal monitoring
                # pass detection heuristics:
                if self.last_dist is not None:
                    # if distance suddenly increases and exceeded max_pass_distance, consider skipped/passed
                    if d > (active['tolerance_m'] + self.max_pass_distance) and d > self.last_dist + 0.1:
                        self.get_logger().warn(f"Checkpoint {self.active_idx} appears passed (d={d:.2f} m). Skipping.")
                        self.mark_visited_and_advance(False)
                # else continue

        # update last distance
        self.last_dist = d

        # If alignment_done flag set by external node
        if self.alignment_done:
            # finalize: set /point False, mark visited, advance
            self.get_logger().info("Alignment done - clearing /point and advancing.")
            self.clear_and_advance()

    # ---------- helpers ----------
    def publish_point(self, val: bool):
        self.pub_point.publish(Bool(data=bool(val)))
        # publish idx as well
        idx_val = self.active_idx if val else Int32(data=-1)
        if val:
            self.pub_point_idx.publish(Int32(data=self.active_idx))
        else:
            self.pub_point_idx.publish(Int32(data=-1))

    def set_active(self, idx: int):
        if idx is None:
            self.get_logger().warn("No available checkpoint to activate")
            return
        self.active_idx = idx
        self.point_flag = False
        self.alignment_done = False
        self.alignment_requested = False
        self.last_dist = None
        self.get_logger().info(f"Set active checkpoint -> idx={idx}, name={self.checkpoints[idx]['name']}")
        # initial publish of idx (not setting point True)
        self.pub_point_idx.publish(Int32(data=idx))

    def select_next_checkpoint(self):
        # Choose next checkpoint based on mode
        unvisited = [i for i, p in enumerate(self.checkpoints) if not p['visited']]
        if not unvisited:
            self.get_logger().info("All checkpoints visited.")
            return None
        if self.mode == 'sequential':
            return unvisited[0]
        # closest mode
        best = None
        bestd = None
        for i in unvisited:
            p = self.checkpoints[i]
            d = haversine_m(self.gps_lat, self.gps_lon, p['lat'], p['lon'])
            if best is None or d < bestd:
                best = i
                bestd = d
        return best

    def mark_visited_and_advance(self, aligned_was_true: bool):
        # mark current as visited and pick next
        if self.active_idx is None or self.active_idx < 0:
            return
        now = time.time()
        self.checkpoints[self.active_idx]['visited'] = True
        self.checkpoints[self.active_idx]['last_visited_time'] = now
        self.get_logger().info(f"Marked checkpoint {self.active_idx} visited.")
        # reset state
        self.active_idx = -1
        self.point_flag = False
        self.alignment_done = False
        # choose next
        next_idx = self.select_next_checkpoint()
        if next_idx is not None:
            self.set_active(next_idx)

    def clear_and_advance(self):
        # clear point flag, publish false, mark visited and move on
        self.publish_point(False)
        self.pub_alignment_req.publish(Bool(data=False))
        # mark visited & move
        self.mark_visited_and_advance(aligned_was_true=self.alignment_done)

def main(args=None):
    rclpy.init(args=args)
    node = GNSSCheckpointMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

