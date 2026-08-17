#!/usr/bin/env python3
"""
gnss_heading_node.py

GNSS-only heading publisher for ROS2 (publishes jmoab/ahrs as Float32MultiArray):
 - Computes earth-referenced heading (bearing / course-over-ground) from successive NavSatFix messages.
 - Smooths the bearing using a simple 1D Kalman filter and optional exponential moving average.
 - Publishes [roll, pitch, heading_deg] on topic 'jmoab/ahrs' (Float32MultiArray).

Notes:
 - Heading is in degrees in range [0, 360).
 - At low speeds GPS bearing is unreliable; node uses a speed threshold and/or distance threshold to accept bearing samples.
 - Designed to be drop-in replacement for AHRS_SIM's GNSS bearing functionality.

Author: Adjusted for user by ChatGPT (GPT-5 Thinking mini)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import math
import time
import numpy as np
if not hasattr(np, 'float'):
    np.float = float

# -------------------------
# Utility functions
# -------------------------
def bearing_deg(lat1, lon1, lat2, lon2):
    """
    Calculate forward bearing in degrees from (lat1,lon1) -> (lat2,lon2).
    Result in [0, 360).
    """
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    dlonr = math.radians(lon2 - lon1)
    y = math.sin(dlonr) * math.cos(lat2r)
    x = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlonr)
    brng = math.degrees(math.atan2(y, x))
    return brng % 360.0

def shortest_signed_deg_diff(target_deg, current_deg):
    """
    Smallest signed difference between two headings in degrees (target - current)
    returned in range (-180, 180]
    """
    d = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return d

def wrap360(deg):
    return deg % 360.0

# -------------------------
# Simple 1D Kalman filter for heading offsets (works with circular wrap)
# -------------------------
class HeadingKalman1D:
    def __init__(self, initial=0.0, initial_error=10.0, measurement_var=8.0, process_var=0.01):
        # state = heading_deg (we will manage wrap-around externally)
        self.state = initial
        self.error = initial_error
        self.measurement_var = measurement_var
        self.process_var = process_var

    def update(self, measurement_deg):
        """
        Standard scalar Kalman update for 1D measurement.
        measurement_deg and state are treated as plain scalars. Use shortest diff when computing residual.
        """
        # compute innovation (use shortest angular difference)
        residual = shortest_signed_deg_diff(measurement_deg, self.state)

        # Kalman gain
        K = self.error / (self.error + self.measurement_var)

        # State update (add residual, not raw measurement)
        self.state = wrap360(self.state + K * residual)

        # Error update
        self.error = (1 - K) * self.error + abs(self.state) * 0.0  # keep stable, optionally add small process noise below
        # Add process noise to error (prediction step)
        self.error = self.error + self.process_var

        return self.state

# -------------------------
# Node
# -------------------------
class GNSSHeadingNode(Node):
    def __init__(self):
        super().__init__('gnss_heading_node')
        self.get_logger().info("Starting GNSS Heading Node (GNSS-only bearing → jmoab/ahrs)")

        # ---------- parameters ----------
        # thresholds & gating
        self.declare_parameter('min_speed_m_s', 0.5)         # below this, GNSS COG unreliable (m/s)
        self.declare_parameter('min_distance_m', 0.5)        # minimal displacement to compute bearing (meters)
        self.declare_parameter('max_sample_age_s', 2.0)      # maximum allowed time between fixes used to compute bearing
        # Kalman / smoothing
        self.declare_parameter('kalman_measurement_var', 8.0)  # variance of measurement (deg^2)
        self.declare_parameter('kalman_process_var', 0.01)     # process variance
        self.declare_parameter('ema_alpha', 0.6)               # Optional Exponential Moving Average for published heading (0..1), 1 -> no EMA
        # Logging
        self.declare_parameter('show_log', False)

        # load parameters
        self.min_speed = float(self.get_parameter('min_speed_m_s').value)
        self.min_distance = float(self.get_parameter('min_distance_m').value)
        self.max_sample_age = float(self.get_parameter('max_sample_age_s').value)
        self.kalman_measurement_var = float(self.get_parameter('kalman_measurement_var').value)
        self.kalman_process_var = float(self.get_parameter('kalman_process_var').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.show_log = bool(self.get_parameter('show_log').value)

        # dynamic parameter callback
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---------- state ----------
        self.prev_lat = None
        self.prev_lon = None
        self.prev_time = None
        self.prev_speed = 0.0

        # latest computed heading (degrees)
        self.kalman = HeadingKalman1D(initial=0.0,
                                      initial_error=10.0,
                                      measurement_var=self.kalman_measurement_var,
                                      process_var=self.kalman_process_var)

        self.ema_heading = None  # used only if ema_alpha < 1

        # roll/pitch from other sources (we set zeros; GNSS-only does not provide roll/pitch)
        self.roll = 0.0
        self.pitch = 0.0

        # ---------- ROS pubs/subs ----------
        self.ahrs_pub = self.create_publisher(Float32MultiArray, 'jmoab/ahrs', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)

        # timer to regularly publish (even when no new heading) - keeps topic alive
        self.publish_hz = 5.0
        self.create_timer(1.0 / self.publish_hz, self._publish_timer_cb)

        self.get_logger().info("GNSS Heading Node initialized - waiting for /fix messages")

    # parameter update handler
    def _on_param_change(self, params):
        for p in params:
            if p.name == 'min_speed_m_s':
                self.min_speed = float(p.value)
            elif p.name == 'min_distance_m':
                self.min_distance = float(p.value)
            elif p.name == 'max_sample_age_s':
                self.max_sample_age = float(p.value)
            elif p.name == 'kalman_measurement_var':
                self.kalman_measurement_var = float(p.value)
                self.kalman.measurement_var = self.kalman_measurement_var
            elif p.name == 'kalman_process_var':
                self.kalman_process_var = float(p.value)
                self.kalman.process_var = self.kalman_process_var
            elif p.name == 'ema_alpha':
                self.ema_alpha = float(p.value)
            elif p.name == 'show_log':
                self.show_log = bool(p.value)
        self.get_logger().info("Parameters updated")
        return SetParametersResult(successful=True)

    # ----------------------
    # GPS callback
    # ----------------------
    def gps_callback(self, msg: NavSatFix):
        """
        Called on each GNSS fix. Compute bearing from previous accepted fix -> this fix.
        Use gating: time between fixes, minimal displacement, and optional speed check (if your NavSatFix provides speed separately).
        """
        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
        except Exception as e:
            self.get_logger().warn(f"Bad GPS fix: {e}")
            return

        now = time.time()

        # if first fix, just store
        if self.prev_lat is None:
            self.prev_lat = lat
            self.prev_lon = lon
            self.prev_time = now
            if self.show_log:
                self.get_logger().info("Stored first GNSS sample, no bearing computed yet.")
            return

        # compute time and distance since last stored
        dt = now - (self.prev_time or now)
        # approximate linear distance (haversine)
        # small-distance approximation (meters)
        # Use haversine formula for safety
        R = 6371000.0
        dlat = math.radians(lat - self.prev_lat)
        dlon = math.radians(lon - self.prev_lon)
        a = (math.sin(dlat/2.0)**2 +
             math.cos(math.radians(self.prev_lat)) * math.cos(math.radians(lat)) * math.sin(dlon/2.0)**2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        dist_m = R * c

        # optional speed gating: if your NavSatFix contains velocity you might use it;
        # otherwise we use distance/time to estimate speed
        est_speed = dist_m / max(1e-6, dt)

        # Accept this sample for bearing computation only if it meets thresholds
        accept_sample = False
        if dist_m >= self.min_distance and dt <= max(self.max_sample_age, 10.0):
            # Good displacement and not too old
            # Also accept if estimated speed > min_speed (avoid tiny jitter when static)
            if est_speed >= self.min_speed or self.min_speed <= 0.0:
                accept_sample = True

        # If not accepted, we still update previous with current only if time window exceeded (avoid stale)
        if not accept_sample:
            # if sample too old, just refresh prev_time (but do not compute bearing)
            # update prev when dt is large to allow future bearings
            if dt > self.max_sample_age:
                self.prev_lat = lat
                self.prev_lon = lon
                self.prev_time = now
                if self.show_log:
                    self.get_logger().info(f"GNSS sample refreshed (no bearing) dist={dist_m:.2f} dt={dt:.2f}s speed={est_speed:.2f}m/s")
            return

        # compute raw bearing from prev -> current
        raw_bearing = bearing_deg(self.prev_lat, self.prev_lon, lat, lon)  # degrees 0..360

        # update kalman with this measurement
        filtered_heading = self.kalman.update(raw_bearing)

        # optionally apply EMA smoothing on top of kalman (helps remove step changes)
        if self.ema_heading is None:
            self.ema_heading = filtered_heading
        else:
            # handle circular wrap when computing EMA: compute signed diff, then apply
            diff = shortest_signed_deg_diff(filtered_heading, self.ema_heading)
            self.ema_heading = wrap360(self.ema_heading + self.ema_alpha * diff)

        # Save the current fix as previous for next bearing computation
        self.prev_lat = lat
        self.prev_lon = lon
        self.prev_time = now
        self.prev_speed = est_speed

        if self.show_log:
            self.get_logger().info(f"GNSS raw_brg={raw_bearing:.2f}°, kalman={filtered_heading:.2f}°, ema={self.ema_heading:.2f}°, dist={dist_m:.2f}m dt={dt:.2f}s speed={est_speed:.2f}m/s")

    # ----------------------
    # Periodic publisher
    # ----------------------
    def _publish_timer_cb(self):
        # If no computed heading yet, do nothing
        if self.ema_heading is None:
            # still publish zeros occasionally so downstream nodes don't wait forever
            msg = Float32MultiArray()
            msg.data = [self.roll, self.pitch, 0.0]
            self.ahrs_pub.publish(msg)
            return

        # publish [roll, pitch, heading_deg]
        hdg = wrap360(self.ema_heading)
        msg = Float32MultiArray()
        msg.data = [float(self.roll), float(self.pitch), float(hdg)]
        self.ahrs_pub.publish(msg)

    # clean shutdown not required special


def main(args=None):
    rclpy.init(args=args)
    node = GNSSHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

