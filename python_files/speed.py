#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
import csv
import os

from std_msgs.msg import Float32, Float32MultiArray, Float64, Bool


# -------------------------
# Simple PID controller
# -------------------------
class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, integrator_limit=None, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator = 0.0
        self.prev_error = None
        self.prev_time = None
        self.integrator_limit = integrator_limit
        self.output_limit = output_limit

    def reset(self):
        self.integrator = 0.0
        self.prev_error = None
        self.prev_time = None

    def step(self, error, now=None):
        if now is None:
            now = time.time()
        dt = 0.0
        if self.prev_time is not None:
            dt = now - self.prev_time

        # Proportional
        p = self.kp * error

        # Integral
        if dt > 0:
            self.integrator += error * dt
            if self.integrator_limit is not None:
                self.integrator = max(-self.integrator_limit, min(self.integrator_limit, self.integrator))
        i = self.ki * self.integrator

        # Derivative
        d = 0.0
        if self.prev_error is not None and dt > 0:
            d = self.kd * (error - self.prev_error) / dt

        out = p + i + d

        self.prev_error = error
        self.prev_time = now

        if self.output_limit is not None:
            out = max(-self.output_limit, min(self.output_limit, out))

        return out


# -------------------------
# Heading controller with waypoint switching triggered by /done topic
# -------------------------
class HeadingController(Node):
    def __init__(self):
        super().__init__('heading_controller')

        # Parameters
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('steering_feedback_topic', '/steering_ang')
        self.declare_parameter('steering_command_topic', '/steering_angle')
        self.declare_parameter('ahrs_topic', '/jmoab/ahrs')
        self.declare_parameter('done_topic', '/done')
        self.declare_parameter('waypoint_csv', 'clean.csv')
        self.declare_parameter('deadband_yaw', 2.0)        # degrees
        self.declare_parameter('hold_time', 0.01)
        self.declare_parameter('max_pwm', 30.0)
        self.declare_parameter('min_pwm', 10.0)
        self.declare_parameter('pwm_rate_limit', 200.0)
        self.declare_parameter('steering_map_k', 1.0)
        self.declare_parameter('pid_kp', 2.0)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.2)
        self.declare_parameter('pid_integrator_limit', 50.0)
        self.declare_parameter('pid_output_limit', 100.0)

        # Load parameters
        self.rate_hz = self.get_parameter('rate_hz').value
        self.steer_feedback_topic = self.get_parameter('steering_feedback_topic').value
        self.steer_command_topic = self.get_parameter('steering_command_topic').value
        self.ahrs_topic = self.get_parameter('ahrs_topic').value
        self.done_topic = self.get_parameter('done_topic').value
        self.csv_file = self.get_parameter('waypoint_csv').value
        self.deadband_yaw = self.get_parameter('deadband_yaw').value
        self.hold_time = self.get_parameter('hold_time').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.pwm_rate_limit = self.get_parameter('pwm_rate_limit').value
        self.steering_map_k = self.get_parameter('steering_map_k').value

        pid_kp = self.get_parameter('pid_kp').value
        pid_ki = self.get_parameter('pid_ki').value
        pid_kd = self.get_parameter('pid_kd').value
        pid_int_lim = self.get_parameter('pid_integrator_limit').value
        pid_out_lim = self.get_parameter('pid_output_limit').value

        # PID controller
        self.pid = PID(kp=pid_kp, ki=pid_ki, kd=pid_kd,
                       integrator_limit=pid_int_lim, output_limit=pid_out_lim)

        # Subscriptions
        self.create_subscription(Float32, self.steer_feedback_topic, self.cb_steer_feedback, 10)
        self.create_subscription(Float32MultiArray, self.ahrs_topic, self.cb_ahrs, 10)
        self.create_subscription(Bool, self.done_topic, self.cb_done, 10)

        # Publisher
        self.steer_pub = self.create_publisher(Float32, self.steer_command_topic, 10)

        # State variables
        self.current_steer = 0.0
        self.current_heading = None
        self.target_heading = None
        self.last_pwm = 0.0
        self.last_pwm_time = time.time()
        self.done_triggered = False

        # Waypoints
        self.waypoints = self.load_waypoints(self.csv_file)
        self.wp_index = 0
        self.last_reach_time = None

        if self.waypoints:
            self.target_heading = self.waypoints[0]
            self.get_logger().info(f"Loaded {len(self.waypoints)} headings: {self.waypoints}")
            self.get_logger().info(f"Starting with heading {self.target_heading}°")
        else:
            self.get_logger().warn("No waypoints found in CSV.")

        # Timer
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.control_loop)

    # -------------------------
    # Waypoint Management
    # -------------------------
    def load_waypoints(self, filename):
        if not os.path.exists(filename):
            self.get_logger().warn(f"CSV {filename} not found.")
            return []
        waypoints = []
        with open(filename, newline='') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for row in reader:
                try:
                    hdg = round(float(row[0]), 1)
                    waypoints.append(hdg)
                except Exception:
                    continue
        return waypoints

    def next_waypoint(self):
        if self.wp_index + 1 < len(self.waypoints):
            self.wp_index += 1
            self.target_heading = self.waypoints[self.wp_index]
            self.pid.reset()
            self.done_triggered = False
            self.get_logger().info(f"Switched to heading {self.target_heading}° (waypoint {self.wp_index+1}/{len(self.waypoints)})")
        else:
            self.get_logger().info("✅ All waypoints completed!")
            self.target_heading = None

    # -------------------------
    # Callbacks
    # -------------------------
    def cb_steer_feedback(self, msg: Float32):
        try:
            self.current_steer = round(float(msg.data), 1)
        except Exception:
            pass

    def cb_ahrs(self, msg: Float32MultiArray):
        try:
            _, _, hdg = msg.data
            self.current_heading = round(float(hdg), 1)
        except Exception:
            pass

    def cb_done(self, msg: Bool):
        if msg.data and not self.done_triggered:
            self.get_logger().info("✅ /done=True received → Switching waypoint")
            self.done_triggered = True
            self.next_waypoint()

    # -------------------------
    # Helpers
    # -------------------------
    def heading_error(self, target, current):
        error = target - current
        return round((error + 180) % 360 - 180, 1)

    def rate_limit_pwm(self, desired_pwm, now):
        dt = max(1e-6, now - self.last_pwm_time)
        max_delta = self.pwm_rate_limit * dt
        delta = desired_pwm - self.last_pwm
        if delta > max_delta:
            desired_pwm = self.last_pwm + max_delta
        elif delta < -max_delta:
            desired_pwm = self.last_pwm - max_delta
        return desired_pwm

    # -------------------------
    # Control Loop
    # -------------------------
    def control_loop(self):
        now = time.time()
        if self.current_heading is None or self.target_heading is None:
            return

        # Compute heading error
        error_heading = self.heading_error(self.target_heading, self.current_heading)

        # PID control
        desired_steer = self.steering_map_k * error_heading
        steer_error = desired_steer - self.current_steer
        pwm_cmd = self.pid.step(steer_error, now=now)

        # Limit PWM
        pwm_cmd = max(-self.max_pwm, min(self.max_pwm, pwm_cmd))
        if abs(pwm_cmd) > 1e-3 and abs(pwm_cmd) < self.min_pwm:
            pwm_cmd = math.copysign(self.min_pwm, pwm_cmd)

        pwm_cmd = self.rate_limit_pwm(pwm_cmd, now)
        self.last_pwm = pwm_cmd
        self.last_pwm_time = now

        # Publish PWM
        out_msg = Float32()
        out_msg.data = round(float(pwm_cmd), 2)
        self.steer_pub.publish(out_msg)

        self.get_logger().info(
            f"Target={self.target_heading}°, Curr={self.current_heading}°, Err={error_heading}°, PWM={pwm_cmd:.2f}"
        )


# -------------------------
# Main
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Controller stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

