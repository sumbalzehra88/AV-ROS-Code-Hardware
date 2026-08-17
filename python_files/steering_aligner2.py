#!/usr/bin/env python3
"""
Steering Aligner Node — ROS2 Humble

📘 Description:
Smoothly aligns the steering to a target heading using Exponential Moving Average (EMA).
After alignment (within tolerance), it holds and stabilizes the heading for a short duration
before sending a Boolean flag on `/alignment_done` = True. During the hold phase,
small corrections are made to ensure perfect heading stability before handing control
to the Stanley controller.

💡 Topics:
  - Subscribes:
      /aligned (std_msgs/Float32) → Target heading [degrees]
      /jmoab_compass (std_msgs/Float32MultiArray) → [roll, pitch, heading]
  - Publishes:
      /steering_angle (std_msgs/Float32)
      /alignment_done (std_msgs/Bool)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy


class SteeringAligner(Node):
    def __init__(self):
        super().__init__('steering_aligner')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.tolerance_deg = 2.0             # ± tolerance in degrees
        self.max_pwm = 17
        self.min_pwm = 0.0
        self.ema_alpha = 0.5                 # smoothing factor (0.0–1.0)
        self.hold_duration = 0.26             # seconds to hold and stabilize after alignment
        self.disable_time = 0.0              # seconds to pause after alignment
        self.last_disable_time = 0.0

        # --------------------------------------------------
        # Internal state
        # --------------------------------------------------
        self.target_heading = None
        self.current_heading = None
        self.smoothed_pwm = 0.0
        self.active = True                   # Whether aligner is actively controlling
        self.holding = False                 # Whether in holding/stabilization phase
        self.hold_start_time = None

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.steer_pub = self.create_publisher(Float32, '/steering_angle', qos_policy)
        self.align_done_pub = self.create_publisher(Bool, '/alignment_done', 10)

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
        self.create_subscription(Float32, '/aligned', self.aligned_callback, 10)
        self.create_subscription(Float32MultiArray, '/jmoab_compass', self.compass_callback, 10)

        # --------------------------------------------------
        # Timer
        # --------------------------------------------------
        self.create_timer(0.005, self.control_loop)
        self.get_logger().info("🧭 Steering Aligner Node started.")

    # ======================================================
    #  Callbacks
    # ======================================================
    def aligned_callback(self, msg: Float32):
        """New target heading (degrees)"""
        if time.time() - self.last_disable_time < self.disable_time:
            return  # Ignore if still in cooldown
        self.target_heading = msg.data % 360.0
        self.active = True
        self.holding = False
        self.get_logger().info(f"🎯 New target heading received: {self.target_heading:.2f}°")

    def compass_callback(self, msg: Float32MultiArray):
        """Current heading from compass (degrees)"""
        if len(msg.data) >= 3:
            self.current_heading = msg.data[2] % 360.0

    # ======================================================
    #  Helper Functions
    # ======================================================
    def angle_diff(self, a, b):
        """Smallest signed difference between two headings (degrees)"""
        d = (a - b + 180.0) % 360.0 - 180.0
        return d

    def apply_ema(self, new_val):
        """Smooths PWM updates"""
        self.smoothed_pwm = (self.ema_alpha * new_val) + (1 - self.ema_alpha) * self.smoothed_pwm
        return self.smoothed_pwm

    # ===========================================================
    #  Control Loop
    # ===========================================================
    def control_loop(self):
        if self.current_heading is None or self.target_heading is None:
            return

        # Skip if not active or not holding
        if not self.active and not self.holding:
            return

        # Compute heading error
        error = self.angle_diff(self.target_heading, self.current_heading)

        # ==================================================
        # 1️⃣ ALIGNING PHASE
        # ==================================================
        if not self.holding:
            if abs(error) <= self.tolerance_deg:
                # Enter holding phase
                self.holding = True
                self.hold_start_time = time.time()
                self.get_logger().info(
                    f"✅ Aligned! Holding for {self.hold_duration:.1f}s to stabilize..."
                )
                return
            else:
                # Keep aligning
                pwm_cmd = (abs(error) / 2.0) * self.max_pwm
                pwm_cmd = min(max(pwm_cmd, self.min_pwm), self.max_pwm)
                if error < 0:
                    pwm_cmd = -pwm_cmd
                pwm_cmd = self.apply_ema(pwm_cmd)
                self.publish_pwm(pwm_cmd)

        # ==================================================
        # 2️⃣ HOLDING PHASE (STABILIZATION)
        # ==================================================
        elif self.holding:
            hold_elapsed = time.time() - self.hold_start_time

            # If still within hold time
            if hold_elapsed < self.hold_duration:
                # Keep small corrections
                pwm_cmd = (abs(error) / 2.0) * self.max_pwm
                pwm_cmd = min(max(pwm_cmd, self.min_pwm), self.max_pwm)
                if error < 0:
                    pwm_cmd = -pwm_cmd
                pwm_cmd = self.apply_ema(pwm_cmd)
                self.publish_pwm(pwm_cmd)
                return

            # ==================================================
            # 3️⃣ FINALIZE AFTER HOLD
            # ==================================================
            else:
                self.publish_alignment_done()
                self.active = False
                self.holding = False
                self.last_disable_time = time.time()
                self.get_logger().info(
                    f"📡 Alignment complete and stabilized — released control."
                )

    # ======================================================
    #  Publishers
    # ======================================================
    def publish_pwm(self, pwm_value):
        msg = Float32()
        msg.data = pwm_value
        self.steer_pub.publish(msg)

    def publish_alignment_done(self):
        msg = Bool()
        msg.data = True
        self.align_done_pub.publish(msg)
        self.get_logger().info("📡 Published /alignment_done = True")

# ==========================================================
#  Main
# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = SteeringAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Steering Aligner interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

