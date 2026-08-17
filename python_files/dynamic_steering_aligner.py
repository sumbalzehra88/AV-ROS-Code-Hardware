#!/usr/bin/env python3
"""
Adaptive & Robust Steering Aligner Node — ROS2 Humble

Features:
 - Adaptive gain: stronger correction for large errors, gentle for small ones.
 - Deadband: avoids micro-corrections and hunting.
 - EMA smoothing of published steering command.
 - Slew-rate limiter to avoid abrupt jumps (prevents overshoot).
 - Oscillation detector: if repeated sign changes occur, temporarily reduce aggressiveness.
 - Separate, gentler behavior during HOLD/STABILIZE phase.

Topics:
 - Subscribes:
     /aligned (std_msgs/Float32)           -> target heading [deg]
     /jmoab_compass (std_msgs/Float32MultiArray) -> [roll,pitch,heading]
 - Publishes:
     /steering_angle (std_msgs/Float32)
     /alignment_done (std_msgs/Bool)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
import math
import time
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy


class SteeringAligner(Node):
    def __init__(self):
        super().__init__('steering_aligner')

        # ---------------------------
        # TUNABLE PARAMETERS (start here)
        # ---------------------------
        # Alignment tolerance (deg). Within ± this -> start hold phase.
        self.tolerance_deg = 2.0

        # Deadband (deg): errors inside this are considered zero (reduces hunting).
        self.deadband_deg = 0.25

        # PWM output limits (signed).
        self.max_pwm = 17.0
        self.min_pwm = -17.0

        # Adaptive gain shaping:
        self.k_min = 3.08    # gentle proportional scale for very small errors
        self.k_max = 0.9     # maximum proportional scale for large errors
        self.gain_scale = 6.0  # degrees over which gain rises toward k_max (larger -> slower ramp)

        # EMA smoothing for command: base alpha (0..1). Lower = more smoothing.
        self.ema_alpha_base = 0.25

        # Slew-rate limit (PWM units per second)
        self.max_slew_rate = 80.0

        # Hold / stabilization behavior
        self.hold_duration = 5.0
        self.hold_k_max_factor = 1.1  # fraction of k_max used during hold (much gentler)
        self.hold_ema_alpha_factor = 0.95  # stronger smoothing in hold

        # Oscillation detection parameters
        self.osc_window_sec = 1.2
        self.osc_sign_changes_threshold = 5
        self.osc_reduction_factor = 0.45  # reduce k_max by this factor while oscillating
        self.osc_min_time_between_warnings = 1.5

        # Timer frequency
        self.loop_hz = 100.0  # 100 Hz control loop

        # Cooldown after alignment (seconds)
        self.disable_time = 0.0

        # ---------------------------
        # Internal state
        # ---------------------------
        self.target_heading = None
        self.current_heading = None
        self.smoothed_pwm = 0.0
        self.last_pwm_cmd = 0.0
        self.last_cmd_time = time.time()
        self.active = True
        self.holding = False
        self.hold_start_time = None
        self.last_disable_time = 0.0

        # For oscillation detection (deque of (timestamp, sign))
        self.sign_history = deque()
        self.last_osc_warn_time = 0.0

        # ---------------------------
        # Publishers & Subscribers
        # ---------------------------
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.steer_pub = self.create_publisher(Float32, '/steering_angle', qos_policy)
        self.align_done_pub = self.create_publisher(Bool, '/alignment_done', qos_policy)

        self.create_subscription(Float32, '/aligned', self.aligned_callback, 10)
        self.create_subscription(Float32MultiArray, '/jmoab_compass', self.compass_callback, 10)

        self.create_timer(1.0 / self.loop_hz, self.control_loop)
        self.get_logger().info("🧭 Steering Aligner Node started (adaptive, deadband, slew, EMA, osc-detector).")

    # ---------------------------
    # Callbacks
    # ---------------------------
    def aligned_callback(self, msg: Float32):
        """New target heading in degrees (0..360)."""
        if time.time() - self.last_disable_time < self.disable_time:
            return
        self.target_heading = msg.data % 360.0
        self.active = True
        self.holding = False
        self.get_logger().info(f"🎯 New target heading: {self.target_heading:.2f}°")

    def compass_callback(self, msg: Float32MultiArray):
        """Expecting [roll, pitch, heading]."""
        if len(msg.data) >= 3:
            self.current_heading = msg.data[2] % 360.0

    # ---------------------------
    # Utility helpers
    # ---------------------------
    @staticmethod
    def angle_diff(a, b):
        """
        Smallest signed difference between headings a and b (degrees).
        Return in range (-180, +180].
        """
        d = (a - b + 180.0) % 360.0 - 180.0
        return d

    def apply_deadband(self, value, deadband_deg):
        """Convert deadband in degrees to approximate pwm deadband using k_min and scaling.
           But here we use direct degree deadband: if error_abs < deadband -> zero."""
        return 0.0 if abs(value) <= deadband_deg else value

    def adaptive_gain(self, error_abs, k_max_effective):
        """Exponential approach mapping |error| -> gain in [k_min, k_max_effective]."""
        # 1 - exp(-e/gain_scale) gives a smooth curve
        return self.k_min + (k_max_effective - self.k_min) * (1.0 - math.exp(-error_abs / max(1e-6, self.gain_scale)))

    def apply_ema(self, new_val, alpha):
        """EMA smoothing of command value."""
        self.smoothed_pwm = (alpha * new_val) + (1.0 - alpha) * self.smoothed_pwm
        return self.smoothed_pwm

    def slew_limit(self, desired, last, dt):
        """Limit rate of change of PWM command."""
        if dt <= 0:
            return desired
        max_delta = self.max_slew_rate * dt
        delta = desired - last
        if abs(delta) > max_delta:
            desired = last + math.copysign(max_delta, delta)
        return desired

    def update_sign_history(self, error):
        """Append sign record and prune old entries."""
        now = time.time()
        if error > 0:
            sign = 1
        elif error < 0:
            sign = -1
        else:
            sign = 0
        self.sign_history.append((now, sign))
        cutoff = now - self.osc_window_sec
        while self.sign_history and self.sign_history[0][0] < cutoff:
            self.sign_history.popleft()

    def detect_oscillation(self):
        """Return True if sign changes in recent window exceed threshold."""
        signs = [s for (_, s) in self.sign_history if s != 0]
        if len(signs) < 2:
            return False
        changes = 0
        prev = signs[0]
        for s in signs[1:]:
            if s != prev:
                changes += 1
                prev = s
        return changes >= self.osc_sign_changes_threshold

    # ---------------------------
    # Main control loop
    # ---------------------------
    def control_loop(self):
        now = time.time()
        dt = max(1e-6, now - self.last_cmd_time)
        self.last_cmd_time = now

        if self.current_heading is None or self.target_heading is None:
            return

        if not self.active and not self.holding:
            return

        # Signed heading error (target - current), positive => need to turn "positive" direction
        error = self.angle_diff(self.target_heading, self.current_heading)
        error_abs = abs(error)

        # Update oscillation detector sign history & check
        self.update_sign_history(error)
        oscillating = self.detect_oscillation()

        # If detected oscillation, reduce k_max temporarily and increase smoothing
        k_max_effective = self.k_max
        ema_alpha = self.ema_alpha_base
        if oscillating:
            k_max_effective = max(self.k_min, self.k_max * self.osc_reduction_factor)
            ema_alpha = max(0.02, self.ema_alpha_base * 0.6)
            # throttle warning logs
            if now - self.last_osc_warn_time > self.osc_min_time_between_warnings:
                self.get_logger().warn("⚠️ Oscillation detected — reducing aggressiveness temporarily.")
                self.last_osc_warn_time = now

        # If in aligning phase
        if not self.holding:
            # If within tolerance -> enter hold/stabilize phase
            if error_abs <= self.tolerance_deg:
                self.holding = True
                self.hold_start_time = now
                self.get_logger().info(f"✅ Aligned within ±{self.tolerance_deg:.2f}°. Entering hold for {self.hold_duration}s.")
                # don't send immediate zero; let hold handle gentle corrections
                return

            # Compute adaptive proportional gain
            gain = self.adaptive_gain(error_abs, k_max_effective)

            # Proportional command: sign preserved
            pwm_cmd = gain * error_abs

            # Clip to allowed range
            pwm_cmd = max(-abs(self.max_pwm), min(self.max_pwm, pwm_cmd))
            pwm_cmd = pwm_cmd if error >= 0 else -pwm_cmd

            # Apply a deadband in degrees -> convert to minimal pwm threshold
            # Simpler: if error_abs < deadband_deg -> 0
            if error_abs <= self.deadband_deg:
                pwm_cmd = 0.0

            # Slew-rate limit (smooth sudden jumps)
            pwm_cmd = self.slew_limit(pwm_cmd, self.last_pwm_cmd, dt)

            # EMA smoothing
            pwm_cmd = self.apply_ema(pwm_cmd, ema_alpha)

            # Publish
            self.publish_pwm(pwm_cmd)
            self.last_pwm_cmd = pwm_cmd

        # Holding / stabilization phase: make much gentler corrections and stronger smoothing
        else:
            hold_elapsed = now - self.hold_start_time
            if hold_elapsed < self.hold_duration:
                # Use much smaller max gain and smaller commands
                hold_k_max = max(self.k_min, k_max_effective * self.hold_k_max_factor)
                gain = self.adaptive_gain(error_abs, hold_k_max)

                pwm_cmd = gain * error_abs
                pwm_cmd = max(-abs(self.max_pwm), min(self.max_pwm, pwm_cmd))
                pwm_cmd = pwm_cmd if error >= 0 else -pwm_cmd

                # Slightly larger deadband during hold to avoid new micro corrections
                if error_abs <= (self.deadband_deg * 1.5):
                    pwm_cmd = 0.0

                # More conservative slew & EMA in hold
                pwm_cmd = self.slew_limit(pwm_cmd, self.last_pwm_cmd, dt)
                pwm_cmd = self.apply_ema(pwm_cmd, max(0.01, ema_alpha * self.hold_ema_alpha_factor))

                self.publish_pwm(pwm_cmd)
                self.last_pwm_cmd = pwm_cmd
                return
            else:
                # Finished hold phase -> publish alignment done and release control
                self.publish_alignment_done()
                self.active = False
                self.holding = False
                self.last_disable_time = now
                self.get_logger().info("📡 Alignment complete and stabilized — released control.")
                return

    # ---------------------------
    # Publishers
    # ---------------------------
    def publish_pwm(self, pwm_value):
        msg = Float32()
        msg.data = float(pwm_value)
        self.steer_pub.publish(msg)

    def publish_alignment_done(self):
        msg = Bool()
        msg.data = True
        self.align_done_pub.publish(msg)
        self.get_logger().info("📡 Published /alignment_done = True")


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

