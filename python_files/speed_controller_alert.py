#!/usr/bin/env python3
"""
ROS2 node: GPS speed controller that mimics a human foot "tap" on accelerator:
 - Subscribes to:
      /speed    (Float32)   - current speed km/h
      /alert    (Bool)      - from GNSS checkpoint TRUE/FALSE
 - Publishes:
      /pwm_a    (Int32)     - servo actuator command

Alert Behavior:
 - When /alert = True:
       Immediately apply rebound_angle (170) to reduce speed.
       Hold this until /alert becomes False.
 - When /alert = False:
       Normal speed control state machine resumes.

Author: Inzamam ul Haq
Date: Updated November 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool


class GpsSpeedController(Node):
    def __init__(self):
        super().__init__("gps_speed_controller")

        # --------------------- Subscribers ---------------------
        self.create_subscription(Float32, "/speed", self.speed_callback, 10)
        self.create_subscription(Bool, "/alert", self.alert_callback, 10)

        # --------------------- Publisher -----------------------
        self.accel_pub = self.create_publisher(Int32, "/pwm_a", 10)

        # --------------------- Parameters -----------------------
        self.declare_parameter("target_speed_kmh", 1.0)
        self.declare_parameter("tolerance_kmh", 0.0)
        self.declare_parameter("servo_neutral", 164)
        self.declare_parameter("servo_full", 158)
        self.declare_parameter("rebound_angle", 170)
        self.declare_parameter("speed_max_kmh", 5.0)
        self.declare_parameter("timer_hz", 20.0)

        # --------------------- Load Parameters ------------------
        self.target_speed_kmh = float(self.get_parameter("target_speed_kmh").value)
        self.tolerance_kmh = float(self.get_parameter("tolerance_kmh").value)
        self.servo_neutral = int(self.get_parameter("servo_neutral").value)
        self.servo_full = int(self.get_parameter("servo_full").value)
        self.rebound_angle = int(self.get_parameter("rebound_angle").value)
        self.speed_max_kmh = float(self.get_parameter("speed_max_kmh").value)
        self.timer_hz = float(self.get_parameter("timer_hz").value)

        # --------------------- Internal States ------------------
        self.latest_speed_kmh = 0.0
        self.current_servo = self.servo_neutral
        self.state = "pressing"
        self._rebound_tick_count = 0

        # Alert flag
        self.alert_active = False

        # --------------------- Main Timer -----------------------
        period = 1.0 / max(1.0, self.timer_hz)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"🚀 Speed Controller started | Target={self.target_speed_kmh:.2f} km/h"
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def speed_callback(self, msg: Float32):
        self.latest_speed_kmh = msg.data

    def alert_callback(self, msg: Bool):
        """
        GNSS checkpoint alert.
        True  -> vehicle must slow down immediately (apply rebound)
        False -> resume normal speed control operation
        """
        self.alert_active = msg.data

        if self.alert_active:
            self.get_logger().warn("🔴 ALERT RECEIVED → Applying rebound angle to slow down!")
        else:
            self.get_logger().info("🟢 ALERT CLEARED → Resuming normal speed control.")

    # ================================================================
    # SERVO PUBLISH HELPER
    # ================================================================
    def publish_servo(self, angle: int):
        angle = max(0, min(255, int(angle)))
        self.current_servo = angle
        msg = Int32()
        msg.data = angle
        self.accel_pub.publish(msg)

    # ================================================================
    # MAIN CONTROL LOOP
    # ================================================================
    def control_loop(self):
        """
        Core logic. Now enhanced with ALERT override.
        """

        # -----------------------------------------------------
        # 1. ALERT OVERRIDE MODE
        # -----------------------------------------------------
        if self.alert_active:
            # Alert forces vehicle to slow down → apply rebound angle continuously
            self.publish_servo(self.rebound_angle)
            return  # exit — do NOT run the normal FSM

        # -----------------------------------------------------
        # 2. SAFETY OVERSPEED STOP
        # -----------------------------------------------------
        if self.latest_speed_kmh > self.speed_max_kmh:
            if self.state != "neutral":
                self.get_logger().warn(
                    f"⚠️ Overspeed {self.latest_speed_kmh:.2f} km/h → Neutral mode."
                )
            self.state = "neutral"
            self.publish_servo(self.servo_neutral)
            return

        # If in neutral, wait to resume
        if self.state == "neutral":
            if self.latest_speed_kmh <= (self.speed_max_kmh - 0.05):
                self.get_logger().info("🔄 Speed safe again → Resuming pressing mode.")
                self.state = "pressing"
            else:
                self.publish_servo(self.servo_neutral)
                return

        # -----------------------------------------------------
        # 3. NORMAL SPEED CONTROL STATE MACHINE
        # -----------------------------------------------------
        if self.state == "pressing":
            self.publish_servo(self.servo_full)

            if abs(self.latest_speed_kmh - self.target_speed_kmh) <= self.tolerance_kmh:
                self.get_logger().info("🎯 Target reached → Begin rebound cycle.")
                self.state = "rebound1"
                self._rebound_tick_count = 0
                self.publish_servo(self.rebound_angle)
                return

        elif self.state == "rebound1":
            if self._rebound_tick_count < 1:
                self.publish_servo(self.rebound_angle)
                self._rebound_tick_count += 1
                return
            else:
                self.state = "rebound2"
                self._rebound_tick_count = 0
                self.publish_servo(self.servo_neutral)
                return

        elif self.state == "rebound2":
            if self._rebound_tick_count < 1:
                self.publish_servo(self.servo_neutral)
                self._rebound_tick_count += 1
                return
            else:
                self.state = "pressing"
                self._rebound_tick_count = 0
                self.publish_servo(self.servo_full)
                return

        else:
            self.get_logger().warn(f"Unknown state '{self.state}' → Resetting.")
            self.state = "neutral"
            self.publish_servo(self.servo_neutral)

    # ================================================================
    # END OF CLASS
    # ================================================================


def main(args=None):
    rclpy.init(args=args)
    node = GpsSpeedController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

