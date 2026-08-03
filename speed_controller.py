#!/usr/bin/env python3
"""
ROS2 node: GPS speed controller that mimics a human foot "tap" on accelerator:
 - Subscribes to `/speed` (Float32) to get current vehicle speed in km/h.
 - Publishes Int32 commands to `/pwm_a` for servo control.
 - Presses servo to `servo_full` (e.g., 154) until the target speed (km/h) is reached.
 - Once the target is reached, performs a rebound sequence:
     -> set servo to rebound_angle (170),
     -> then neutral (161),
     -> then back to pressing cycle.
 - Includes overspeed protection and safe fallback states.

Author: Inzamam ul Haq
Date: October 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class GpsSpeedController(Node):
    def __init__(self):
        super().__init__("gps_speed_controller")

        # --- Subscribers ---
        self.create_subscription(Float32, "/speed", self.speed_callback, 10)

        # --- Publishers ---
        self.accel_pub = self.create_publisher(Int32, "/pwm_a", 10)

        # --- Parameters (tweak if needed) ---
        self.declare_parameter("target_speed_kmh", 0.8)  # km/h
        self.declare_parameter("tolerance_kmh", 0.0)    # acceptable deviation around target
        self.declare_parameter("servo_neutral", 160)     # pedal not pressed
        self.declare_parameter("servo_full", 150)        # pedal fully pressed
        self.declare_parameter("rebound_angle", 180)     # angle to lift foot temporarily
        self.declare_parameter("speed_max_kmh", 4.0)     # safety max speed (km/h)
        self.declare_parameter("timer_hz", 20)           # control loop frequency

        # --- Load parameters ---
        self.target_speed_kmh = float(self.get_parameter("target_speed_kmh").value)
        self.tolerance_kmh = float(self.get_parameter("tolerance_kmh").value)
        self.servo_neutral = int(self.get_parameter("servo_neutral").value)
        self.servo_full = int(self.get_parameter("servo_full").value)
        self.rebound_angle = int(self.get_parameter("rebound_angle").value)
        self.speed_max_kmh = float(self.get_parameter("speed_max_kmh").value)
        self.timer_hz = float(self.get_parameter("timer_hz").value)

        # --- Internal state ---
        self.latest_speed_kmh = 0.0
        self.current_servo = self.servo_neutral
        self.state = "pressing"
        self._rebound_tick_count = 0  # for rebound timing

        # --- Timer for control loop ---
        period = 1.0 / max(1.0, self.timer_hz)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"✅ Speed Controller started: Target = {self.target_speed_kmh:.3f} km/h | State = '{self.state}'"
        )

    # -------------------------------------------------
    # Callback: Get current car speed from /speed topic
    # -------------------------------------------------
    def speed_callback(self, msg: Float32):
        """Receive the current car speed (km/h)."""
        self.latest_speed_kmh = msg.data

    # -------------------------------------------------
    # Publish servo command
    # -------------------------------------------------
    def publish_servo(self, angle: int):
        """Publish servo command safely to /pwm_a."""
        angle = int(round(angle))
        if angle < 0:
            angle = 0
        if angle > 255:
            angle = 255

        self.current_servo = angle
        msg = Int32()
        msg.data = angle
        try:
            self.accel_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish servo command: {e}")

    # -------------------------------------------------
    # Control Loop (main logic)
    # -------------------------------------------------
    def control_loop(self):
        """
        Main state machine for servo control.
        """
        # Safety override: stop if overspeed
        if self.latest_speed_kmh > self.speed_max_kmh:
            if self.state != "neutral":
                self.get_logger().warn(
                    f"⚠️ OVERSPEED: {self.latest_speed_kmh:.3f} km/h > {self.speed_max_kmh:.3f} km/h → Switching to 'neutral'."
                )
            self.state = "neutral"
            self.publish_servo(self.servo_neutral)
            return

        # If in neutral (due to overspeed), wait until speed is below threshold
        if self.state == "neutral":
            if self.latest_speed_kmh <= (self.speed_max_kmh - 0.05):
                self.get_logger().info("✅ Safe again — resuming 'pressing' mode.")
                self.state = "pressing"
            else:
                self.publish_servo(self.servo_neutral)
                return

        # --- Normal state machine behavior ---
        if self.state == "pressing":
            self.publish_servo(self.servo_full)

            # Check if target reached
            if abs(self.target_speed_kmh - self.latest_speed_kmh) <= self.tolerance_kmh:
                self.get_logger().info(
                    f"🎯 Target reached: {self.latest_speed_kmh:.3f} km/h. Starting rebound sequence."
                )
                self.state = "rebound1"
                self._rebound_tick_count = 0
                self.publish_servo(self.rebound_angle)
                return

        elif self.state == "rebound1":
            if self._rebound_tick_count == 0:
                self.publish_servo(self.rebound_angle)
                self._rebound_tick_count += 1
                return
            else:
                self.state = "rebound2"
                self._rebound_tick_count = 0
                self.publish_servo(self.servo_neutral)
                return

        elif self.state == "rebound2":
            if self._rebound_tick_count == 0:
                self.publish_servo(self.servo_neutral)
                self._rebound_tick_count += 1
                return
            else:
                self.state = "pressing"
                self._rebound_tick_count = 0
                self.publish_servo(self.servo_full)
                return

        else:
            self.get_logger().warn(f"Unknown state '{self.state}' — resetting to neutral.")
            self.state = "neutral"
            self.publish_servo(self.servo_neutral)

        # Debug info (optional)
        self.get_logger().debug(
            f"[Loop] State={self.state}, Speed={self.latest_speed_kmh:.3f} km/h, Servo={self.current_servo}"
        )


# -------------------------------------------------
# Main Entry Point
# -------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GpsSpeedController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
 
