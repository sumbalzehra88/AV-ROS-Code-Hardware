#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class SteeringMonitorNode(Node):
    def __init__(self):
        super().__init__('steering_monitor')

        # Subscribers
        self.steering_sub = self.create_subscription(
            Float32, '/steering_angle', self.steering_callback, 10
        )

        # Publisher
        self.point_pub = self.create_publisher(Bool, '/point', 10)

        # State variables
        self.steering_state = "IDLE"
        self.rebound_direction = 1
        self.rebound_completed = False
        self.theta_deg = 0.0
        self.point_sent = False

        self.get_logger().info("Steering Monitor Node Started")

    def steering_callback(self, msg: Float32):
        self.theta_deg = msg.data
        point_msg = Bool()
        point_msg.data = False  # Default, will become True if sweep completes

        # --- Rebound / sweep logic ---
        if self.steering_state == "IDLE":
            if self.theta_deg >= 20.0:
                self.steering_state = "HIT_POS_LIMIT"
                self.rebound_direction = -1
            elif self.theta_deg <= -24.0:
                self.steering_state = "HIT_NEG_LIMIT"
                self.rebound_direction = 1

        elif self.steering_state == "HIT_POS_LIMIT":
            self.theta_deg = 20.0
            self.steering_state = "REBOUNDING"

        elif self.steering_state == "HIT_NEG_LIMIT":
            self.theta_deg = -24.0
            self.steering_state = "REBOUNDING"

        elif self.steering_state == "REBOUNDING":
            # Simulate rebound step (2° per callback, for example)
            self.theta_deg += self.rebound_direction * 1.0

            # Detect hitting limits and reverse
            if self.rebound_direction == -1 and self.theta_deg <= -24.0:
                self.rebound_direction = 1
                self.rebound_completed = True
                self.theta_deg = -24.0
            elif self.rebound_direction == 1 and self.theta_deg >= 20.0:
                self.rebound_direction = -1
                self.rebound_completed = True
                self.theta_deg = 20.0

            # --- Lock center condition ---
            if self.rebound_completed and ((self.rebound_direction == 1 and self.theta_deg <= 0) or
                                           (self.rebound_direction == -1 and self.theta_deg >= 0)):
                self.steering_state = "LOCK_CENTER"
                self.theta_deg = 0.0

        # --- Publish /point True if rebound sweep completed ---
        if self.rebound_completed and not self.point_sent:
            point_msg.data = True
            self.point_sent = True
            self.get_logger().info("✅ Full rebound sweep completed → Publishing /point True")
        else:
            point_msg.data = False

        # Publish
        self.point_pub.publish(point_msg)
        self.get_logger().info(f"Steering: {self.theta_deg:.2f}°, State: {self.steering_state}, /point: {point_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SteeringMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

