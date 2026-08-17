#!/usr/bin/env python3
"""
gps_distance_velocity_node.py

ROS 2 Node to compute:
- Ground (horizontal) speed from GPS velocity data
- Distance traveled between consecutive GPS coordinate updates
- Convert ground speed (m/s) to car speed (km/h)
- Publish current car speed on topic `/speed`

Author: Inzamam ul Haq
Date: October 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import numpy as np


# -------------------------------------------------
# Utility: Speed conversion chart and function
# -------------------------------------------------
speed_chart: dict[str, float] = {
    "km/h": 1.0,
    "m/s": 3.6,
    "mph": 1.609344,
    "knot": 1.852,
}

speed_chart_inverse: dict[str, float] = {
    "km/h": 1.0,
    "m/s": 0.277777778,
    "mph": 0.621371192,
    "knot": 0.539956803,
}


def convert_speed(speed: float, unit_from: str, unit_to: str) -> float:
    """
    Convert speed from one unit to another using the charts above.
    Example:
      convert_speed(10, "m/s", "km/h") → 36.0
    """
    if unit_to not in speed_chart or unit_from not in speed_chart_inverse:
        msg = (
            f"Incorrect 'from_type' or 'to_type' value: {unit_from!r}, {unit_to!r}\n"
            f"Valid values are: {', '.join(speed_chart_inverse)}"
        )
        raise ValueError(msg)
    return round(speed * speed_chart[unit_from] * speed_chart_inverse[unit_to], 3)


# -------------------------------------------------
# Main Node Class
# -------------------------------------------------
class GPSDistanceVelocityNode(Node):
    def __init__(self):
        super().__init__('gps_distance_velocity_node')

        # ---- Variables ----
        self.prev_lat = None
        self.prev_lon = None
        self.total_distance = 0.0  # meters
        self.gps_abs_vel = 0.0     # m/s
        self.car_speed_kmh = 0.0   # km/h

        # ---- Publishers ----
        self.speed_pub = self.create_publisher(Float32, '/speed', 10)
        self.speed_pub_mps = self.create_publisher(Float32, '/speed_mps', 10)

        # ---- Subscribers ----
        self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            '/vel',
            self.gps_vel_callback,
            10
        )

        self.get_logger().info("✅ GPS Distance & Velocity Node Started")

        # ---- Timer for periodic logging ----
        self.timer = self.create_timer(2.0, self.timer_callback)

    # -------------------------------------------------
    # Callback: GPS Position
    # -------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        """Called whenever a new GPS fix message is received."""
        lat = msg.latitude
        lon = msg.longitude

        if self.prev_lat is not None and self.prev_lon is not None:
            dist = self.get_distance(self.prev_lat, self.prev_lon, lat, lon)
            self.total_distance += dist
            self.get_logger().info(
                f"📍 Distance from last point: {dist:.3f} m  |  Total: {self.total_distance:.3f} m"
            )

        # Update previous coordinates
        self.prev_lat = lat
        self.prev_lon = lon

    # -------------------------------------------------
    # Callback: GPS Velocity
    # -------------------------------------------------
    def gps_vel_callback(self, msg: TwistStamped):
        """Compute horizontal ground speed and convert to km/h."""
        vel_x = msg.twist.linear.x
        vel_y = msg.twist.linear.y

        # Ground speed (horizontal magnitude)
        self.gps_abs_vel = np.sqrt(vel_x ** 2 + vel_y ** 2)

        # Convert m/s → km/h
        self.car_speed_kmh = convert_speed(self.gps_abs_vel, "m/s", "km/h")

        # Publish the car speed in km/h
        speed_msg_mps = Float32()
        speed_msg_mps.data = self.gps_abs_vel
        self.speed_pub_mps.publish(speed_msg_mps)
        speed_msg = Float32()
        speed_msg.data = self.car_speed_kmh
        self.speed_pub.publish(speed_msg)

        self.get_logger().info(
            f"🚗 Ground Speed: {self.gps_abs_vel:.3f} m/s  |  Car Speed: {self.car_speed_kmh:.3f} km/h"
        )

    # -------------------------------------------------
    # Compute Distance between two GPS points
    # -------------------------------------------------
    def get_distance(self, lat1, lon1, lat2, lon2):
        """Returns distance in meters using the Haversine formula."""
        R = 6371.0 * 1000.0  # Earth radius in m

        lat_start = np.radians(lat1)
        lon_start = np.radians(lon1)
        lat_end = np.radians(lat2)
        lon_end = np.radians(lon2)

        dLat = lat_end - lat_start
        dLon = lon_end - lon_start

        a = (
            np.sin(dLat / 2.0) ** 2
            + np.cos(lat_start) * np.cos(lat_end) * np.sin(dLon / 2.0) ** 2
        )
        c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        d = c * R
        return d

    # -------------------------------------------------
    # Timer: Periodic summary
    # -------------------------------------------------
    def timer_callback(self):
        """Print current speed and total distance periodically."""
        self.get_logger().info(
            f"⏱️ Speed: {self.gps_abs_vel:.2f} m/s  ({self.car_speed_kmh:.2f} km/h)  |  Distance: {self.total_distance:.2f} m"
        )


# -------------------------------------------------
# Main Entry Point
# -------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GPSDistanceVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('❌ Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

