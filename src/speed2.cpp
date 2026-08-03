// ==========================================
// Basic Libraries & Headers
// ==========================================
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <chrono>
#include <string>
#include <optional>
#include <unordered_map>
#include <stdexcept>
#include "heading_controller_pkg/speed2_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using NavSatFix = sensor_msgs::msg::NavSatFix;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Float32 = std_msgs::msg::Float32;

// ==========================================
// GPS Distance & Velocity Node Class
// (SPEED_CHART / SPEED_CHART_INVERSE / convert_speed() / get_distance()
// now live in gps_utils.hpp so they can be unit-tested directly with
// gtest, without needing a running rclcpp::Node)
// ==========================================
class GPSDistanceVelocityNode : public rclcpp::Node {
public:
    GPSDistanceVelocityNode() : Node("gps_distance_velocity_node") {
        // ----------------------------------------------------
        // Initialize State Variables
        // ----------------------------------------------------
        prev_lat_ = std::nullopt;
        prev_lon_ = std::nullopt;
        total_distance_ = 0.0f;
        gps_abs_vel_ = 0.0f;
        car_speed_kmh_ = 0.0f;

        // ----------------------------------------------------
        // Setup Subscriptions and Publications
        // ----------------------------------------------------
        speed_pub_ = this->create_publisher<Float32>("/speed", 10);
        speed_pub_mps_ = this->create_publisher<Float32>("/speed_mps", 10);

        gps_sub_ = this->create_subscription<NavSatFix>(
            "/fix", 10,
            std::bind(&GPSDistanceVelocityNode::gps_callback, this, std::placeholders::_1)
        );

        gps_vel_sub_ = this->create_subscription<TwistStamped>(
            "/vel", 10,
            std::bind(&GPSDistanceVelocityNode::gps_vel_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "✅ GPS Distance & Velocity Node Started");

        // ----------------------------------------------------
        // Setup Periodic Timer for Summary Logs
        // ----------------------------------------------------
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(2.0f),
            std::bind(&GPSDistanceVelocityNode::timer_callback, this)
        );
    }

private:
    // ==========================================
    // Callback: GPS Position
    // ==========================================
    void gps_callback(const NavSatFix::SharedPtr msg) {
        double lat = msg->latitude;
        double lon = msg->longitude;

        if (prev_lat_.has_value() && prev_lon_.has_value()) {
            double dist = get_distance(prev_lat_.value(), prev_lon_.value(), lat, lon);
            total_distance_ += static_cast<float>(dist);
            RCLCPP_INFO(this->get_logger(),
                        "📍 Distance from last point: %.3f m  |  Total: %.3f m",
                        dist, total_distance_);
        }

        prev_lat_ = lat;
        prev_lon_ = lon;
    }

    // ==========================================
    // Callback: GPS Velocity
    // ==========================================
    void gps_vel_callback(const TwistStamped::SharedPtr msg) {
        float vel_x = static_cast<float>(msg->twist.linear.x);
        float vel_y = static_cast<float>(msg->twist.linear.y);

        // Ground speed (horizontal magnitude)
        gps_abs_vel_ = std::sqrt(vel_x * vel_x + vel_y * vel_y);

        // Convert m/s → km/h
        try {
            car_speed_kmh_ = convert_speed(gps_abs_vel_, "m/s", "km/h");
        } catch (...) {
            car_speed_kmh_ = 0.0f;
        }

        // Publish speed in m/s and km/h
        Float32 speed_msg_mps;
        speed_msg_mps.data = gps_abs_vel_;
        speed_pub_mps_->publish(speed_msg_mps);

        Float32 speed_msg_kmh;
        speed_msg_kmh.data = car_speed_kmh_;
        speed_pub_->publish(speed_msg_kmh);

        RCLCPP_INFO(this->get_logger(),
                    "🚗 Ground Speed: %.3f m/s  |  Car Speed: %.3f km/h",
                    gps_abs_vel_, car_speed_kmh_);
    }

    // ==========================================
    // Timer: Periodic summary
    // ==========================================
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(),
                    "⏱️ Speed: %.2f m/s  (%.2f km/h)  |  Distance: %.2f m",
                    gps_abs_vel_, car_speed_kmh_, total_distance_);
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Publisher<Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<Float32>::SharedPtr speed_pub_mps_;
    rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr gps_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==========================================
    // Internal State Variables
    // ==========================================
    std::optional<double> prev_lat_;
    std::optional<double> prev_lon_;
    float total_distance_;
    float gps_abs_vel_;
    float car_speed_kmh_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSDistanceVelocityNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "❌ Node stopped by user.");
    }

    rclcpp::shutdown();
    return 0;
}