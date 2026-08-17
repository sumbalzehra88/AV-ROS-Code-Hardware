// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include "heading_controller_pkg/steering_aligner2_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using Float32 = std_msgs::msg::Float32;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Bool = std_msgs::msg::Bool;

// ==========================================
// Steering Aligner Node Class (simpler variant)
// Proportional alignment with EMA smoothing, then a short hold/stabilize
// phase before publishing /alignment_done. No adaptive gain, slew
// limiting, or oscillation detection -- see dynamic_steering_aligner.cpp
// for the more advanced version.
//
// NOTE: this node uses the SAME ROS node name ("steering_aligner") as
// dynamic_steering_aligner.cpp. Running both at once will fail with a
// duplicate-node-name error -- remap one at launch if you need both,
// e.g. --ros-args -r __node:=steering_aligner2.
//
// (angle_diff() / apply_ema() / compute_aligner2_step() now live in
// steering_aligner2_utils.hpp so they can be unit-tested directly with
// gtest, without needing a running rclcpp::Node)
// ==========================================
class SteeringAligner : public rclcpp::Node {
public:
    SteeringAligner() : Node("steering_aligner") {
        // ----------------------------------------------------
        // Declare ROS 2 Parameters (defaults match the Python node)
        // ----------------------------------------------------
        this->declare_parameter<float>("tolerance_deg", 2.0f);
        this->declare_parameter<float>("max_pwm", 17.0f);
        this->declare_parameter<float>("min_pwm", 0.0f);
        this->declare_parameter<float>("ema_alpha", 0.5f);
        this->declare_parameter<float>("hold_duration", 0.26f);
        this->declare_parameter<float>("disable_time", 0.0f);

        // ----------------------------------------------------
        // Load Parameters into Member Variables
        // ----------------------------------------------------
        params_.tolerance_deg = static_cast<float>(this->get_parameter("tolerance_deg").as_double());
        params_.max_pwm = static_cast<float>(this->get_parameter("max_pwm").as_double());
        params_.min_pwm = static_cast<float>(this->get_parameter("min_pwm").as_double());
        params_.ema_alpha = static_cast<float>(this->get_parameter("ema_alpha").as_double());
        params_.hold_duration = static_cast<float>(this->get_parameter("hold_duration").as_double());
        params_.disable_time = static_cast<double>(this->get_parameter("disable_time").as_double());

        // ----------------------------------------------------
        // Setup Publishers
        // ----------------------------------------------------
        rclcpp::QoS qos_policy(1);
        qos_policy.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_policy.durability(rclcpp::DurabilityPolicy::Volatile);

        steer_pub_ = this->create_publisher<Float32>("/steering_angle", qos_policy);
        // Matches the Python: align_done_pub uses default QoS (depth 10),
        // NOT the custom qos_policy used for steer_pub.
        align_done_pub_ = this->create_publisher<Bool>("/alignment_done", 10);

        // ----------------------------------------------------
        // Setup Subscriptions
        // ----------------------------------------------------
        aligned_sub_ = this->create_subscription<Float32>(
            "/aligned", 10, std::bind(&SteeringAligner::aligned_callback, this, std::placeholders::_1));
        compass_sub_ = this->create_subscription<Float32MultiArray>(
            "/jmoab_compass", 10, std::bind(&SteeringAligner::compass_callback, this, std::placeholders::_1));

        // ----------------------------------------------------
        // Initialize State
        // ----------------------------------------------------
        state_ = Aligner2State{};

        // ----------------------------------------------------
        // Setup Control Loop Timer (200Hz, matching the Python's 0.005s period)
        // ----------------------------------------------------
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.005),
            std::bind(&SteeringAligner::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🧭 Steering Aligner Node started.");
    }

private:
    // ==========================================
    // Callbacks
    // ==========================================
    void aligned_callback(const Float32::SharedPtr msg) {
        double now = this->now().seconds();
        if (now - state_.last_disable_time < params_.disable_time) {
            return;  // Ignore if still in cooldown
        }
        state_.target_heading = wrap360(msg->data);
        state_.active = true;
        state_.holding = false;
        RCLCPP_INFO(this->get_logger(), "🎯 New target heading received: %.2f°", state_.target_heading.value());
    }

    void compass_callback(const Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            state_.current_heading = wrap360(msg->data[2]);
        }
    }

    // ==========================================
    // Control Loop
    // ==========================================
    void control_loop() {
        double now = this->now().seconds();
        Aligner2StepResult result = compute_aligner2_step(state_, now, params_);

        if (result.entered_hold) {
            RCLCPP_INFO(this->get_logger(), "✅ Aligned! Holding for %.1fs to stabilize...", params_.hold_duration);
            return;
        }

        if (result.should_publish_pwm) {
            publish_pwm(result.pwm_value);
            return;
        }

        if (result.alignment_done) {
            publish_alignment_done();
            RCLCPP_INFO(this->get_logger(), "📡 Alignment complete and stabilized — released control.");
            return;
        }
    }

    // ==========================================
    // Publishers
    // ==========================================
    void publish_pwm(float pwm_value) {
        Float32 msg;
        msg.data = pwm_value;
        steer_pub_->publish(msg);
    }

    void publish_alignment_done() {
        Bool msg;
        msg.data = true;
        align_done_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "📡 Published /alignment_done = True");
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Publisher<Float32>::SharedPtr steer_pub_;
    rclcpp::Publisher<Bool>::SharedPtr align_done_pub_;
    rclcpp::Subscription<Float32>::SharedPtr aligned_sub_;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr compass_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==========================================
    // Configuration Parameters
    // ==========================================
    Aligner2Params params_;

    // ==========================================
    // Internal State
    // ==========================================
    Aligner2State state_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SteeringAligner>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "🛑 Steering Aligner interrupted by user.");
    }

    rclcpp::shutdown();
    return 0;
}