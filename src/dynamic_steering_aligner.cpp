// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/qos.hpp>
#include "heading_controller_pkg/dynamic_steering_aligner_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using Float32 = std_msgs::msg::Float32;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Bool = std_msgs::msg::Bool;

// ==========================================
// Steering Aligner Node Class
// Adaptive proportional gain + deadband + EMA smoothing + slew-rate
// limiting + oscillation detection, with a gentler hold/stabilize phase
// once the target heading is reached.
// (angle_diff() / adaptive_gain() / apply_ema() / slew_limit() /
// update_sign_history() / detect_oscillation() / compute_aligner_step()
// now live in dynamic_steering_aligner_utils.hpp so they can be
// unit-tested directly with gtest, without needing a running rclcpp::Node)
//
// NOTE: min_pwm is accepted as a parameter for parity with the Python
// node, but -- exactly like the Python -- it is NOT used for clamping;
// the clamp uses -abs(max_pwm) instead. Preserved faithfully.
// ==========================================
class SteeringAligner : public rclcpp::Node {
public:
    SteeringAligner() : Node("steering_aligner") {
        // ----------------------------------------------------
        // Declare ROS 2 Parameters (defaults match the Python node)
        // ----------------------------------------------------
        this->declare_parameter<float>("tolerance_deg", 2.0f);
        this->declare_parameter<float>("deadband_deg", 0.25f);
        this->declare_parameter<float>("max_pwm", 17.0f);
        this->declare_parameter<float>("min_pwm", -17.0f);
        this->declare_parameter<float>("k_min", 3.08f);
        this->declare_parameter<float>("k_max", 0.9f);
        this->declare_parameter<float>("gain_scale", 6.0f);
        this->declare_parameter<float>("ema_alpha_base", 0.25f);
        this->declare_parameter<float>("max_slew_rate", 80.0f);
        this->declare_parameter<float>("hold_duration", 5.0f);
        this->declare_parameter<float>("hold_k_max_factor", 1.1f);
        this->declare_parameter<float>("hold_ema_alpha_factor", 0.95f);
        this->declare_parameter<float>("osc_window_sec", 1.2f);
        this->declare_parameter<int>("osc_sign_changes_threshold", 5);
        this->declare_parameter<float>("osc_reduction_factor", 0.45f);
        this->declare_parameter<float>("osc_min_time_between_warnings", 1.5f);
        this->declare_parameter<float>("disable_time", 0.0f);
        this->declare_parameter<float>("loop_hz", 100.0f);

        // ----------------------------------------------------
        // Load Parameters into Member Variables
        // ----------------------------------------------------
        params_.tolerance_deg = static_cast<float>(this->get_parameter("tolerance_deg").as_double());
        params_.deadband_deg = static_cast<float>(this->get_parameter("deadband_deg").as_double());
        params_.max_pwm = static_cast<float>(this->get_parameter("max_pwm").as_double());
        params_.min_pwm = static_cast<float>(this->get_parameter("min_pwm").as_double());
        params_.k_min = static_cast<float>(this->get_parameter("k_min").as_double());
        params_.k_max = static_cast<float>(this->get_parameter("k_max").as_double());
        params_.gain_scale = static_cast<float>(this->get_parameter("gain_scale").as_double());
        params_.ema_alpha_base = static_cast<float>(this->get_parameter("ema_alpha_base").as_double());
        params_.max_slew_rate = static_cast<float>(this->get_parameter("max_slew_rate").as_double());
        params_.hold_duration = static_cast<float>(this->get_parameter("hold_duration").as_double());
        params_.hold_k_max_factor = static_cast<float>(this->get_parameter("hold_k_max_factor").as_double());
        params_.hold_ema_alpha_factor = static_cast<float>(this->get_parameter("hold_ema_alpha_factor").as_double());
        params_.osc_window_sec = static_cast<double>(this->get_parameter("osc_window_sec").as_double());
        params_.osc_sign_changes_threshold = this->get_parameter("osc_sign_changes_threshold").as_int();
        params_.osc_reduction_factor = static_cast<float>(this->get_parameter("osc_reduction_factor").as_double());
        params_.osc_min_time_between_warnings = static_cast<double>(this->get_parameter("osc_min_time_between_warnings").as_double());
        params_.disable_time = static_cast<double>(this->get_parameter("disable_time").as_double());
        float loop_hz = static_cast<float>(this->get_parameter("loop_hz").as_double());

        // ----------------------------------------------------
        // Setup Publishers & Subscribers
        // ----------------------------------------------------
        rclcpp::QoS qos_policy(1);
        qos_policy.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_policy.durability(rclcpp::DurabilityPolicy::Volatile);

        steer_pub_ = this->create_publisher<Float32>("/steering_angle", qos_policy);
        align_done_pub_ = this->create_publisher<Bool>("/alignment_done", qos_policy);

        aligned_sub_ = this->create_subscription<Float32>(
            "/aligned", 10, std::bind(&SteeringAligner::aligned_callback, this, std::placeholders::_1));
        compass_sub_ = this->create_subscription<Float32MultiArray>(
            "/jmoab_compass", 10, std::bind(&SteeringAligner::compass_callback, this, std::placeholders::_1));

        // ----------------------------------------------------
        // Initialize State
        // ----------------------------------------------------
        state_ = AlignerState{};
        state_.last_cmd_time = this->now().seconds();

        // ----------------------------------------------------
        // Setup Control Loop Timer
        // ----------------------------------------------------
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(1.0f / std::max(1.0f, loop_hz)),
            std::bind(&SteeringAligner::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
                    "🧭 Steering Aligner Node started (adaptive, deadband, slew, EMA, osc-detector).");
    }

private:
    // ==========================================
    // Callbacks
    // ==========================================
    void aligned_callback(const Float32::SharedPtr msg) {
        double now = this->now().seconds();
        if (now - state_.last_disable_time < params_.disable_time) {
            return;
        }
        state_.target_heading = wrap360(msg->data);
        state_.active = true;
        state_.holding = false;
        RCLCPP_INFO(this->get_logger(), "🎯 New target heading: %.2f°", state_.target_heading.value());
    }

    void compass_callback(const Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            state_.current_heading = wrap360(msg->data[2]);
        }
    }

    // ==========================================
    // Main control loop
    // ==========================================
    void control_loop() {
        double now = this->now().seconds();
        AlignerStepResult result = compute_aligner_step(state_, now, params_);

        if (result.oscillation_warning) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Oscillation detected — reducing aggressiveness temporarily.");
        }

        if (result.entered_hold) {
            RCLCPP_INFO(this->get_logger(), "✅ Aligned within ±%.2f°. Entering hold for %.1fs.",
                        params_.tolerance_deg, params_.hold_duration);
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
    AlignerParams params_;

    // ==========================================
    // Internal State
    // ==========================================
    AlignerState state_;
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