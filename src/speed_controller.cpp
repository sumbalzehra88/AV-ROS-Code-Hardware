// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include "heading_controller_pkg/speed_controller_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using Float32 = std_msgs::msg::Float32;
using Int32 = std_msgs::msg::Int32;

// ==========================================
// GPS Speed Controller Node Class
// Mimics a human foot "tap" on the accelerator: presses the servo to
// servo_full until the target speed is reached, then performs a rebound
// sequence (rebound_angle -> neutral -> back to pressing).
// (ControllerState / ServoParams / compute_control_step() / clamp_servo()
// now live in speed_controller_utils.hpp so they can be unit-tested
// directly with gtest, without needing a running rclcpp::Node)
// ==========================================
class GpsSpeedController : public rclcpp::Node {
public:
    GpsSpeedController() : Node("gps_speed_controller") {
        // ----------------------------------------------------
        // Declare ROS 2 Parameters
        // ----------------------------------------------------
        this->declare_parameter<float>("target_speed_kmh", 0.8f);
        this->declare_parameter<float>("tolerance_kmh", 0.0f);
        this->declare_parameter<int>("servo_neutral", 160);
        this->declare_parameter<int>("servo_full", 150);
        this->declare_parameter<int>("rebound_angle", 180);
        this->declare_parameter<float>("speed_max_kmh", 4.0f);
        this->declare_parameter<float>("timer_hz", 20.0f);

        // ----------------------------------------------------
        // Load Parameters into Member Variables
        // ----------------------------------------------------
        params_.target_speed_kmh = static_cast<float>(this->get_parameter("target_speed_kmh").as_double());
        params_.tolerance_kmh = static_cast<float>(this->get_parameter("tolerance_kmh").as_double());
        params_.servo_neutral = this->get_parameter("servo_neutral").as_int();
        params_.servo_full = this->get_parameter("servo_full").as_int();
        params_.rebound_angle = this->get_parameter("rebound_angle").as_int();
        params_.speed_max_kmh = static_cast<float>(this->get_parameter("speed_max_kmh").as_double());
        float timer_hz = static_cast<float>(this->get_parameter("timer_hz").as_double());

        // ----------------------------------------------------
        // Setup Subscriptions and Publications
        // ----------------------------------------------------
        speed_sub_ = this->create_subscription<Float32>(
            "/speed", 10,
            std::bind(&GpsSpeedController::speed_callback, this, std::placeholders::_1)
        );

        accel_pub_ = this->create_publisher<Int32>("/pwm_a", 10);

        // ----------------------------------------------------
        // Initialize State Variables
        // ----------------------------------------------------
        latest_speed_kmh_ = 0.0f;
        current_servo_ = params_.servo_neutral;
        state_ = ControllerState::Pressing;
        rebound_tick_count_ = 0;

        // ----------------------------------------------------
        // Setup Control Loop Timer
        // ----------------------------------------------------
        float period = 1.0f / std::max(1.0f, timer_hz);
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(period),
            std::bind(&GpsSpeedController::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(),
                    "✅ Speed Controller started: Target = %.3f km/h | State = 'pressing'",
                    params_.target_speed_kmh);
    }

private:
    // ==========================================
    // Callback: Get current car speed from /speed topic
    // ==========================================
    void speed_callback(const Float32::SharedPtr msg) {
        latest_speed_kmh_ = msg->data;
    }

    // ==========================================
    // Publish servo command safely to /pwm_a
    // ==========================================
    void publish_servo(int angle) {
        int clamped = clamp_servo(angle);
        current_servo_ = clamped;
        Int32 msg;
        msg.data = clamped;
        accel_pub_->publish(msg);
    }

    // ==========================================
    // Helper: human-readable state name for logging
    // ==========================================
    static const char* state_name(ControllerState s) {
        switch (s) {
            case ControllerState::Pressing: return "pressing";
            case ControllerState::Rebound1: return "rebound1";
            case ControllerState::Rebound2: return "rebound2";
            case ControllerState::Neutral: return "neutral";
        }
        return "unknown";
    }

    // ==========================================
    // Core Execution Control Loop (main state machine)
    // ==========================================
    void control_loop() {
        ControlResult result = compute_control_step(state_, rebound_tick_count_, latest_speed_kmh_, params_);

        if (result.entered_overspeed) {
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ OVERSPEED: %.3f km/h > %.3f km/h → Switching to 'neutral'.",
                        latest_speed_kmh_, params_.speed_max_kmh);
        }
        if (result.resumed_from_overspeed) {
            RCLCPP_INFO(this->get_logger(), "✅ Safe again — resuming 'pressing' mode.");
        }
        if (result.target_reached) {
            RCLCPP_INFO(this->get_logger(),
                        "🎯 Target reached: %.3f km/h. Starting rebound sequence.",
                        latest_speed_kmh_);
        }

        state_ = result.next_state;
        rebound_tick_count_ = result.next_tick_count;
        publish_servo(result.servo_command);

        RCLCPP_DEBUG(this->get_logger(),
                     "[Loop] State=%s, Speed=%.3f km/h, Servo=%d",
                     state_name(state_), latest_speed_kmh_, current_servo_);
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Subscription<Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<Int32>::SharedPtr accel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==========================================
    // Configuration Parameters
    // ==========================================
    ServoParams params_;

    // ==========================================
    // Internal State Variables
    // ==========================================
    float latest_speed_kmh_;
    int current_servo_;
    ControllerState state_;
    int rebound_tick_count_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsSpeedController>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "🛑 Controller stopped by user.");
    }

    rclcpp::shutdown();
    return 0;
}