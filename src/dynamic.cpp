// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "heading_controller_pkg/dynamic_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;

// ==========================================
// Steering Monitor Node Class
// Watches /steering_angle for a full lock-to-lock rebound sweep (hit one
// steering limit, sweep to the opposite limit, then lock to center) and
// publishes /point = True once, the moment that sweep completes.
// (SteeringState / SweepParams / SweepState / compute_steering_step() now
// live in steering_monitor_utils.hpp so they can be unit-tested directly
// with gtest, without needing a running rclcpp::Node)
// ==========================================
class SteeringMonitorNode : public rclcpp::Node {
public:
    SteeringMonitorNode() : Node("steering_monitor") {
        // ----------------------------------------------------
        // Declare ROS 2 Parameters
        // ----------------------------------------------------
        this->declare_parameter<float>("pos_limit_deg", 20.0f);
        this->declare_parameter<float>("neg_limit_deg", -24.0f);
        this->declare_parameter<float>("rebound_step_deg", 1.0f);

        // ----------------------------------------------------
        // Load Parameters into Member Variables
        // ----------------------------------------------------
        params_.pos_limit_deg = static_cast<float>(this->get_parameter("pos_limit_deg").as_double());
        params_.neg_limit_deg = static_cast<float>(this->get_parameter("neg_limit_deg").as_double());
        params_.rebound_step_deg = static_cast<float>(this->get_parameter("rebound_step_deg").as_double());

        // ----------------------------------------------------
        // Setup Subscriptions and Publications
        // ----------------------------------------------------
        steering_sub_ = this->create_subscription<Float32>(
            "/steering_angle", 10,
            std::bind(&SteeringMonitorNode::steering_callback, this, std::placeholders::_1)
        );

        point_pub_ = this->create_publisher<Bool>("/point", 10);

        // ----------------------------------------------------
        // Initialize State Variables
        // ----------------------------------------------------
        sweep_state_ = SweepState{};

        RCLCPP_INFO(this->get_logger(), "Steering Monitor Node Started");
    }

private:
    // ==========================================
    // Helper: human-readable state name for logging
    // ==========================================
    static const char* state_name(SteeringState s) {
        switch (s) {
            case SteeringState::Idle: return "IDLE";
            case SteeringState::HitPosLimit: return "HIT_POS_LIMIT";
            case SteeringState::HitNegLimit: return "HIT_NEG_LIMIT";
            case SteeringState::Rebounding: return "REBOUNDING";
            case SteeringState::LockCenter: return "LOCK_CENTER";
        }
        return "UNKNOWN";
    }

    // ==========================================
    // Callback: New steering angle reading
    // ==========================================
    void steering_callback(const Float32::SharedPtr msg) {
        SweepResult result = compute_steering_step(sweep_state_, msg->data, params_);
        sweep_state_ = result.next;

        if (result.point_value) {
            RCLCPP_INFO(this->get_logger(), "✅ Full rebound sweep completed → Publishing /point True");
        }

        Bool point_msg;
        point_msg.data = result.point_value;
        point_pub_->publish(point_msg);

        RCLCPP_INFO(this->get_logger(), "Steering: %.2f°, State: %s, /point: %s",
                    sweep_state_.theta_deg, state_name(sweep_state_.state),
                    result.point_value ? "True" : "False");
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Subscription<Float32>::SharedPtr steering_sub_;
    rclcpp::Publisher<Bool>::SharedPtr point_pub_;

    // ==========================================
    // Configuration Parameters
    // ==========================================
    SweepParams params_;

    // ==========================================
    // Internal State Variables
    // ==========================================
    SweepState sweep_state_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SteeringMonitorNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "🛑 Steering monitor stopped by user.");
    }

    rclcpp::shutdown();
    return 0;
}