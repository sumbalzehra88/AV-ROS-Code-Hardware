// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <optional>
#include <algorithm>
#include <sstream>
#include <vector>
#include "heading_controller_pkg/speed_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using Float32 = std_msgs::msg::Float32;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Bool = std_msgs::msg::Bool;

// ==========================================
// Heading Controller Node Class
// (PID class and free helper functions now live in heading_utils.hpp so
// they can be unit-tested directly with gtest, without needing a running
// rclcpp::Node)
// ==========================================
class HeadingController : public rclcpp::Node {
public:
    HeadingController() : Node("heading_controller") {
        // ----------------------------------------------------
        // Declare ROS 2 Parameters (All using float data types)
        // ----------------------------------------------------
        this->declare_parameter<float>("rate_hz", 20.0f);
        this->declare_parameter<std::string>("steering_feedback_topic", "/steering_ang");
        this->declare_parameter<std::string>("steering_command_topic", "/steering_angle");
        this->declare_parameter<std::string>("ahrs_topic", "/jmoab/ahrs");
        this->declare_parameter<std::string>("done_topic", "/done");
        this->declare_parameter<std::string>("waypoint_csv", "clean.csv");
        this->declare_parameter<float>("deadband_yaw", 2.0f); 
        this->declare_parameter<float>("hold_time", 0.01f);
        this->declare_parameter<float>("max_pwm", 30.0f);
        this->declare_parameter<float>("min_pwm", 10.0f);
        this->declare_parameter<float>("pwm_rate_limit", 200.0f);
        this->declare_parameter<float>("steering_map_k", 1.0f);
        this->declare_parameter<float>("pid_kp", 2.0f);
        this->declare_parameter<float>("pid_ki", 0.0f);
        this->declare_parameter<float>("pid_kd", 0.2f);
        this->declare_parameter<float>("pid_integrator_limit", 50.0f);
        this->declare_parameter<float>("pid_output_limit", 100.0f);

        // ----------------------------------------------------
        // Load Parameters into Member Variables
        // ----------------------------------------------------
        rate_hz_ = this->get_parameter("rate_hz").as_double();
        steer_feedback_topic_ = this->get_parameter("steering_feedback_topic").as_string();
        steer_command_topic_ = this->get_parameter("steering_command_topic").as_string();
        ahrs_topic_ = this->get_parameter("ahrs_topic").as_string();
        done_topic_ = this->get_parameter("done_topic").as_string();
        csv_file_ = this->get_parameter("waypoint_csv").as_string();
        deadband_yaw_ = this->get_parameter("deadband_yaw").as_double();
        hold_time_ = this->get_parameter("hold_time").as_double();
        max_pwm_ = this->get_parameter("max_pwm").as_double();
        min_pwm_ = this->get_parameter("min_pwm").as_double();
        pwm_rate_limit_ = this->get_parameter("pwm_rate_limit").as_double();
        steering_map_k_ = this->get_parameter("steering_map_k").as_double();

        float pid_kp = static_cast<float>(this->get_parameter("pid_kp").as_double());
        float pid_ki = static_cast<float>(this->get_parameter("pid_ki").as_double());
        float pid_kd = static_cast<float>(this->get_parameter("pid_kd").as_double());
        float pid_int_lim = static_cast<float>(this->get_parameter("pid_integrator_limit").as_double());
        float pid_out_lim = static_cast<float>(this->get_parameter("pid_output_limit").as_double());

        // Initialize PID controller with parameter values
        pid_ = PID(pid_kp, pid_ki, pid_kd, pid_int_lim, pid_out_lim);

        // ----------------------------------------------------
        // Setup Subscriptions and Publications
        // ----------------------------------------------------
        steer_feedback_sub_ = this->create_subscription<Float32>(
            steer_feedback_topic_, 10,
            std::bind(&HeadingController::cb_steer_feedback, this, std::placeholders::_1)
        );

        ahrs_sub_ = this->create_subscription<Float32MultiArray>(
            ahrs_topic_, 10,
            std::bind(&HeadingController::cb_ahrs, this, std::placeholders::_1)
        );

        done_sub_ = this->create_subscription<Bool>(
            done_topic_, 10,
            std::bind(&HeadingController::cb_done, this, std::placeholders::_1)
        );

        steer_command_pub_ = this->create_publisher<Float32>(steer_command_topic_, 10);

        // ----------------------------------------------------
        // Initialize State Variables
        // ----------------------------------------------------
        current_steer_ = 0.0f;
        current_heading_ = std::nullopt;
        target_heading_ = std::nullopt;
        last_pwm_ = 0.0f;
        
        auto now_sec = std::chrono::steady_clock::now().time_since_epoch();
        last_pwm_time_ = std::chrono::duration<float>(now_sec).count();
        done_triggered_ = false;

        // Load waypoints from CSV file
        waypoints_ = load_waypoints(csv_file_);
        wp_index_ = 0;
        last_reach_time_ = std::nullopt;

        if (!waypoints_.empty()) {
            target_heading_ = waypoints_[0];
            RCLCPP_INFO(this->get_logger(), "Loaded %zu headings", waypoints_.size());
            RCLCPP_INFO(this->get_logger(), "Starting with heading %.2f°", target_heading_.value());
        } else {
            RCLCPP_WARN(this->get_logger(), "No waypoints found in CSV.");
        }

        // ----------------------------------------------------
        // Setup Control Loop Timer
        // ----------------------------------------------------
        float period = 1.0f / std::max(1.0f, rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(period),
            std::bind(&HeadingController::control_loop, this)
        );
    }

private:
    // ==========================================
    // Waypoint File Parser Utility
    // ==========================================
    std::vector<float> load_waypoints(const std::string& filename) {
        if (!std::filesystem::exists(filename)) {
            RCLCPP_WARN(this->get_logger(), "CSV %s not found.", filename.c_str());
            return {};
        }

        // Parsing itself is delegated to parse_waypoints_csv() in
        // heading_utils.hpp, which is unit-tested directly with gtest
        // (see test/test_heading_controller.cpp). Only the file-existence
        // check and logging stay here since they need this->get_logger().
        std::ifstream file(filename);
        std::vector<float> waypoints = parse_waypoints_csv(file);

        if (waypoints.empty()) {
            RCLCPP_WARN(this->get_logger(), "[load_waypoints] no waypoints parsed from %s (empty file, header-only, or all rows malformed)",
                        filename.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "[load_waypoints] parsed %zu waypoint(s) from %s",
                    waypoints.size(), filename.c_str());
        return waypoints;
    }

    // ==========================================
    // Waypoint Transition Logic
    // ==========================================
    void next_waypoint() {
        if (wp_index_ + 1 < waypoints_.size()) {
            wp_index_++;
            target_heading_ = waypoints_[wp_index_];
            pid_.reset();
            done_triggered_ = false;
            RCLCPP_INFO(this->get_logger(), "Switched to heading %.1f° (waypoint %zu/%zu)", 
                        target_heading_.value(), wp_index_ + 1, waypoints_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ All waypoints completed!");
            target_heading_ = std::nullopt;

            // Command the actuator to neutral so it doesn't hold the last
            // steering command indefinitely once there is nothing left to track.
            Float32 stop_msg;
            stop_msg.data = 0.0f;
            steer_command_pub_->publish(stop_msg);
            last_pwm_ = 0.0f;
        }
    }

    // ==========================================
    // Topic Callback Functions
    // ==========================================
    void cb_steer_feedback(const Float32::SharedPtr msg) {
        try {
            current_steer_ = std::round(msg->data * 10.0f) / 10.0f;
            RCLCPP_DEBUG(this->get_logger(), "[cb_steer_feedback] raw=%.3f -> current_steer_=%.1f",
                         msg->data, current_steer_);
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "[cb_steer_feedback] failed to parse steering feedback message");
        }
    }

    void cb_ahrs(const Float32MultiArray::SharedPtr msg) {
        try {
            if (msg->data.size() >= 3) {
                float hdg = msg->data[2];
                current_heading_ = std::round(hdg * 10.0f) / 10.0f;
                RCLCPP_DEBUG(this->get_logger(), "[cb_ahrs] raw=%.3f -> current_heading_=%.1f",
                             hdg, current_heading_.value());
            } else {
                RCLCPP_WARN(this->get_logger(), "[cb_ahrs] message has only %zu elements, need >= 3", msg->data.size());
            }
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "[cb_ahrs] failed to parse AHRS message");
        }
    }

    void cb_done(const Bool::SharedPtr msg) {
        if (msg->data && !done_triggered_) {
            RCLCPP_INFO(this->get_logger(), "✅ /done=True received → Switching waypoint");
            done_triggered_ = true;
            next_waypoint();
        }
    }

    // ==========================================
    // Mathematical & Helper Functions
    // (heading_error() itself is now the free function from
    // heading_utils.hpp — called directly below, no wrapper needed)
    // ==========================================
    float rate_limit_pwm(float desired_pwm, float now) {
        float dt = std::max(1e-6f, now - last_pwm_time_);
        float max_delta = pwm_rate_limit_ * dt;
        float delta = desired_pwm - last_pwm_;
        
        if (delta > max_delta) {
            desired_pwm = last_pwm_ + max_delta;
        } else if (delta < -max_delta) {
            desired_pwm = last_pwm_ - max_delta;
        }
        return desired_pwm;
    }

    // ==========================================
    // Core Execution Control Loop
    // ==========================================
    void control_loop() {
        float now = std::chrono::duration<float>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();

        // Ensure valid readings before computation
        if (!current_heading_.has_value() || !target_heading_.has_value()) {
            return;
        }

        // Compute angular error relative to current target heading
        float error_heading = heading_error(target_heading_.value(), current_heading_.value());

        // Inside the deadband, hold the actuator still instead of dithering
        // around the setpoint with repeated minimum-PWM corrections.
        if (std::abs(error_heading) <= deadband_yaw_) {
            if (std::abs(last_pwm_) > 1e-3f) {
                Float32 out_msg;
                out_msg.data = 0.0f;
                steer_command_pub_->publish(out_msg);
                last_pwm_ = 0.0f;
                last_pwm_time_ = now;
                pid_.reset();
            }
            RCLCPP_INFO(this->get_logger(), "Target=%.1f°, Curr=%.1f°, Err=%.1f° (within deadband)",
                        target_heading_.value(), current_heading_.value(), error_heading);
            return;
        }

        // Process error through PID controller loop
        float desired_steer = steering_map_k_ * error_heading;
        float steer_error = desired_steer - current_steer_;
        float pwm_cmd = pid_.step(steer_error, now);
        RCLCPP_DEBUG(this->get_logger(),
                     "[control_loop] desired_steer=%.2f current_steer_=%.2f steer_error=%.2f pid_raw_out=%.2f",
                     desired_steer, current_steer_, steer_error, pwm_cmd);

        // Enforce physical constraints on PWM output
        float pwm_before_floor = pwm_cmd;
        pwm_cmd = std::clamp(pwm_cmd, -max_pwm_, max_pwm_);
        if (std::abs(pwm_cmd) > 1e-3f && std::abs(pwm_cmd) < min_pwm_) {
            pwm_cmd = std::copysign(min_pwm_, pwm_cmd);
        }
        if (pwm_before_floor != pwm_cmd) {
            RCLCPP_DEBUG(this->get_logger(), "[control_loop] pwm clamped/floored: %.2f -> %.2f",
                         pwm_before_floor, pwm_cmd);
        }

        // Apply rate limitation and save current timestamp state
        float pwm_before_rate_limit = pwm_cmd;
        pwm_cmd = rate_limit_pwm(pwm_cmd, now);
        if (pwm_before_rate_limit != pwm_cmd) {
            RCLCPP_DEBUG(this->get_logger(), "[control_loop] pwm rate-limited: %.2f -> %.2f",
                         pwm_before_rate_limit, pwm_cmd);
        }
        last_pwm_ = pwm_cmd;
        last_pwm_time_ = now;

        // Publish resulting steering command
        Float32 out_msg;
        out_msg.data = std::round(pwm_cmd * 100.0f) / 100.0f;
        steer_command_pub_->publish(out_msg);

        // Log loop metrics
        RCLCPP_INFO(this->get_logger(), "Target=%.1f°, Curr=%.1f°, Err=%.1f°, PWM=%.2f",
                    target_heading_.value(), current_heading_.value(), error_heading, pwm_cmd);
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Subscription<Float32>::SharedPtr steer_feedback_sub_;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr ahrs_sub_;
    rclcpp::Subscription<Bool>::SharedPtr done_sub_;
    rclcpp::Publisher<Float32>::SharedPtr steer_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==========================================
    // Configuration Parameters (Float-based)
    // ==========================================
    float rate_hz_;
    std::string steer_feedback_topic_;
    std::string steer_command_topic_;
    std::string ahrs_topic_;
    std::string done_topic_;
    std::string csv_file_;
    float deadband_yaw_;
    float hold_time_;
    float max_pwm_;
    float min_pwm_;
    float pwm_rate_limit_;
    float steering_map_k_;

    // ==========================================
    // Internal State Variables
    // ==========================================
    PID pid_;
    float current_steer_;
    std::optional<float> current_heading_;
    std::optional<float> target_heading_;
    float last_pwm_;
    float last_pwm_time_;
    bool done_triggered_;
    std::vector<float> waypoints_;
    size_t wp_index_;
    std::optional<float> last_reach_time_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeadingController>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "🛑 Controller stopped by user.");
    }
    
    rclcpp::shutdown();
    return 0;
}