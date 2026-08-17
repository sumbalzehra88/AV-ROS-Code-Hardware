// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "heading_controller_pkg/ahrs_sim_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using NavSatFix = sensor_msgs::msg::NavSatFix;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

// ==========================================
// GNSS Heading Node Class
// GNSS-only heading publisher: computes course-over-ground bearing from
// successive NavSatFix messages, smooths it with a 1D Kalman filter plus
// optional EMA, and publishes [roll, pitch, heading_deg] on jmoab/ahrs.
// (bearing_deg() / shortest_signed_deg_diff() / wrap360() / gnss_distance_m()
// / HeadingKalman1D / process_gps_fix() / update_ema_heading() now live in
// gnss_heading_utils.hpp so they can be unit-tested directly with gtest.)
// ==========================================
class GNSSHeadingNode : public rclcpp::Node {
public:
    GNSSHeadingNode() : Node("gnss_heading_node") {
        RCLCPP_INFO(this->get_logger(), "Starting GNSS Heading Node (GNSS-only bearing → jmoab/ahrs)");

        // ----------------------------------------------------
        // Declare ROS 2 Parameters
        // ----------------------------------------------------
        this->declare_parameter<float>("min_speed_m_s", 0.5f);
        this->declare_parameter<float>("min_distance_m", 0.5f);
        this->declare_parameter<float>("max_sample_age_s", 2.0f);
        this->declare_parameter<float>("kalman_measurement_var", 8.0f);
        this->declare_parameter<float>("kalman_process_var", 0.01f);
        this->declare_parameter<float>("ema_alpha", 0.6f);
        this->declare_parameter<bool>("show_log", false);

        // ----------------------------------------------------
        // Load Parameters into Member Variables
        // ----------------------------------------------------
        params_.min_speed_m_s = static_cast<float>(this->get_parameter("min_speed_m_s").as_double());
        params_.min_distance_m = static_cast<float>(this->get_parameter("min_distance_m").as_double());
        params_.max_sample_age_s = static_cast<float>(this->get_parameter("max_sample_age_s").as_double());
        float kalman_measurement_var = static_cast<float>(this->get_parameter("kalman_measurement_var").as_double());
        float kalman_process_var = static_cast<float>(this->get_parameter("kalman_process_var").as_double());
        ema_alpha_ = static_cast<float>(this->get_parameter("ema_alpha").as_double());
        show_log_ = this->get_parameter("show_log").as_bool();

        // ----------------------------------------------------
        // Dynamic Parameter Callback (mirrors Python's _on_param_change)
        // ----------------------------------------------------
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&GNSSHeadingNode::on_param_change, this, std::placeholders::_1));

        // ----------------------------------------------------
        // Initialize State
        // ----------------------------------------------------
        fix_state_ = FixState{};
        kalman_ = HeadingKalman1D(0.0f, 10.0f, kalman_measurement_var, kalman_process_var);
        ema_heading_ = std::nullopt;
        roll_ = 0.0f;
        pitch_ = 0.0f;

        // ----------------------------------------------------
        // Setup Subscriptions and Publications
        // ----------------------------------------------------
        ahrs_pub_ = this->create_publisher<Float32MultiArray>("jmoab/ahrs", 10);
        gps_sub_ = this->create_subscription<NavSatFix>(
            "/fix", 10,
            std::bind(&GNSSHeadingNode::gps_callback, this, std::placeholders::_1)
        );

        // ----------------------------------------------------
        // Setup Periodic Publish Timer (keeps topic alive even with no new heading)
        // ----------------------------------------------------
        publish_hz_ = 5.0f;
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(1.0f / publish_hz_),
            std::bind(&GNSSHeadingNode::publish_timer_cb, this)
        );

        RCLCPP_INFO(this->get_logger(), "GNSS Heading Node initialized - waiting for /fix messages");
    }

private:
    // ==========================================
    // Dynamic Parameter Callback
    // ==========================================
    SetParametersResult on_param_change(const std::vector<rclcpp::Parameter>& params) {
        for (const auto& p : params) {
            if (p.get_name() == "min_speed_m_s") {
                params_.min_speed_m_s = static_cast<float>(p.as_double());
            } else if (p.get_name() == "min_distance_m") {
                params_.min_distance_m = static_cast<float>(p.as_double());
            } else if (p.get_name() == "max_sample_age_s") {
                params_.max_sample_age_s = static_cast<float>(p.as_double());
            } else if (p.get_name() == "kalman_measurement_var") {
                kalman_.set_measurement_var(static_cast<float>(p.as_double()));
            } else if (p.get_name() == "kalman_process_var") {
                kalman_.set_process_var(static_cast<float>(p.as_double()));
            } else if (p.get_name() == "ema_alpha") {
                ema_alpha_ = static_cast<float>(p.as_double());
            } else if (p.get_name() == "show_log") {
                show_log_ = p.as_bool();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Parameters updated");
        SetParametersResult result;
        result.successful = true;
        return result;
    }

    // ==========================================
    // Callback: GPS Position
    // ==========================================
    void gps_callback(const NavSatFix::SharedPtr msg) {
        double lat = msg->latitude;
        double lon = msg->longitude;
        double now = this->now().seconds();

        GpsUpdateResult result = process_gps_fix(fix_state_, lat, lon, now, params_);
        fix_state_ = result.next;

        if (result.is_first_fix) {
            if (show_log_) {
                RCLCPP_INFO(this->get_logger(), "Stored first GNSS sample, no bearing computed yet.");
            }
            return;
        }

        if (!result.accepted) {
            if (result.refreshed_stale && show_log_) {
                RCLCPP_INFO(this->get_logger(),
                            "GNSS sample refreshed (no bearing) dist=%.2f dt=%.2fs speed=%.2fm/s",
                            result.distance_m, result.dt_s, result.est_speed_mps);
            }
            return;
        }

        float filtered_heading = kalman_.update(static_cast<float>(result.raw_bearing_deg));
        ema_heading_ = update_ema_heading(ema_heading_, filtered_heading, ema_alpha_);

        if (show_log_) {
            RCLCPP_INFO(this->get_logger(),
                        "GNSS raw_brg=%.2f°, kalman=%.2f°, ema=%.2f°, dist=%.2fm dt=%.2fs speed=%.2fm/s",
                        result.raw_bearing_deg, filtered_heading, ema_heading_.value(),
                        result.distance_m, result.dt_s, result.est_speed_mps);
        }
    }

    // ==========================================
    // Periodic Publisher
    // ==========================================
    void publish_timer_cb() {
        Float32MultiArray msg;
        if (!ema_heading_.has_value()) {
            msg.data = {roll_, pitch_, 0.0f};
            ahrs_pub_->publish(msg);
            return;
        }

        float hdg = wrap360(ema_heading_.value());
        msg.data = {roll_, pitch_, hdg};
        ahrs_pub_->publish(msg);
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Publisher<Float32MultiArray>::SharedPtr ahrs_pub_;
    rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // ==========================================
    // Configuration Parameters
    // ==========================================
    GnssHeadingParams params_;
    float ema_alpha_;
    bool show_log_;
    float publish_hz_;

    // ==========================================
    // Internal State Variables
    // ==========================================
    FixState fix_state_;
    HeadingKalman1D kalman_{0.0f, 10.0f, 8.0f, 0.01f};
    std::optional<float> ema_heading_;
    float roll_;
    float pitch_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSHeadingNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "🛑 GNSS heading node stopped by user.");
    }

    rclcpp::shutdown();
    return 0;
}