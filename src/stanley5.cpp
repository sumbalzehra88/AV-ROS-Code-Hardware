// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <array>
#include "heading_controller_pkg/stanley5_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using AckermannControlCommand = ackermann_msgs::msg::AckermannDriveStamped;
using Path = nav_msgs::msg::Path;
using Odometry = nav_msgs::msg::Odometry;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;

// ==========================================
// Stanley Controller Node Class
// (quat2eulers() / quaternion_rotation_matrix() / transpose() / matmul() /
// compute_yaw_from_rotation() / compute_cross_track_error() /
// compute_stanley_output() / StanleyGateState / handle_point_message() /
// handle_alignment_done() now live in stanley5_utils.hpp so they can be
// unit-tested directly with gtest, without needing a running rclcpp::Node)
// ==========================================
class StanleyControllerNode : public rclcpp::Node {
public:
    StanleyControllerNode() : Node("stanley_controller") {
        RCLCPP_INFO(this->get_logger(), "Stanley Controller start!");

        // ----------------------------------------------------
        // State
        // ----------------------------------------------------
        gate_ = StanleyGateState{};
        speed_ = 0.0;
        K_ = 0.01;

        // ----------------------------------------------------
        // Setup Subscriptions
        // ----------------------------------------------------
        current_pose_sub_ = this->create_subscription<Odometry>(
            "/odometry", 10,
            std::bind(&StanleyControllerNode::current_pose_listener_callback, this, std::placeholders::_1));
        point_sub_ = this->create_subscription<Bool>(
            "/point", 10,
            std::bind(&StanleyControllerNode::point_callback, this, std::placeholders::_1));
        alignment_done_sub_ = this->create_subscription<Bool>(
            "/alignment_done", 10,
            std::bind(&StanleyControllerNode::alignment_done_callback, this, std::placeholders::_1));
        reference_trajectory_sub_ = this->create_subscription<Path>(
            "/path_points", 10,
            std::bind(&StanleyControllerNode::reference_trajectory_listener_callback, this, std::placeholders::_1));

        // ----------------------------------------------------
        // Setup Publishers
        // ----------------------------------------------------
        error_pub_ = this->create_publisher<Float32>("/cross_track_error", 10);

        rclcpp::QoS qos_policy(1);
        qos_policy.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_policy.durability(rclcpp::DurabilityPolicy::Volatile);

        control_pub_ = this->create_publisher<AckermannControlCommand>("/control/command/control_cmd", qos_policy);
        steering_angle_pub_ = this->create_publisher<Float32>("/steering_angle", qos_policy);

        // ----------------------------------------------------
        // Setup Timers
        // ----------------------------------------------------
        control_publisher_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 100.0),
            std::bind(&StanleyControllerNode::control_publisher_timer_callback, this));

        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.2),
            std::bind(&StanleyControllerNode::control_timer_callback, this));
    }

private:
    // ==========================================
    // Callbacks
    // ==========================================
    void point_callback(const Bool::SharedPtr msg) {
        bool was_enabled = gate_.enable_control;
        bool was_waiting = gate_.wait_alignment;
        handle_point_message(gate_, msg->data);

        if (msg->data && was_enabled && !gate_.enable_control) {
            RCLCPP_INFO(this->get_logger(), "🟡 Entered checkpoint zone → Stanley paused steering.");
        } else if (!msg->data && !was_waiting && !was_enabled && gate_.enable_control) {
            RCLCPP_INFO(this->get_logger(), "🟢 Outside checkpoint → Stanley resumed control.");
        }
    }

    void alignment_done_callback(const Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "✅ Alignment done → Stanley resuming steering.");
            handle_alignment_done(gate_, true);
        }
    }

    void current_pose_listener_callback(const Odometry::SharedPtr msg) {
        // Previous (only used as a "have we received >=2 updates" gate,
        // matching the Python -- the actual values aren't used in the
        // Stanley computation itself).
        last_time_ = curr_time_;
        last_x_ = curr_x_;
        last_y_ = curr_y_;

        // Position
        curr_x_ = msg->pose.pose.position.x;
        curr_y_ = msg->pose.pose.position.y;
        curr_z_ = msg->pose.pose.position.z;

        // Orientation
        curr_qw_ = msg->pose.pose.orientation.w;
        curr_qx_ = msg->pose.pose.orientation.x;
        curr_qy_ = msg->pose.pose.orientation.y;
        curr_qz_ = msg->pose.pose.orientation.z;

        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        speed_ = std::sqrt(vx * vx + vy * vy);

        curr_time_ = msg->header.stamp.nanosec;
    }

    void reference_trajectory_listener_callback(const Path::SharedPtr msg) {
        ref_path_.clear();
        for (const auto& pose_stamped : msg->poses) {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            ref_side_ = pose_stamped.pose.position.z;
            double qx = pose_stamped.pose.orientation.x;
            double qy = pose_stamped.pose.orientation.y;
            double qz = pose_stamped.pose.orientation.z;
            double qw = pose_stamped.pose.orientation.w;
            ref_path_.push_back({x, y, qx, qy, qz, qw});
        }
    }

    // ==========================================
    // Publish helper
    // ==========================================
    void publish_control(double theta, double accel) {
        AckermannControlCommand acc;
        acc.drive.steering_angle = static_cast<float>(theta);
        acc.drive.acceleration = static_cast<float>(accel);
        control_pub_->publish(acc);
    }

    // ==========================================
    // Timer: republish latest Ackermann command at 100Hz
    // ==========================================
    void control_publisher_timer_callback() {
        if (!gate_.enable_control) {
            RCLCPP_DEBUG(this->get_logger(), "Stanley paused → waiting for alignment.");
            return;
        }
        if (theta_.has_value() && acceleration_.has_value()) {
            publish_control(theta_.value(), acceleration_.value());
            RCLCPP_INFO(this->get_logger(), "Controller output: theta: %.4f, acceleration: %.4f",
                        theta_.value(), acceleration_.value());
        } else {
            RCLCPP_INFO(this->get_logger(), "Stanley Controller wrong control!");
        }
    }

    // ==========================================
    // Timer: compute Stanley control law at 5Hz
    // ==========================================
    void control_timer_callback() {
        if (!gate_.enable_control) {
            return;  // pause publishing during alignment
        }
        if (ref_path_.empty() || !curr_x_.has_value() || !last_time_.has_value()) {
            return;
        }

        // ref_path_ entries: [x, y, qx, qy, qz, qw]
        const auto& ref0 = ref_path_[0];
        Matrix3 Ra = quaternion_rotation_matrix(ref0[5], ref0[2], ref0[3], ref0[4]);
        Matrix3 Rb = quaternion_rotation_matrix(curr_qw_.value(), curr_qx_.value(),
                                                 curr_qy_.value(), curr_qz_.value());
        Matrix3 R = matmul(transpose(Ra), Rb);
        double yaw = compute_yaw_from_rotation(R);

        double e = compute_cross_track_error(curr_x_.value(), curr_y_.value(), ref0[0], ref0[1], ref_side_);

        RCLCPP_INFO(this->get_logger(), "e: %.4f, yaw: %.4f", e, yaw);

        StanleyOutput out = compute_stanley_output(yaw, e, speed_, K_);
        theta_ = out.theta_deg;
        acceleration_ = 0.3;

        Float32 steer_msg;
        steer_msg.data = static_cast<float>(theta_.value());
        steering_angle_pub_->publish(steer_msg);

        Float32 error_msg;
        error_msg.data = static_cast<float>(out.cross_track_error);
        error_pub_->publish(error_msg);
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Subscription<Odometry>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<Bool>::SharedPtr point_sub_;
    rclcpp::Subscription<Bool>::SharedPtr alignment_done_sub_;
    rclcpp::Subscription<Path>::SharedPtr reference_trajectory_sub_;
    rclcpp::Publisher<Float32>::SharedPtr error_pub_;
    rclcpp::Publisher<AckermannControlCommand>::SharedPtr control_pub_;
    rclcpp::Publisher<Float32>::SharedPtr steering_angle_pub_;
    rclcpp::TimerBase::SharedPtr control_publisher_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ==========================================
    // Gating State (checkpoint/alignment pause logic)
    // ==========================================
    StanleyGateState gate_;

    // ==========================================
    // Pose / Orientation State
    // ==========================================
    std::optional<double> curr_x_, curr_y_, curr_z_;
    std::optional<double> curr_qw_, curr_qx_, curr_qy_, curr_qz_;
    std::optional<uint32_t> curr_time_;
    std::optional<uint32_t> last_time_;
    std::optional<double> last_x_, last_y_;
    double speed_;

    // ==========================================
    // Reference Trajectory State
    // ==========================================
    std::vector<std::array<double, 6>> ref_path_;  // [x, y, qx, qy, qz, qw]
    double ref_side_ = 0.0;

    // ==========================================
    // Control Output State
    // ==========================================
    std::optional<double> theta_;
    std::optional<double> acceleration_;
    double K_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}