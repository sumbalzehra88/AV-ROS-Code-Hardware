// ==========================================
// Basic Libraries & Headers
// ==========================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "heading_controller_pkg/gnss_checkpoint_monitor_utils.hpp"

// ==========================================
// Type Aliases for Standard ROS 2 Messages
// ==========================================
using NavSatFix = sensor_msgs::msg::NavSatFix;
using Bool = std_msgs::msg::Bool;
using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;
using String = std_msgs::msg::String;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using TwistStamped = geometry_msgs::msg::TwistStamped;

// ==========================================
// GNSS Checkpoint Monitor Node Class
// (haversine_m() / bearing_deg() / Checkpoint / parse_checkpoints_csv() /
// select_next_checkpoint() / decide_loop_action() now live in
// checkpoint_monitor_utils.hpp so they can be unit-tested directly with
// gtest, without needing a running rclcpp::Node)
// ==========================================
class GNSSCheckpointMonitor : public rclcpp::Node {
public:
    GNSSCheckpointMonitor() : Node("gnss_checkpoint_monitor") {
        // ----------------------------------------------------
        // Declare ROS 2 Parameters
        // ----------------------------------------------------
        this->declare_parameter<std::string>("checkpoints_file", "gnss_cheakpoint.yaml");
        this->declare_parameter<std::string>("file_type", "yaml");
        this->declare_parameter<std::string>("mode", "sequential");
        this->declare_parameter<float>("tolerance_m_default", 2.0f);
        this->declare_parameter<float>("heading_tolerance_deg", 1.0f);
        this->declare_parameter<float>("revisit_timeout", 30.0f);
        this->declare_parameter<float>("publish_aligned_rate", 5.0f);
        this->declare_parameter<float>("min_gps_age", 0.5f);
        this->declare_parameter<float>("max_pass_distance", 3.0f);

        // ----------------------------------------------------
        // Load Parameters into Member Variables
        // ----------------------------------------------------
        checkpoints_file_ = this->get_parameter("checkpoints_file").as_string();
        file_type_ = to_lower(this->get_parameter("file_type").as_string());
        mode_ = to_lower(this->get_parameter("mode").as_string());
        tol_default_ = static_cast<double>(this->get_parameter("tolerance_m_default").as_double());
        heading_tolerance_ = static_cast<double>(this->get_parameter("heading_tolerance_deg").as_double());
        revisit_timeout_ = static_cast<double>(this->get_parameter("revisit_timeout").as_double());
        publish_aligned_rate_ = static_cast<double>(this->get_parameter("publish_aligned_rate").as_double());
        max_pass_distance_ = static_cast<double>(this->get_parameter("max_pass_distance").as_double());

        // ----------------------------------------------------
        // Initialize State
        // ----------------------------------------------------
        active_idx_ = -1;
        gps_lat_ = std::nullopt;
        gps_lon_ = std::nullopt;
        hdg_ = std::nullopt;
        speed_ = 0.0;
        last_dist_ = std::nullopt;
        alignment_requested_ = false;
        alignment_publish_last_ = 0.0;
        point_flag_ = false;
        alignment_done_ = false;

        // ----------------------------------------------------
        // Setup Publications
        // ----------------------------------------------------
        pub_point_ = this->create_publisher<Bool>("/point", 10);
        pub_point_idx_ = this->create_publisher<Int32>("/point_idx", 10);
        pub_aligned_ = this->create_publisher<Float32>("/aligned", 10);
        pub_alignment_req_ = this->create_publisher<Bool>("/alignment_req", 10);
        pub_point_info_ = this->create_publisher<String>("/point_info", 10);

        // ----------------------------------------------------
        // Setup Subscriptions
        // ----------------------------------------------------
        gps_sub_ = this->create_subscription<NavSatFix>(
            "/fix", 10, std::bind(&GNSSCheckpointMonitor::gps_callback, this, std::placeholders::_1));
        compass_sub_ = this->create_subscription<Float32MultiArray>(
            "/jmoab_compass", 10, std::bind(&GNSSCheckpointMonitor::compass_callback, this, std::placeholders::_1));
        vel_sub_ = this->create_subscription<TwistStamped>(
            "/vel", 10, std::bind(&GNSSCheckpointMonitor::gps_vel_callback, this, std::placeholders::_1));
        alignment_done_sub_ = this->create_subscription<Bool>(
            "/alignment_done", 10, std::bind(&GNSSCheckpointMonitor::alignment_done_cb, this, std::placeholders::_1));

        // ----------------------------------------------------
        // Load Checkpoints
        // ----------------------------------------------------
        load_checkpoints(checkpoints_file_, file_type_);

        // ----------------------------------------------------
        // Setup Main Loop Timer (20 Hz)
        // ----------------------------------------------------
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.05),
            std::bind(&GNSSCheckpointMonitor::main_loop, this));

        RCLCPP_INFO(this->get_logger(), "GNSS Checkpoint Monitor started with %zu checkpoints, mode=%s",
                    checkpoints_.size(), mode_.c_str());
    }

private:
    static std::string to_lower(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
        return s;
    }

    // ==========================================
    // File loaders
    // ==========================================
    void load_checkpoints(const std::string& path_in, const std::string& ftype) {
        std::filesystem::path path(path_in);
        if (!path.is_absolute()) {
            path = std::filesystem::current_path() / path;
        }

        try {
            if (ftype == "yaml") {
                YAML::Node data = YAML::LoadFile(path.string());
                YAML::Node items = data["checkpoints"];
                int row_index = 0;
                for (const auto& it : items) {
                    Checkpoint cp;
                    std::optional<double> lat = it["lat"] ? std::optional<double>(it["lat"].as<double>()) : std::nullopt;
                    std::optional<double> latitude = it["latitude"] ? std::optional<double>(it["latitude"].as<double>()) : std::nullopt;
                    std::optional<double> lon = it["lon"] ? std::optional<double>(it["lon"].as<double>()) : std::nullopt;
                    std::optional<double> longitude = it["longitude"] ? std::optional<double>(it["longitude"].as<double>()) : std::nullopt;
                    cp.lat = truthy_or(lat, latitude, 0.0);
                    cp.lon = truthy_or(lon, longitude, 0.0);
                    cp.id = it["id"] ? it["id"].as<int>() : row_index;
                    cp.name = it["name"] ? it["name"].as<std::string>() : ("pt_" + std::to_string(cp.id));
                    std::optional<double> tol = it["tolerance_m"] ? std::optional<double>(it["tolerance_m"].as<double>()) : std::nullopt;
                    cp.tolerance_m = truthy_or_single(tol, tol_default_);
                    if (it["desired_heading_deg"] && !it["desired_heading_deg"].IsNull()) {
                        cp.desired_heading_deg = it["desired_heading_deg"].as<double>();
                    } else {
                        cp.desired_heading_deg = std::nullopt;
                    }
                    cp.behavior = it["behavior"] ? it["behavior"].as<std::string>() : "align_then_continue";
                    cp.visited = false;
                    cp.last_visited_time = 0.0;
                    checkpoints_.push_back(cp);
                    row_index++;
                }
            } else {
                std::ifstream file(path.string());
                checkpoints_ = parse_checkpoints_csv(file, tol_default_);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu checkpoints from %s",
                        checkpoints_.size(), path.string().c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load checkpoints from %s: %s",
                         path.string().c_str(), e.what());
        }
    }

    // ==========================================
    // Callbacks
    // ==========================================
    void gps_callback(const NavSatFix::SharedPtr msg) {
        gps_lat_ = msg->latitude;
        gps_lon_ = msg->longitude;
    }

    void compass_callback(const Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            hdg_ = msg->data[2];
        } else {
            RCLCPP_WARN(this->get_logger(), "compass_callback error: message has only %zu elements", msg->data.size());
        }
    }

    void gps_vel_callback(const TwistStamped::SharedPtr msg) {
        double vx = msg->twist.linear.x;
        double vy = msg->twist.linear.y;
        speed_ = std::sqrt(vx * vx + vy * vy);
    }

    void alignment_done_cb(const Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Received /alignment_done = True");
            alignment_done_ = true;
        }
    }

    // ==========================================
    // Main loop
    // ==========================================
    void main_loop() {
        if (!gps_lat_.has_value() || !gps_lon_.has_value()) {
            return;
        }

        if (active_idx_ == -1) {
            auto idx = select_next_checkpoint(checkpoints_, mode_, gps_lat_.value(), gps_lon_.value());
            if (!idx.has_value()) {
                return;
            }
            set_active(static_cast<int>(idx.value()));
        }

        const Checkpoint& active = checkpoints_[active_idx_];
        double d = haversine_m(gps_lat_.value(), gps_lon_.value(), active.lat, active.lon);

        publish_point_info(active, d);

        LoopDecision decision = decide_loop_action(active, d, point_flag_, alignment_done_, last_dist_, max_pass_distance_);
        point_flag_ = decision.point_flag_next;

        switch (decision.action) {
            case LoopAction::EnteredTolerance:
                publish_point(true);
                RCLCPP_INFO(this->get_logger(), "Checkpoint %d (%s) reached (d=%.3f m). publishing /point True",
                            active_idx_, active.name.c_str(), d);
                maybe_publish_aligned(active);
                break;

            case LoopAction::RequestAlignment:
                maybe_publish_aligned(active);
                break;

            case LoopAction::ClearAndAdvance:
                if (alignment_done_) {
                    RCLCPP_INFO(this->get_logger(), "Left tolerance after alignment done. Clearing point and advancing.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Left tolerance zone without alignment -> marking passed and advancing");
                }
                clear_and_advance();
                break;

            case LoopAction::MarkPassedAndAdvance:
                RCLCPP_WARN(this->get_logger(), "Checkpoint %d appears passed (d=%.2f m). Skipping.", active_idx_, d);
                mark_visited_and_advance();
                break;

            case LoopAction::None:
                break;
        }

        last_dist_ = d;

        if (alignment_done_ && active_idx_ != -1) {
            RCLCPP_INFO(this->get_logger(), "Alignment done - clearing /point and advancing.");
            clear_and_advance();
        }
    }

    // ==========================================
    // Helpers
    // ==========================================
    void maybe_publish_aligned(const Checkpoint& active) {
        if (!active.desired_heading_deg.has_value() || alignment_done_) {
            return;
        }
        double now = this->now().seconds();
        double period = 1.0 / std::max(1.0, publish_aligned_rate_);
        if (now - alignment_publish_last_ >= period) {
            Float32 aligned_msg;
            aligned_msg.data = static_cast<float>(active.desired_heading_deg.value());
            pub_aligned_->publish(aligned_msg);

            Bool req_msg;
            req_msg.data = true;
            pub_alignment_req_->publish(req_msg);

            alignment_publish_last_ = now;
        }
    }

    void publish_point_info(const Checkpoint& active, double d) {
        std::ostringstream oss;
        oss << "{\"active_idx\": " << active_idx_
            << ", \"active_name\": \"" << active.name << "\""
            << ", \"distance_m\": " << d
            << ", \"tolerance_m\": " << active.tolerance_m
            << ", \"gps\": [" << gps_lat_.value() << ", " << gps_lon_.value() << "]"
            << ", \"heading\": " << (hdg_.has_value() ? std::to_string(hdg_.value()) : "null")
            << ", \"speed_m_s\": " << speed_
            << "}";
        String msg;
        msg.data = oss.str();
        pub_point_info_->publish(msg);
    }

    void publish_point(bool val) {
        Bool point_msg;
        point_msg.data = val;
        pub_point_->publish(point_msg);

        Int32 idx_msg;
        idx_msg.data = val ? active_idx_ : -1;
        pub_point_idx_->publish(idx_msg);
    }

    void set_active(int idx) {
        active_idx_ = idx;
        point_flag_ = false;
        alignment_done_ = false;
        alignment_requested_ = false;
        last_dist_ = std::nullopt;
        RCLCPP_INFO(this->get_logger(), "Set active checkpoint -> idx=%d, name=%s",
                    idx, checkpoints_[idx].name.c_str());
        Int32 idx_msg;
        idx_msg.data = idx;
        pub_point_idx_->publish(idx_msg);
    }

    void mark_visited_and_advance() {
        if (active_idx_ < 0) {
            return;
        }
        checkpoints_[active_idx_].visited = true;
        checkpoints_[active_idx_].last_visited_time = this->now().seconds();
        RCLCPP_INFO(this->get_logger(), "Marked checkpoint %d visited.", active_idx_);

        active_idx_ = -1;
        point_flag_ = false;
        alignment_done_ = false;

        if (!gps_lat_.has_value() || !gps_lon_.has_value()) {
            return;
        }
        auto next_idx = select_next_checkpoint(checkpoints_, mode_, gps_lat_.value(), gps_lon_.value());
        if (next_idx.has_value()) {
            set_active(static_cast<int>(next_idx.value()));
        }
    }

    void clear_and_advance() {
        publish_point(false);
        Bool req_msg;
        req_msg.data = false;
        pub_alignment_req_->publish(req_msg);
        mark_visited_and_advance();
    }

    // ==========================================
    // ROS 2 Communication Handles
    // ==========================================
    rclcpp::Publisher<Bool>::SharedPtr pub_point_;
    rclcpp::Publisher<Int32>::SharedPtr pub_point_idx_;
    rclcpp::Publisher<Float32>::SharedPtr pub_aligned_;
    rclcpp::Publisher<Bool>::SharedPtr pub_alignment_req_;
    rclcpp::Publisher<String>::SharedPtr pub_point_info_;
    rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr compass_sub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<Bool>::SharedPtr alignment_done_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==========================================
    // Configuration Parameters
    // ==========================================
    std::string checkpoints_file_;
    std::string file_type_;
    std::string mode_;
    double tol_default_;
    double heading_tolerance_;   // unused by this node's own logic, kept for parity with Python
    double revisit_timeout_;     // currently unused, kept for parity with Python
    double publish_aligned_rate_;
    double max_pass_distance_;

    // ==========================================
    // Internal State Variables
    // ==========================================
    std::vector<Checkpoint> checkpoints_;
    int active_idx_;
    std::optional<double> gps_lat_;
    std::optional<double> gps_lon_;
    std::optional<double> hdg_;
    double speed_;
    std::optional<double> last_dist_;
    bool alignment_requested_;
    double alignment_publish_last_;
    bool point_flag_;
    bool alignment_done_;
};

// ==========================================
// Main Execution Entry Point
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSCheckpointMonitor>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "🛑 Checkpoint monitor stopped by user.");
    }

    rclcpp::shutdown();
    return 0;
}