#ifndef HEADING_CONTROLLER_PKG__SPEED_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__SPEED_UTILS_HPP_

#include <cmath>
#include <chrono>
#include <optional>
#include <algorithm>
#include <istream>
#include <sstream>
#include <string>
#include <vector>

// ==========================================
// PID Controller Class Implementation
// ==========================================
class PID {
private:
    // Controller Tuning Gains
    float kp_;
    float ki_;
    float kd_;

    // Internal Controller States
    float integrator_;
    std::optional<float> prev_error_;
    std::optional<float> prev_time_;

    // Controller Safety Boundaries
    std::optional<float> integrator_limit_;
    std::optional<float> output_limit_;

public:
    // Constructor with default values
    PID(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f,
        std::optional<float> integrator_limit = std::nullopt,
        std::optional<float> output_limit = std::nullopt)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          integrator_(0.0f),
          prev_error_(std::nullopt),
          prev_time_(std::nullopt),
          integrator_limit_(integrator_limit),
          output_limit_(output_limit) {}

    // Reset the internal states of the controller
    void reset() {
        integrator_ = 0.0f;
        prev_error_ = std::nullopt;
        prev_time_ = std::nullopt;
    }

    // Compute PID step output given an error and optional timestamp
    float step(float error, std::optional<float> now = std::nullopt) {
        // Fetch current timestamp in seconds if not provided
        float current_time = now.value_or(
            std::chrono::duration<float>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count()
        );

        // Calculate time delta (dt)
        float dt = 0.0f;
        if (prev_time_.has_value()) {
            dt = current_time - prev_time_.value();
        }

        // Clamp dt to non-negative values
        if (dt <= 0.0f) {
            dt = 0.0f;
        }

        // Compute Proportional output component
        float p_out = kp_ * error;

        // Compute Integral output component with optional anti-windup clamping
        if (dt > 0.0f) {
            integrator_ += error * dt;

            if (integrator_limit_.has_value()) {
                integrator_ = std::clamp(integrator_, -integrator_limit_.value(), integrator_limit_.value());
            }
        }
        float i_out = ki_ * integrator_;

        // Compute Derivative output component
        float d_out = 0.0f;
        if (prev_error_.has_value() && dt > 0.0f) {
            float derivative = (error - prev_error_.value()) / dt;
            d_out = kd_ * derivative;
        }

        // Sum up total control output
        float output = p_out + i_out + d_out;

        // Apply final output limits if specified
        if (output_limit_.has_value()) {
            output = std::clamp(output, -output_limit_.value(), output_limit_.value());
        }

        // Store states for the next iteration cycle
        prev_time_ = current_time;
        prev_error_ = error;

        return output;
    }
};

// ==========================================
// Mathematical & Helper Functions (free functions so gtest can call them
// directly without needing a running rclcpp::Node)
// ==========================================

// Compute the shortest signed angular error between a target and current
// heading, wrapped into the range [-180, 180] degrees.
inline float heading_error(float target, float current) {
    float error = target - current;
    float wrapped = std::fmod(error + 180.0f, 360.0f);
    // std::fmod keeps the sign of the dividend, so a negative result
    // here needs to be shifted back into the [0, 360) range before
    // the final [-180, 180] centering below.
    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    return std::round((wrapped - 180.0f) * 10.0f) / 10.0f;
}

// Parse waypoint headings (degrees) from a CSV-formatted stream. The first
// line is treated as a header and skipped. Only the first column of each
// subsequent row is read; unparsable rows are skipped. Taking a std::istream
// (rather than a filename) lets tests feed in-memory data via
// std::istringstream without touching the filesystem.
inline std::vector<float> parse_waypoints_csv(std::istream& stream) {
    std::vector<float> waypoints;
    std::string line;

    // Skip CSV header line and process rows
    if (std::getline(stream, line)) {
        while (std::getline(stream, line)) {
            std::stringstream ss(line);
            std::string cell;
            if (std::getline(ss, cell, ',')) {
                try {
                    float hdg = std::round(std::stof(cell) * 10.0f) / 10.0f;
                    waypoints.push_back(hdg);
                } catch (...) {
                    continue;
                }
            }
        }
    }
    return waypoints;
}

#endif  // HEADING_CONTROLLER_PKG__SPEED_UTILS_HPP_