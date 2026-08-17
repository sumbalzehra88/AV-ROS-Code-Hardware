#ifndef HEADING_CONTROLLER_PKG__STEERING_ALIGNER2_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__STEERING_ALIGNER2_UTILS_HPP_

#include <cmath>
#include <algorithm>
#include <optional>

// ==========================================
// Utility helpers
// ==========================================
inline float angle_diff(float a, float b) {
    float d = std::fmod(a - b + 180.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

inline float wrap360(float deg) {
    float w = std::fmod(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    return w;
}

inline float apply_ema(float new_val, float alpha, float prev_smoothed) {
    return alpha * new_val + (1.0f - alpha) * prev_smoothed;
}

// ==========================================
// Tunable Parameters (defaults match the Python node exactly)
// ==========================================
struct Aligner2Params {
    float tolerance_deg = 2.0f;
    float max_pwm = 17.0f;
    float min_pwm = 0.0f;
    float ema_alpha = 0.5f;
    float hold_duration = 0.26f;
    double disable_time = 0.0;
};

// ==========================================
// Full aligner state, carried between control_loop ticks
// ==========================================
struct Aligner2State {
    std::optional<float> target_heading;
    std::optional<float> current_heading;
    float smoothed_pwm = 0.0f;
    bool active = true;
    bool holding = false;
    std::optional<double> hold_start_time;
    double last_disable_time = 0.0;
};

// ==========================================
// Result of one control-loop tick
// ==========================================
struct Aligner2StepResult {
    bool should_publish_pwm = false;
    float pwm_value = 0.0f;
    bool entered_hold = false;
    bool alignment_done = false;
};

// ==========================================
// Pure(-ish) step function: mutates the passed-in Aligner2State the way
// control_loop() mutates `self`, and returns what side effects the caller
// should perform. No ROS dependency, directly unit-testable with gtest.
// ==========================================
inline Aligner2StepResult compute_aligner2_step(Aligner2State& s, double now, const Aligner2Params& p) {
    Aligner2StepResult result;

    if (!s.current_heading.has_value() || !s.target_heading.has_value()) {
        return result;
    }
    if (!s.active && !s.holding) {
        return result;
    }

    float error = angle_diff(s.target_heading.value(), s.current_heading.value());

    if (!s.holding) {
        if (std::abs(error) <= p.tolerance_deg) {
            s.holding = true;
            s.hold_start_time = now;
            result.entered_hold = true;
            return result;
        }

        float pwm_cmd = (std::abs(error) / 2.0f) * p.max_pwm;
        pwm_cmd = std::clamp(pwm_cmd, p.min_pwm, p.max_pwm);
        if (error < 0.0f) pwm_cmd = -pwm_cmd;
        pwm_cmd = apply_ema(pwm_cmd, p.ema_alpha, s.smoothed_pwm);
        s.smoothed_pwm = pwm_cmd;

        result.should_publish_pwm = true;
        result.pwm_value = pwm_cmd;
        return result;
    }

    // Holding / stabilization phase
    double hold_elapsed = now - s.hold_start_time.value();
    if (hold_elapsed < p.hold_duration) {
        float pwm_cmd = (std::abs(error) / 2.0f) * p.max_pwm;
        pwm_cmd = std::clamp(pwm_cmd, p.min_pwm, p.max_pwm);
        if (error < 0.0f) pwm_cmd = -pwm_cmd;
        pwm_cmd = apply_ema(pwm_cmd, p.ema_alpha, s.smoothed_pwm);
        s.smoothed_pwm = pwm_cmd;

        result.should_publish_pwm = true;
        result.pwm_value = pwm_cmd;
        return result;
    }

    // Hold duration elapsed -- finish alignment and release control.
    result.alignment_done = true;
    s.active = false;
    s.holding = false;
    s.last_disable_time = now;
    return result;
}

#endif  // HEADING_CONTROLLER_PKG__STEERING_ALIGNER2_UTILS_HPP_