#ifndef HEADING_CONTROLLER_PKG__DYNAMIC_STEERING_ALIGNER_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__DYNAMIC_STEERING_ALIGNER_UTILS_HPP_

#include <cmath>
#include <algorithm>
#include <optional>
#include <deque>
#include <vector>

// ==========================================
// Utility helpers
// ==========================================

// Smallest signed difference between headings a and b (degrees), range (-180, 180].
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

// Present but currently unused by the aligner's own control loop, mirroring
// the Python (the actual deadband logic is inlined separately).
inline float apply_deadband(float value, float deadband_deg) {
    return (std::abs(value) <= deadband_deg) ? 0.0f : value;
}

// Exponential approach mapping |error| -> gain in [k_min, k_max_effective].
// NOTE: as configured by the node's default params (k_min > k_max), this
// produces LARGER gain for SMALL errors and SMALLER gain for LARGE errors
// -- the opposite of the "stronger correction for large errors" intent
// described in the original docstring. Preserved faithfully.
inline float adaptive_gain(float error_abs, float k_min, float k_max_effective, float gain_scale) {
    return k_min + (k_max_effective - k_min) *
           (1.0f - std::exp(-error_abs / std::max(1e-6f, gain_scale)));
}

inline float apply_ema(float new_val, float alpha, float prev_smoothed) {
    return alpha * new_val + (1.0f - alpha) * prev_smoothed;
}

inline float slew_limit(float desired, float last, double dt, float max_slew_rate) {
    if (dt <= 0.0) return desired;
    float max_delta = max_slew_rate * static_cast<float>(dt);
    float delta = desired - last;
    if (std::abs(delta) > max_delta) {
        desired = last + std::copysign(max_delta, delta);
    }
    return desired;
}

// ==========================================
// Oscillation detection
// ==========================================
struct SignSample {
    double time;
    int sign;
};
using SignHistory = std::deque<SignSample>;

inline void update_sign_history(SignHistory& history, double now, float error, double window_sec) {
    int sign = (error > 0.0f) ? 1 : (error < 0.0f ? -1 : 0);
    history.push_back({now, sign});
    double cutoff = now - window_sec;
    while (!history.empty() && history.front().time < cutoff) {
        history.pop_front();
    }
}

inline bool detect_oscillation(const SignHistory& history, int threshold) {
    std::vector<int> signs;
    for (const auto& s : history) {
        if (s.sign != 0) signs.push_back(s.sign);
    }
    if (signs.size() < 2) return false;
    int changes = 0;
    int prev = signs[0];
    for (size_t i = 1; i < signs.size(); ++i) {
        if (signs[i] != prev) {
            changes++;
            prev = signs[i];
        }
    }
    return changes >= threshold;
}

// ==========================================
// Tunable Parameters (defaults match the Python node exactly)
// ==========================================
struct AlignerParams {
    float tolerance_deg = 2.0f;
    float deadband_deg = 0.25f;
    float max_pwm = 17.0f;
    float min_pwm = -17.0f;  // kept for parity though NOT used in clamping (see note in .cpp)
    float k_min = 3.08f;
    float k_max = 0.9f;
    float gain_scale = 6.0f;
    float ema_alpha_base = 0.25f;
    float max_slew_rate = 80.0f;
    float hold_duration = 5.0f;
    float hold_k_max_factor = 1.1f;
    float hold_ema_alpha_factor = 0.95f;
    double osc_window_sec = 1.2;
    int osc_sign_changes_threshold = 5;
    float osc_reduction_factor = 0.45f;
    double osc_min_time_between_warnings = 1.5;
    double disable_time = 0.0;
};

// ==========================================
// Full aligner state, carried between control_loop ticks
// ==========================================
struct AlignerState {
    std::optional<float> target_heading;
    std::optional<float> current_heading;
    float smoothed_pwm = 0.0f;
    float last_pwm_cmd = 0.0f;
    double last_cmd_time = 0.0;
    bool active = true;
    bool holding = false;
    std::optional<double> hold_start_time;
    double last_disable_time = 0.0;
    SignHistory sign_history;
    double last_osc_warn_time = 0.0;
};

// ==========================================
// Result of one control-loop tick
// ==========================================
struct AlignerStepResult {
    bool should_publish_pwm = false;
    float pwm_value = 0.0f;
    bool entered_hold = false;
    bool oscillation_warning = false;
    bool alignment_done = false;
};

// ==========================================
// Pure(-ish) step function: mutates the passed-in AlignerState exactly the
// way control_loop() mutates `self`, and returns what side effects (log
// messages, publishes) the caller should perform. No ROS dependency, so
// it's directly unit-testable with gtest.
// ==========================================
inline AlignerStepResult compute_aligner_step(AlignerState& s, double now, const AlignerParams& p) {
    AlignerStepResult result;
    double dt = std::max(1e-6, now - s.last_cmd_time);
    s.last_cmd_time = now;

    if (!s.current_heading.has_value() || !s.target_heading.has_value()) {
        return result;
    }
    if (!s.active && !s.holding) {
        return result;
    }

    float error = angle_diff(s.target_heading.value(), s.current_heading.value());
    float error_abs = std::abs(error);

    update_sign_history(s.sign_history, now, error, p.osc_window_sec);
    bool oscillating = detect_oscillation(s.sign_history, p.osc_sign_changes_threshold);

    float k_max_effective = p.k_max;
    float ema_alpha = p.ema_alpha_base;
    if (oscillating) {
        k_max_effective = std::max(p.k_min, p.k_max * p.osc_reduction_factor);
        ema_alpha = std::max(0.02f, p.ema_alpha_base * 0.6f);
        if (now - s.last_osc_warn_time > p.osc_min_time_between_warnings) {
            result.oscillation_warning = true;
            s.last_osc_warn_time = now;
        }
    }

    if (!s.holding) {
        if (error_abs <= p.tolerance_deg) {
            s.holding = true;
            s.hold_start_time = now;
            result.entered_hold = true;
            return result;  // matches Python: no PWM published this tick
        }

        float gain = adaptive_gain(error_abs, p.k_min, k_max_effective, p.gain_scale);
        float pwm_cmd = gain * error_abs;
        pwm_cmd = std::clamp(pwm_cmd, -std::abs(p.max_pwm), p.max_pwm);
        pwm_cmd = (error >= 0.0f) ? pwm_cmd : -pwm_cmd;

        if (error_abs <= p.deadband_deg) {
            pwm_cmd = 0.0f;
        }

        pwm_cmd = slew_limit(pwm_cmd, s.last_pwm_cmd, dt, p.max_slew_rate);
        pwm_cmd = apply_ema(pwm_cmd, ema_alpha, s.smoothed_pwm);
        s.smoothed_pwm = pwm_cmd;

        result.should_publish_pwm = true;
        result.pwm_value = pwm_cmd;
        s.last_pwm_cmd = pwm_cmd;
        return result;
    }

    // Holding / stabilization phase
    double hold_elapsed = now - s.hold_start_time.value();
    if (hold_elapsed < p.hold_duration) {
        float hold_k_max = std::max(p.k_min, k_max_effective * p.hold_k_max_factor);
        float gain = adaptive_gain(error_abs, p.k_min, hold_k_max, p.gain_scale);
        float pwm_cmd = gain * error_abs;
        pwm_cmd = std::clamp(pwm_cmd, -std::abs(p.max_pwm), p.max_pwm);
        pwm_cmd = (error >= 0.0f) ? pwm_cmd : -pwm_cmd;

        if (error_abs <= (p.deadband_deg * 1.5f)) {
            pwm_cmd = 0.0f;
        }

        pwm_cmd = slew_limit(pwm_cmd, s.last_pwm_cmd, dt, p.max_slew_rate);
        pwm_cmd = apply_ema(pwm_cmd, std::max(0.01f, ema_alpha * p.hold_ema_alpha_factor), s.smoothed_pwm);
        s.smoothed_pwm = pwm_cmd;

        result.should_publish_pwm = true;
        result.pwm_value = pwm_cmd;
        s.last_pwm_cmd = pwm_cmd;
        return result;
    }

    // Hold duration elapsed -- finish alignment and release control.
    result.alignment_done = true;
    s.active = false;
    s.holding = false;
    s.last_disable_time = now;
    return result;
}

#endif  // HEADING_CONTROLLER_PKG__DYNAMIC_STEERING_ALIGNER_UTILS_HPP_