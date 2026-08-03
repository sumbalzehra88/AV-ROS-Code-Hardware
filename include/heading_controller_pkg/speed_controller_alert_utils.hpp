#ifndef HEADING_CONTROLLER_PKG__SPEED_CONTROLLER_ALERT_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__SPEED_CONTROLLER_ALERT_UTILS_HPP_

#include <cmath>
#include <algorithm>

// ==========================================
// Controller State
// ==========================================
enum class ControllerState {
    Pressing,
    Rebound1,
    Rebound2,
    Neutral
};

// ==========================================
// Tunable Parameters
// ==========================================
struct ServoParams {
    float target_speed_kmh = 1.0f;
    float tolerance_kmh = 0.0f;
    int servo_neutral = 164;
    int servo_full = 158;
    int rebound_angle = 170;
    float speed_max_kmh = 5.0f;
};

// ==========================================
// Result of a single control-loop step
// ==========================================
struct ControlResult {
    ControllerState next_state;
    int next_tick_count;
    int servo_command;
    bool target_reached;
    bool entered_overspeed;
    bool resumed_from_overspeed;
};

// ==========================================
// Pure state-machine step function, now with an alert override layered
// on top. Mirrors the Python control_loop() logic exactly, but has no
// ROS/logging/publishing side effects, so it can be unit-tested directly.
// ==========================================
inline ControlResult compute_control_step_with_alert(ControllerState state, int tick_count,
                                                       float speed_kmh, bool alert_active,
                                                       const ServoParams& p) {
    // 1. ALERT OVERRIDE -- highest priority, bypasses the FSM entirely.
    // State and tick count are frozen while the alert is active, so the
    // FSM resumes exactly where it left off once the alert clears.
    if (alert_active) {
        return {state, tick_count, p.rebound_angle, false, false, false};
    }

    // 2. SAFETY OVERSPEED STOP
    if (speed_kmh > p.speed_max_kmh) {
        bool entered = (state != ControllerState::Neutral);
        return {ControllerState::Neutral, 0, p.servo_neutral, false, entered, false};
    }

    // If in neutral (due to overspeed), wait until speed drops back below
    // threshold (with a small hysteresis margin) before resuming.
    ControllerState working_state = state;
    bool resumed = false;
    if (working_state == ControllerState::Neutral) {
        if (speed_kmh <= (p.speed_max_kmh - 0.05f)) {
            resumed = true;
            working_state = ControllerState::Pressing;
        } else {
            return {ControllerState::Neutral, 0, p.servo_neutral, false, false, false};
        }
    }

    // 3. NORMAL SPEED CONTROL STATE MACHINE
    switch (working_state) {
        case ControllerState::Pressing: {
            if (std::abs(speed_kmh - p.target_speed_kmh) <= p.tolerance_kmh) {
                return {ControllerState::Rebound1, 0, p.rebound_angle, true, false, resumed};
            }
            return {ControllerState::Pressing, 0, p.servo_full, false, false, resumed};
        }

        case ControllerState::Rebound1: {
            if (tick_count < 1) {
                return {ControllerState::Rebound1, tick_count + 1, p.rebound_angle, false, false, resumed};
            }
            return {ControllerState::Rebound2, 0, p.servo_neutral, false, false, resumed};
        }

        case ControllerState::Rebound2: {
            if (tick_count < 1) {
                return {ControllerState::Rebound2, tick_count + 1, p.servo_neutral, false, false, resumed};
            }
            return {ControllerState::Pressing, 0, p.servo_full, false, false, resumed};
        }

        default:
            // Unknown state fallback -- mirrors the Python "else" branch.
            return {ControllerState::Neutral, 0, p.servo_neutral, false, false, resumed};
    }
}

// ==========================================
// Clamp a raw servo command into the valid [0, 255] output range.
// ==========================================
inline int clamp_servo(int angle) {
    return std::clamp(angle, 0, 255);
}

#endif  // HEADING_CONTROLLER_PKG__SPEED_CONTROLLER_ALERT_UTILS_HPP_