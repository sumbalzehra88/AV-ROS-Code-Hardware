#ifndef HEADING_CONTROLLER_PKG__DYNAMIC_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__DYNAMIC_UTILS_HPP_

// ==========================================
// Steering Sweep State
// ==========================================
enum class SteeringState {
    Idle,
    HitPosLimit,
    HitNegLimit,
    Rebounding,
    LockCenter
};

// ==========================================
// Tunable Parameters
// ==========================================
struct SweepParams {
    float pos_limit_deg = 20.0f;
    float neg_limit_deg = -24.0f;
    float rebound_step_deg = 1.0f;
};

// ==========================================
// Full state bundle carried between callbacks
// ==========================================
struct SweepState {
    SteeringState state = SteeringState::Idle;
    float theta_deg = 0.0f;
    int rebound_direction = 1;
    bool rebound_completed = false;
    bool point_sent = false;
};

// ==========================================
// Result of processing one incoming steering-angle reading
// ==========================================
struct SweepResult {
    SweepState next;    // updated full state bundle to carry into the next call
    bool point_value;   // what to publish on /point this tick
};

// ==========================================
// Pure state-machine step function. Mirrors the Python steering_callback()
// logic exactly (including its quirks -- see README/PR notes), with no
// ROS/logging/publishing side effects, so it can be unit-tested directly.
// ==========================================
inline SweepResult compute_steering_step(SweepState s, float measured_theta_deg, const SweepParams& p) {
    // The Python always overwrites theta_deg with the incoming reading
    // first, before any state-specific logic runs.
    s.theta_deg = measured_theta_deg;

    if (s.state == SteeringState::Idle) {
        if (s.theta_deg >= p.pos_limit_deg) {
            s.state = SteeringState::HitPosLimit;
            s.rebound_direction = -1;
        } else if (s.theta_deg <= p.neg_limit_deg) {
            s.state = SteeringState::HitNegLimit;
            s.rebound_direction = 1;
        }
    } else if (s.state == SteeringState::HitPosLimit) {
        s.theta_deg = p.pos_limit_deg;
        s.state = SteeringState::Rebounding;
    } else if (s.state == SteeringState::HitNegLimit) {
        s.theta_deg = p.neg_limit_deg;
        s.state = SteeringState::Rebounding;
    } else if (s.state == SteeringState::Rebounding) {
        // Simulate rebound step
        s.theta_deg += static_cast<float>(s.rebound_direction) * p.rebound_step_deg;

        // Detect hitting limits and reverse
        if (s.rebound_direction == -1 && s.theta_deg <= p.neg_limit_deg) {
            s.rebound_direction = 1;
            s.rebound_completed = true;
            s.theta_deg = p.neg_limit_deg;
        } else if (s.rebound_direction == 1 && s.theta_deg >= p.pos_limit_deg) {
            s.rebound_direction = -1;
            s.rebound_completed = true;
            s.theta_deg = p.pos_limit_deg;
        }

        // Lock center condition
        if (s.rebound_completed &&
            ((s.rebound_direction == 1 && s.theta_deg <= 0.0f) ||
             (s.rebound_direction == -1 && s.theta_deg >= 0.0f))) {
            s.state = SteeringState::LockCenter;
            s.theta_deg = 0.0f;
        }
    }
    // LockCenter: intentionally no handling branch here, mirroring the
    // Python (which has no corresponding elif for this state either).

    // Publish /point True exactly once, on the tick the sweep completes.
    bool point_value = false;
    if (s.rebound_completed && !s.point_sent) {
        point_value = true;
        s.point_sent = true;
    }

    return {s, point_value};
}

#endif  // HEADING_CONTROLLER_PKG__DYNAMIC_UTILS_HPP_