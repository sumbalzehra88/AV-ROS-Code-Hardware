#ifndef HEADING_CONTROLLER_PKG__AHRS_SIM_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__AHRS_SIM_UTILS_HPP_

#include <cmath>
#include <algorithm>
#include <optional>

// ==========================================
// Utility functions
// ==========================================

// Forward bearing in degrees from (lat1,lon1) -> (lat2,lon2). Result in [0, 360).
inline double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double lat1r = lat1 * M_PI / 180.0;
    double lat2r = lat2 * M_PI / 180.0;
    double dlonr = (lon2 - lon1) * M_PI / 180.0;
    double y = std::sin(dlonr) * std::cos(lat2r);
    double x = std::cos(lat1r) * std::sin(lat2r) - std::sin(lat1r) * std::cos(lat2r) * std::cos(dlonr);
    double brng = std::atan2(y, x) * 180.0 / M_PI;
    // Wrap into [0, 360) -- fmod keeps the sign of the dividend in C++, so
    // a negative bearing needs the same re-wrap as heading_error() elsewhere
    // in this package.
    double wrapped = std::fmod(brng, 360.0);
    if (wrapped < 0.0) wrapped += 360.0;
    return wrapped;
}

// Smallest signed difference (target - current) in degrees, range (-180, 180].
inline float shortest_signed_deg_diff(float target_deg, float current_deg) {
    float d = std::fmod(target_deg - current_deg + 180.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

// Wrap any degree value into [0, 360).
inline float wrap360(float deg) {
    float w = std::fmod(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    return w;
}

// Haversine distance in meters between two lat/lon points, double precision.
inline double gnss_distance_m(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    if (a > 1.0) a = 1.0;
    if (a < 0.0) a = 0.0;
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c;
}

// ==========================================
// Simple 1D Kalman filter for heading, with circular-wrap-aware residuals.
// ==========================================
class HeadingKalman1D {
public:
    HeadingKalman1D(float initial = 0.0f, float initial_error = 10.0f,
                     float measurement_var = 8.0f, float process_var = 0.01f)
        : state_(initial), error_(initial_error),
          measurement_var_(measurement_var), process_var_(process_var) {}

    float update(float measurement_deg) {
        float residual = shortest_signed_deg_diff(measurement_deg, state_);
        float K = error_ / (error_ + measurement_var_);
        state_ = wrap360(state_ + K * residual);
        // (Python had a dead "+ abs(state) * 0.0" term here -- always zero,
        // omitted as a no-op.)
        error_ = (1.0f - K) * error_;
        error_ = error_ + process_var_;
        return state_;
    }

    float state() const { return state_; }
    void set_measurement_var(float v) { measurement_var_ = v; }
    void set_process_var(float v) { process_var_ = v; }

private:
    float state_;
    float error_;
    float measurement_var_;
    float process_var_;
};

// ==========================================
// GPS fix gating/acceptance logic
// ==========================================
struct GnssHeadingParams {
    float min_speed_m_s = 0.5f;
    float min_distance_m = 0.5f;
    float max_sample_age_s = 2.0f;
};

struct FixState {
    bool has_prev = false;
    double prev_lat = 0.0;
    double prev_lon = 0.0;
    double prev_time = 0.0;
};

struct GpsUpdateResult {
    FixState next;
    bool is_first_fix = false;
    bool accepted = false;
    bool refreshed_stale = false;
    double raw_bearing_deg = 0.0;
    double distance_m = 0.0;
    double est_speed_mps = 0.0;
    double dt_s = 0.0;
};

// Pure function mirroring gps_callback()'s gating logic exactly (including
// the max_sample_age_s inconsistency noted above), with no ROS dependency.
inline GpsUpdateResult process_gps_fix(FixState state, double lat, double lon, double now,
                                        const GnssHeadingParams& p) {
    if (!state.has_prev) {
        GpsUpdateResult r;
        r.next = FixState{true, lat, lon, now};
        r.is_first_fix = true;
        return r;
    }

    double dt = now - state.prev_time;
    double dist_m = gnss_distance_m(state.prev_lat, state.prev_lon, lat, lon);
    double est_speed = dist_m / std::max(1e-6, dt);

    bool accept = false;
    if (dist_m >= p.min_distance_m && dt <= std::max(static_cast<double>(p.max_sample_age_s), 10.0)) {
        if (est_speed >= p.min_speed_m_s || p.min_speed_m_s <= 0.0f) {
            accept = true;
        }
    }

    GpsUpdateResult r;
    r.distance_m = dist_m;
    r.est_speed_mps = est_speed;
    r.dt_s = dt;

    if (!accept) {
        // Mirrors Python: only refresh prev if dt exceeds the RAW
        // max_sample_age_s (not the floored-to-10s value used above).
        if (dt > p.max_sample_age_s) {
            r.next = FixState{true, lat, lon, now};
            r.refreshed_stale = true;
        } else {
            r.next = state;  // unchanged, silent reject
        }
        return r;
    }

    r.accepted = true;
    r.raw_bearing_deg = bearing_deg(state.prev_lat, state.prev_lon, lat, lon);
    r.next = FixState{true, lat, lon, now};
    return r;
}

// EMA smoothing on top of the Kalman output, circular-wrap-aware.
inline float update_ema_heading(std::optional<float> ema_prev, float filtered_heading, float alpha) {
    if (!ema_prev.has_value()) {
        return filtered_heading;
    }
    float diff = shortest_signed_deg_diff(filtered_heading, ema_prev.value());
    return wrap360(ema_prev.value() + alpha * diff);
}

#endif  // HEADING_CONTROLLER_PKG__AHRS_SIM_UTILS_HPP_