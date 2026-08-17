#ifndef HEADING_CONTROLLER_PKG__STANLEY5_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__STANLEY5_UTILS_HPP_

#include <cmath>
#include <algorithm>
#include <array>

// ==========================================
// 3x3 matrix type
// ==========================================
using Matrix3 = std::array<std::array<double, 3>, 3>;

// ==========================================
// Quaternion -> Euler angles (roll, pitch, yaw), radians.
// NOTE: present for parity with the Python source, but currently unused
// by the Stanley controller's own logic -- kept as a standalone utility.
// ==========================================
inline std::array<double, 3> quat2eulers(double q0, double q1, double q2, double q3) {
    double roll = std::atan2(2.0 * ((q2 * q3) + (q0 * q1)),
                              q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    double pitch = std::asin(2.0 * ((q1 * q3) - (q0 * q2)));
    double yaw = std::atan2(2.0 * ((q1 * q2) + (q0 * q3)),
                             q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    return {roll, pitch, yaw};
}

// ==========================================
// Quaternion -> full 3x3 rotation matrix. Q = (q0=w, q1=x, q2=y, q3=z).
// ==========================================
inline Matrix3 quaternion_rotation_matrix(double q0, double q1, double q2, double q3) {
    Matrix3 m;
    m[0][0] = 2.0 * (q0 * q0 + q1 * q1) - 1.0;
    m[0][1] = 2.0 * (q1 * q2 - q0 * q3);
    m[0][2] = 2.0 * (q1 * q3 + q0 * q2);

    m[1][0] = 2.0 * (q1 * q2 + q0 * q3);
    m[1][1] = 2.0 * (q0 * q0 + q2 * q2) - 1.0;
    m[1][2] = 2.0 * (q2 * q3 - q0 * q1);

    m[2][0] = 2.0 * (q1 * q3 - q0 * q2);
    m[2][1] = 2.0 * (q2 * q3 + q0 * q1);
    m[2][2] = 2.0 * (q0 * q0 + q3 * q3) - 1.0;
    return m;
}

inline Matrix3 transpose(const Matrix3& m) {
    Matrix3 t;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            t[i][j] = m[j][i];
    return t;
}

inline Matrix3 matmul(const Matrix3& a, const Matrix3& b) {
    Matrix3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += a[i][k] * b[k][j];
            }
            r[i][j] = sum;
        }
    }
    return r;
}

// yaw = -atan2(R[1][0], R[0][0])
inline double compute_yaw_from_rotation(const Matrix3& R) {
    return -std::atan2(R[1][0], R[0][0]);
}

// ==========================================
// Cross-track error. NOTE: the sign convention only negates when
// ref_side == 0 exactly -- any other ref_side value always yields a
// positive (unsigned) error regardless of true geometric side. Preserved
// faithfully from the Python.
// ==========================================
inline double compute_cross_track_error(double curr_x, double curr_y,
                                         double ref_x, double ref_y, double ref_side) {
    double e = std::sqrt((curr_x - ref_x) * (curr_x - ref_x) +
                          (curr_y - ref_y) * (curr_y - ref_y));
    if (ref_side == 0.0) {
        e = -e;
    }
    return e;
}

// ==========================================
// Stanley control law: yaw + arctan(K*e/v), clamped to [-20, 20] degrees.
// ==========================================
struct StanleyOutput {
    double theta_deg;
    double cross_track_error;
};

inline StanleyOutput compute_stanley_output(double yaw_rad, double e, double speed, double K) {
    double v = std::max(speed, 0.1);
    double delta2 = std::atan((K * e) / v);
    double theta_deg = (yaw_rad + delta2) * 180.0 / M_PI;
    theta_deg = std::clamp(theta_deg, -20.0, 20.0);
    return {theta_deg, e};
}

// ==========================================
// Point / alignment gating state machine, extracted from
// point_callback() and alignment_done_callback(). NOTE: if a checkpoint
// is marked passed/skipped without alignment_done ever firing,
// wait_alignment stays true and enable_control is never re-set here --
// this mirrors the Python exactly, including that risk.
// ==========================================
struct StanleyGateState {
    bool enable_control = true;
    bool wait_alignment = false;
};

inline void handle_point_message(StanleyGateState& s, bool point_active) {
    if (point_active) {
        if (s.enable_control) {
            s.enable_control = false;
            s.wait_alignment = true;
        }
    } else {
        if (!s.wait_alignment) {
            if (!s.enable_control) {
                s.enable_control = true;
            }
        }
    }
}

inline void handle_alignment_done(StanleyGateState& s, bool done) {
    if (done) {
        s.wait_alignment = false;
        s.enable_control = true;
    }
}

#endif  // HEADING_CONTROLLER_PKG__STANLEY5_UTILS_HPP_