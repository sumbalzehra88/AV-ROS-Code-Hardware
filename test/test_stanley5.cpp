#include <gtest/gtest.h>
#include <cmath>
#include "heading_controller_pkg/stanley5_utils.hpp"

// ==========================================
// quaternion_rotation_matrix() / transpose() / matmul()
// ==========================================
TEST(RotationMatrix, IdentityQuaternionGivesIdentityMatrix) {
    // q = (1, 0, 0, 0) is the identity quaternion.
    Matrix3 m = quaternion_rotation_matrix(1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(m[0][0], 1.0, 1e-9);
    EXPECT_NEAR(m[1][1], 1.0, 1e-9);
    EXPECT_NEAR(m[2][2], 1.0, 1e-9);
    EXPECT_NEAR(m[0][1], 0.0, 1e-9);
    EXPECT_NEAR(m[1][0], 0.0, 1e-9);
}

TEST(RotationMatrix, TransposeSwapsOffDiagonal) {
    Matrix3 m = quaternion_rotation_matrix(0.7071, 0.7071, 0.0, 0.0);
    Matrix3 t = transpose(m);
    EXPECT_NEAR(t[0][1], m[1][0], 1e-6);
    EXPECT_NEAR(t[1][0], m[0][1], 1e-6);
}

TEST(RotationMatrix, MatmulIdentityIsUnchanged) {
    Matrix3 identity = quaternion_rotation_matrix(1.0, 0.0, 0.0, 0.0);
    Matrix3 m = quaternion_rotation_matrix(0.9, 0.1, 0.2, 0.3);
    Matrix3 r = matmul(identity, m);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(r[i][j], m[i][j], 1e-9);
}

TEST(RotationMatrix, YawFromIdentityRotationIsZero) {
    Matrix3 identity = quaternion_rotation_matrix(1.0, 0.0, 0.0, 0.0);
    double yaw = compute_yaw_from_rotation(identity);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
}

// ==========================================
// compute_cross_track_error() -- documents the ref_side==0-only sign quirk
// ==========================================
TEST(CrossTrackError, ZeroDistanceIsZero) {
    double e = compute_cross_track_error(1.0, 1.0, 1.0, 1.0, 5.0);
    EXPECT_NEAR(e, 0.0, 1e-9);
}

TEST(CrossTrackError, RefSideZeroNegatesDistance) {
    double e = compute_cross_track_error(1.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_LT(e, 0.0);
    EXPECT_NEAR(e, -1.0, 1e-9);
}

TEST(CrossTrackError, AnyNonZeroRefSideStaysPositive_RegressionCase) {
    // Documents the quirk: only ref_side==0 triggers negation; any other
    // value (e.g. 1, -5, 100) leaves the error positive regardless of
    // true geometric side.
    double e = compute_cross_track_error(1.0, 0.0, 0.0, 0.0, 1.0);
    EXPECT_GT(e, 0.0);
    EXPECT_NEAR(e, 1.0, 1e-9);
}

// ==========================================
// compute_stanley_output()
// ==========================================
TEST(StanleyOutput, ZeroYawZeroErrorGivesZeroTheta) {
    auto out = compute_stanley_output(0.0, 0.0, 5.0, 0.01);
    EXPECT_NEAR(out.theta_deg, 0.0, 1e-6);
}

TEST(StanleyOutput, ClampsToPositive20) {
    // Large yaw forces clamp at +20.
    auto out = compute_stanley_output(M_PI, 0.0, 5.0, 0.01);
    EXPECT_NEAR(out.theta_deg, 20.0, 1e-6);
}

TEST(StanleyOutput, ClampsToNegative20) {
    auto out = compute_stanley_output(-M_PI, 0.0, 5.0, 0.01);
    EXPECT_NEAR(out.theta_deg, -20.0, 1e-6);
}

TEST(StanleyOutput, SpeedFloorPreventsDivideByZero) {
    // speed=0 should use the 0.1 floor internally, not blow up.
    auto out1 = compute_stanley_output(0.0, 5.0, 0.0, 0.01);
    auto out2 = compute_stanley_output(0.0, 5.0, 0.1, 0.01);
    EXPECT_NEAR(out1.theta_deg, out2.theta_deg, 1e-6);
}

TEST(StanleyOutput, CrossTrackErrorPassthrough) {
    auto out = compute_stanley_output(0.0, 3.5, 5.0, 0.01);
    EXPECT_NEAR(out.cross_track_error, 3.5, 1e-9);
}

// ==========================================
// StanleyGateState -- point/alignment gating logic
// ==========================================
TEST(GateState, EnteringPointZonePausesControl) {
    StanleyGateState s;
    handle_point_message(s, true);
    EXPECT_FALSE(s.enable_control);
    EXPECT_TRUE(s.wait_alignment);
}

TEST(GateState, LeavingZoneResumesControlWhenNotWaitingForAlignment) {
    StanleyGateState s;
    s.enable_control = false;
    s.wait_alignment = false;
    handle_point_message(s, false);
    EXPECT_TRUE(s.enable_control);
}

TEST(GateState, LeavingZoneDoesNotResumeWhileStillWaitingForAlignment) {
    StanleyGateState s;
    s.enable_control = false;
    s.wait_alignment = true;
    handle_point_message(s, false);
    EXPECT_FALSE(s.enable_control);  // still paused
}

TEST(GateState, AlignmentDoneClearsWaitAndResumesControl) {
    StanleyGateState s;
    s.enable_control = false;
    s.wait_alignment = true;
    handle_alignment_done(s, true);
    EXPECT_TRUE(s.enable_control);
    EXPECT_FALSE(s.wait_alignment);
}

TEST(GateState, DeadlockScenario_PassedWithoutAlignmentDone_RegressionCase) {
    // Documents the cross-node risk: if a checkpoint gets marked passed
    // without alignment_done ever firing, /point goes False while
    // wait_alignment is still True -- control never resumes here.
    StanleyGateState s;
    handle_point_message(s, true);   // entered zone, paused
    ASSERT_TRUE(s.wait_alignment);
    handle_point_message(s, false);  // left zone WITHOUT alignment_done firing
    EXPECT_FALSE(s.enable_control);  // stuck paused
    EXPECT_TRUE(s.wait_alignment);
}

TEST(GateState, NormalFullCycleResumesControl) {
    StanleyGateState s;
    handle_point_message(s, true);        // enter zone
    handle_alignment_done(s, true);       // aligner finishes
    handle_point_message(s, false);       // leave zone
    EXPECT_TRUE(s.enable_control);
    EXPECT_FALSE(s.wait_alignment);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}