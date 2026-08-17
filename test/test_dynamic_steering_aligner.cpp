#include <gtest/gtest.h>
#include "heading_controller_pkg/dynamic_steering_aligner_utils.hpp"

// ==========================================
// angle_diff() / wrap360()
// ==========================================
TEST(AngleDiff, SimpleCase) {
    EXPECT_NEAR(angle_diff(10.0f, 0.0f), 10.0f, 0.001f);
}

TEST(AngleDiff, WrapsAcrossZero) {
    EXPECT_NEAR(angle_diff(10.0f, 350.0f), 20.0f, 0.001f);
}

TEST(AngleDiff, LargeWrapStaysInRange) {
    float d = angle_diff(10.0f, 200.0f);
    EXPECT_LE(std::abs(d), 180.0f);
}

TEST(Wrap360, HandlesNegative) {
    EXPECT_NEAR(wrap360(-10.0f), 350.0f, 0.001f);
}

// ==========================================
// adaptive_gain() -- documents the counterintuitive k_min > k_max behavior
// ==========================================
TEST(AdaptiveGain, ApproachesKMinAtZeroError) {
    float g = adaptive_gain(0.0f, 3.08f, 0.9f, 6.0f);
    EXPECT_NEAR(g, 3.08f, 0.01f);
}

TEST(AdaptiveGain, ApproachesKMaxEffectiveAtLargeError) {
    float g = adaptive_gain(1000.0f, 3.08f, 0.9f, 6.0f);
    EXPECT_NEAR(g, 0.9f, 0.05f);
}

TEST(AdaptiveGain, SmallErrorGetsLargerGainThanLargeError_RegressionCase) {
    // With the node's default params (k_min=3.08 > k_max=0.9), small
    // errors get a LARGER gain than large errors -- the opposite of the
    // "stronger correction for large errors" behavior the docstring
    // describes. This test documents that this is the actual behavior.
    float small_error_gain = adaptive_gain(0.5f, 3.08f, 0.9f, 6.0f);
    float large_error_gain = adaptive_gain(50.0f, 3.08f, 0.9f, 6.0f);
    EXPECT_GT(small_error_gain, large_error_gain);
}

// ==========================================
// apply_ema() / slew_limit()
// ==========================================
TEST(ApplyEma, SmoothsTowardNewValue) {
    float result = apply_ema(10.0f, 0.5f, 0.0f);
    EXPECT_NEAR(result, 5.0f, 0.001f);
}

TEST(SlewLimit, LimitsLargeJump) {
    // max_slew_rate=80/s, dt=0.01s -> max_delta=0.8
    float result = slew_limit(100.0f, 0.0f, 0.01, 80.0f);
    EXPECT_NEAR(result, 0.8f, 0.001f);
}

TEST(SlewLimit, PassesThroughSmallChange) {
    float result = slew_limit(1.0f, 0.9f, 0.01, 80.0f);
    EXPECT_NEAR(result, 1.0f, 0.001f);
}

TEST(SlewLimit, ZeroDtReturnsDesiredUnchanged) {
    float result = slew_limit(100.0f, 0.0f, 0.0, 80.0f);
    EXPECT_NEAR(result, 100.0f, 0.001f);
}

// ==========================================
// Oscillation detection
// ==========================================
TEST(OscillationDetection, NoOscillationWithConsistentSign) {
    SignHistory h;
    for (int i = 0; i < 5; ++i) {
        update_sign_history(h, i * 0.1, 5.0f, 1.2);
    }
    EXPECT_FALSE(detect_oscillation(h, 5));
}

TEST(OscillationDetection, DetectsFrequentSignFlips) {
    SignHistory h;
    float errors[] = {1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f};
    for (int i = 0; i < 6; ++i) {
        update_sign_history(h, i * 0.1, errors[i], 1.2);
    }
    EXPECT_TRUE(detect_oscillation(h, 5));
}

TEST(OscillationDetection, OldSamplesArePruned) {
    SignHistory h;
    update_sign_history(h, 0.0, 1.0f, 1.2);
    update_sign_history(h, 5.0, -1.0f, 1.2);  // far beyond window, should prune the first
    EXPECT_EQ(h.size(), 1u);
}

// ==========================================
// compute_aligner_step() -- full integration of the state machine
// ==========================================
AlignerParams default_aligner_params() {
    AlignerParams p;
    return p;  // uses struct defaults, matching the Python node exactly
}

TEST(AlignerStep, NoActionWithoutHeadingData) {
    auto p = default_aligner_params();
    AlignerState s;
    auto r = compute_aligner_step(s, 0.0, p);
    EXPECT_FALSE(r.should_publish_pwm);
    EXPECT_FALSE(r.entered_hold);
    EXPECT_FALSE(r.alignment_done);
}

TEST(AlignerStep, LargeErrorPublishesNonZeroPwm) {
    auto p = default_aligner_params();
    AlignerState s;
    s.target_heading = 90.0f;
    s.current_heading = 0.0f;
    s.last_cmd_time = 0.0;
    auto r = compute_aligner_step(s, 0.05, p);
    EXPECT_TRUE(r.should_publish_pwm);
    EXPECT_GT(r.pwm_value, 0.0f);
}

TEST(AlignerStep, WithinToleranceEntersHold) {
    auto p = default_aligner_params();  // tolerance_deg = 2.0
    AlignerState s;
    s.target_heading = 1.0f;
    s.current_heading = 0.0f;
    s.last_cmd_time = 0.0;
    auto r = compute_aligner_step(s, 0.05, p);
    EXPECT_TRUE(r.entered_hold);
    EXPECT_TRUE(s.holding);
    EXPECT_FALSE(r.should_publish_pwm);
}

TEST(AlignerStep, ErrorWithinDeadbandProducesZeroPwm) {
    auto p = default_aligner_params();  // deadband_deg = 0.25, tolerance_deg = 2.0
    AlignerState s;
    // Error must be inside deadband but outside tolerance to hit the
    // deadband-zeroing branch in the "aligning" (not holding) path --
    // deadband(0.25) < tolerance(2.0) here so this combination can't
    // actually occur with defaults; use a wider deadband via custom params.
    AlignerParams custom = p;
    custom.deadband_deg = 5.0f;
    custom.tolerance_deg = 1.0f;
    s.target_heading = 3.0f;
    s.current_heading = 0.0f;
    s.last_cmd_time = 0.0;
    auto r = compute_aligner_step(s, 0.05, custom);
    EXPECT_TRUE(r.should_publish_pwm);
    EXPECT_NEAR(r.pwm_value, 0.0f, 0.001f);
}

TEST(AlignerStep, HoldPhaseThenAlignmentDoneAfterDuration) {
    auto p = default_aligner_params();
    p.hold_duration = 0.1f;  // short for testing
    AlignerState s;
    s.target_heading = 1.0f;
    s.current_heading = 0.0f;
    s.last_cmd_time = 0.0;

    auto r1 = compute_aligner_step(s, 0.05, p);
    EXPECT_TRUE(r1.entered_hold);

    // Still within hold_duration
    auto r2 = compute_aligner_step(s, 0.10, p);
    EXPECT_TRUE(r2.should_publish_pwm);
    EXPECT_FALSE(r2.alignment_done);

    // Past hold_duration (0.05 + 0.1 = 0.15)
    auto r3 = compute_aligner_step(s, 0.20, p);
    EXPECT_TRUE(r3.alignment_done);
    EXPECT_FALSE(s.active);
    EXPECT_FALSE(s.holding);
}

TEST(AlignerStep, InactiveAndNotHoldingProducesNoAction) {
    auto p = default_aligner_params();
    AlignerState s;
    s.target_heading = 90.0f;
    s.current_heading = 0.0f;
    s.active = false;
    s.holding = false;
    auto r = compute_aligner_step(s, 0.05, p);
    EXPECT_FALSE(r.should_publish_pwm);
    EXPECT_FALSE(r.entered_hold);
    EXPECT_FALSE(r.alignment_done);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}