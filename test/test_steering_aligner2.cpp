#include <gtest/gtest.h>
#include "heading_controller_pkg/steering_aligner2_utils.hpp"

// ==========================================
// angle_diff() / wrap360()
// ==========================================
TEST(AngleDiff2, SimpleCase) {
    EXPECT_NEAR(angle_diff(10.0f, 0.0f), 10.0f, 0.001f);
}

TEST(AngleDiff2, WrapsAcrossZero) {
    EXPECT_NEAR(angle_diff(10.0f, 350.0f), 20.0f, 0.001f);
}

TEST(Wrap3602, HandlesNegative) {
    EXPECT_NEAR(wrap360(-10.0f), 350.0f, 0.001f);
}

// ==========================================
// apply_ema()
// ==========================================
TEST(ApplyEma2, SmoothsTowardNewValue) {
    float result = apply_ema(10.0f, 0.5f, 0.0f);
    EXPECT_NEAR(result, 5.0f, 0.001f);
}

TEST(ApplyEma2, AlphaOneIsNoSmoothing) {
    float result = apply_ema(10.0f, 1.0f, 999.0f);
    EXPECT_NEAR(result, 10.0f, 0.001f);
}

// ==========================================
// compute_aligner2_step()
// ==========================================
Aligner2Params default_params() {
    Aligner2Params p;
    return p;  // struct defaults match the Python node exactly
}

TEST(Aligner2Step, NoActionWithoutHeadingData) {
    auto p = default_params();
    Aligner2State s;
    auto r = compute_aligner2_step(s, 0.0, p);
    EXPECT_FALSE(r.should_publish_pwm);
    EXPECT_FALSE(r.entered_hold);
}

TEST(Aligner2Step, LargeErrorPublishesProportionalPwm) {
    auto p = default_params();  // max_pwm=17
    Aligner2State s;
    s.target_heading = 90.0f;
    s.current_heading = 0.0f;
    auto r = compute_aligner2_step(s, 0.0, p);
    EXPECT_TRUE(r.should_publish_pwm);
    // pwm_cmd before EMA = (90/2)*17 = 765, clamped to max_pwm=17, then
    // EMA'd from smoothed_pwm=0 with alpha=0.5 -> 8.5
    EXPECT_NEAR(r.pwm_value, 8.5f, 0.01f);
}

TEST(Aligner2Step, NegativeErrorGivesNegativePwm) {
    auto p = default_params();
    Aligner2State s;
    s.target_heading = 0.0f;
    s.current_heading = 90.0f;  // error = angle_diff(0,90) = -90
    auto r = compute_aligner2_step(s, 0.0, p);
    EXPECT_LT(r.pwm_value, 0.0f);
}

TEST(Aligner2Step, WithinToleranceEntersHold) {
    auto p = default_params();  // tolerance_deg = 2.0
    Aligner2State s;
    s.target_heading = 1.0f;
    s.current_heading = 0.0f;
    auto r = compute_aligner2_step(s, 0.0, p);
    EXPECT_TRUE(r.entered_hold);
    EXPECT_TRUE(s.holding);
    EXPECT_FALSE(r.should_publish_pwm);
}

TEST(Aligner2Step, HoldPhaseThenAlignmentDoneAfterDuration) {
    auto p = default_params();
    p.hold_duration = 0.1f;
    Aligner2State s;
    s.target_heading = 1.0f;
    s.current_heading = 0.0f;

    auto r1 = compute_aligner2_step(s, 0.0, p);
    EXPECT_TRUE(r1.entered_hold);

    // Still within hold_duration
    auto r2 = compute_aligner2_step(s, 0.05, p);
    EXPECT_FALSE(r2.alignment_done);

    // Past hold_duration
    auto r3 = compute_aligner2_step(s, 0.2, p);
    EXPECT_TRUE(r3.alignment_done);
    EXPECT_FALSE(s.active);
    EXPECT_FALSE(s.holding);
}

TEST(Aligner2Step, InactiveAndNotHoldingProducesNoAction) {
    auto p = default_params();
    Aligner2State s;
    s.target_heading = 90.0f;
    s.current_heading = 0.0f;
    s.active = false;
    s.holding = false;
    auto r = compute_aligner2_step(s, 0.0, p);
    EXPECT_FALSE(r.should_publish_pwm);
    EXPECT_FALSE(r.entered_hold);
    EXPECT_FALSE(r.alignment_done);
}

TEST(Aligner2Step, PwmNeverExceedsMaxMagnitude) {
    auto p = default_params();
    Aligner2State s;
    s.target_heading = 179.0f;
    s.current_heading = 0.0f;  // near-maximal possible error
    auto r = compute_aligner2_step(s, 0.0, p);
    EXPECT_LE(std::abs(r.pwm_value), p.max_pwm);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}