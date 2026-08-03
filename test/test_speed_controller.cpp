#include <gtest/gtest.h>
#include "heading_controller_pkg/speed_controller_utils.hpp"

// ==========================================
// Test fixture with default parameters matching the node's defaults
// ==========================================
ServoParams default_params() {
    ServoParams p;
    p.target_speed_kmh = 0.8f;
    p.tolerance_kmh = 0.0f;
    p.servo_neutral = 160;
    p.servo_full = 150;
    p.rebound_angle = 180;
    p.speed_max_kmh = 4.0f;
    return p;
}

// ==========================================
// Overspeed protection
// ==========================================
TEST(SpeedController, OverspeedTriggersNeutralFromPressing) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Pressing, 0, 5.0f, p);
    EXPECT_EQ(r.next_state, ControllerState::Neutral);
    EXPECT_EQ(r.servo_command, p.servo_neutral);
    EXPECT_TRUE(r.entered_overspeed);
}

TEST(SpeedController, OverspeedFromReboundAlsoTriggersNeutral) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Rebound1, 1, 4.5f, p);
    EXPECT_EQ(r.next_state, ControllerState::Neutral);
    EXPECT_TRUE(r.entered_overspeed);
}

TEST(SpeedController, StayingInNeutralDoesNotRelogEnteredOverspeed) {
    auto p = default_params();
    // Already in Neutral and still overspeed -> entered_overspeed should be
    // false since we didn't just transition into it.
    auto r = compute_control_step(ControllerState::Neutral, 0, 5.0f, p);
    EXPECT_EQ(r.next_state, ControllerState::Neutral);
    EXPECT_FALSE(r.entered_overspeed);
}

// ==========================================
// Resuming from neutral
// ==========================================
TEST(SpeedController, StaysNeutralUntilBelowHysteresisMargin) {
    auto p = default_params();  // speed_max_kmh = 4.0, margin = 0.05
    auto r = compute_control_step(ControllerState::Neutral, 0, 3.98f, p);
    EXPECT_EQ(r.next_state, ControllerState::Neutral);
    EXPECT_FALSE(r.resumed_from_overspeed);
}

TEST(SpeedController, ResumesToPressingBelowHysteresisMargin) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Neutral, 0, 3.90f, p);
    EXPECT_EQ(r.next_state, ControllerState::Pressing);
    EXPECT_TRUE(r.resumed_from_overspeed);
    EXPECT_EQ(r.servo_command, p.servo_full);
}

// ==========================================
// Pressing state / target detection
// ==========================================
TEST(SpeedController, PressingCommandsServoFullBelowTarget) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Pressing, 0, 0.3f, p);
    EXPECT_EQ(r.next_state, ControllerState::Pressing);
    EXPECT_EQ(r.servo_command, p.servo_full);
    EXPECT_FALSE(r.target_reached);
}

TEST(SpeedController, TargetReachedStartsRebound) {
    auto p = default_params();  // tolerance = 0.0, so needs an exact match
    auto r = compute_control_step(ControllerState::Pressing, 0, 0.8f, p);
    EXPECT_EQ(r.next_state, ControllerState::Rebound1);
    EXPECT_EQ(r.servo_command, p.rebound_angle);
    EXPECT_TRUE(r.target_reached);
    EXPECT_EQ(r.next_tick_count, 0);
}

TEST(SpeedController, ToleranceWidensTargetWindow) {
    auto p = default_params();
    p.tolerance_kmh = 0.1f;
    auto r = compute_control_step(ControllerState::Pressing, 0, 0.75f, p);
    EXPECT_TRUE(r.target_reached);
}

// ==========================================
// Rebound sequence (mirrors the Python control_loop tick-by-tick)
// ==========================================
TEST(SpeedController, Rebound1FirstTickHoldsReboundAngle) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Rebound1, 0, 0.8f, p);
    EXPECT_EQ(r.next_state, ControllerState::Rebound1);
    EXPECT_EQ(r.next_tick_count, 1);
    EXPECT_EQ(r.servo_command, p.rebound_angle);
}

TEST(SpeedController, Rebound1SecondTickMovesToRebound2) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Rebound1, 1, 0.8f, p);
    EXPECT_EQ(r.next_state, ControllerState::Rebound2);
    EXPECT_EQ(r.next_tick_count, 0);
    EXPECT_EQ(r.servo_command, p.servo_neutral);
}

TEST(SpeedController, Rebound2FirstTickHoldsNeutral) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Rebound2, 0, 0.8f, p);
    EXPECT_EQ(r.next_state, ControllerState::Rebound2);
    EXPECT_EQ(r.next_tick_count, 1);
    EXPECT_EQ(r.servo_command, p.servo_neutral);
}

TEST(SpeedController, Rebound2SecondTickReturnsToPressing) {
    auto p = default_params();
    auto r = compute_control_step(ControllerState::Rebound2, 1, 0.8f, p);
    EXPECT_EQ(r.next_state, ControllerState::Pressing);
    EXPECT_EQ(r.next_tick_count, 0);
    EXPECT_EQ(r.servo_command, p.servo_full);
}

TEST(SpeedController, FullReboundSequenceEndsBackAtPressing) {
    // Drives the state machine tick-by-tick, the same way the real timer
    // callback would, and confirms the full cycle: rebound_angle x2 ->
    // neutral x2 -> pressing.
    auto p = default_params();
    ControllerState state = ControllerState::Rebound1;
    int tick = 0;
    std::vector<int> servo_sequence;

    for (int i = 0; i < 4; ++i) {
        auto r = compute_control_step(state, tick, 0.8f, p);
        servo_sequence.push_back(r.servo_command);
        state = r.next_state;
        tick = r.next_tick_count;
    }

    ASSERT_EQ(servo_sequence.size(), 4u);
    EXPECT_EQ(servo_sequence[0], p.rebound_angle);
    EXPECT_EQ(servo_sequence[1], p.servo_neutral);
    EXPECT_EQ(servo_sequence[2], p.servo_neutral);
    EXPECT_EQ(servo_sequence[3], p.servo_full);
    EXPECT_EQ(state, ControllerState::Pressing);
}

// ==========================================
// clamp_servo() tests
// ==========================================
TEST(ClampServo, WithinRangeUnchanged) {
    EXPECT_EQ(clamp_servo(150), 150);
}

TEST(ClampServo, ClampsBelowZero) {
    EXPECT_EQ(clamp_servo(-10), 0);
}

TEST(ClampServo, ClampsAboveMax) {
    EXPECT_EQ(clamp_servo(300), 255);
}

TEST(ClampServo, BoundaryValuesUnchanged) {
    EXPECT_EQ(clamp_servo(0), 0);
    EXPECT_EQ(clamp_servo(255), 255);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}