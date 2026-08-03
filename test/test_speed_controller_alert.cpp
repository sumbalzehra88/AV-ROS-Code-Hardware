#include <gtest/gtest.h>
#include "heading_controller_pkg/speed_controller_alert_utils.hpp"

ServoParams default_params() {
    ServoParams p;
    p.target_speed_kmh = 1.0f;
    p.tolerance_kmh = 0.0f;
    p.servo_neutral = 164;
    p.servo_full = 158;
    p.rebound_angle = 170;
    p.speed_max_kmh = 5.0f;
    return p;
}

// ==========================================
// Alert override -- highest priority
// ==========================================
TEST(SpeedControllerAlert, AlertForcesReboundAngleFromPressing) {
    auto p = default_params();
    auto r = compute_control_step_with_alert(ControllerState::Pressing, 0, 0.3f, true, p);
    EXPECT_EQ(r.servo_command, p.rebound_angle);
}

TEST(SpeedControllerAlert, AlertFreezesStateAndTickCount) {
    auto p = default_params();
    // Even mid-rebound with tick_count=1, alert should freeze it exactly
    // as-is rather than advancing the FSM.
    auto r = compute_control_step_with_alert(ControllerState::Rebound1, 1, 0.9f, true, p);
    EXPECT_EQ(r.next_state, ControllerState::Rebound1);
    EXPECT_EQ(r.next_tick_count, 1);
    EXPECT_EQ(r.servo_command, p.rebound_angle);
}

TEST(SpeedControllerAlert, AlertOverridesEvenAtOverspeed) {
    auto p = default_params();
    // Alert check happens before the overspeed check -- confirm it wins.
    auto r = compute_control_step_with_alert(ControllerState::Pressing, 0, 10.0f, true, p);
    EXPECT_EQ(r.servo_command, p.rebound_angle);
    EXPECT_FALSE(r.entered_overspeed);
}

TEST(SpeedControllerAlert, ClearingAlertResumesExactStateNextTick) {
    auto p = default_params();
    // While alert is active, state is frozen at Rebound2, tick=1.
    auto during = compute_control_step_with_alert(ControllerState::Rebound2, 1, 0.9f, true, p);
    EXPECT_EQ(during.next_state, ControllerState::Rebound2);
    EXPECT_EQ(during.next_tick_count, 1);

    // Once alert clears, the FSM should resume from exactly that point --
    // Rebound2 with tick_count=1 advances to Pressing.
    auto after = compute_control_step_with_alert(during.next_state, during.next_tick_count, 0.9f, false, p);
    EXPECT_EQ(after.next_state, ControllerState::Pressing);
    EXPECT_EQ(after.servo_command, p.servo_full);
}

// ==========================================
// Normal FSM behavior (alert_active = false), same as base controller
// ==========================================
TEST(SpeedControllerAlert, OverspeedTriggersNeutralFromPressing) {
    auto p = default_params();
    auto r = compute_control_step_with_alert(ControllerState::Pressing, 0, 6.0f, false, p);
    EXPECT_EQ(r.next_state, ControllerState::Neutral);
    EXPECT_EQ(r.servo_command, p.servo_neutral);
    EXPECT_TRUE(r.entered_overspeed);
}

TEST(SpeedControllerAlert, ResumesToPressingBelowHysteresisMargin) {
    auto p = default_params();  // speed_max_kmh = 5.0, margin = 0.05
    auto r = compute_control_step_with_alert(ControllerState::Neutral, 0, 4.90f, false, p);
    EXPECT_EQ(r.next_state, ControllerState::Pressing);
    EXPECT_TRUE(r.resumed_from_overspeed);
}

TEST(SpeedControllerAlert, PressingCommandsServoFullBelowTarget) {
    auto p = default_params();
    auto r = compute_control_step_with_alert(ControllerState::Pressing, 0, 0.3f, false, p);
    EXPECT_EQ(r.next_state, ControllerState::Pressing);
    EXPECT_EQ(r.servo_command, p.servo_full);
    EXPECT_FALSE(r.target_reached);
}

TEST(SpeedControllerAlert, TargetReachedStartsRebound) {
    auto p = default_params();
    auto r = compute_control_step_with_alert(ControllerState::Pressing, 0, 1.0f, false, p);
    EXPECT_EQ(r.next_state, ControllerState::Rebound1);
    EXPECT_EQ(r.servo_command, p.rebound_angle);
    EXPECT_TRUE(r.target_reached);
}

TEST(SpeedControllerAlert, FullReboundSequenceEndsBackAtPressing) {
    auto p = default_params();
    ControllerState state = ControllerState::Rebound1;
    int tick = 0;
    std::vector<int> servo_sequence;

    for (int i = 0; i < 4; ++i) {
        auto r = compute_control_step_with_alert(state, tick, 1.0f, false, p);
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

TEST(SpeedControllerAlert, UnknownStateFallsBackToNeutral) {
    auto p = default_params();
    // Cast an out-of-range int into ControllerState to simulate an
    // unexpected/corrupt state value, mirroring the Python "else" branch.
    auto bogus_state = static_cast<ControllerState>(99);
    auto r = compute_control_step_with_alert(bogus_state, 0, 1.0f, false, p);
    EXPECT_EQ(r.next_state, ControllerState::Neutral);
    EXPECT_EQ(r.servo_command, p.servo_neutral);
}

// ==========================================
// clamp_servo() tests
// ==========================================
TEST(ClampServoAlert, ClampsBelowZero) {
    EXPECT_EQ(clamp_servo(-5), 0);
}

TEST(ClampServoAlert, ClampsAboveMax) {
    EXPECT_EQ(clamp_servo(400), 255);
}

TEST(ClampServoAlert, WithinRangeUnchanged) {
    EXPECT_EQ(clamp_servo(170), 170);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}