#include <gtest/gtest.h>
#include "heading_controller_pkg/dynamic_utils.hpp"

SweepParams default_params() {
    SweepParams p;
    p.pos_limit_deg = 20.0f;
    p.neg_limit_deg = -24.0f;
    p.rebound_step_deg = 1.0f;
    return p;
}

// ==========================================
// IDLE state transitions
// ==========================================
TEST(SteeringMonitor, IdleStaysIdleBelowLimits) {
    auto p = default_params();
    SweepState s;
    auto r = compute_steering_step(s, 5.0f, p);
    EXPECT_EQ(r.next.state, SteeringState::Idle);
}

TEST(SteeringMonitor, IdleHitsPositiveLimit) {
    auto p = default_params();
    SweepState s;
    auto r = compute_steering_step(s, 20.0f, p);
    EXPECT_EQ(r.next.state, SteeringState::HitPosLimit);
    EXPECT_EQ(r.next.rebound_direction, -1);
}

TEST(SteeringMonitor, IdleHitsNegativeLimit) {
    auto p = default_params();
    SweepState s;
    auto r = compute_steering_step(s, -24.0f, p);
    EXPECT_EQ(r.next.state, SteeringState::HitNegLimit);
    EXPECT_EQ(r.next.rebound_direction, 1);
}

// ==========================================
// HIT_POS_LIMIT / HIT_NEG_LIMIT -> REBOUNDING
// ==========================================
TEST(SteeringMonitor, HitPosLimitTransitionsToRebounding) {
    auto p = default_params();
    SweepState s;
    s.state = SteeringState::HitPosLimit;
    auto r = compute_steering_step(s, 21.0f, p);  // incoming reading is overwritten
    EXPECT_EQ(r.next.state, SteeringState::Rebounding);
    EXPECT_FLOAT_EQ(r.next.theta_deg, 20.0f);
}

TEST(SteeringMonitor, HitNegLimitTransitionsToRebounding) {
    auto p = default_params();
    SweepState s;
    s.state = SteeringState::HitNegLimit;
    auto r = compute_steering_step(s, -25.0f, p);
    EXPECT_EQ(r.next.state, SteeringState::Rebounding);
    EXPECT_FLOAT_EQ(r.next.theta_deg, -24.0f);
}

// ==========================================
// REBOUNDING: incremental sweep
// ==========================================
TEST(SteeringMonitor, ReboundingStepsTowardOppositeLimit) {
    // NOTE: during Rebounding, theta_deg is first overwritten by the
    // incoming "measured" reading, THEN stepped -- it does not silently
    // ignore the input. To simulate a sensor that's actually tracking the
    // sweep, feed the current theta_deg back in as the measured value.
    auto p = default_params();
    SweepState s;
    s.state = SteeringState::Rebounding;
    s.theta_deg = 20.0f;
    s.rebound_direction = -1;

    auto r = compute_steering_step(s, s.theta_deg, p);
    EXPECT_EQ(r.next.state, SteeringState::Rebounding);
    EXPECT_FLOAT_EQ(r.next.theta_deg, 19.0f);
    EXPECT_FALSE(r.next.rebound_completed);
}

// ==========================================
// REBOUNDING -> reaching the second limit -> immediate LockCenter
// (documents the "trivially satisfied" lock-center condition quirk)
// ==========================================
TEST(SteeringMonitor, ReachingSecondLimitTriggersImmediateLockCenter) {
    auto p = default_params();
    SweepState s;
    s.state = SteeringState::Rebounding;
    s.theta_deg = -23.0f;
    s.rebound_direction = -1;

    // Feed the current theta_deg as the measured reading (simulating a
    // sensor tracking the sweep). One more step of -1.0 pushes theta to
    // -24.0, hitting the negative limit. This SAME tick also satisfies
    // the lock-center condition (direction flips to +1, theta=-24 <= 0),
    // so it jumps straight to LockCenter and snaps theta to 0 -- it does
    // not gradually sweep back through zero on a separate leg.
    auto r = compute_steering_step(s, s.theta_deg, p);
    EXPECT_TRUE(r.next.rebound_completed);
    EXPECT_EQ(r.next.state, SteeringState::LockCenter);
    EXPECT_FLOAT_EQ(r.next.theta_deg, 0.0f);
}

TEST(SteeringMonitor, ReachingSecondLimitFromPositiveSideAlsoLocksImmediately) {
    auto p = default_params();
    SweepState s;
    s.state = SteeringState::Rebounding;
    s.theta_deg = 19.0f;
    s.rebound_direction = 1;

    auto r = compute_steering_step(s, s.theta_deg, p);
    EXPECT_TRUE(r.next.rebound_completed);
    EXPECT_EQ(r.next.state, SteeringState::LockCenter);
    EXPECT_FLOAT_EQ(r.next.theta_deg, 0.0f);
}

// ==========================================
// LOCK_CENTER: no handling branch -- state and theta pass through
// unmodified except theta getting overwritten by the raw input reading
// (documents the "falls through silently" quirk)
// ==========================================
TEST(SteeringMonitor, LockCenterHasNoActiveHandling) {
    auto p = default_params();
    SweepState s;
    s.state = SteeringState::LockCenter;
    s.rebound_completed = true;
    s.point_sent = true;

    auto r = compute_steering_step(s, 7.5f, p);
    // State stays LockCenter (no branch changes it), but theta_deg is
    // overwritten with whatever the raw input was, since that overwrite
    // happens unconditionally at the top of the function.
    EXPECT_EQ(r.next.state, SteeringState::LockCenter);
    EXPECT_FLOAT_EQ(r.next.theta_deg, 7.5f);
}

// ==========================================
// /point published exactly once
// ==========================================
TEST(SteeringMonitor, PointPublishesTrueOnlyOnceOnCompletion) {
    auto p = default_params();
    SweepState s;
    s.state = SteeringState::Rebounding;
    s.theta_deg = -23.0f;
    s.rebound_direction = -1;

    auto r1 = compute_steering_step(s, s.theta_deg, p);
    EXPECT_TRUE(r1.point_value);
    EXPECT_TRUE(r1.next.point_sent);

    // Next call, even though rebound_completed is still true, point_sent
    // is now true too, so it should not fire again. Feed the resulting
    // theta_deg (0.0, now in LockCenter) back in as the next reading.
    auto r2 = compute_steering_step(r1.next, r1.next.theta_deg, p);
    EXPECT_FALSE(r2.point_value);
}

TEST(SteeringMonitor, PointStaysFalseThroughoutNormalSweep) {
    auto p = default_params();
    SweepState s;
    auto r1 = compute_steering_step(s, 20.0f, p);   // Idle -> HitPosLimit
    EXPECT_FALSE(r1.point_value);
    auto r2 = compute_steering_step(r1.next, 20.0f, p);  // -> Rebounding
    EXPECT_FALSE(r2.point_value);
    auto r3 = compute_steering_step(r2.next, 19.0f, p);  // one rebound step
    EXPECT_FALSE(r3.point_value);
}

// ==========================================
// Full sequence: idle -> pos limit -> rebound -> neg limit -> lock center
// ==========================================
TEST(SteeringMonitor, FullSweepSequenceFromIdleToLockCenter) {
    auto p = default_params();
    SweepState s;

    auto r = compute_steering_step(s, 20.0f, p);          // Idle -> HitPosLimit
    EXPECT_EQ(r.next.state, SteeringState::HitPosLimit);

    r = compute_steering_step(r.next, 20.0f, p);          // -> Rebounding, theta=20
    EXPECT_EQ(r.next.state, SteeringState::Rebounding);
    EXPECT_FLOAT_EQ(r.next.theta_deg, 20.0f);

    // Step all the way down to the negative limit (44 steps of -1.0 from
    // 20 to -24). Feed each step's resulting theta_deg back in as the
    // next measured reading -- this is what a real sensor tracking the
    // sweep would report, and it's what compute_steering_step actually
    // steps from internally.
    bool locked = false;
    for (int i = 0; i < 60 && !locked; ++i) {
        r = compute_steering_step(r.next, r.next.theta_deg, p);
        if (r.next.state == SteeringState::LockCenter) {
            locked = true;
        }
    }

    EXPECT_TRUE(locked);
    EXPECT_TRUE(r.next.rebound_completed);
    EXPECT_FLOAT_EQ(r.next.theta_deg, 0.0f);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}