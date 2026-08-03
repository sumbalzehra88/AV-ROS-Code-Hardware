#include <gtest/gtest.h>
#include <sstream>
#include <cmath>
#include "heading_controller_pkg/speed_utils.hpp"

// ==========================================
// heading_error() tests
// ==========================================
TEST(HeadingError, TrivialPositiveError) {
    EXPECT_FLOAT_EQ(heading_error(10.0f, 0.0f), 10.0f);
}

TEST(HeadingError, SimpleWrapNegative) {
    EXPECT_FLOAT_EQ(heading_error(350.0f, 10.0f), -20.0f);
}

TEST(HeadingError, SimpleWrapPositive) {
    EXPECT_FLOAT_EQ(heading_error(10.0f, 350.0f), 20.0f);
}

// Regression test for the std::fmod sign bug: fmod() keeps the sign of the
// dividend, so without the re-wrap fix this used to return -190 instead of
// the correct short-way-around answer of +170.
TEST(HeadingError, LargeWrapPositive_RegressionCase) {
    float result = heading_error(10.0f, 200.0f);
    EXPECT_FLOAT_EQ(result, 170.0f);
    EXPECT_LE(std::abs(result), 180.0f);
}

TEST(HeadingError, LargeWrapNegative_RegressionCase) {
    float result = heading_error(200.0f, 10.0f);
    EXPECT_FLOAT_EQ(result, -170.0f);
    EXPECT_LE(std::abs(result), 180.0f);
}

TEST(HeadingError, ZeroError) {
    EXPECT_FLOAT_EQ(heading_error(0.0f, 0.0f), 0.0f);
}

TEST(HeadingError, BoundaryCaseStaysWithinRange) {
    // 180 vs 0 is the ambiguous antipodal case; the important invariant is
    // that the result never exceeds the [-180, 180] range regardless of sign.
    float result = heading_error(180.0f, 0.0f);
    EXPECT_LE(std::abs(result), 180.0f);
}

TEST(HeadingError, OutputAlwaysWithinValidRange) {
    // Sweep a range of target/current combinations and assert the invariant
    // holds everywhere, not just at the specific cases above.
    for (float target = 0.0f; target < 360.0f; target += 37.0f) {
        for (float current = 0.0f; current < 360.0f; current += 53.0f) {
            float result = heading_error(target, current);
            EXPECT_LE(std::abs(result), 180.0f)
                << "target=" << target << " current=" << current << " result=" << result;
        }
    }
}

// ==========================================
// parse_waypoints_csv() tests
// ==========================================
TEST(ParseWaypointsCsv, NormalFile) {
    std::istringstream in("heading\n90\n180\n270\n0\n");
    auto wp = parse_waypoints_csv(in);
    ASSERT_EQ(wp.size(), 4u);
    EXPECT_FLOAT_EQ(wp[0], 90.0f);
    EXPECT_FLOAT_EQ(wp[1], 180.0f);
    EXPECT_FLOAT_EQ(wp[2], 270.0f);
    EXPECT_FLOAT_EQ(wp[3], 0.0f);
}

TEST(ParseWaypointsCsv, EmptyFile) {
    std::istringstream in("");
    auto wp = parse_waypoints_csv(in);
    EXPECT_TRUE(wp.empty());
}

TEST(ParseWaypointsCsv, HeaderOnlyNoDataRows) {
    std::istringstream in("heading\n");
    auto wp = parse_waypoints_csv(in);
    EXPECT_TRUE(wp.empty());
}

TEST(ParseWaypointsCsv, MalformedRowsAreSkippedGoodRowsKept) {
    std::istringstream in("heading\nabc\n90\n,,,\n180\n");
    auto wp = parse_waypoints_csv(in);
    ASSERT_EQ(wp.size(), 2u);
    EXPECT_FLOAT_EQ(wp[0], 90.0f);
    EXPECT_FLOAT_EQ(wp[1], 180.0f);
}

TEST(ParseWaypointsCsv, SingleWaypoint) {
    std::istringstream in("heading\n45\n");
    auto wp = parse_waypoints_csv(in);
    ASSERT_EQ(wp.size(), 1u);
    EXPECT_FLOAT_EQ(wp[0], 45.0f);
}

TEST(ParseWaypointsCsv, RoundsToOneDecimalPlace) {
    std::istringstream in("heading\n90.37\n");
    auto wp = parse_waypoints_csv(in);
    ASSERT_EQ(wp.size(), 1u);
    EXPECT_FLOAT_EQ(wp[0], 90.4f);
}

TEST(ParseWaypointsCsv, NegativeHeadingsParseCorrectly) {
    std::istringstream in("heading\n-45\n-180\n");
    auto wp = parse_waypoints_csv(in);
    ASSERT_EQ(wp.size(), 2u);
    EXPECT_FLOAT_EQ(wp[0], -45.0f);
    EXPECT_FLOAT_EQ(wp[1], -180.0f);
}

// ==========================================
// PID controller tests
// ==========================================
TEST(PidController, ProportionalOnlyOutputOnFirstStep) {
    // First call has no prev_time_, so dt=0 and only the P term contributes.
    PID pid(2.0f, 0.0f, 0.0f);
    float out = pid.step(5.0f, 0.0f);
    EXPECT_FLOAT_EQ(out, 10.0f);  // kp * error = 2.0 * 5.0
}

TEST(PidController, IntegralAccumulatesOverTime) {
    PID pid(0.0f, 1.0f, 0.0f);
    pid.step(2.0f, 0.0f);               // dt=0, no integration yet
    float out = pid.step(2.0f, 1.0f);   // dt=1s -> integrator += 2*1 = 2
    EXPECT_FLOAT_EQ(out, 2.0f);
}

TEST(PidController, IntegratorClampsToLimit) {
    PID pid(0.0f, 1.0f, 0.0f, 5.0f);    // integrator_limit = 5
    pid.step(100.0f, 0.0f);
    float out = pid.step(100.0f, 10.0f);  // large dt would blow past the limit
    EXPECT_LE(std::abs(out), 5.0f);
}

TEST(PidController, OutputClampsToLimit) {
    PID pid(10.0f, 0.0f, 0.0f, std::nullopt, 3.0f);  // output_limit = 3
    float out = pid.step(100.0f, 0.0f);
    EXPECT_FLOAT_EQ(out, 3.0f);
}

TEST(PidController, DerivativeRespondsToChangingError) {
    PID pid(0.0f, 0.0f, 1.0f);
    pid.step(0.0f, 0.0f);
    float out = pid.step(10.0f, 1.0f);  // error changed by 10 over dt=1
    EXPECT_FLOAT_EQ(out, 10.0f);
}

TEST(PidController, NegativeDtIsTreatedAsZero) {
    // Guards against out-of-order timestamps corrupting the integrator.
    PID pid(0.0f, 1.0f, 0.0f);
    pid.step(5.0f, 5.0f);
    float out = pid.step(5.0f, 2.0f);  // "now" earlier than prev_time_
    EXPECT_FLOAT_EQ(out, 0.0f);        // dt clamped to 0, so no new integration
}

TEST(PidController, ResetClearsInternalState) {
    PID pid(0.0f, 1.0f, 0.0f);
    pid.step(5.0f, 0.0f);
    pid.step(5.0f, 1.0f);
    pid.reset();
    float out = pid.step(5.0f, 0.0f);  // behaves like a brand-new controller
    EXPECT_FLOAT_EQ(out, 0.0f);        // first step after reset: dt=0, i_out=0
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}