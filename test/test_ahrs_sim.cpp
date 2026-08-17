#include <gtest/gtest.h>
#include <cmath>
#include "heading_controller_pkg/ahrs_sim_utils.hpp"

// ==========================================
// bearing_deg() tests
// ==========================================
TEST(BearingDeg, DueNorth) {
    // Moving due north: longitude unchanged, latitude increases -> bearing ~0
    double b = bearing_deg(0.0, 0.0, 1.0, 0.0);
    EXPECT_NEAR(b, 0.0, 0.5);
}

TEST(BearingDeg, DueEast) {
    double b = bearing_deg(0.0, 0.0, 0.0, 1.0);
    EXPECT_NEAR(b, 90.0, 0.5);
}

TEST(BearingDeg, DueSouth) {
    double b = bearing_deg(1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(b, 180.0, 0.5);
}

TEST(BearingDeg, AlwaysInZeroTo360Range) {
    double b1 = bearing_deg(10.0, 10.0, 5.0, -5.0);
    double b2 = bearing_deg(-10.0, -10.0, -20.0, 20.0);
    EXPECT_GE(b1, 0.0);
    EXPECT_LT(b1, 360.0);
    EXPECT_GE(b2, 0.0);
    EXPECT_LT(b2, 360.0);
}

// ==========================================
// shortest_signed_deg_diff() / wrap360() tests
// ==========================================
TEST(WrapMath, ShortestDiffSimple) {
    EXPECT_NEAR(shortest_signed_deg_diff(10.0f, 0.0f), 10.0f, 0.001f);
}

TEST(WrapMath, ShortestDiffWrapsAcrossZero) {
    // Regression case for the Python-modulo-vs-C++-fmod sign issue.
    float d = shortest_signed_deg_diff(10.0f, 350.0f);
    EXPECT_NEAR(d, 20.0f, 0.001f);
}

TEST(WrapMath, ShortestDiffLargeWrap) {
    float d = shortest_signed_deg_diff(10.0f, 200.0f);
    EXPECT_NEAR(d, 170.0f, 0.001f);
    EXPECT_LE(std::abs(d), 180.0f);
}

TEST(WrapMath, Wrap360HandlesNegative) {
    EXPECT_NEAR(wrap360(-10.0f), 350.0f, 0.001f);
}

TEST(WrapMath, Wrap360HandlesOver360) {
    EXPECT_NEAR(wrap360(370.0f), 10.0f, 0.001f);
}

TEST(WrapMath, Wrap360Identity) {
    EXPECT_NEAR(wrap360(180.0f), 180.0f, 0.001f);
}

// ==========================================
// HeadingKalman1D tests
// ==========================================
TEST(HeadingKalman, ConvergesTowardRepeatedMeasurement) {
    HeadingKalman1D kf(0.0f, 10.0f, 8.0f, 0.01f);
    float first = kf.update(90.0f);
    float second = kf.update(90.0f);
    // Should move toward 90 and get closer with each consistent measurement.
    EXPECT_GT(second, first);
    EXPECT_LE(second, 90.0f);
}

TEST(HeadingKalman, HandlesWrapAroundZero) {
    // Starting near 350, measuring near 10 -- should move the short way
    // (through 360/0), not the long way through 180.
    HeadingKalman1D kf(350.0f, 10.0f, 8.0f, 0.01f);
    float result = kf.update(10.0f);
    // Result should be closer to 350->360->10 path, i.e. either just above
    // 350 or just above 0, not somewhere down near 180.
    bool near_wrap = (result > 300.0f || result < 60.0f);
    EXPECT_TRUE(near_wrap);
}

TEST(HeadingKalman, StaysWithinValidRange) {
    HeadingKalman1D kf(0.0f, 10.0f, 8.0f, 0.01f);
    for (int i = 0; i < 20; ++i) {
        float s = kf.update(static_cast<float>(i * 17 % 360));
        EXPECT_GE(s, 0.0f);
        EXPECT_LT(s, 360.0f);
    }
}

// ==========================================
// update_ema_heading() tests
// ==========================================
TEST(EmaHeading, FirstCallReturnsInputUnchanged) {
    float result = update_ema_heading(std::nullopt, 45.0f, 0.6f);
    EXPECT_FLOAT_EQ(result, 45.0f);
}

TEST(EmaHeading, SmoothsTowardNewValue) {
    float result = update_ema_heading(0.0f, 100.0f, 0.5f);
    EXPECT_NEAR(result, 50.0f, 0.001f);
}

TEST(EmaHeading, HandlesWrapAroundZero) {
    // prev=350, new=10 -- short way is +20 through zero, not -340.
    float result = update_ema_heading(350.0f, 10.0f, 1.0f);
    EXPECT_NEAR(result, 10.0f, 0.5f);
}

// ==========================================
// process_gps_fix() tests
// ==========================================
GnssHeadingParams default_gnss_params() {
    GnssHeadingParams p;
    p.min_speed_m_s = 0.5f;
    p.min_distance_m = 0.5f;
    p.max_sample_age_s = 2.0f;
    return p;
}

TEST(ProcessGpsFix, FirstFixIsStoredNotComputed) {
    auto p = default_gnss_params();
    FixState s;  // has_prev = false
    auto r = process_gps_fix(s, 10.0, 20.0, 100.0, p);
    EXPECT_TRUE(r.is_first_fix);
    EXPECT_FALSE(r.accepted);
    EXPECT_TRUE(r.next.has_prev);
    EXPECT_DOUBLE_EQ(r.next.prev_lat, 10.0);
}

TEST(ProcessGpsFix, AcceptsGoodDisplacementAboveSpeedThreshold) {
    auto p = default_gnss_params();
    FixState s{true, 0.0, 0.0, 0.0};
    // ~111m north over 1 second -> well above min_distance and min_speed
    auto r = process_gps_fix(s, 0.001, 0.0, 1.0, p);
    EXPECT_TRUE(r.accepted);
    EXPECT_FALSE(r.is_first_fix);
    EXPECT_GT(r.est_speed_mps, p.min_speed_m_s);
}

TEST(ProcessGpsFix, RejectsTinyDisplacementSilently) {
    auto p = default_gnss_params();
    FixState s{true, 0.0, 0.0, 0.0};
    // Tiny movement, well under min_distance_m, and not stale -> silent reject
    auto r = process_gps_fix(s, 0.0000001, 0.0, 0.5, p);
    EXPECT_FALSE(r.accepted);
    EXPECT_FALSE(r.refreshed_stale);
    // prev should stay unchanged since not stale
    EXPECT_DOUBLE_EQ(r.next.prev_lat, 0.0);
}

TEST(ProcessGpsFix, RefreshesStalePrevWithoutComputingBearing) {
    auto p = default_gnss_params();  // max_sample_age_s = 2.0
    FixState s{true, 0.0, 0.0, 0.0};
    // dt = 5s > max_sample_age_s -> stale, should refresh prev, not accept
    auto r = process_gps_fix(s, 0.0000001, 0.0, 5.0, p);
    EXPECT_FALSE(r.accepted);
    EXPECT_TRUE(r.refreshed_stale);
    EXPECT_DOUBLE_EQ(r.next.prev_lat, 0.0000001);
    EXPECT_DOUBLE_EQ(r.next.prev_time, 5.0);
}

TEST(ProcessGpsFix, ZeroMinSpeedAlwaysPassesSpeedGate) {
    auto p = default_gnss_params();
    p.min_speed_m_s = 0.0f;
    FixState s{true, 0.0, 0.0, 0.0};
    // Good distance, very slow speed (large dt) -- should still be accepted
    // since min_speed_m_s <= 0 bypasses the speed gate entirely.
    auto r = process_gps_fix(s, 0.001, 0.0, 9.0, p);
    EXPECT_TRUE(r.accepted);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}