#include <gtest/gtest.h>
#include <cmath>
#include "heading_controller_pkg/speed2_utils.hpp"

// ==========================================
// convert_speed() tests
// ==========================================

// Regression test for the swapped SPEED_CHART / SPEED_CHART_INVERSE bug:
// converting 10 m/s to km/h must give 36.0, not 2.778 (the old, inverted
// result).
TEST(ConvertSpeed, MpsToKmh_RegressionCase) {
    float result = convert_speed(10.0f, "m/s", "km/h");
    EXPECT_NEAR(result, 36.0f, 0.001f);
}

TEST(ConvertSpeed, KmhToMps) {
    float result = convert_speed(36.0f, "km/h", "m/s");
    EXPECT_NEAR(result, 10.0f, 0.001f);
}

TEST(ConvertSpeed, MpsToMph) {
    // 10 m/s = 36 km/h = 36 * 0.621371192 mph = 22.3693... mph
    float result = convert_speed(10.0f, "m/s", "mph");
    EXPECT_NEAR(result, 22.369f, 0.01f);
}

TEST(ConvertSpeed, KnotToKmh) {
    // 1 knot = 1.852 km/h
    float result = convert_speed(1.0f, "knot", "km/h");
    EXPECT_NEAR(result, 1.852f, 0.001f);
}

TEST(ConvertSpeed, SameUnitIsIdentity) {
    float result = convert_speed(42.0f, "m/s", "m/s");
    EXPECT_NEAR(result, 42.0f, 0.001f);
}

TEST(ConvertSpeed, ZeroSpeedStaysZero) {
    float result = convert_speed(0.0f, "m/s", "knot");
    EXPECT_NEAR(result, 0.0f, 0.001f);
}

TEST(ConvertSpeed, RoundTripConversionReturnsOriginal) {
    // m/s -> mph -> m/s should return (approximately) the original value.
    float to_mph = convert_speed(15.0f, "m/s", "mph");
    float back_to_mps = convert_speed(to_mph, "mph", "m/s");
    EXPECT_NEAR(back_to_mps, 15.0f, 0.01f);
}

TEST(ConvertSpeed, InvalidUnitFromThrows) {
    EXPECT_THROW(convert_speed(10.0f, "furlong/fortnight", "km/h"), std::invalid_argument);
}

TEST(ConvertSpeed, InvalidUnitToThrows) {
    EXPECT_THROW(convert_speed(10.0f, "m/s", "furlong/fortnight"), std::invalid_argument);
}

// ==========================================
// get_distance() tests
// ==========================================
TEST(GetDistance, SamePointIsZero) {
    double d = get_distance(37.7749, -122.4194, 37.7749, -122.4194);
    EXPECT_NEAR(d, 0.0, 0.01);
}

TEST(GetDistance, KnownShortDistance) {
    // Two points ~111m apart (0.001 degree of latitude at the equator-ish
    // scale is roughly 111 meters).
    double d = get_distance(0.0, 0.0, 0.001, 0.0);
    EXPECT_NEAR(d, 111.19, 1.0);
}

TEST(GetDistance, KnownLongerDistance) {
    // San Francisco (37.7749, -122.4194) to Los Angeles (34.0522, -118.2437)
    // is approximately 559 km.
    double d = get_distance(37.7749, -122.4194, 34.0522, -118.2437);
    EXPECT_NEAR(d, 559000.0, 5000.0);  // within 5 km tolerance
}

TEST(GetDistance, DistanceIsSymmetric) {
    double d1 = get_distance(37.7749, -122.4194, 34.0522, -118.2437);
    double d2 = get_distance(34.0522, -118.2437, 37.7749, -122.4194);
    EXPECT_NEAR(d1, d2, 0.01);
}

TEST(GetDistance, SmallIncrementalMovementIsAccurate) {
    // Regression-style test for the float precision-loss issue: two GPS
    // fixes ~1 meter apart should not collapse to 0 or produce a wildly
    // inaccurate result due to catastrophic cancellation.
    double lat1 = 40.712800;
    double lon1 = -74.006000;
    double lat2 = 40.712809;  // roughly ~1m north
    double lon2 = -74.006000;
    double d = get_distance(lat1, lon1, lat2, lon2);
    EXPECT_GT(d, 0.5);
    EXPECT_LT(d, 1.5);
}

TEST(GetDistance, NeverReturnsNaN) {
    // Antipodal-ish points can push the haversine `a` term right at the
    // sqrt(1-a) boundary; confirm the clamp prevents NaN.
    double d = get_distance(0.0, 0.0, 0.0, 180.0);
    EXPECT_FALSE(std::isnan(d));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}