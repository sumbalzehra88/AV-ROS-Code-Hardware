#include <gtest/gtest.h>
#include <sstream>
#include "heading_controller_pkg/gnss_checkpoint_monitor_utils.hpp"

// ==========================================
// haversine_m() / bearing_deg() sanity checks
// ==========================================
TEST(CheckpointMath, SamePointIsZeroDistance) {
    EXPECT_NEAR(haversine_m(10.0, 20.0, 10.0, 20.0), 0.0, 0.01);
}

TEST(CheckpointMath, KnownShortDistance) {
    double d = haversine_m(0.0, 0.0, 0.001, 0.0);
    EXPECT_NEAR(d, 111.19, 1.0);
}

TEST(CheckpointMath, BearingAlwaysInRange) {
    double b = bearing_deg(10.0, 10.0, 5.0, -5.0);
    EXPECT_GE(b, 0.0);
    EXPECT_LT(b, 360.0);
}

// ==========================================
// truthy_or() / truthy_or_single() -- documents the zero-fallback quirk
// ==========================================
TEST(TruthyOr, UsesFirstValueWhenNonZero) {
    EXPECT_DOUBLE_EQ(truthy_or(5.0, 10.0, 0.0), 5.0);
}

TEST(TruthyOr, ZeroFirstValueFallsThroughToSecond_RegressionCase) {
    // Documents the inherited Python bug: an explicit 0.0 is treated as
    // "not set" and falls through, even though it's a valid coordinate.
    EXPECT_DOUBLE_EQ(truthy_or(0.0, 7.0, 99.0), 7.0);
}

TEST(TruthyOr, BothZeroFallsThroughToFallback) {
    EXPECT_DOUBLE_EQ(truthy_or(0.0, 0.0, 42.0), 42.0);
}

TEST(TruthyOr, MissingFirstUsesSecond) {
    EXPECT_DOUBLE_EQ(truthy_or(std::nullopt, 3.0, 0.0), 3.0);
}

TEST(TruthyOrSingle, ZeroToleranceFallsBackToDefault_RegressionCase) {
    // An explicit tolerance_m: 0.0 gets silently replaced by the default --
    // same inherited quirk, kept faithful to the Python.
    EXPECT_DOUBLE_EQ(truthy_or_single(0.0, 2.5), 2.5);
}

TEST(TruthyOrSingle, NonZeroValuePreserved) {
    EXPECT_DOUBLE_EQ(truthy_or_single(1.5, 2.5), 1.5);
}

// ==========================================
// select_next_checkpoint()
// ==========================================
std::vector<Checkpoint> make_checkpoints() {
    Checkpoint a; a.id = 0; a.name = "A"; a.lat = 0.0; a.lon = 0.0;
    Checkpoint b; b.id = 1; b.name = "B"; b.lat = 0.01; b.lon = 0.0;
    Checkpoint c; c.id = 2; c.name = "C"; c.lat = 0.0001; c.lon = 0.0;
    return {a, b, c};
}

TEST(SelectNextCheckpoint, SequentialReturnsFirstUnvisited) {
    auto cps = make_checkpoints();
    auto idx = select_next_checkpoint(cps, "sequential", 0.0, 0.0);
    ASSERT_TRUE(idx.has_value());
    EXPECT_EQ(idx.value(), 0u);
}

TEST(SelectNextCheckpoint, SequentialSkipsVisited) {
    auto cps = make_checkpoints();
    cps[0].visited = true;
    auto idx = select_next_checkpoint(cps, "sequential", 0.0, 0.0);
    ASSERT_TRUE(idx.has_value());
    EXPECT_EQ(idx.value(), 1u);
}

TEST(SelectNextCheckpoint, ClosestReturnsNearestUnvisited) {
    auto cps = make_checkpoints();
    cps[0].visited = true;  // A sits exactly at (0,0), which would trivially
                             // win with distance 0 -- exclude it so this
                             // actually tests the comparison between B and C.
    // C (idx 2) is closest to (0,0) among the remaining unvisited (B, C).
    auto idx = select_next_checkpoint(cps, "closest", 0.0, 0.0);
    ASSERT_TRUE(idx.has_value());
    EXPECT_EQ(idx.value(), 2u);
}

TEST(SelectNextCheckpoint, ReturnsNulloptWhenAllVisited) {
    auto cps = make_checkpoints();
    for (auto& c : cps) c.visited = true;
    auto idx = select_next_checkpoint(cps, "sequential", 0.0, 0.0);
    EXPECT_FALSE(idx.has_value());
}

// ==========================================
// decide_loop_action()
// ==========================================
Checkpoint default_checkpoint() {
    Checkpoint cp;
    cp.tolerance_m = 2.0;
    cp.desired_heading_deg = 90.0;
    return cp;
}

TEST(DecideLoopAction, EnteringToleranceFirstTimeSetsPointTrue) {
    auto cp = default_checkpoint();
    auto d = decide_loop_action(cp, 1.5, false, false, std::nullopt, 3.0);
    EXPECT_EQ(d.action, LoopAction::EnteredTolerance);
    EXPECT_TRUE(d.point_flag_next);
}

TEST(DecideLoopAction, StaysInsideRequestsAlignmentWhenNotDone) {
    auto cp = default_checkpoint();
    auto d = decide_loop_action(cp, 1.5, true, false, 1.6, 3.0);
    EXPECT_EQ(d.action, LoopAction::RequestAlignment);
}

TEST(DecideLoopAction, StaysInsideNoActionWhenAlignedOrNoHeadingNeeded) {
    auto cp = default_checkpoint();
    auto d = decide_loop_action(cp, 1.5, true, true, 1.6, 3.0);
    EXPECT_EQ(d.action, LoopAction::None);
    EXPECT_TRUE(d.point_flag_next);
}

TEST(DecideLoopAction, LeavingAfterInsideAlwaysAdvances_AlignedOrNot) {
    auto cp = default_checkpoint();
    auto d1 = decide_loop_action(cp, 5.0, true, true, 1.5, 3.0);
    EXPECT_EQ(d1.action, LoopAction::ClearAndAdvance);
    auto d2 = decide_loop_action(cp, 5.0, true, false, 1.5, 3.0);
    EXPECT_EQ(d2.action, LoopAction::ClearAndAdvance);
}

TEST(DecideLoopAction, PassDetectionTriggersWhenDistanceJumpsPastThreshold) {
    auto cp = default_checkpoint();  // tolerance=2.0, so pass threshold = 2.0+3.0=5.0
    auto d = decide_loop_action(cp, 6.0, false, false, 5.5, 3.0);
    EXPECT_EQ(d.action, LoopAction::MarkPassedAndAdvance);
}

TEST(DecideLoopAction, NoPassDetectionWithoutPriorDistance) {
    auto cp = default_checkpoint();
    auto d = decide_loop_action(cp, 6.0, false, false, std::nullopt, 3.0);
    EXPECT_EQ(d.action, LoopAction::None);
}

TEST(DecideLoopAction, NormalMonitoringOutsideIsNoOp) {
    auto cp = default_checkpoint();
    auto d = decide_loop_action(cp, 4.0, false, false, 3.9, 3.0);
    EXPECT_EQ(d.action, LoopAction::None);
    EXPECT_FALSE(d.point_flag_next);
}

// ==========================================
// parse_checkpoints_csv()
// ==========================================
TEST(ParseCheckpointsCsv, NormalFile) {
    std::istringstream in(
        "id,name,lat,lon,tolerance_m,desired_heading_deg,behavior\n"
        "0,start,10.0,20.0,1.5,90.0,align_then_continue\n"
        "1,end,10.001,20.001,2.0,,align_then_continue\n");
    auto cps = parse_checkpoints_csv(in, 2.0);
    ASSERT_EQ(cps.size(), 2u);
    EXPECT_EQ(cps[0].name, "start");
    EXPECT_DOUBLE_EQ(cps[0].lat, 10.0);
    EXPECT_TRUE(cps[0].desired_heading_deg.has_value());
    EXPECT_FALSE(cps[1].desired_heading_deg.has_value());
}

TEST(ParseCheckpointsCsv, MissingFieldsUseDefaults) {
    std::istringstream in("id,lat,lon\n5,1.0,2.0\n");
    auto cps = parse_checkpoints_csv(in, 3.5);
    ASSERT_EQ(cps.size(), 1u);
    EXPECT_EQ(cps[0].id, 5);
    EXPECT_EQ(cps[0].name, "pt_5");
    EXPECT_DOUBLE_EQ(cps[0].tolerance_m, 3.5);
    EXPECT_EQ(cps[0].behavior, "align_then_continue");
}

TEST(ParseCheckpointsCsv, ZeroToleranceFallsBackToDefault_RegressionCase) {
    std::istringstream in("id,lat,lon,tolerance_m\n0,1.0,2.0,0.0\n");
    auto cps = parse_checkpoints_csv(in, 4.0);
    ASSERT_EQ(cps.size(), 1u);
    EXPECT_DOUBLE_EQ(cps[0].tolerance_m, 4.0);
}

TEST(ParseCheckpointsCsv, EmptyFileReturnsEmpty) {
    std::istringstream in("");
    auto cps = parse_checkpoints_csv(in, 2.0);
    EXPECT_TRUE(cps.empty());
}

TEST(ParseCheckpointsCsv, LatitudeColumnNameFallback) {
    std::istringstream in("id,latitude,longitude\n0,1.5,2.5\n");
    auto cps = parse_checkpoints_csv(in, 2.0);
    ASSERT_EQ(cps.size(), 1u);
    EXPECT_DOUBLE_EQ(cps[0].lat, 1.5);
    EXPECT_DOUBLE_EQ(cps[0].lon, 2.5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}