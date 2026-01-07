#include <gtest/gtest.h>
#include "fastplanner/core/state.hpp"
#include <cmath>

using namespace fastplanner;

// ============================================================================
// Utility Function Tests
// ============================================================================

TEST(UtilityTests, AngleDifference) {
    // Test basic cases
    EXPECT_NEAR(angleDifference(0.0, 0.0), 0.0, 1e-6);
    EXPECT_NEAR(angleDifference(M_PI/2, 0.0), M_PI/2, 1e-6);
    EXPECT_NEAR(angleDifference(0.0, M_PI/2), -M_PI/2, 1e-6);
    
    // Test wraparound at ±π
    EXPECT_NEAR(angleDifference(M_PI, -M_PI), 0.0, 1e-6);
    EXPECT_NEAR(angleDifference(M_PI * 0.9, -M_PI * 0.9), -M_PI * 0.2, 1e-5);
    
    // Test that result is in [-π, π]
    double diff = angleDifference(3.0, -3.0);
    EXPECT_GE(diff, -M_PI);
    EXPECT_LE(diff, M_PI);
}

TEST(UtilityTests, NormalizeAngle) {
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-6);
    EXPECT_NEAR(normalizeAngle(M_PI), M_PI, 1e-6);
    EXPECT_NEAR(normalizeAngle(-M_PI), -M_PI, 1e-6);
    
    // Test wraparound
    EXPECT_NEAR(normalizeAngle(3 * M_PI), M_PI, 1e-5);
    EXPECT_NEAR(normalizeAngle(-3 * M_PI), -M_PI, 1e-5);
    EXPECT_NEAR(normalizeAngle(2 * M_PI), 0.0, 1e-5);
}

// ============================================================================
// Bounds Tests
// ============================================================================

TEST(BoundsTests, Construction) {
    Bounds bounds(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 10));
    
    EXPECT_EQ(bounds.position_min, Eigen::Vector2d(0, 0));
    EXPECT_EQ(bounds.position_max, Eigen::Vector2d(10, 10));
}

TEST(BoundsTests, PositionValidation) {
    Bounds bounds(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 10));
    
    EXPECT_TRUE(bounds.isPositionValid(Eigen::Vector2d(5, 5)));
    EXPECT_TRUE(bounds.isPositionValid(Eigen::Vector2d(0, 0)));
    EXPECT_TRUE(bounds.isPositionValid(Eigen::Vector2d(10, 10)));
    
    EXPECT_FALSE(bounds.isPositionValid(Eigen::Vector2d(-1, 5)));
    EXPECT_FALSE(bounds.isPositionValid(Eigen::Vector2d(11, 5)));
    EXPECT_FALSE(bounds.isPositionValid(Eigen::Vector2d(5, -1)));
    EXPECT_FALSE(bounds.isPositionValid(Eigen::Vector2d(5, 11)));
}

TEST(BoundsTests, Sampling) {
    Bounds bounds(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 10),
                  -M_PI, M_PI, 0.0, 5.0);
    
    // Sample multiple times and verify all are valid
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector2d pos = bounds.samplePosition();
        EXPECT_TRUE(bounds.isPositionValid(pos));
        
        double heading = bounds.sampleHeading();
        EXPECT_GE(heading, -M_PI);
        EXPECT_LE(heading, M_PI);
        
        double velocity = bounds.sampleVelocity();
        EXPECT_GE(velocity, 0.0);
        EXPECT_LE(velocity, 5.0);
    }
}

// ============================================================================
// State Tests
// ============================================================================

TEST(StateTests, DefaultConstruction) {
    State s;
    
    EXPECT_EQ(s.position, Eigen::Vector2d(0, 0));
    EXPECT_DOUBLE_EQ(s.heading, 0.0);
    EXPECT_DOUBLE_EQ(s.velocity, 0.0);
    EXPECT_DOUBLE_EQ(s.time, 0.0);
}

TEST(StateTests, ParameterizedConstruction) {
    State s(Eigen::Vector2d(1, 2), M_PI/4, 1.5, 1.0);
    
    EXPECT_EQ(s.position, Eigen::Vector2d(1, 2));
    EXPECT_NEAR(s.heading, M_PI/4, 1e-6);
    EXPECT_DOUBLE_EQ(s.velocity, 1.5);
    EXPECT_DOUBLE_EQ(s.time, 1.0);
}

TEST(StateTests, XYConstruction) {
    State s(1.0, 2.0, M_PI/4, 1.5, 1.0);
    
    EXPECT_DOUBLE_EQ(s.position.x(), 1.0);
    EXPECT_DOUBLE_EQ(s.position.y(), 2.0);
    EXPECT_NEAR(s.heading, M_PI/4, 1e-6);
}

TEST(StateTests, Distance) {
    State s1(0, 0);
    State s2(3, 4);
    
    EXPECT_DOUBLE_EQ(s1.distanceTo(s2), 5.0);
    EXPECT_DOUBLE_EQ(s2.distanceTo(s1), 5.0);
}

TEST(StateTests, ConfigurationDistance) {
    State s1(0, 0, 0.0);
    State s2(1, 0, M_PI/2);
    
    // Distance should include both position and heading
    double dist = s1.configurationDistanceTo(s2, 0.5);
    
    // Position distance = 1.0, heading difference = π/2
    // Expected: 1.0 + 0.5 * π/2 ≈ 1.785
    EXPECT_NEAR(dist, 1.0 + 0.5 * M_PI/2, 1e-5);
}

TEST(StateTests, Interpolation) {
    State s1(0, 0, 0.0, 0.0, 0.0);
    State s2(4, 0, M_PI, 2.0, 2.0);
    
    // Midpoint
    State mid = s1.interpolate(s2, 0.5);
    EXPECT_NEAR(mid.position.x(), 2.0, 1e-6);
    EXPECT_NEAR(mid.position.y(), 0.0, 1e-6);
    EXPECT_NEAR(mid.velocity, 1.0, 1e-6);
    EXPECT_NEAR(mid.time, 1.0, 1e-6);
    
    // Heading should interpolate with wraparound handling
    EXPECT_GE(mid.heading, -M_PI);
    EXPECT_LE(mid.heading, M_PI);
    
    // Endpoints
    State start = s1.interpolate(s2, 0.0);
    EXPECT_EQ(start.position, s1.position);
    
    State end = s1.interpolate(s2, 1.0);
    EXPECT_EQ(end.position, s2.position);
}

TEST(StateTests, HeadingNormalization) {
    State s(0, 0, 3 * M_PI);
    
    // Heading should be normalized to [-π, π]
    EXPECT_GE(s.heading, -M_PI);
    EXPECT_LE(s.heading, M_PI);
    EXPECT_NEAR(s.heading, M_PI, 1e-5);
}

TEST(StateTests, Validation) {
    Bounds bounds(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 10),
                  -M_PI, M_PI, 0.0, 5.0);
    
    State valid(5, 5, 0.0, 2.0);
    EXPECT_TRUE(valid.isValid(bounds));
    
    State invalid_pos(-1, 5, 0.0, 2.0);
    EXPECT_FALSE(invalid_pos.isValid(bounds));
    
    State invalid_vel(5, 5, 0.0, 10.0);
    EXPECT_FALSE(invalid_vel.isValid(bounds));
}

// ============================================================================
// Trajectory Tests
// ============================================================================

TEST(TrajectoryTests, Construction) {
    Trajectory traj;
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0);
}

TEST(TrajectoryTests, AddStates) {
    Trajectory traj;
    
    traj.addState(State(0, 0));
    EXPECT_EQ(traj.size(), 1);
    EXPECT_FALSE(traj.empty());
    
    traj.addState(State(1, 1));
    EXPECT_EQ(traj.size(), 2);
    
    EXPECT_EQ(traj[0].position, Eigen::Vector2d(0, 0));
    EXPECT_EQ(traj[1].position, Eigen::Vector2d(1, 1));
}

TEST(TrajectoryTests, Length) {
    Trajectory traj;
    
    // Empty trajectory
    EXPECT_DOUBLE_EQ(traj.getLength(), 0.0);
    
    // Single state
    traj.addState(State(0, 0));
    EXPECT_DOUBLE_EQ(traj.getLength(), 0.0);
    
    // Multiple states forming a 3-4-5 triangle
    traj.addState(State(3, 0));
    traj.addState(State(3, 4));
    
    EXPECT_DOUBLE_EQ(traj.getLength(), 7.0);  // 3 + 4
}

TEST(TrajectoryTests, Duration) {
    Trajectory traj;
    
    traj.addState(State(0, 0, 0, 0, 0.0));
    traj.addState(State(1, 0, 0, 0, 1.5));
    traj.addState(State(2, 0, 0, 0, 3.0));
    
    EXPECT_DOUBLE_EQ(traj.getDuration(), 3.0);
}

TEST(TrajectoryTests, Curvature) {
    Trajectory traj;
    
    // Straight line - should have zero curvature
    traj.addState(State(0, 0, 0.0));
    traj.addState(State(1, 0, 0.0));
    traj.addState(State(2, 0, 0.0));
    
    EXPECT_NEAR(traj.getCurvature(), 0.0, 1e-6);
    
    // Path with turn
    Trajectory curved;
    curved.addState(State(0, 0, 0.0));
    curved.addState(State(1, 0, M_PI/2));
    curved.addState(State(1, 1, M_PI/2));
    
    // Should have non-zero curvature due to heading change
    EXPECT_GT(curved.getCurvature(), 0.0);
}

TEST(TrajectoryTests, Cost) {
    Trajectory traj;
    traj.addState(State(0, 0, 0.0));
    traj.addState(State(3, 0, 0.0));
    traj.addState(State(3, 4, M_PI/2));
    
    double length = traj.getLength();  // 7.0
    double curvature = traj.getCurvature();
    
    // Cost with weight = 1.0
    double cost = traj.getCost(1.0);
    EXPECT_NEAR(cost, length + curvature, 1e-6);
    
    // Cost with different weight
    double cost2 = traj.getCost(2.0);
    EXPECT_NEAR(cost2, length + 2.0 * curvature, 1e-6);
}

TEST(TrajectoryTests, Downsample) {
    Trajectory traj;
    for (int i = 0; i < 10; ++i) {
        traj.addState(State(i, 0));
    }
    
    Trajectory downsampled = traj.downsample(2);
    
    // Should keep first, every 2nd, and last
    // Indices: 0, 2, 4, 6, 8, 9
    EXPECT_EQ(downsampled.size(), 6);
    EXPECT_EQ(downsampled[0].position.x(), 0.0);
    EXPECT_EQ(downsampled[1].position.x(), 2.0);
    EXPECT_EQ(downsampled[5].position.x(), 9.0);  // Last point
}

TEST(TrajectoryTests, Reverse) {
    Trajectory traj;
    traj.addState(State(0, 0));
    traj.addState(State(1, 0));
    traj.addState(State(2, 0));
    
    Trajectory reversed = traj.reverse();
    
    EXPECT_EQ(reversed.size(), 3);
    EXPECT_EQ(reversed[0].position.x(), 2.0);
    EXPECT_EQ(reversed[1].position.x(), 1.0);
    EXPECT_EQ(reversed[2].position.x(), 0.0);
}

TEST(TrajectoryTests, Clear) {
    Trajectory traj;
    traj.addState(State(0, 0));
    traj.addState(State(1, 1));
    
    EXPECT_FALSE(traj.empty());
    
    traj.clear();
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0);
}

TEST(TrajectoryTests, InterpolateToTimeStep) {
    Trajectory traj;
    traj.addState(State(0, 0, 0, 0, 0.0));
    traj.addState(State(2, 0, 0, 0, 1.0));
    traj.addState(State(2, 2, 0, 0, 2.0));
    
    // Interpolate with 0.5s time step
    Trajectory interpolated = traj.interpolateToTimeStep(0.5);
    
    // Should have waypoints at t = 0, 0.5, 1.0, 1.5, 2.0
    EXPECT_GE(interpolated.size(), 5);
    
    // Check first and last are preserved
    EXPECT_EQ(interpolated[0].position, traj[0].position);
    EXPECT_EQ(interpolated.getStates().back().position, traj.getStates().back().position);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(EdgeCaseTests, StateInterpolationWithWraparound) {
    // Test heading interpolation across ±π boundary
    State s1(0, 0, M_PI * 0.9);
    State s2(0, 0, -M_PI * 0.9);
    
    State mid = s1.interpolate(s2, 0.5);
    
    // Should interpolate the short way (through π), not long way
    // Expected: near π or -π
    EXPECT_GE(mid.heading, -M_PI);
    EXPECT_LE(mid.heading, M_PI);
    EXPECT_GT(std::abs(mid.heading), M_PI * 0.8);  // Should be near boundary
}

TEST(EdgeCaseTests, ZeroDistanceInterpolation) {
    State s(1, 1);
    State same = s.interpolate(s, 0.5);
    
    EXPECT_EQ(same.position, s.position);
    EXPECT_DOUBLE_EQ(same.heading, s.heading);
}

TEST(EdgeCaseTests, OutOfBoundsInterpolation) {
    State s1(0, 0);
    State s2(1, 1);
    
    // t < 0 should clamp to 0
    State before = s1.interpolate(s2, -0.5);
    EXPECT_EQ(before.position, s1.position);
    
    // t > 1 should clamp to 1
    State after = s1.interpolate(s2, 1.5);
    EXPECT_EQ(after.position, s2.position);
}

// Main function
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
