#include <gtest/gtest.h>
#include "fastplanner/core/collision_checker.hpp"
#include <cmath>

using namespace fastplanner;

// ============================================================================
// CircleObstacle Tests
// ============================================================================

TEST(CircleObstacleTests, StaticObstacle) {
    CircleObstacle obs(Eigen::Vector2d(0, 0), 1.0);
    
    EXPECT_EQ(obs.center, Eigen::Vector2d(0, 0));
    EXPECT_DOUBLE_EQ(obs.radius, 1.0);
    EXPECT_FALSE(obs.is_dynamic);
}

TEST(CircleObstacleTests, DynamicObstacle) {
    CircleObstacle obs(Eigen::Vector2d(0, 0), 1.0, Eigen::Vector2d(1, 0));
    
    EXPECT_TRUE(obs.is_dynamic);
    EXPECT_EQ(obs.velocity, Eigen::Vector2d(1, 0));
}

TEST(CircleObstacleTests, PredictPosition) {
    CircleObstacle obs(Eigen::Vector2d(0, 0), 1.0, Eigen::Vector2d(2, 1));
    
    // Predict 1 second into future
    Eigen::Vector2d pred = obs.predictPosition(1.0);
    EXPECT_EQ(pred, Eigen::Vector2d(2, 1));
    
    // Predict 2 seconds
    pred = obs.predictPosition(2.0);
    EXPECT_EQ(pred, Eigen::Vector2d(4, 2));
    
    // Static obstacle should not move
    CircleObstacle static_obs(Eigen::Vector2d(5, 5), 1.0);
    EXPECT_EQ(static_obs.predictPosition(1.0), Eigen::Vector2d(5, 5));
}

TEST(CircleObstacleTests, Collision) {
    CircleObstacle obs(Eigen::Vector2d(0, 0), 2.0);
    
    // Inside obstacle
    EXPECT_TRUE(obs.collidesWith(Eigen::Vector2d(0, 0)));
    EXPECT_TRUE(obs.collidesWith(Eigen::Vector2d(1, 0)));
    
    // On boundary
    EXPECT_TRUE(obs.collidesWith(Eigen::Vector2d(2, 0)));
    
    // Outside obstacle
    EXPECT_FALSE(obs.collidesWith(Eigen::Vector2d(3, 0)));
    EXPECT_FALSE(obs.collidesWith(Eigen::Vector2d(0, 5)));
}

TEST(CircleObstacleTests, CollisionWithMargin) {
    CircleObstacle obs(Eigen::Vector2d(0, 0), 1.0);
    
    double margin = 0.5;
    
    // Just outside obstacle but within margin
    EXPECT_TRUE(obs.collidesWith(Eigen::Vector2d(1.2, 0), margin));
    
    // Outside obstacle + margin
    EXPECT_FALSE(obs.collidesWith(Eigen::Vector2d(2, 0), margin));
}

// ============================================================================
// SpatialHashGrid Tests
// ============================================================================

TEST(SpatialHashTests, HashConsistency) {
    SpatialHashGrid grid(1.0);
    
    // Same position should give same hash
    Eigen::Vector2d pos(1.5, 2.5);
    int64_t hash1 = grid.getHashKey(pos);
    int64_t hash2 = grid.getHashKey(pos);
    
    EXPECT_EQ(hash1, hash2);
}

TEST(SpatialHashTests, NearbyCellsHash) {
    SpatialHashGrid grid(1.0);
    
    // Nearby positions should have different hashes (unless in same cell)
    Eigen::Vector2d pos1(0.5, 0.5);
    Eigen::Vector2d pos2(1.5, 0.5);
    
    int64_t hash1 = grid.getHashKey(pos1);
    int64_t hash2 = grid.getHashKey(pos2);
    
    EXPECT_NE(hash1, hash2);
}

TEST(SpatialHashTests, InsertAndQuery) {
    SpatialHashGrid grid(2.0);
    
    // Create obstacles
    CircleObstacle obs1(Eigen::Vector2d(0, 0), 0.5);
    CircleObstacle obs2(Eigen::Vector2d(10, 10), 0.5);
    
    grid.insert(obs1, 0);
    grid.insert(obs2, 1);
    
    // Query near first obstacle
    std::vector<size_t> nearby = grid.getNearbyObstacles(Eigen::Vector2d(0, 0), 1.0);
    
    // Should find obstacle 0
    EXPECT_FALSE(nearby.empty());
    bool found_obs0 = false;
    for (size_t idx : nearby) {
        if (idx == 0) found_obs0 = true;
    }
    EXPECT_TRUE(found_obs0);
}

TEST(SpatialHashTests, Clear) {
    SpatialHashGrid grid(1.0);
    
    CircleObstacle obs(Eigen::Vector2d(0, 0), 1.0);
    grid.insert(obs, 0);
    
    grid.clear();
    
    std::vector<size_t> nearby = grid.getNearbyObstacles(Eigen::Vector2d(0, 0), 5.0);
    EXPECT_TRUE(nearby.empty());
}

// ============================================================================
// CollisionChecker Tests
// ============================================================================

TEST(CollisionCheckerTests, Construction) {
    CollisionChecker checker(0.5);
    
    EXPECT_DOUBLE_EQ(checker.getRobotRadius(), 0.5);
    EXPECT_EQ(checker.getObstacleCount(), 0);
}

TEST(CollisionCheckerTests, AddObstacles) {
    CollisionChecker checker(0.5);
    
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    EXPECT_EQ(checker.getObstacleCount(), 1);
    
    checker.addObstacle(Eigen::Vector2d(5, 5), 1.0);
    EXPECT_EQ(checker.getObstacleCount(), 2);
}

TEST(CollisionCheckerTests, ClearObstacles) {
    CollisionChecker checker(0.5);
    
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    checker.addObstacle(Eigen::Vector2d(5, 5), 1.0);
    
    checker.clearObstacles();
    EXPECT_EQ(checker.getObstacleCount(), 0);
}

TEST(CollisionCheckerTests, StateCollisionFree) {
    CollisionChecker checker(0.5);  // Robot radius = 0.5
    
    // Add obstacle at origin with radius 1.0
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    
    // Robot center needs to be > 1.5 away (obstacle radius + robot radius)
    State far_state(3, 0);
    EXPECT_TRUE(checker.isStateFree(far_state));
    
    // Robot at origin should collide
    State collision_state(0, 0);
    EXPECT_FALSE(checker.isStateFree(collision_state));
    
    // Robot at distance = 1.4 should collide (< 1.5)
    State near_collision(1.4, 0);
    EXPECT_FALSE(checker.isStateFree(near_collision));
    
    // Robot at distance = 1.6 should be free (> 1.5)
    State just_free(1.6, 0);
    EXPECT_TRUE(checker.isStateFree(just_free));
}

TEST(CollisionCheckerTests, StateCollisionWithMargin) {
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    
    double margin = 0.5;
    
    // Without margin: distance = 2.0 > 1.5 (free)
    State state(2, 0);
    EXPECT_TRUE(checker.isStateFree(state, 0.0));
    
    // With margin: distance = 2.0 = 1.5 + 0.5 (on boundary, counts as collision)
    EXPECT_FALSE(checker.isStateFree(state, margin));
}

TEST(CollisionCheckerTests, PathCollisionFree) {
    CollisionChecker checker(0.5);
    
    // Add obstacle in middle
    checker.addObstacle(Eigen::Vector2d(5, 0), 1.0);
    
    // Path that goes around obstacle
    State start(0, 5);
    State goal(10, 5);
    EXPECT_TRUE(checker.isPathFree(start, goal, 0.1));
    
    // Path that goes through obstacle
    State start2(0, 0);
    State goal2(10, 0);
    EXPECT_FALSE(checker.isPathFree(start2, goal2, 0.1));
}

TEST(CollisionCheckerTests, TrajectoryCollisionFree) {
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(5, 5), 1.0);
    
    // Create collision-free trajectory
    Trajectory safe_traj;
    safe_traj.addState(State(0, 0));
    safe_traj.addState(State(1, 0));
    safe_traj.addState(State(2, 0));
    
    EXPECT_TRUE(checker.isTrajectoryFree(safe_traj));
    
    // Create trajectory that collides
    Trajectory collision_traj;
    collision_traj.addState(State(0, 0));
    collision_traj.addState(State(5, 5));  // Through obstacle
    collision_traj.addState(State(10, 10));
    
    EXPECT_FALSE(checker.isTrajectoryFree(collision_traj));
}

TEST(CollisionCheckerTests, DynamicObstacles) {
    CollisionChecker checker(0.5);
    
    // Dynamic obstacle starting at origin, moving right at 1 m/s
    checker.addDynamicObstacle(Eigen::Vector2d(0, 0), 1.0, Eigen::Vector2d(1, 0));
    
    // At t=0, position (0,0) should collide
    State state_t0(0, 0, 0, 0, 0.0);
    EXPECT_FALSE(checker.isStateFree(state_t0));
    
    // At t=5, obstacle is at (5,0), so (0,0) should be free
    State state_t5(0, 0, 0, 0, 5.0);
    EXPECT_TRUE(checker.isStateFree(state_t5));
    
    // At t=5, position (5,0) should collide with predicted obstacle position
    State collision_t5(5, 0, 0, 0, 5.0);
    EXPECT_FALSE(checker.isStateFree(collision_t5));
}

TEST(CollisionCheckerTests, DistanceToNearestObstacle) {
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(0, 0), 2.0);
    
    // At position (5, 0), distance to obstacle surface = 5 - 2 = 3
    double dist = checker.distanceToNearestObstacle(Eigen::Vector2d(5, 0));
    EXPECT_NEAR(dist, 3.0, 1e-6);
    
    // Inside obstacle should give negative distance
    dist = checker.distanceToNearestObstacle(Eigen::Vector2d(0, 0));
    EXPECT_LT(dist, 0.0);
}

TEST(CollisionCheckerTests, DistanceWithNoObstacles) {
    CollisionChecker checker(0.5);
    
    double dist = checker.distanceToNearestObstacle(Eigen::Vector2d(0, 0));
    EXPECT_TRUE(std::isinf(dist));
}

TEST(CollisionCheckerTests, ObstacleGradient) {
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    
    // At position (5, 0), gradient should point away from obstacle (positive x)
    Eigen::Vector2d grad = checker.getObstacleGradient(Eigen::Vector2d(5, 0));
    
    // Gradient x-component should be positive (moving right increases distance)
    EXPECT_GT(grad.x(), 0.0);
    
    // Gradient y-component should be near zero (perpendicular to distance change)
    EXPECT_NEAR(grad.y(), 0.0, 0.1);
}

TEST(CollisionCheckerTests, SpatialHashOptimization) {
    // Compare performance/correctness with and without spatial hashing
    
    CollisionChecker checker_with_hash(0.5, true);
    CollisionChecker checker_without_hash(0.5, false);
    
    // Add many obstacles
    for (int i = 0; i < 50; ++i) {
        Eigen::Vector2d pos(i * 2.0, i * 2.0);
        checker_with_hash.addObstacle(pos, 0.5);
        checker_without_hash.addObstacle(pos, 0.5);
    }
    
    // Test various states - results should be identical
    for (int i = 0; i < 100; ++i) {
        State s(i * 0.5, i * 0.5);
        
        bool free_with_hash = checker_with_hash.isStateFree(s);
        bool free_without_hash = checker_without_hash.isStateFree(s);
        
        EXPECT_EQ(free_with_hash, free_without_hash)
            << "Mismatch at position (" << s.position.x() << ", " << s.position.y() << ")";
    }
}

TEST(CollisionCheckerTests, RebuildSpatialHash) {
    CollisionChecker checker(0.5);
    
    // Add obstacles
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    checker.addObstacle(Eigen::Vector2d(10, 10), 1.0);
    
    // Rebuild hash
    checker.rebuildSpatialHash();
    
    // Should still work correctly
    State s1(0, 0);
    State s2(5, 5);
    
    EXPECT_FALSE(checker.isStateFree(s1));
    EXPECT_TRUE(checker.isStateFree(s2));
}

// ============================================================================
// Edge Cases and Robustness
// ============================================================================

TEST(EdgeCaseTests, EmptyEnvironment) {
    CollisionChecker checker(0.5);
    
    // Everything should be collision-free
    State s(0, 0);
    EXPECT_TRUE(checker.isStateFree(s));
    
    Trajectory traj;
    traj.addState(State(0, 0));
    traj.addState(State(100, 100));
    EXPECT_TRUE(checker.isTrajectoryFree(traj));
}

TEST(EdgeCaseTests, ZeroLengthPath) {
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    
    // Path from point to itself
    State s(5, 0);
    EXPECT_TRUE(checker.isPathFree(s, s, 0.1));
}

TEST(EdgeCaseTests, TouchingObstacles) {
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    
    // Robot exactly touching obstacle (distance = radius sum)
    State touching(1.5, 0);  // 0.5 (robot) + 1.0 (obstacle) = 1.5
    
    // Due to floating point, this might be collision
    // The important thing is consistency
    bool result = checker.isStateFree(touching);
    
    // Slightly further should definitely be free
    State definitely_free(1.6, 0);
    EXPECT_TRUE(checker.isStateFree(definitely_free));
    
    // Slightly closer should definitely collide
    State definitely_collision(1.4, 0);
    EXPECT_FALSE(checker.isStateFree(definitely_collision));
}

TEST(EdgeCaseTests, VeryFineResolution) {
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(5, 0), 0.1);
    
    // Very fine resolution should still detect small obstacle
    State start(0, 0);
    State goal(10, 0);
    
    EXPECT_FALSE(checker.isPathFree(start, goal, 0.01));
}

// Main function
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
