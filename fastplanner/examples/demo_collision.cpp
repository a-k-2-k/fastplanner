/**
 * @file demo_collision.cpp
 * @brief Demonstration of collision checking functionality
 * 
 * This example shows how to:
 * - Create static and dynamic obstacles
 * - Check point and path collisions
 * - Use spatial hashing for performance
 * - Query nearest obstacles
 * - Validate entire trajectories
 */

#include "fastplanner/core/collision_checker.hpp"
#include "fastplanner/core/state.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace fastplanner;

void printSeparator(const std::string& title = "") {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    if (!title.empty()) {
        std::cout << title << std::endl;
        std::cout << std::string(70, '=') << std::endl;
    }
}

void printObstacle(const CircleObstacle& obs, const std::string& label) {
    std::cout << label << ": center=(" << obs.center.x() << ", " << obs.center.y() 
              << "), radius=" << obs.radius;
    if (obs.is_dynamic) {
        std::cout << ", velocity=(" << obs.velocity.x() << ", " << obs.velocity.y() << ")";
    }
    std::cout << std::endl;
}

int main() {
    std::cout << std::fixed << std::setprecision(3);
    
    printSeparator("FastPlanner Collision Checker Demo");
    
    // =============================================================================
    // Part 1: Basic Obstacle Creation
    // =============================================================================
    printSeparator("Part 1: Creating Obstacles");
    
    CircleObstacle static_obs(10.0, 10.0, 2.0);
    CircleObstacle dynamic_obs(20.0, 5.0, 1.5, 1.0, 0.5);
    
    printObstacle(static_obs, "Static obstacle");
    printObstacle(dynamic_obs, "Dynamic obstacle");
    
    std::cout << "\nDynamic obstacle predictions:" << std::endl;
    for (double t = 0.0; t <= 5.0; t += 1.0) {
        Eigen::Vector2d pred = dynamic_obs.predictPosition(t);
        std::cout << "  t=" << t << "s: position=(" << pred.x() << ", " << pred.y() << ")" << std::endl;
    }
    
    // =============================================================================
    // Part 2: CollisionChecker Setup
    // =============================================================================
    printSeparator("Part 2: Setting Up Collision Checker");
    
    CollisionChecker checker(100.0, 100.0, 5.0);
    std::cout << "Created checker: 100x100m world, 5m cell size" << std::endl;
    
    // Add obstacles
    checker.addObstacle(CircleObstacle(10.0, 10.0, 2.0));
    checker.addObstacle(CircleObstacle(30.0, 30.0, 3.0));
    checker.addObstacle(CircleObstacle(50.0, 20.0, 1.5));
    checker.addObstacle(CircleObstacle(70.0, 70.0, 2.5));
    checker.addObstacle(CircleObstacle(25.0, 5.0, 1.0, 0.5, 0.0)); // Dynamic
    
    std::cout << "Added " << checker.getObstacles().size() << " obstacles" << std::endl;
    
    // =============================================================================
    // Part 3: Point Collision Checking
    // =============================================================================
    printSeparator("Part 3: Point Collision Checking");
    
    std::vector<State> test_points = {
        State(10.0, 10.0),  // Inside first obstacle
        State(5.0, 5.0),    // Near first obstacle
        State(50.0, 50.0),  // Empty space
        State(30.0, 30.0),  // Inside second obstacle
        State(15.0, 15.0)   // Between obstacles
    };
    
    double robot_radius = 0.5;
    std::cout << "Testing collision with robot radius=" << robot_radius << "m:\n" << std::endl;
    
    for (size_t i = 0; i < test_points.size(); ++i) {
        const State& s = test_points[i];
        bool collision = checker.isInCollision(s, robot_radius);
        double distance = checker.getDistanceToNearestObstacle(s.position);
        
        std::cout << "Point " << i+1 << " at (" << s.position.x() << ", " << s.position.y() << "): "
                  << (collision ? "COLLISION" : "FREE")
                  << ", distance to nearest=" << distance << "m" << std::endl;
    }
    
    // =============================================================================
    // Part 4: Path Collision Checking
    // =============================================================================
    printSeparator("Part 4: Path Collision Checking");
    
    struct PathTest {
        State from;
        State to;
        std::string description;
    };
    
    std::vector<PathTest> paths = {
        {State(0.0, 0.0), State(5.0, 5.0), "Short path (free)"},
        {State(0.0, 0.0), State(20.0, 20.0), "Path through obstacle"},
        {State(40.0, 40.0), State(60.0, 60.0), "Diagonal path (free)"},
        {State(50.0, 15.0), State(50.0, 25.0), "Vertical path near obstacle"}
    };
    
    std::cout << "Testing paths with resolution=0.1m:\n" << std::endl;
    
    for (const auto& path : paths) {
        bool is_free = checker.isPathCollisionFree(path.from, path.to, robot_radius, 0.1);
        std::cout << path.description << ": " << (is_free ? "FREE" : "BLOCKED") << std::endl;
    }
    
    // =============================================================================
    // Part 5: Trajectory Validation
    // =============================================================================
    printSeparator("Part 5: Trajectory Validation");
    
    // Create a safe trajectory
    Trajectory safe_traj;
    safe_traj.addState(State(0.0, 0.0));
    safe_traj.addState(State(5.0, 0.0));
    safe_traj.addState(State(10.0, 0.0));
    safe_traj.addState(State(15.0, 0.0));
    
    // Create an unsafe trajectory (passes through obstacle)
    Trajectory unsafe_traj;
    unsafe_traj.addState(State(5.0, 5.0));
    unsafe_traj.addState(State(10.0, 10.0));
    unsafe_traj.addState(State(15.0, 15.0));
    
    std::cout << "Safe trajectory (along y=0): " 
              << (checker.isTrajectoryCollisionFree(safe_traj, robot_radius) ? "VALID" : "INVALID")
              << std::endl;
    
    std::cout << "Unsafe trajectory (through obstacles): " 
              << (checker.isTrajectoryCollisionFree(unsafe_traj, robot_radius) ? "VALID" : "INVALID")
              << std::endl;
    
    // =============================================================================
    // Part 6: Nearest Obstacle Queries
    // =============================================================================
    printSeparator("Part 6: Nearest Obstacle Queries");
    
    std::vector<Eigen::Vector2d> query_points = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(25.0, 25.0),
        Eigen::Vector2d(60.0, 60.0),
        Eigen::Vector2d(90.0, 90.0)
    };
    
    std::cout << "Finding nearest obstacles:\n" << std::endl;
    
    for (const auto& point : query_points) {
        const CircleObstacle* nearest = checker.getNearestObstacle(point);
        double distance = checker.getDistanceToNearestObstacle(point);
        
        std::cout << "Query at (" << point.x() << ", " << point.y() << "):" << std::endl;
        if (nearest != nullptr) {
            std::cout << "  Nearest obstacle: center=(" << nearest->center.x() << ", " 
                      << nearest->center.y() << "), radius=" << nearest->radius << std::endl;
            std::cout << "  Distance to surface: " << distance << "m" << std::endl;
        } else {
            std::cout << "  No obstacles found" << std::endl;
        }
    }
    
    // =============================================================================
    // Part 7: Regional Queries
    // =============================================================================
    printSeparator("Part 7: Regional Queries");
    
    Eigen::Vector2d region_center(30.0, 20.0);
    double region_radius = 25.0;
    
    std::cout << "Finding obstacles within " << region_radius << "m of (" 
              << region_center.x() << ", " << region_center.y() << "):" << std::endl;
    
    auto nearby = checker.getObstaclesInRegion(region_center, region_radius);
    std::cout << "Found " << nearby.size() << " obstacles in region:" << std::endl;
    
    for (size_t i = 0; i < nearby.size(); ++i) {
        printObstacle(nearby[i], "  Obstacle " + std::to_string(i+1));
    }
    
    // =============================================================================
    // Part 8: Dynamic Obstacle Collision
    // =============================================================================
    printSeparator("Part 8: Dynamic Obstacle Collision");
    
    CollisionChecker dynamic_checker(100.0, 100.0, 5.0);
    dynamic_checker.addObstacle(CircleObstacle(10.0, 10.0, 2.0, 2.0, 0.0)); // Moving right
    
    State test_point(20.0, 10.0);
    
    std::cout << "Dynamic obstacle moving at 2 m/s to the right" << std::endl;
    std::cout << "Test point at (20, 10):\n" << std::endl;
    
    for (double t = 0.0; t <= 6.0; t += 1.0) {
        bool collision = dynamic_checker.isInCollision(test_point.position, t, robot_radius);
        std::cout << "  t=" << t << "s: " << (collision ? "COLLISION" : "FREE") << std::endl;
    }
    
    // =============================================================================
    // Part 9: Spatial Hash Performance
    // =============================================================================
    printSeparator("Part 9: Spatial Hash Statistics");
    
    CollisionChecker perf_checker(100.0, 100.0, 2.0);
    
    // Add many obstacles
    std::cout << "Adding 100 random obstacles..." << std::endl;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist_pos(0.0, 100.0);
    std::uniform_real_distribution<double> dist_radius(0.5, 2.0);
    
    for (int i = 0; i < 100; ++i) {
        perf_checker.addObstacle(CircleObstacle(
            dist_pos(rng), dist_pos(rng), dist_radius(rng)
        ));
    }
    
    double avg_per_cell;
    size_t max_per_cell, num_occupied;
    perf_checker.getSpatialHashStats(avg_per_cell, max_per_cell, num_occupied);
    
    std::cout << "\nSpatial hash statistics:" << std::endl;
    std::cout << "  Occupied cells: " << num_occupied << std::endl;
    std::cout << "  Avg obstacles per cell: " << avg_per_cell << std::endl;
    std::cout << "  Max obstacles per cell: " << max_per_cell << std::endl;
    
    // Benchmark collision checks
    std::cout << "\nPerformance test (10000 collision checks):" << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    int num_checks = 10000;
    int num_collisions = 0;
    
    for (int i = 0; i < num_checks; ++i) {
        State random_state(dist_pos(rng), dist_pos(rng));
        if (perf_checker.isInCollision(random_state, robot_radius)) {
            num_collisions++;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "  Total time: " << duration.count() / 1000.0 << " ms" << std::endl;
    std::cout << "  Average per check: " << duration.count() / static_cast<double>(num_checks) 
              << " μs" << std::endl;
    std::cout << "  Collisions detected: " << num_collisions << " / " << num_checks 
              << " (" << (100.0 * num_collisions / num_checks) << "%)" << std::endl;
    
    printSeparator();
    std::cout << "\n✓ Collision checker demo complete!\n" << std::endl;
    std::cout << "Key takeaways:" << std::endl;
    std::cout << "  - Spatial hashing enables O(1) collision queries" << std::endl;
    std::cout << "  - Dynamic obstacles use constant velocity prediction" << std::endl;
    std::cout << "  - Path checking uses configurable resolution" << std::endl;
    std::cout << "  - Performance scales well with many obstacles\n" << std::endl;
    
    return 0;
}

