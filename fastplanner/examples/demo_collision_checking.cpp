/**
 * @file demo_collision_checking.cpp
 * @brief Demonstration of collision checking functionality
 * 
 * This example shows:
 * - Creating obstacles (static and dynamic)
 * - Checking state and path collisions
 * - Computing distance to obstacles
 * - Using obstacle gradients for optimization
 * - Performance with spatial hashing
 */

#include "fastplanner/core/collision_checker.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace fastplanner;

void printObstacle(const CircleObstacle& obs, int idx) {
    std::cout << "  Obstacle " << idx << ": "
              << "center=(" << obs.center.transpose() << "), "
              << "radius=" << obs.radius << ", "
              << (obs.is_dynamic ? "dynamic" : "static");
    if (obs.is_dynamic) {
        std::cout << ", vel=(" << obs.velocity.transpose() << ")";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "=== FastPlanner Collision Checking Demo ===\n\n";
    
    // ========================================================================
    // Part 1: Creating a collision checker and adding obstacles
    // ========================================================================
    
    std::cout << "1. Setting Up Environment\n";
    std::cout << "-------------------------\n";
    
    CollisionChecker checker(0.5);  // Robot radius = 0.5m
    std::cout << "Robot radius: " << checker.getRobotRadius() << " m\n\n";
    
    // Add static obstacles
    checker.addObstacle(Eigen::Vector2d(5, 0), 1.0);
    checker.addObstacle(Eigen::Vector2d(10, 5), 1.5);
    checker.addObstacle(Eigen::Vector2d(15, -3), 0.8);
    
    // Add dynamic obstacles
    checker.addDynamicObstacle(Eigen::Vector2d(0, 5), 1.0, Eigen::Vector2d(1, 0));
    checker.addDynamicObstacle(Eigen::Vector2d(20, 0), 1.2, Eigen::Vector2d(-0.5, 0.5));
    
    std::cout << "Created environment with " << checker.getObstacleCount() << " obstacles:\n";
    for (size_t i = 0; i < checker.getObstacles().size(); ++i) {
        printObstacle(checker.getObstacles()[i], i);
    }
    
    // ========================================================================
    // Part 2: State collision checking
    // ========================================================================
    
    std::cout << "\n2. State Collision Checking\n";
    std::cout << "---------------------------\n";
    
    std::vector<State> test_states = {
        State(0, 0),      // Far from obstacles
        State(5, 0),      // At obstacle center
        State(6, 0),      // Near obstacle
        State(3, 0),      // Between robot and obstacle
        State(0, 5, 0, 0, 0),    // At dynamic obstacle (t=0)
        State(5, 5, 0, 0, 5),    // Where dynamic obstacle will be (t=5)
    };
    
    for (size_t i = 0; i < test_states.size(); ++i) {
        const State& s = test_states[i];
        bool is_free = checker.isStateFree(s);
        double dist = checker.distanceToNearestObstacle(s.position, s.time);
        
        std::cout << "State " << i << " at (" << s.position.transpose() 
                  << "), t=" << s.time << ": "
                  << (is_free ? "FREE" : "COLLISION")
                  << " (distance to nearest: " << std::fixed << std::setprecision(2) 
                  << dist << " m)\n";
    }
    
    // ========================================================================
    // Part 3: Path collision checking
    // ========================================================================
    
    std::cout << "\n3. Path Collision Checking\n";
    std::cout << "--------------------------\n";
    
    struct PathTest {
        State start;
        State goal;
        std::string description;
    };
    
    std::vector<PathTest> paths = {
        {State(0, 0), State(20, 0), "Straight through obstacles"},
        {State(0, 0), State(0, 10), "Clear vertical path"},
        {State(0, -5), State(20, -5), "Clear horizontal path below"},
        {State(4, 0), State(6, 0), "Short path near obstacle"},
    };
    
    for (const auto& path : paths) {
        bool is_free = checker.isPathFree(path.start, path.goal, 0.1);
        double length = path.start.distanceTo(path.goal);
        
        std::cout << path.description << ": "
                  << (is_free ? "FREE" : "COLLISION")
                  << " (length=" << std::fixed << std::setprecision(2) << length << " m)\n";
    }
    
    // ========================================================================
    // Part 4: Trajectory validation
    // ========================================================================
    
    std::cout << "\n4. Trajectory Validation\n";
    std::cout << "------------------------\n";
    
    // Safe trajectory
    Trajectory safe_traj;
    for (double y = 0; y <= 10; y += 1.0) {
        safe_traj.addState(State(0, y));
    }
    
    // Collision trajectory
    Trajectory collision_traj;
    for (double x = 0; x <= 20; x += 2.0) {
        collision_traj.addState(State(x, 0));
    }
    
    std::cout << "Safe trajectory (vertical): "
              << (checker.isTrajectoryFree(safe_traj) ? "VALID" : "INVALID") << "\n";
    std::cout << "Collision trajectory (horizontal): "
              << (checker.isTrajectoryFree(collision_traj) ? "VALID" : "INVALID") << "\n";
    
    // ========================================================================
    // Part 5: Distance field and gradients
    // ========================================================================
    
    std::cout << "\n5. Distance Field and Gradients\n";
    std::cout << "-------------------------------\n";
    
    std::vector<Eigen::Vector2d> query_points = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(5, 5),
        Eigen::Vector2d(10, 0),
        Eigen::Vector2d(8, 0),
    };
    
    for (const auto& point : query_points) {
        double dist = checker.distanceToNearestObstacle(point);
        Eigen::Vector2d grad = checker.getObstacleGradient(point);
        
        std::cout << "Point (" << point.transpose() << "):\n";
        std::cout << "  Distance: " << std::fixed << std::setprecision(3) << dist << " m\n";
        std::cout << "  Gradient: (" << grad.transpose() << ")\n";
        std::cout << "  Gradient magnitude: " << grad.norm() << "\n";
    }
    
    // ========================================================================
    // Part 6: Dynamic obstacle prediction
    // ========================================================================
    
    std::cout << "\n6. Dynamic Obstacle Prediction\n";
    std::cout << "------------------------------\n";
    
    // The first dynamic obstacle starts at (0, 5) and moves at (1, 0) m/s
    std::cout << "Dynamic obstacle trajectory:\n";
    for (double t = 0; t <= 10; t += 2.0) {
        Eigen::Vector2d predicted = checker.getObstacles()[3].predictPosition(t);
        State test_state(predicted.x(), predicted.y(), 0, 0, t);
        
        std::cout << "  t=" << t << "s: position=(" << predicted.transpose() 
                  << "), collision=" << (!checker.isStateFree(test_state) ? "YES" : "NO")
                  << "\n";
    }
    
    // ========================================================================
    // Part 7: Safety margins
    // ========================================================================
    
    std::cout << "\n7. Safety Margins\n";
    std::cout << "-----------------\n";
    
    State test_state(6.5, 0);  // Near obstacle at (5, 0) with radius 1.0
    
    std::cout << "State at (" << test_state.position.transpose() << "):\n";
    for (double margin = 0.0; margin <= 1.0; margin += 0.2) {
        bool is_free = checker.isStateFree(test_state, margin);
        std::cout << "  Margin=" << std::fixed << std::setprecision(1) << margin 
                  << "m: " << (is_free ? "FREE" : "COLLISION") << "\n";
    }
    
    // ========================================================================
    // Part 8: Performance comparison
    // ========================================================================
    
    std::cout << "\n8. Performance Comparison\n";
    std::cout << "-------------------------\n";
    
    // Create dense environment
    CollisionChecker checker_with_hash(0.5, true);
    CollisionChecker checker_without_hash(0.5, false);
    
    std::cout << "Adding 100 obstacles to environment...\n";
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector2d pos(i * 0.5, (i % 10) * 2.0);
        checker_with_hash.addObstacle(pos, 0.3);
        checker_without_hash.addObstacle(pos, 0.3);
    }
    
    const int num_queries = 10000;
    std::vector<State> query_states;
    for (int i = 0; i < num_queries; ++i) {
        query_states.emplace_back(i * 0.01, (i % 100) * 0.1);
    }
    
    // Benchmark with spatial hashing
    auto start_with = std::chrono::high_resolution_clock::now();
    int free_count_with = 0;
    for (const auto& s : query_states) {
        if (checker_with_hash.isStateFree(s)) free_count_with++;
    }
    auto end_with = std::chrono::high_resolution_clock::now();
    auto duration_with = std::chrono::duration_cast<std::chrono::microseconds>(end_with - start_with);
    
    // Benchmark without spatial hashing
    auto start_without = std::chrono::high_resolution_clock::now();
    int free_count_without = 0;
    for (const auto& s : query_states) {
        if (checker_without_hash.isStateFree(s)) free_count_without++;
    }
    auto end_without = std::chrono::high_resolution_clock::now();
    auto duration_without = std::chrono::duration_cast<std::chrono::microseconds>(end_without - start_without);
    
    std::cout << "\n" << num_queries << " collision queries:\n";
    std::cout << "  With spatial hashing:    " << duration_with.count() / 1000.0 << " ms ("
              << num_queries * 1000000.0 / duration_with.count() << " queries/sec)\n";
    std::cout << "  Without spatial hashing: " << duration_without.count() / 1000.0 << " ms ("
              << num_queries * 1000000.0 / duration_without.count() << " queries/sec)\n";
    std::cout << "  Speedup: " << std::fixed << std::setprecision(1) 
              << static_cast<double>(duration_without.count()) / duration_with.count() << "x\n";
    
    // Sanity check: results should match
    std::cout << "\nResults match: " << (free_count_with == free_count_without ? "YES" : "NO") << "\n";
    
    std::cout << "\n=== Demo Complete ===\n";
    
    return 0;
}

