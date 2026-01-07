/**
 * @file benchmark_collision_checking.cpp
 * @brief Performance benchmarks for collision checking
 * 
 * Benchmarks:
 * - State collision checking with varying obstacle counts
 * - Path collision checking with different resolutions
 * - Spatial hashing vs brute force
 * - Dynamic obstacle prediction
 */

#include <benchmark/benchmark.h>
#include "fastplanner/core/collision_checker.hpp"

using namespace fastplanner;

// ============================================================================
// Benchmark: State collision checking
// ============================================================================

static void BM_StateCollisionChecking(benchmark::State& state) {
    int num_obstacles = state.range(0);
    bool use_spatial_hash = state.range(1);
    
    CollisionChecker checker(0.5, use_spatial_hash);
    
    // Create obstacles in grid
    for (int i = 0; i < num_obstacles; ++i) {
        double x = (i % 10) * 2.0;
        double y = (i / 10) * 2.0;
        checker.addObstacle(Eigen::Vector2d(x, y), 0.5);
    }
    
    // Query states
    std::vector<State> queries;
    for (int i = 0; i < 1000; ++i) {
        queries.emplace_back(i * 0.05, (i % 20) * 0.5);
    }
    
    for (auto _ : state) {
        for (const auto& s : queries) {
            benchmark::DoNotOptimize(checker.isStateFree(s));
        }
    }
    
    state.SetItemsProcessed(state.iterations() * queries.size());
}

BENCHMARK(BM_StateCollisionChecking)
    ->Args({10, false})   // 10 obstacles, no spatial hash
    ->Args({10, true})    // 10 obstacles, with spatial hash
    ->Args({50, false})
    ->Args({50, true})
    ->Args({100, false})
    ->Args({100, true})
    ->Args({200, false})
    ->Args({200, true})
    ->Unit(benchmark::kMillisecond);

// ============================================================================
// Benchmark: Path collision checking
// ============================================================================

static void BM_PathCollisionChecking(benchmark::State& state) {
    double resolution = state.range(0) / 100.0;  // Convert to meters
    
    CollisionChecker checker(0.5);
    
    // Add some obstacles
    for (int i = 0; i < 20; ++i) {
        checker.addObstacle(Eigen::Vector2d(i * 2.0, i % 5), 0.5);
    }
    
    State start(0, 0);
    State goal(50, 10);
    
    for (auto _ : state) {
        benchmark::DoNotOptimize(checker.isPathFree(start, goal, resolution));
    }
}

BENCHMARK(BM_PathCollisionChecking)
    ->Arg(5)     // 0.05m resolution
    ->Arg(10)    // 0.1m
    ->Arg(20)    // 0.2m
    ->Arg(50)    // 0.5m
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// Benchmark: Distance queries
// ============================================================================

static void BM_DistanceToNearestObstacle(benchmark::State& state) {
    int num_obstacles = state.range(0);
    
    CollisionChecker checker(0.5);
    
    for (int i = 0; i < num_obstacles; ++i) {
        checker.addObstacle(Eigen::Vector2d(i * 1.5, (i % 10) * 1.5), 0.5);
    }
    
    std::vector<Eigen::Vector2d> queries;
    for (int i = 0; i < 100; ++i) {
        queries.emplace_back(i * 0.3, (i % 10) * 0.3);
    }
    
    for (auto _ : state) {
        for (const auto& q : queries) {
            benchmark::DoNotOptimize(checker.distanceToNearestObstacle(q));
        }
    }
    
    state.SetItemsProcessed(state.iterations() * queries.size());
}

BENCHMARK(BM_DistanceToNearestObstacle)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// Benchmark: Gradient computation
// ============================================================================

static void BM_ObstacleGradient(benchmark::State& state) {
    CollisionChecker checker(0.5);
    
    // Add some obstacles
    for (int i = 0; i < 20; ++i) {
        checker.addObstacle(Eigen::Vector2d(i * 2.0, 0), 0.5);
    }
    
    Eigen::Vector2d query(5.0, 0.0);
    
    for (auto _ : state) {
        benchmark::DoNotOptimize(checker.getObstacleGradient(query));
    }
}

BENCHMARK(BM_ObstacleGradient)->Unit(benchmark::kMicrosecond);

// ============================================================================
// Benchmark: Dynamic obstacles
// ============================================================================

static void BM_DynamicObstacleChecking(benchmark::State& state) {
    int num_dynamic = state.range(0);
    
    CollisionChecker checker(0.5);
    
    // Add dynamic obstacles with random velocities
    for (int i = 0; i < num_dynamic; ++i) {
        Eigen::Vector2d pos(i * 2.0, 0);
        Eigen::Vector2d vel(0.5, 0.2);
        checker.addDynamicObstacle(pos, 0.5, vel);
    }
    
    // Query at different times
    std::vector<State> queries;
    for (int i = 0; i < 100; ++i) {
        queries.emplace_back(i * 0.5, 0, 0, 0, i * 0.1);
    }
    
    for (auto _ : state) {
        for (const auto& s : queries) {
            benchmark::DoNotOptimize(checker.isStateFree(s));
        }
    }
    
    state.SetItemsProcessed(state.iterations() * queries.size());
}

BENCHMARK(BM_DynamicObstacleChecking)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// Benchmark: Trajectory validation
// ============================================================================

static void BM_TrajectoryValidation(benchmark::State& state) {
    int traj_length = state.range(0);
    
    CollisionChecker checker(0.5);
    
    // Add obstacles
    for (int i = 0; i < 30; ++i) {
        checker.addObstacle(Eigen::Vector2d(i * 2.0, 5), 0.5);
    }
    
    // Create trajectory
    Trajectory traj;
    for (int i = 0; i < traj_length; ++i) {
        traj.addState(State(i * 0.5, 0));  // Below obstacles
    }
    
    for (auto _ : state) {
        benchmark::DoNotOptimize(checker.isTrajectoryFree(traj));
    }
}

BENCHMARK(BM_TrajectoryValidation)
    ->Arg(10)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();

