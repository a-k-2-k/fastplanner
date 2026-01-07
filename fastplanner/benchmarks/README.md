# Benchmarks

Performance benchmarks for FastPlanner algorithms.

## Status: 🚧 Phase 3 (Planned)

Benchmarks will be implemented in Phase 3 after core planners are complete.

## Planned Benchmarks

### Algorithm Comparisons
- `benchmark_rrt.cpp`: RRT performance across scenarios
- `benchmark_rrt_star.cpp`: RRT* convergence analysis
- `benchmark_hybrid_astar.cpp`: Hybrid A* grid resolution impact
- `benchmark_comparison.cpp`: Head-to-head algorithm comparison

### Performance Tests
- `benchmark_collision.cpp`: Collision checking with varying obstacle counts
- `benchmark_nearest_neighbor.cpp`: KD-tree vs linear search
- `benchmark_spatial_hash.cpp`: Spatial hashing cell size optimization

## Benchmark Scenarios

### 1. Empty Space
- **Description**: Direct path with no obstacles
- **Metrics**: Planning time, path length
- **Expected**: All algorithms fast, RRT* slightly longer

### 2. Dense Clutter
- **Description**: 50+ random obstacles
- **Metrics**: Success rate, nodes explored, collision checks
- **Expected**: Spatial hashing advantage clear

### 3. Narrow Passage
- **Description**: 1m wide corridor between walls
- **Metrics**: Success rate, planning time
- **Expected**: RRT struggles, Hybrid A* excels

### 4. Long Distance
- **Description**: 50m+ planning distance
- **Metrics**: Path quality, memory usage
- **Expected**: RRT* finds better paths over time

### 5. Dynamic Obstacles
- **Description**: Moving obstacles with prediction
- **Metrics**: Replanning frequency, trajectory safety
- **Expected**: Tests dynamic collision checking

## Usage (Future)

```bash
cd build
./benchmarks/benchmark_comparison

# Run specific benchmark
./benchmarks/benchmark_collision --benchmark_filter="DenseClutter"

# Output to JSON
./benchmarks/benchmark_comparison --benchmark_format=json --benchmark_out=results.json
```

## Expected Output Format

```
--------------------------------------------------------------------
Benchmark                          Time             CPU   Iterations
--------------------------------------------------------------------
BM_RRT_EmptySpace               0.245 ms        0.244 ms         2856
BM_RRT_DenseClutter             12.3 ms         12.2 ms            57
BM_RRTStar_EmptySpace           0.891 ms        0.889 ms          787
BM_HybridAStar_EmptySpace       0.156 ms        0.155 ms         4498
```

## Benchmark Guidelines

1. **Reproducibility**: Use fixed random seeds
2. **Warmup**: Run iterations before timing
3. **Statistics**: Report mean, stddev, min, max
4. **Comparison**: Relative performance vs baseline
5. **Documentation**: Explain scenario setup

## Coming Soon

Check back after Phase 2 completion for implemented benchmarks!

