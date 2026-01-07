# Getting Started with FastPlanner

This guide will walk you through building, testing, and using FastPlanner for the first time.

## Prerequisites

### Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgtest-dev \
    libbenchmark-dev
```

### macOS

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake eigen googletest google-benchmark
```

### Windows (WSL2 recommended)

We recommend using Windows Subsystem for Linux (WSL2) with Ubuntu, then follow the Linux instructions above.

## Building the Library

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/fastplanner.git
cd fastplanner
```

### 2. Create Build Directory

```bash
mkdir build
cd build
```

### 3. Configure with CMake

```bash
# Release build (optimized)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Or Debug build (for development)
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

### 4. Compile

```bash
# Use all available cores
make -j$(nproc)  # Linux
make -j$(sysctl -n hw.ncpu)  # macOS
```

Expected output:
```
[  5%] Building CXX object src/core/CMakeFiles/fastplanner.dir/state.cpp.o
[ 10%] Building CXX object src/core/CMakeFiles/fastplanner.dir/collision_checker.cpp.o
...
[100%] Built target fastplanner
```

## Running Tests

### Run All Tests

```bash
# In build directory
ctest --output-on-failure

# Or directly
./tests/fastplanner_tests
```

Expected output:
```
[==========] Running 50+ tests from 10 test suites.
[----------] Global test environment set-up.
[----------] 5 tests from StateTests
[ RUN      ] StateTests.DefaultConstruction
[       OK ] StateTests.DefaultConstruction (0 ms)
...
[==========] 50+ tests from 10 test suites ran. (X ms total)
[  PASSED  ] 50+ tests.
```

### Run Specific Test Suites

```bash
# Only state tests
./tests/fastplanner_tests --gtest_filter="StateTests.*"

# Only collision checker tests
./tests/fastplanner_tests --gtest_filter="CollisionCheckerTests.*"
```

## Running Examples

### Example 1: State and Trajectory Demo

```bash
./examples/demo_state_trajectory
```

This demonstrates:
- Creating and manipulating states
- Building trajectories
- Computing metrics (length, curvature, cost)
- Interpolation and downsampling

### Example 2: Collision Checking Demo

```bash
./examples/demo_collision_checking
```

This demonstrates:
- Adding static and dynamic obstacles
- Checking state and path collisions
- Computing distance fields
- Performance with spatial hashing

## Running Benchmarks

If Google Benchmark is installed:

```bash
# Run all benchmarks
./benchmarks/fastplanner_benchmarks

# Run specific benchmark
./benchmarks/fastplanner_benchmarks --benchmark_filter="BM_StateCollision"

# Save results to JSON
./benchmarks/fastplanner_benchmarks --benchmark_format=json > results.json
```

## Your First Program

Create a file `my_first_planner.cpp`:

```cpp
#include "fastplanner/core/state.hpp"
#include "fastplanner/core/collision_checker.hpp"
#include <iostream>

using namespace fastplanner;

int main() {
    // Create collision checker
    CollisionChecker checker(0.5);  // robot radius = 0.5m
    
    // Add some obstacles
    checker.addObstacle(Eigen::Vector2d(5, 0), 1.0);
    checker.addObstacle(Eigen::Vector2d(10, 5), 1.5);
    
    std::cout << "Environment has " << checker.getObstacleCount() 
              << " obstacles\n";
    
    // Create start and goal states
    State start(0, 0);
    State goal(15, 10);
    
    std::cout << "Distance: " << start.distanceTo(goal) << " m\n";
    
    // Check if straight path is collision-free
    bool path_free = checker.isPathFree(start, goal, 0.1);
    std::cout << "Straight path is " 
              << (path_free ? "collision-free" : "blocked") << "\n";
    
    // Build a simple trajectory
    Trajectory traj;
    traj.addState(start);
    traj.addState(State(5, 5));
    traj.addState(State(10, 8));
    traj.addState(goal);
    
    std::cout << "\nTrajectory metrics:\n";
    std::cout << "  Length: " << traj.getLength() << " m\n";
    std::cout << "  Curvature: " << traj.getCurvature() << "\n";
    std::cout << "  Valid: " 
              << (checker.isTrajectoryFree(traj) ? "YES" : "NO") << "\n";
    
    return 0;
}
```

### Compile and Run

```bash
# In your project directory
g++ -std=c++17 -O3 \
    -I/path/to/fastplanner/include \
    -I/usr/include/eigen3 \
    my_first_planner.cpp \
    -L/path/to/fastplanner/build \
    -lfastplanner \
    -o my_first_planner

# Run it
./my_first_planner
```

Or add to CMakeLists.txt:

```cmake
add_executable(my_first_planner my_first_planner.cpp)
target_link_libraries(my_first_planner PRIVATE fastplanner)
```

## Using FastPlanner in Your Project

### Option 1: Install System-Wide

```bash
cd build
sudo make install
```

Then in your CMakeLists.txt:

```cmake
find_package(FastPlanner REQUIRED)
target_link_libraries(your_target PRIVATE FastPlanner::fastplanner)
```

### Option 2: Add as Subdirectory

```cmake
add_subdirectory(external/fastplanner)
target_link_libraries(your_target PRIVATE fastplanner)
```

### Option 3: Use with FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(
  fastplanner
  GIT_REPOSITORY https://github.com/yourusername/fastplanner.git
  GIT_TAG main
)
FetchContent_MakeAvailable(fastplanner)

target_link_libraries(your_target PRIVATE fastplanner)
```

## Next Steps

Now that you have FastPlanner running:

1. **Explore the API**: Read the header files in `include/fastplanner/`
2. **Study the Examples**: Check out `examples/` for more usage patterns
3. **Read the Tests**: `tests/` contains many usage examples
4. **Try the Benchmarks**: See performance characteristics
5. **Wait for Phase 2**: RRT and planning algorithms coming soon!

## Troubleshooting

### CMake can't find Eigen3

```bash
# Specify Eigen3 location manually
cmake -DEigen3_DIR=/usr/share/eigen3/cmake ..
```

### Tests fail to compile

Make sure Google Test is installed:

```bash
# Ubuntu
sudo apt install libgtest-dev

# macOS
brew install googletest
```

### Linker errors

Make sure you're linking against the fastplanner library:

```cmake
target_link_libraries(your_target PRIVATE fastplanner)
```

And that the library path is in your `LD_LIBRARY_PATH` (Linux) or `DYLD_LIBRARY_PATH` (macOS):

```bash
export LD_LIBRARY_PATH=/path/to/fastplanner/build:$LD_LIBRARY_PATH
```

## Getting Help

- **Documentation**: Check `docs/` directory
- **Examples**: See `examples/` for working code
- **Issues**: Open an issue on GitHub
- **Email**: your.email@berkeley.edu

## Common Workflows

### Development Workflow

```bash
# Make changes to code
vim include/fastplanner/core/state.hpp

# Rebuild
cd build
make -j$(nproc)

# Test changes
./tests/fastplanner_tests --gtest_filter="StateTests.*"

# Run examples to verify
./examples/demo_state_trajectory
```

### Performance Testing

```bash
# Build in Release mode
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run benchmarks
./benchmarks/fastplanner_benchmarks

# Compare results
./benchmarks/fastplanner_benchmarks --benchmark_filter="BM_StateCollision" \
    --benchmark_repetitions=10
```

### Debugging

```bash
# Build in Debug mode
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)

# Run with debugger
gdb ./examples/demo_collision_checking

# Or with valgrind
valgrind --leak-check=full ./tests/fastplanner_tests
```

---

**Happy Planning! 🚀**

