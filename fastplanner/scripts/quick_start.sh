#!/bin/bash
# Quick start script for FastPlanner
# Builds the project and runs tests and examples

set -e  # Exit on error

echo "======================================"
echo "FastPlanner Quick Start"
echo "======================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo -e "${RED}Error: CMakeLists.txt not found${NC}"
    echo "Please run this script from the fastplanner root directory"
    exit 1
fi

# Step 1: Create build directory
echo -e "${BLUE}[1/5] Creating build directory...${NC}"
mkdir -p build
cd build

# Step 2: Configure with CMake
echo -e "${BLUE}[2/5] Configuring with CMake...${NC}"
cmake -DCMAKE_BUILD_TYPE=Release ..

# Step 3: Build
echo -e "${BLUE}[3/5] Building FastPlanner...${NC}"
if [ "$(uname)" == "Darwin" ]; then
    # macOS
    make -j$(sysctl -n hw.ncpu)
else
    # Linux
    make -j$(nproc)
fi

echo ""
echo -e "${GREEN}✓ Build successful!${NC}"
echo ""

# Step 4: Run tests
echo -e "${BLUE}[4/5] Running tests...${NC}"
if [ -f "tests/fastplanner_tests" ]; then
    ./tests/fastplanner_tests
    echo ""
    echo -e "${GREEN}✓ All tests passed!${NC}"
else
    echo -e "${RED}Warning: Tests not built (GTest may not be installed)${NC}"
fi
echo ""

# Step 5: Run examples
echo -e "${BLUE}[5/5] Running examples...${NC}"
echo ""

if [ -f "examples/demo_state_trajectory" ]; then
    echo "-----------------------------------"
    echo "Running State & Trajectory Demo..."
    echo "-----------------------------------"
    ./examples/demo_state_trajectory
    echo ""
    echo -e "${GREEN}✓ State demo completed!${NC}"
    echo ""
else
    echo -e "${RED}Warning: State demo not built${NC}"
fi

if [ -f "examples/demo_collision_checking" ]; then
    echo "-----------------------------------"
    echo "Running Collision Checking Demo..."
    echo "-----------------------------------"
    ./examples/demo_collision_checking
    echo ""
    echo -e "${GREEN}✓ Collision demo completed!${NC}"
    echo ""
else
    echo -e "${RED}Warning: Collision demo not built${NC}"
fi

# Optional: Run benchmarks
if [ -f "benchmarks/fastplanner_benchmarks" ]; then
    echo ""
    echo -e "${BLUE}Optional: Run benchmarks? (y/N)${NC}"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        echo ""
        echo "-----------------------------------"
        echo "Running Benchmarks..."
        echo "-----------------------------------"
        ./benchmarks/fastplanner_benchmarks --benchmark_filter="BM_StateCollision" --benchmark_repetitions=3
    fi
fi

echo ""
echo "======================================"
echo -e "${GREEN}FastPlanner Quick Start Complete!${NC}"
echo "======================================"
echo ""
echo "Next steps:"
echo "  - Read the API reference: docs/API_REFERENCE.md"
echo "  - Check out examples: examples/"
echo "  - Start coding your planner!"
echo ""
echo "Build artifacts are in: $(pwd)"
echo ""

