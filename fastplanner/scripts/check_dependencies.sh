#!/bin/bash

# FastPlanner Dependency Checker
# Checks if all required and optional dependencies are installed

set -e

echo "========================================"
echo "FastPlanner Dependency Checker"
echo "========================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

MISSING_REQUIRED=0
MISSING_OPTIONAL=0

check_command() {
    if command -v "$1" &> /dev/null; then
        echo -e "${GREEN}✓${NC} $2: $(command -v $1)"
        if [ ! -z "$3" ]; then
            VERSION=$($3 2>&1 || echo "unknown")
            echo "  Version: $VERSION"
        fi
        return 0
    else
        echo -e "${RED}✗${NC} $2: NOT FOUND"
        return 1
    fi
}

check_library() {
    if pkg-config --exists "$1" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $2: Found"
        VERSION=$(pkg-config --modversion "$1" 2>/dev/null || echo "unknown")
        echo "  Version: $VERSION"
        echo "  Path: $(pkg-config --cflags-only-I "$1" | sed 's/-I//' || echo "unknown")"
        return 0
    else
        echo -e "${RED}✗${NC} $2: NOT FOUND"
        return 1
    fi
}

echo "=== Required Dependencies ==="
echo ""

# CMake
if check_command cmake "CMake" "cmake --version | head -1"; then
    CMAKE_VERSION=$(cmake --version | head -1 | awk '{print $3}')
    if [ "$(printf '%s\n' "3.15" "$CMAKE_VERSION" | sort -V | head -n1)" = "3.15" ]; then
        echo -e "  ${GREEN}Version >= 3.15 ✓${NC}"
    else
        echo -e "  ${YELLOW}Warning: CMake < 3.15 (found $CMAKE_VERSION)${NC}"
    fi
else
    MISSING_REQUIRED=1
fi
echo ""

# C++ Compiler
if check_command c++ "C++ Compiler" "c++ --version | head -1"; then
    :
elif check_command g++ "C++ Compiler" "g++ --version | head -1"; then
    :
elif check_command clang++ "C++ Compiler" "clang++ --version | head -1"; then
    :
else
    MISSING_REQUIRED=1
fi
echo ""

# Eigen3
if check_library eigen3 "Eigen3"; then
    EIGEN_VERSION=$(pkg-config --modversion eigen3 2>/dev/null)
    if [ "$(printf '%s\n' "3.3.0" "$EIGEN_VERSION" | sort -V | head -n1)" = "3.3.0" ]; then
        echo -e "  ${GREEN}Version >= 3.3.0 ✓${NC}"
    else
        echo -e "  ${YELLOW}Warning: Eigen < 3.3.0 (found $EIGEN_VERSION)${NC}"
    fi
else
    # Check alternate locations
    if [ -d "/usr/include/eigen3" ]; then
        echo -e "${YELLOW}⚠${NC} Eigen3: Found in /usr/include/eigen3 (but pkg-config not configured)"
    elif [ -d "/opt/homebrew/include/eigen3" ]; then
        echo -e "${YELLOW}⚠${NC} Eigen3: Found in /opt/homebrew/include/eigen3 (but pkg-config not configured)"
    else
        MISSING_REQUIRED=1
    fi
fi
echo ""

echo "=== Optional Dependencies ==="
echo ""

# Google Test
if [ -f "/usr/lib/libgtest.a" ] || [ -f "/usr/local/lib/libgtest.a" ] || \
   [ -f "/opt/homebrew/lib/libgtest.a" ]; then
    echo -e "${GREEN}✓${NC} Google Test: Found"
    find /usr/lib /usr/local/lib /opt/homebrew/lib -name "libgtest.a" 2>/dev/null | head -1
elif pkg-config --exists gtest 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Google Test: Found (via pkg-config)"
else
    echo -e "${YELLOW}⚠${NC} Google Test: NOT FOUND (tests will be disabled)"
    MISSING_OPTIONAL=1
fi
echo ""

# Google Benchmark
if check_library benchmark "Google Benchmark"; then
    :
else
    echo -e "${YELLOW}⚠${NC} Google Benchmark: NOT FOUND (benchmarks will be disabled)"
    MISSING_OPTIONAL=1
fi
echo ""

# Emscripten (for WebAssembly)
if check_command emcc "Emscripten" "emcc --version | head -1"; then
    :
else
    echo -e "${YELLOW}⚠${NC} Emscripten: NOT FOUND (WebAssembly build will be disabled)"
    MISSING_OPTIONAL=1
fi
echo ""

echo "========================================"
echo "Summary"
echo "========================================"
echo ""

if [ $MISSING_REQUIRED -eq 0 ]; then
    echo -e "${GREEN}✓ All required dependencies found!${NC}"
    echo ""
    echo "You can build FastPlanner with:"
    echo "  mkdir build && cd build"
    echo "  cmake -DCMAKE_BUILD_TYPE=Release .."
    echo "  make -j\$(nproc)"
else
    echo -e "${RED}✗ Missing required dependencies!${NC}"
    echo ""
    echo "Please install missing dependencies:"
    echo ""
    echo "macOS (Homebrew):"
    echo "  brew install cmake eigen"
    echo ""
    echo "Ubuntu/Debian:"
    echo "  sudo apt-get install cmake libeigen3-dev"
    echo ""
    exit 1
fi

if [ $MISSING_OPTIONAL -gt 0 ]; then
    echo -e "${YELLOW}⚠ Some optional dependencies are missing${NC}"
    echo ""
    echo "To enable all features, install:"
    echo ""
    echo "macOS (Homebrew):"
    echo "  brew install googletest google-benchmark emscripten"
    echo ""
    echo "Ubuntu/Debian:"
    echo "  sudo apt-get install libgtest-dev libbenchmark-dev"
    echo ""
fi

echo ""
echo "For detailed installation instructions, see INSTALL.md"
echo ""

