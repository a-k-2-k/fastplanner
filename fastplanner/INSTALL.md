# Installation Guide

## Prerequisites

FastPlanner requires the following dependencies:

### Required
- **CMake** (>= 3.15): Build system
- **C++ Compiler**: GCC 7+, Clang 6+, or MSVC 2017+ with C++17 support
- **Eigen3** (>= 3.3): Linear algebra library

### Optional
- **Google Test**: For unit tests (recommended)
- **Google Benchmark**: For performance benchmarks
- **Emscripten**: For WebAssembly compilation

## Installation by Platform

### macOS

Using Homebrew:
```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake eigen googletest google-benchmark

# Optional: For WebAssembly
brew install emscripten
```

### Ubuntu/Debian

```bash
# Update package list
sudo apt-get update

# Install required packages
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev

# Install optional packages
sudo apt-get install -y \
    libgtest-dev \
    libbenchmark-dev

# Build and install Google Test (if not available via package manager)
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp lib/*.a /usr/lib
```

### Arch Linux

```bash
sudo pacman -S cmake eigen googletest benchmark
```

### Windows

Using vcpkg:
```bash
# Clone vcpkg
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh  # or .bat on Windows

# Install dependencies
./vcpkg install eigen3 gtest benchmark
```

## Building FastPlanner

### Quick Build

```bash
# Clone the repository
git clone https://github.com/yourusername/fastplanner.git
cd fastplanner

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake -DCMAKE_BUILD_TYPE=Release ..

# Build (use -j for parallel compilation)
make -j$(nproc)  # Linux/macOS
# or
cmake --build . --config Release  # Cross-platform

# Run tests (if Google Test is available)
./fastplanner_tests

# Run examples
./examples/demo_state
./examples/demo_collision
```

### Build Types

#### Release (Optimized)
```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

#### Debug (with symbols)
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

#### RelWithDebInfo (Optimized + Symbols)
```bash
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j$(nproc)
```

### Advanced Options

#### Custom Eigen Location
```bash
cmake -DEigen3_DIR=/path/to/eigen3 ..
```

#### Disable Tests
```bash
# Tests are automatically disabled if Google Test is not found
# To explicitly disable:
cmake -DBUILD_TESTING=OFF ..
```

#### Install to Custom Location
```bash
cmake -DCMAKE_INSTALL_PREFIX=/custom/path ..
make install
```

## Verifying Installation

After building, verify the installation:

```bash
# From build directory
./fastplanner_tests  # Should run all tests
./examples/demo_state  # Should run state demo
./examples/demo_collision  # Should run collision demo

# Check library was created
ls libfastplanner.* # Should show .so (Linux), .dylib (macOS), or .dll (Windows)
```

## Troubleshooting

### CMake can't find Eigen3

**Solution**: Install Eigen3 or specify its location:
```bash
cmake -DEigen3_DIR=/path/to/eigen3/share/eigen3/cmake ..
```

On macOS with Homebrew:
```bash
cmake -DEigen3_DIR=/opt/homebrew/share/eigen3/cmake ..
```

### Google Test not found

**Solution**: Tests will be automatically skipped. To enable:
1. Install Google Test (see platform-specific instructions above)
2. Specify GTest location if needed:
```bash
cmake -DGTest_DIR=/path/to/gtest ..
```

### Compiler doesn't support C++17

**Solution**: Update your compiler:
- GCC: `sudo apt-get install g++-9` (or higher)
- Clang: `sudo apt-get install clang-9` (or higher)

Specify compiler explicitly:
```bash
export CXX=g++-9  # or clang++-9
cmake ..
```

### Build errors with Eigen

**Solution**: Ensure Eigen version >= 3.3:
```bash
# Check Eigen version
pkg-config --modversion eigen3

# Or check CMake output for Eigen3 version
```

### Link errors on macOS

**Solution**: May need to specify architecture:
```bash
cmake -DCMAKE_OSX_ARCHITECTURES=arm64 ..  # For Apple Silicon
cmake -DCMAKE_OSX_ARCHITECTURES=x86_64 .. # For Intel
```

## WebAssembly Build (Advanced)

To build for WebAssembly:

```bash
# Install Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh

# Build FastPlanner for WebAssembly
cd /path/to/fastplanner
mkdir build-wasm && cd build-wasm
emcmake cmake -DCMAKE_BUILD_TYPE=Release ..
emmake make
```

## Running Tests

### Run all tests
```bash
cd build
./fastplanner_tests
```

### Run specific test suite
```bash
./fastplanner_tests --gtest_filter="StateTest.*"
./fastplanner_tests --gtest_filter="CollisionCheckerTest.*"
```

### Verbose test output
```bash
./fastplanner_tests --gtest_print_time=1 --gtest_color=yes
```

### Run with CTest
```bash
cd build
ctest --output-on-failure
```

## Performance Benchmarks

If Google Benchmark is available:

```bash
cd build
./benchmarks/benchmark_planning  # (once implemented)
```

## IDE Setup

### Visual Studio Code

1. Install C++ extension
2. Install CMake Tools extension
3. Open FastPlanner directory
4. Select build kit when prompted
5. Press F7 to build

### CLion

1. Open FastPlanner directory
2. CLion will automatically detect CMakeLists.txt
3. Click "Build" → "Build Project"

### Xcode (macOS)

```bash
mkdir build-xcode
cd build-xcode
cmake -G Xcode ..
open FastPlanner.xcodeproj
```

## Next Steps

After successful installation:
1. Read the [README.md](README.md) for project overview
2. Run the example programs in `examples/`
3. Explore the API documentation in header files
4. Start implementing planners (Phase 2)

## Getting Help

If you encounter issues not covered here:
1. Check the [README.md](README.md)
2. Review CMake configuration output carefully
3. Ensure all dependencies meet minimum version requirements
4. Check compiler error messages for missing includes

