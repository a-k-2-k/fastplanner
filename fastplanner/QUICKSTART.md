# FastPlanner Quick Start 🚀

Get FastPlanner up and running in 5 minutes!

## Option 1: Automatic Quick Start (Recommended)

```bash
cd fastplanner
./scripts/quick_start.sh
```

This script will:
1. Configure CMake
2. Build the library
3. Run all tests
4. Run example programs

## Option 2: Using Make

```bash
cd fastplanner

# Build everything
make

# Run tests
make test

# Run examples
make examples

# See all options
make help
```

## Option 3: Manual CMake

```bash
cd fastplanner
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
ctest
```

## First Code Example

Create `my_planner.cpp`:

```cpp
#include "fastplanner/core/state.hpp"
#include "fastplanner/core/collision_checker.hpp"
#include <iostream>

using namespace fastplanner;

int main() {
    // Create environment
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(5, 0), 1.0);
    
    // Check if path is clear
    State start(0, 0);
    State goal(10, 0);
    
    if (checker.isPathFree(start, goal, 0.1)) {
        std::cout << "Path is clear!\n";
    } else {
        std::cout << "Path is blocked!\n";
    }
    
    return 0;
}
```

Compile and run:

```bash
cd build
g++ -std=c++17 -I../include -I/usr/include/eigen3 \
    ../my_planner.cpp -L. -lfastplanner -o my_planner
./my_planner
```

## What's Next?

- 📖 **Read the docs**: `docs/GETTING_STARTED.md`
- 🔍 **API Reference**: `docs/API_REFERENCE.md`
- 💡 **Examples**: Check `examples/` directory
- 🧪 **Tests**: Browse `tests/` for usage patterns

## Common Issues

### Can't find Eigen3?
```bash
# Ubuntu/Debian
sudo apt install libeigen3-dev

# macOS
brew install eigen
```

### Tests not building?
```bash
# Ubuntu/Debian
sudo apt install libgtest-dev

# macOS
brew install googletest
```

### Need help?
- Open an issue on GitHub
- Check `docs/GETTING_STARTED.md`
- Read the FAQ in `README.md`

---

**Ready to build autonomous systems!** 🤖

