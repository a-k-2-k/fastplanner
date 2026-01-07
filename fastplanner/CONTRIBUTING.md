# Contributing to FastPlanner

Thank you for your interest in contributing to FastPlanner! This document provides guidelines and instructions for contributing.

## Table of Contents

1. [Code of Conduct](#code-of-conduct)
2. [Getting Started](#getting-started)
3. [Development Workflow](#development-workflow)
4. [Coding Standards](#coding-standards)
5. [Testing Requirements](#testing-requirements)
6. [Documentation](#documentation)
7. [Submitting Changes](#submitting-changes)

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help others learn and grow
- Maintain a positive environment

## Getting Started

### 1. Fork and Clone

```bash
# Fork the repository on GitHub
# Then clone your fork
git clone https://github.com/YOUR_USERNAME/fastplanner.git
cd fastplanner
git remote add upstream https://github.com/original/fastplanner.git
```

### 2. Create a Branch

```bash
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

### 3. Build and Test

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
make test
```

## Development Workflow

### 1. Make Your Changes

- Write clean, readable code
- Follow the coding standards below
- Add tests for new functionality
- Update documentation

### 2. Run Tests

```bash
# Run all tests
./tests/fastplanner_tests

# Run specific tests
./tests/fastplanner_tests --gtest_filter="YourTests.*"
```

### 3. Check Code Format

```bash
# Format your code
clang-format -i include/**/*.hpp src/**/*.cpp

# Check formatting (dry run)
clang-format --dry-run --Werror include/**/*.hpp
```

### 4. Build Examples

```bash
# Make sure examples still work
./examples/demo_state_trajectory
./examples/demo_collision_checking
```

### 5. Commit Your Changes

```bash
git add .
git commit -m "feat: Add awesome feature"

# Use conventional commits:
# feat: New feature
# fix: Bug fix
# docs: Documentation changes
# test: Adding tests
# refactor: Code refactoring
# perf: Performance improvements
# style: Code style changes
```

## Coding Standards

### Naming Conventions

```cpp
// Classes: PascalCase
class MyAwesomeClass {
    
// Public methods: camelCase
public:
    void doSomething();
    double calculateValue() const;
    
// Private methods: camelCase
private:
    void helperFunction();
    
// Member variables: snake_case with trailing underscore
private:
    double my_value_;
    int counter_;
    std::vector<State> states_;
};

// Functions: camelCase
double computeDistance(const State& s1, const State& s2);

// Constants: UPPER_SNAKE_CASE
const double MAX_VELOCITY = 5.0;
const int DEFAULT_SIZE = 100;

// Local variables: snake_case
int obstacle_count = 0;
double path_length = 0.0;
```

### Code Style

```cpp
// Header guards: Use #pragma once
#pragma once

// Includes: Group and sort
#include "fastplanner/core/state.hpp"        // Project headers first
#include "fastplanner/core/collision.hpp"

#include <Eigen/Dense>                       // Then third-party
#include <vector>
#include <memory>

// Namespace: Don't indent
namespace fastplanner {

class MyClass {
    // Group by access level: public, protected, private
    // Within each, group: types, constructors, methods, members
};

} // namespace fastplanner

// Braces: Same line for functions and classes
void function() {
    if (condition) {
        // Do something
    }
    
    for (int i = 0; i < 10; ++i) {
        // Loop body
    }
}

// Const-correctness: Always use when appropriate
double getValue() const { return value_; }

// Modern C++: Use auto when type is obvious
auto it = container.begin();
auto result = someFunction();

// But be explicit when clarity helps
CollisionChecker checker(0.5);  // Better than auto here
```

### Documentation

```cpp
/**
 * @brief Brief description of the function
 * 
 * Longer description explaining what the function does,
 * any important notes, and design considerations.
 * 
 * @param param1 Description of first parameter
 * @param param2 Description of second parameter
 * @return Description of return value
 * 
 * @throws std::exception When this might throw
 * 
 * @example
 * State s1(0, 0);
 * State s2(1, 1);
 * double dist = s1.distanceTo(s2);
 */
double distanceTo(const State& param1, double param2) const;
```

## Testing Requirements

### Every New Feature Must Have Tests

```cpp
// tests/test_my_feature.cpp
#include <gtest/gtest.h>
#include "fastplanner/core/my_feature.hpp"

TEST(MyFeatureTests, BasicFunctionality) {
    MyFeature feature;
    EXPECT_TRUE(feature.works());
}

TEST(MyFeatureTests, EdgeCases) {
    MyFeature feature;
    EXPECT_NO_THROW(feature.handleEdgeCase());
}

TEST(MyFeatureTests, ErrorConditions) {
    MyFeature feature;
    EXPECT_THROW(feature.invalidOperation(), std::exception);
}
```

### Test Coverage Guidelines

- Aim for >80% code coverage
- Test normal cases
- Test edge cases (empty inputs, boundaries, etc.)
- Test error conditions
- Test performance-critical paths

### Running Coverage

```bash
# Build with coverage
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage" ..
make

# Run tests
make test

# Generate report
lcov --capture --directory . --output-file coverage.info
lcov --remove coverage.info '/usr/*' --output-file coverage.info
genhtml coverage.info --output-directory coverage_html
```

## Documentation

### Update Documentation for Changes

- **README.md**: Update if adding major features
- **API_REFERENCE.md**: Document new public APIs
- **GETTING_STARTED.md**: Update if workflow changes
- **Code comments**: Explain complex algorithms

### Documentation Style

- Use clear, concise language
- Include code examples
- Explain *why*, not just *what*
- Link to relevant papers/resources

## Submitting Changes

### 1. Update from Upstream

```bash
git fetch upstream
git rebase upstream/main
```

### 2. Push to Your Fork

```bash
git push origin feature/your-feature-name
```

### 3. Create Pull Request

Go to GitHub and create a pull request with:

**Title**: Clear, descriptive summary

**Description Template**:
```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
- [ ] All tests pass
- [ ] Added new tests
- [ ] Tested manually

## Checklist
- [ ] Code follows style guidelines
- [ ] Comments added for complex code
- [ ] Documentation updated
- [ ] No new warnings

## Related Issues
Closes #123
```

### 4. Code Review Process

- Maintainer will review your code
- Address feedback promptly
- Be open to suggestions
- Discussion is encouraged

### 5. After Merge

```bash
# Update your local main
git checkout main
git pull upstream main

# Delete feature branch
git branch -d feature/your-feature-name
git push origin --delete feature/your-feature-name
```

## Areas for Contribution

### High Priority

- [ ] KD-tree implementation for RRT nearest neighbor
- [ ] Polygon obstacle support
- [ ] Dubins/Reeds-Shepp curves
- [ ] More unit tests
- [ ] Performance optimizations

### Medium Priority

- [ ] Python bindings (pybind11)
- [ ] ROS2 integration
- [ ] Additional motion models
- [ ] Visualization tools
- [ ] More examples

### Low Priority

- [ ] GPU acceleration
- [ ] 3D planning support
- [ ] Additional distance metrics
- [ ] Configuration file support

## Questions?

- Open an issue for discussion
- Email: your.email@berkeley.edu
- Check existing issues and PRs

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

Thank you for contributing to FastPlanner! 🚀
