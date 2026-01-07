# FastPlanner API Reference

Complete API documentation for FastPlanner Phase 1 (Core Infrastructure).

## Table of Contents

1. [State Module](#state-module)
2. [Collision Checking Module](#collision-checking-module)
3. [Utility Functions](#utility-functions)

---

## State Module

### Header: `fastplanner/core/state.hpp`

#### Class: `State`

Represents a configuration space state for a 2D robot.

**Members:**
```cpp
Eigen::Vector2d position;  // Position (x, y) in meters
double heading;            // Heading angle in radians [-π, π]
double velocity;           // Forward velocity in m/s
double time;               // Timestamp in seconds
```

**Constructors:**

```cpp
State();  // Default: all zeros

State(const Eigen::Vector2d& pos, double head = 0.0, 
      double vel = 0.0, double t = 0.0);

State(double x, double y, double head = 0.0, 
      double vel = 0.0, double t = 0.0);
```

**Methods:**

```cpp
// Compute Euclidean distance to another state
double distanceTo(const State& other) const;

// Compute configuration space distance (includes heading)
double configurationDistanceTo(const State& other, 
                               double heading_weight = 0.5) const;

// Linear interpolation between states
State interpolate(const State& other, double t) const;

// Normalize heading to [-π, π]
void normalizeHeading();

// Check if state is within bounds
bool isValid(const Bounds& bounds) const;
```

**Example:**

```cpp
State start(0, 0, 0, 1.0, 0.0);
State goal(10, 5, M_PI/4, 1.0, 10.0);

double dist = start.distanceTo(goal);
State mid = start.interpolate(goal, 0.5);
```

---

#### Class: `Trajectory`

Represents a sequence of states forming a trajectory.

**Constructors:**

```cpp
Trajectory();  // Empty trajectory
explicit Trajectory(const std::vector<State>& states);
```

**Methods:**

```cpp
// Add state to trajectory
void addState(const State& state);

// Get states
const std::vector<State>& getStates() const;
std::vector<State>& getStates();

// Size and access
size_t size() const;
bool empty() const;
void clear();
const State& at(size_t index) const;
const State& operator[](size_t index) const;

// Metrics
double getLength() const;           // Total path length
double getDuration() const;         // Time span
double getCurvature() const;        // Smoothness metric
double getCost(double curvature_weight = 1.0) const;

// Operations
Trajectory interpolateToTimeStep(double dt) const;
Trajectory downsample(size_t factor) const;
Trajectory reverse() const;
```

**Example:**

```cpp
Trajectory traj;
traj.addState(State(0, 0));
traj.addState(State(5, 5));
traj.addState(State(10, 10));

double length = traj.getLength();      // ~14.14 meters
double curvature = traj.getCurvature();
double cost = traj.getCost(1.0);

Trajectory smooth = traj.interpolateToTimeStep(0.1);
```

---

#### Class: `Bounds`

Configuration space bounds for sampling and validation.

**Members:**

```cpp
Eigen::Vector2d position_min;
Eigen::Vector2d position_max;
double heading_min;
double heading_max;
double velocity_min;
double velocity_max;
```

**Constructor:**

```cpp
Bounds(const Eigen::Vector2d& pos_min = Eigen::Vector2d(-10, -10),
       const Eigen::Vector2d& pos_max = Eigen::Vector2d(10, 10),
       double h_min = -M_PI,
       double h_max = M_PI,
       double v_min = 0.0,
       double v_max = 5.0);
```

**Methods:**

```cpp
bool isPositionValid(const Eigen::Vector2d& position) const;
Eigen::Vector2d samplePosition() const;
double sampleHeading() const;
double sampleVelocity() const;
```

**Example:**

```cpp
Bounds bounds(Eigen::Vector2d(0, 0), 
              Eigen::Vector2d(100, 100));

for (int i = 0; i < 100; ++i) {
    Eigen::Vector2d pos = bounds.samplePosition();
    double heading = bounds.sampleHeading();
    State random_state(pos, heading);
}
```

---

## Collision Checking Module

### Header: `fastplanner/core/collision_checker.hpp`

#### Class: `CircleObstacle`

Represents a circular obstacle (static or dynamic).

**Members:**

```cpp
Eigen::Vector2d center;
double radius;
Eigen::Vector2d velocity;  // For dynamic obstacles
bool is_dynamic;
```

**Constructors:**

```cpp
CircleObstacle(const Eigen::Vector2d& c, double r);  // Static
CircleObstacle(const Eigen::Vector2d& c, double r, 
               const Eigen::Vector2d& v);  // Dynamic
```

**Methods:**

```cpp
Eigen::Vector2d predictPosition(double dt) const;
bool collidesWith(const Eigen::Vector2d& point, 
                  double margin = 0.0) const;
```

**Example:**

```cpp
CircleObstacle static_obs(Eigen::Vector2d(5, 0), 1.0);

CircleObstacle dynamic_obs(Eigen::Vector2d(0, 0), 1.0,
                          Eigen::Vector2d(1, 0));  // Moving right

Eigen::Vector2d future_pos = dynamic_obs.predictPosition(5.0);
```

---

#### Class: `CollisionChecker`

Main collision checking interface with spatial optimization.

**Constructor:**

```cpp
explicit CollisionChecker(double robot_radius = 0.5, 
                         bool use_spatial_hashing = true);
```

**Setup Methods:**

```cpp
void addObstacle(const Eigen::Vector2d& center, double radius);

void addDynamicObstacle(const Eigen::Vector2d& center, 
                       double radius,
                       const Eigen::Vector2d& velocity);

void addObstacle(const CircleObstacle& obstacle);

void clearObstacles();

const std::vector<CircleObstacle>& getObstacles() const;
```

**Collision Queries:**

```cpp
// Check if state is collision-free
bool isStateFree(const State& state, double margin = 0.0) const;

// Check if path segment is collision-free
bool isPathFree(const State& state1, const State& state2,
               double resolution = 0.1, double margin = 0.0) const;

// Check if entire trajectory is collision-free
bool isTrajectoryFree(const Trajectory& trajectory, 
                     double margin = 0.0) const;
```

**Distance Queries:**

```cpp
// Get distance to nearest obstacle
double distanceToNearestObstacle(const Eigen::Vector2d& position, 
                                double time = 0.0) const;

// Get gradient of distance function
Eigen::Vector2d getObstacleGradient(const Eigen::Vector2d& position, 
                                   double time = 0.0) const;
```

**Configuration:**

```cpp
double getRobotRadius() const;
void setRobotRadius(double radius);
size_t getObstacleCount() const;
void rebuildSpatialHash();
```

**Example:**

```cpp
CollisionChecker checker(0.5);  // Robot radius = 0.5m

// Add obstacles
checker.addObstacle(Eigen::Vector2d(5, 0), 1.0);
checker.addDynamicObstacle(Eigen::Vector2d(0, 5), 1.0,
                          Eigen::Vector2d(1, 0));

// Check state
State state(3, 0);
bool is_free = checker.isStateFree(state);

// Check path with 0.1m resolution
State start(0, 0);
State goal(10, 0);
bool path_free = checker.isPathFree(start, goal, 0.1);

// Get distance for optimization
double dist = checker.distanceToNearestObstacle(state.position);
Eigen::Vector2d grad = checker.getObstacleGradient(state.position);

// Validate trajectory
Trajectory traj = ...;
bool traj_valid = checker.isTrajectoryFree(traj);
```

**Performance Notes:**

- Spatial hashing provides ~10-20x speedup for dense environments
- Default cell size is automatically tuned for typical obstacle sizes
- Use safety margins (0.1-0.5m) for robust planning
- Dynamic obstacles use constant velocity prediction

---

## Utility Functions

### Angle Utilities

```cpp
// Compute shortest angular difference in [-π, π]
double angleDifference(double angle1, double angle2);

// Normalize angle to [-π, π]
double normalizeAngle(double angle);
```

**Example:**

```cpp
double diff = angleDifference(3.0, -3.0);  // Accounts for wraparound
double normalized = normalizeAngle(3 * M_PI);  // Returns π
```

---

## Design Patterns and Best Practices

### 1. RAII and Resource Management

All classes follow RAII principles. No manual memory management required.

```cpp
{
    CollisionChecker checker(0.5);
    checker.addObstacle(Eigen::Vector2d(0, 0), 1.0);
    // Automatically cleaned up at end of scope
}
```

### 2. Const-Correctness

Methods that don't modify state are marked `const`:

```cpp
double dist = state.distanceTo(other);  // const method
bool free = checker.isStateFree(state); // const method
```

### 3. Value Semantics

Objects can be copied and returned by value efficiently:

```cpp
State interpolated = state1.interpolate(state2, 0.5);  // Efficient copy
Trajectory reversed = traj.reverse();  // Returns by value
```

### 4. Early Returns

Collision checks return immediately on first collision for performance.

### 5. Thread Safety

- State, Trajectory, Bounds are thread-safe (no shared mutable state)
- CollisionChecker is thread-safe for queries (const methods)
- Adding obstacles is NOT thread-safe (use external synchronization)

---

## Performance Characteristics

| Operation | Time Complexity | Notes |
|-----------|----------------|-------|
| State distance | O(1) | Vector operations |
| State interpolation | O(1) | Linear interpolation |
| Trajectory length | O(n) | n = number of waypoints |
| State collision (with hash) | O(1) avg | O(k) where k = local obstacles |
| State collision (no hash) | O(n) | n = total obstacles |
| Path collision | O(d/r) | d = distance, r = resolution |
| Distance query | O(1) avg | With spatial hashing |
| Gradient computation | O(1) | 3 distance queries |

---

## Error Handling

FastPlanner uses exceptions sparingly. Most errors are prevented through:

1. **Bounds checking**: Use `.at()` for checked access
2. **Validation**: `isValid()` methods
3. **Defensive programming**: Early returns on edge cases

**Exceptions thrown:**

- `std::out_of_range`: Invalid trajectory index with `.at()`

---

## Code Examples

### Complete Planning Example

```cpp
#include "fastplanner/core/state.hpp"
#include "fastplanner/core/collision_checker.hpp"

// Setup environment
CollisionChecker checker(0.5);
checker.addObstacle(Eigen::Vector2d(5, 5), 2.0);
checker.addObstacle(Eigen::Vector2d(15, 5), 1.5);

// Define bounds
Bounds bounds(Eigen::Vector2d(0, 0), Eigen::Vector2d(20, 10));

// Create trajectory
Trajectory traj;
traj.addState(State(0, 0, 0, 1.0, 0.0));

// Sample intermediate waypoints
for (int i = 0; i < 10; ++i) {
    Eigen::Vector2d pos = bounds.samplePosition();
    State candidate(pos, 0, 1.0, i);
    
    if (checker.isStateFree(candidate)) {
        traj.addState(candidate);
    }
}

traj.addState(State(20, 10, 0, 1.0, 10.0));

// Validate and analyze
if (checker.isTrajectoryFree(traj)) {
    std::cout << "Valid trajectory:\n";
    std::cout << "  Length: " << traj.getLength() << " m\n";
    std::cout << "  Duration: " << traj.getDuration() << " s\n";
    std::cout << "  Cost: " << traj.getCost(1.0) << "\n";
}
```

### Dynamic Obstacle Avoidance

```cpp
CollisionChecker checker(0.5);

// Add moving obstacle
checker.addDynamicObstacle(
    Eigen::Vector2d(0, 0),      // Start position
    1.0,                         // Radius
    Eigen::Vector2d(1.0, 0.5)   // Velocity (1 m/s right, 0.5 m/s up)
);

// Check trajectory considering predictions
Trajectory traj = ...;
for (const auto& state : traj.getStates()) {
    // Collision check automatically uses state.time for prediction
    if (!checker.isStateFree(state)) {
        std::cout << "Collision at t=" << state.time << "\n";
    }
}
```

---

## Next Steps

For more information:

- **Examples**: See `examples/` directory
- **Tests**: Browse `tests/` for usage patterns  
- **Benchmarks**: Check `benchmarks/` for performance
- **Phase 2**: Planning algorithms (RRT, RRT*, Hybrid A*) coming soon!

---

**Generated for FastPlanner v1.0.0 - Phase 1**

