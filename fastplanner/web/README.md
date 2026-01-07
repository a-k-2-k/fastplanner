# WebAssembly Demo

Interactive browser-based demonstration of FastPlanner algorithms.

## Status: 🚧 Phase 4 (Planned)

WebAssembly build will be implemented in Phase 4 for final demonstration.

## Planned Features

### Interactive Demo
- **Algorithm Selection**: Choose RRT, RRT*, or Hybrid A* from dropdown
- **Obstacle Placement**: Click to add/remove circular obstacles
- **Start/Goal Setting**: Drag to set planning endpoints
- **Real-time Visualization**: See planning process in action
- **Metrics Display**: Planning time, path length, nodes explored

### Visualization
- **Three.js Rendering**: Smooth 2D/3D visualization
- **Animation**: Tree growth, path refinement
- **Statistics**: Real-time performance metrics
- **Comparison Mode**: Run multiple algorithms side-by-side

## Architecture

```
web/
├── bindings.cpp         # Emscripten C++ to JS bindings
├── index.html          # Main demo page
├── visualizer.js       # Three.js rendering
├── controls.js         # UI interaction handling
├── styles.css          # Styling
└── README.md           # This file
```

## Building for WebAssembly

### Prerequisites
```bash
# Install Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh
```

### Build Commands
```bash
cd fastplanner
mkdir build-wasm && cd build-wasm

# Configure with Emscripten
emcmake cmake -DCMAKE_BUILD_TYPE=Release ..

# Build
emmake make

# Output: fastplanner_web.wasm, fastplanner_web.js
```

## Planned Bindings

```cpp
// Example Emscripten bindings (bindings.cpp)
#include <emscripten/bind.h>
#include "fastplanner/planners/rrt.hpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(fastplanner) {
    class_<State>("State")
        .constructor<double, double>()
        .property("x", &State::position.x())
        .property("y", &State::position.y());
    
    class_<RRTPlanner>("RRTPlanner")
        .constructor<Bounds, CollisionChecker&>()
        .function("plan", &RRTPlanner::plan)
        .function("getPath", &RRTPlanner::getPath);
    
    // ... more bindings
}
```

## JavaScript Usage (Planned)

```javascript
// Load WebAssembly module
const fastplanner = await loadFastPlanner();

// Create planner
const bounds = new fastplanner.Bounds(0, 100, 0, 100);
const checker = new fastplanner.CollisionChecker(100, 100, 2.0);
const planner = new fastplanner.RRTPlanner(bounds, checker);

// Add obstacles
checker.addObstacle(new fastplanner.CircleObstacle(50, 50, 5));

// Plan path
const start = new fastplanner.State(10, 10);
const goal = new fastplanner.State(90, 90);
const path = planner.plan(start, goal);

// Visualize
visualizer.drawPath(path);
```

## Demo Scenarios

### Scenario 1: Simple Navigation
- Empty space with 2-3 obstacles
- Shows basic RRT exploration

### Scenario 2: Maze Solving
- Complex obstacle layout
- Demonstrates RRT* path optimization

### Scenario 3: Parking Maneuver
- Tight space with orientation constraints
- Shows Hybrid A* kinodynamic planning

### Scenario 4: Dynamic Avoidance
- Moving obstacles
- Real-time replanning demo

## Performance Targets

- **Load Time**: < 2 seconds for WASM module
- **Planning Rate**: 10+ Hz in browser
- **Frame Rate**: 60 FPS visualization
- **Memory**: < 100 MB browser usage

## Browser Compatibility

- Chrome 91+
- Firefox 89+
- Safari 15+
- Edge 91+

## Local Development

```bash
# Serve locally
cd web
python3 -m http.server 8000

# Open in browser
open http://localhost:8000
```

## Deployment

Deploy to GitHub Pages for easy sharing with recruiters:

```bash
# Build for production
./scripts/build_wasm.sh

# Deploy to gh-pages branch
git subtree push --prefix web origin gh-pages
```

## Interview Demonstration

This demo enables:
- "I compiled my C++ motion planner to WebAssembly"
- "No server needed - runs entirely in browser"
- "Interactive visualization shows algorithm behavior"
- "Can compare algorithms in real-time"

## Coming Soon

Check back after Phase 3 completion for the live demo!

Meanwhile, explore the C++ examples:
```bash
cd build
./examples/demo_state
./examples/demo_collision
```

