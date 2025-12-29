![CI](https://github.com/jcruden/cpp-2d-robot-sim/actions/workflows/ci.yml/badge.svg)

# C++ 2D Robotics Simulator

Lightweight modern C++ 2D mobile robot simulator built from first principles. Focus on clean architecture and code to demonstrate understanding of basic concepts. A* path planning, differential drive motion model, SVG visualization.

## Features

### Core Math Util
- Vec2 type with norm, arithmetic, distance

### 2D Occupancy Grid
- Row-major storage
- World grid coord conversion
- Obstacle generation

### A* Path Planner
- Grid-based A* implementation with Manhattan heuristic

### Differential-Drive Motion Model
- Discrete time kinematic model

### 2D Range Sensor
- Grid based ray marching

### Simulation Loop
- Integrates sensing, planning, control, motion

### Visualization
- SVG frame with obstacles, trajectory, robot pose/goal

### Testing & CI
- Unit tests for components
- Built with CMake

## Project Structure

```
cpp-2d-robot-sim/
├── src/
│   ├── control/          # Path following controller
│   ├── planning/         # A* path planning
│   ├── sim/              # Core simulation components (robot, sensor, motion model, grid)
│   ├── util/             # Math utilities
│   └── main.cpp          # Entry point
├── tests/                # Unit tests
├── examples/             # Example configurations and maps
└── scripts/              # Utility scripts
```

## Requirements

- C++17 compiler
- CMake >= 3.14

## Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
./sim
./test_runner
```