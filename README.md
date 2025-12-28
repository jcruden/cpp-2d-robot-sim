![CI](https://github.com/jcruden/cpp-2d-robot-sim/actions/workflows/ci.yml/badge.svg)

# C++ 2D Robotics Simulator

Modern C++ 2D mobile robot simulator built from first principles. Focus on clean architecture and code to demonstrate understanding of basic concepts.

## Features

### Core Math Util
- Vec2 type with norm, arithmetic, distance

### 2D Occupancy Grid
- Row-major stoage
- World grid coord conversion
- Obstacle generation

### A* Path Planner
- Grid-based A* implementation with Manhattan heuristic

### Differential-Drive Motion Model
- Discrete time kinematic model

### Unit Tests
- Simple tests for path planner and motion

## Build & Test
```bash
mkdir build && cd build
cmake ..
cmake --build .
./test_motion