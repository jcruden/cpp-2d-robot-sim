cmake_minimum_required(VERSION 3.15)
project(cpp_2d_robot_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Motion model library
add_library(motion_model
    src/sim/motion_model.cpp
)

target_include_directories(motion_model
    PUBLIC
        src
)

# Occupancy grid library
add_library(occupancy_grid
    src/sim/occupancy_grid.cpp
)

target_include_directories(occupancy_grid
    PUBLIC
        src
)

# AStar library
add_library(astar
    src/planning/astar.cpp
)

target_include_directories(astar
    PUBLIC
        src
)

target_link_libraries(astar
    PRIVATE occupancy_grid
)

# Motion model tests
add_executable(test_motion
    tests/test_motion.cpp
)

target_link_libraries(test_motion
    PRIVATE motion_model
)

# Grid tests
add_executable(test_grid
    tests/test_grid.cpp
)

target_link_libraries(test_grid
    PRIVATE occupancy_grid
)

# AStar tests
add_executable(test_astar
    tests/test_astar.cpp
)

target_link_libraries(test_astar
    PRIVATE astar occupancy_grid
)