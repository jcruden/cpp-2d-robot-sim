cmake_minimum_required(VERSION 3.15)
project(cpp_2d_robot_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4 /WX-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Helper function to configure library include directories
function(configure_library target)
    target_include_directories(${target}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<INSTALL_INTERFACE:include>
    )
endfunction()

# Base libraries
add_library(motion_model src/sim/motion_model.cpp)
configure_library(motion_model)

add_library(sensor src/sim/sensor.cpp)
configure_library(sensor)

add_library(occupancy_grid src/sim/occupancy_grid.cpp)
configure_library(occupancy_grid)

add_library(closed_loop src/control/path_follower.cpp)
configure_library(closed_loop)

# Planning library
add_library(astar src/planning/astar.cpp)
configure_library(astar)
target_link_libraries(astar PRIVATE occupancy_grid)

# Simulator library
add_library(simulator
    src/sim/simulator.cpp
    src/sim/robot.cpp
)
configure_library(simulator)
target_link_libraries(simulator PRIVATE
    motion_model
    sensor
    occupancy_grid
    closed_loop
    astar
)

# Catch2
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.4
)
FetchContent_MakeAvailable(Catch2)

# Tests
add_executable(test_runner
    tests/test_runner.cpp
    tests/test_astar.cpp
    tests/test_grid.cpp
    tests/test_motion.cpp
    tests/test_sensor.cpp
    tests/test_closed_loop.cpp
    tests/test_simulator.cpp
)
target_link_libraries(test_runner PRIVATE simulator Catch2::Catch2WithMain)

add_executable(sim
    src/main.cpp
    src/sim/simulator.cpp
    src/sim/robot.cpp
    src/sim/motion_model.cpp
    src/sim/occupancy_grid.cpp
    src/planning/astar.cpp
    src/sim/sensor.cpp
    src/control/path_follower.cpp
)

target_include_directories(sim PRIVATE src)