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

# Motion model tests
add_executable(test_motion
    tests/test_motion.cpp
)

target_link_libraries(test_motion
    PRIVATE motion_model
)
