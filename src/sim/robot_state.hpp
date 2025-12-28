#pragma once

#include "../util/math.hpp"

struct RobotState {
    Vec2 position; // world coords
    double theta; // orientation
    double v_left; // vel of left wheel
    double v_right; // vel of right wheel
};