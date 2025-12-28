#pragma once

#include "robot_state.hpp"

class MotionModel {
public:
    MotionModel(double wheel_base);

    void step(RobotState& state, double dt) const;

private:
    double wheel_base_;
};