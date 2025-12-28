#pragma once

#include "robot_state.hpp"

class MotionModel {
public:
    MotionModel(double wheel_base);

    void step(RobotState& state, double vl, double vr, double dt) const;

private:
    double wheel_base_;
};