#include "motion_model.hpp"
#include "../util/math.hpp"
#include <cmath>

MotionModel::MotionModel(double wheel_base) : wheel_base_(wheel_base) {}

void MotionModel::step(
    RobotState& state,
    double dt
) const {
    double v = (state.v_right + state.v_left) / 2;
    double w = (state.v_right - state.v_left) / wheel_base_;

    // Euler integration
    state.position.x += v * std::cos(state.theta) * dt;
    state.position.y += v * std::sin(state.theta) * dt;
    state.theta += w * dt;

    // [-pi, pi]
    state.theta = wrapToPi(state.theta);
};