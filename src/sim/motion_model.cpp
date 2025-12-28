#include "motion_model.hpp"
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
    if (state.theta > M_PI) {
        state.theta = std::fmod(state.theta + M_PI, 2.0*M_PI) - M_PI;
    }
    else if (state.theta <= -M_PI) {
        state.theta = std::fmod(state.theta - M_PI, 2.0*M_PI) + M_PI;
    }
};