#include "motion_model.hpp"
#include "../util/math.hpp"
#include <cmath>

MotionModel::MotionModel(double wheel_base) : wheel_base_(wheel_base) {}

void MotionModel::step(
    RobotState& state,
    double vl,
    double vr,
    double dt
) const {
    double v = (vl + vr) / 2;
    double w = (vr - vl) / wheel_base_;

    // Euler integration
    state.position.x += v * std::cos(state.theta) * dt;
    state.position.y += v * std::sin(state.theta) * dt;
    state.theta += w * dt;

    // [-pi, pi]
    state.theta = wrapToPi(state.theta);
    state.v_left = vl;
    state.v_right = vr;
};