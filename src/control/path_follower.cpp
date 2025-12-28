#include "path_follower.hpp"
#include <cassert>
#include <cmath>

PathFollower::PathFollower(double wheelbase)
: L_(wheelbase) {}

std::pair<double, double> PathFollower::computeControl (
    RobotState const& state,
    Vec2 const& goal
) const {
    double dx = goal.x - state.position.x;
    double dy = goal.y - state.position.y;

    // dist/heading
    double phi = std::atan2(dy, dx);
    double heading_err = wrapToPi(phi - state.theta);
    double dist_err = std::sqrt(dx * dx + dy * dy);

    // control
    double v = k_rho_ * dist_err;
    double w = k_alpha_ * heading_err;

    // differential drive kinematics
    double v_left = v - w * L_ / 2.0;
    double v_right = v + w * L_ / 2.0;

    return {v_left, v_right};
}