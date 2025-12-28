#pragma once

#include "../sim/robot_state.hpp"
#include "../sim/action.hpp"
#include "../util/math.hpp"
#include <vector>

class PathFollower {
public:
    PathFollower(double wheelbase);

    // return v_left, v_right
    std::pair<double, double> computeControl(
        RobotState const& state,
        Vec2 const& goal
    ) const;

private:
    double L_; // wheelbase
    double k_rho_ = 1.0; // distance
    double k_alpha_ = 2.0; // heading
};