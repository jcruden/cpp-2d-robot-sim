#pragma once
#include "occupancy_grid.hpp"
#include "../util/math.hpp"
#include "robot_state.hpp"
#include <vector>

struct Reading {
    double angle;
    double range;
};

class RangeSensor {
public:
    RangeSensor(
        int num_rays,
        double fov, // span in radians
        double max_range
    );

    std::vector<Reading> sense(
        OccupancyGrid const& grid,
        RobotState const& state
    ) const;

private:
    int num_rays_;
    double fov_;
    double max_range_;

    double castRay(
        OccupancyGrid const& grid,
        Vec2 const& origin,
        double angle
    ) const;
};