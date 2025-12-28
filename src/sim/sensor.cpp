#include "sensor.hpp"
#include <cmath>
#include <cassert>

RangeSensor::RangeSensor(
    int num_rays,
    double fov,
    double max_range
) :
    num_rays_(num_rays),
    fov_(fov),
    max_range_(max_range)
{
    assert(num_rays_ > 0);
    assert(max_range_ > 0.0);
}

std::vector<Reading> RangeSensor::sense(
    OccupancyGrid const& grid,
    RobotState const& state
) const {
    std::vector<Reading> readings;
    readings.reserve(num_rays_);

    double dtheta = 0;
    double start_angle = 0;
    if (num_rays_ != 1) {
        dtheta = fov_ / (num_rays_ - 1);
        start_angle = -fov_ / 2.0;
    }

    for (int i = 0; i < num_rays_; ++i) {
        double rel_angle = start_angle + i * dtheta;
        double world_angle = state.theta + rel_angle;

        double r = castRay(grid, state.position, world_angle);
        readings.push_back({rel_angle, r});
    }

    return readings;
}

// return distance to obstacle (or max range)
double RangeSensor::castRay(
    OccupancyGrid const& grid,
    Vec2 const& origin,
    double angle
) const {
    // unit vec
    Vec2 dir{std::cos(angle), std::sin(angle)};
    double step = 0.5 * grid.resolution();
    if (step < 0.01) step = 0.01; // clamp step
    double traveled = 0.0;
    Vec2 p = origin;

    while (traveled < max_range_) {
        auto[i, j] = grid.worldToGrid(p);

        // check bounds and blockage
        if (i < 0 || i >= grid.height() ||
            j < 0 || j >= grid.width() || 
            grid.isOccupied(i, j)) {
                return traveled;
        }

        p += dir * step;
        traveled += step;
    }

    return max_range_;
}