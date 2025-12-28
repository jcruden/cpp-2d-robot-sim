#pragma once

#include "occupancy_grid.hpp"

class Simulator {
public:
    Simulator();

    void step(double dt);
    void run(int steps, double dt);

private:
    OccupancyGrid grid_;

    // TODO: robot, planner

};