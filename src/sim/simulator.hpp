#pragma once

#include "occupancy_grid.hpp"
#include "robot.hpp"
#include "../planning/astar.hpp"
#include "../control/path_follower.hpp"

class Simulator {
public:
    Simulator(
        OccupancyGrid grid,
        Robot robot,
        AStarPlanner planner,
        Vec2 goal,
        double wheelbase
    );

    void run(double dt, int max_steps);

private:
    OccupancyGrid grid_;
    Robot robot_;
    AStarPlanner planner_;
    PathFollower controller_;
    Vec2 goal_;

    bool reachedGoal() const;
};