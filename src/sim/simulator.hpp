#pragma once

#include "occupancy_grid.hpp"
#include "robot.hpp"
#include "../planning/astar.hpp"
#include "../control/path_follower.hpp"
#include <vector>
#include <string>

class Simulator {
public:
    Simulator(
        OccupancyGrid& grid,
        Robot& robot,
        AStarPlanner& planner,
        Vec2 goal,
        double wheelbase
    );

    void run(double dt, int max_steps, int frame_interval = 100);

private:
    OccupancyGrid& grid_;
    Robot& robot_;
    AStarPlanner& planner_;
    PathFollower controller_;
    Vec2 goal_;
    std::vector<Vec2> trajectory_;
    std::vector<Vec2> planned_path_;

    bool reachedGoal() const;
    void writeFrameSVG(std::string const& filename) const;
    void writeSummarySVG(std::string const& filename) const;
};