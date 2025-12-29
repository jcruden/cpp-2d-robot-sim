#include "simulator.hpp"
#include <cmath>

Simulator::Simulator(
    OccupancyGrid& grid,
    Robot& robot,
    AStarPlanner& planner,
    Vec2 goal,
    double wheelbase
)
    : grid_(grid),
    robot_(robot),
    planner_(planner),
    controller_(wheelbase),
    goal_(goal) {}

void Simulator::run(double dt, int max_steps) {
    auto start = robot_.state().position;
    auto path = planner_.plan(grid_, start, goal_);
    if (path.empty()) return;

    size_t path_idx = 0;

    for (int i = 0; i < max_steps; ++i) {
        auto readings = robot_.sense(grid_);

        Vec2 pos = robot_.state().position;
        
        bool near_goal = (path_idx + 1 == path.size());
        double waypoint_tol = 0.3 * grid_.resolution();
        if (pos.distance(path[path_idx]) < waypoint_tol && !near_goal) {
            path_idx++;
        }
        Vec2 waypoint = path[path_idx];
        if (near_goal) {
            waypoint = goal_;
        }

        auto [vl, vr] = controller_.computeControl(robot_.state(), waypoint);
        robot_.step(vl, vr, dt);

        if (reachedGoal()) {
            break;
        }
    }
}

bool Simulator::reachedGoal() const {
    return robot_.state().position.distance(goal_) < 0.05;
}