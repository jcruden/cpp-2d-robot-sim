#include "simulator.hpp"

Simulator::Simulator(
    OccupancyGrid grid,
    Robot robot,
    AStarPlanner planner,
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

    for (int i = 0; i < max_steps; ++i) {
        auto [vl, vr] = controller_.computeControl(robot_.state(), goal_);
        robot_.step(vl, vr, dt);

        if (robot_.state().position.distance(goal_) < 0.05) {
            break;
        }
    }
}