#include "sim/simulator.hpp"
#include "sim/occupancy_grid.hpp"
#include "sim/robot.hpp"
#include "planning/astar.hpp"

#include <iostream>

int main() {
    // -------------------------------------------------
    // World
    // -------------------------------------------------
    OccupancyGrid grid(
        20, 20,              // width, height (cells)
        0.1,                 // resolution (m)
        Vec2{0.0, 0.0}       // origin
    );

    // Simple vertical obstacle
    for (int i = 5; i < 15; ++i) {
        grid.setOccupied(i, 10);
    }

    // -------------------------------------------------
    // Robot
    // -------------------------------------------------
    RobotState init;
    init.position = Vec2{0.2, 0.2};
    init.theta = 0.0;

    double wheelbase = 0.2;

    MotionModel model{wheelbase};
    RangeSensor sensor{
        36,         // number of rays
        M_PI,       // field of view
        5.0         // max range
    };

    Robot robot(init, model, sensor);

    // -------------------------------------------------
    // Planner + simulator
    // -------------------------------------------------
    AStarPlanner planner;
    Vec2 goal{1.6, 1.6};

    Simulator sim(
        grid,
        robot,
        planner,
        goal,
        wheelbase
    );

    // -------------------------------------------------
    // Run simulation
    // -------------------------------------------------
    double dt = 0.05;
    int max_steps = 1000;
    int frame_interval = 10;

    sim.run(dt, max_steps, frame_interval);

    // -------------------------------------------------
    // Report
    // -------------------------------------------------
    Vec2 final_pos = robot.state().position;
    std::cout << "Final position: ("
              << final_pos.x << ", "
              << final_pos.y << ")\n";

    std::cout << "Wrote frame_*.svg and summary.svg\n";

    return 0;
}
