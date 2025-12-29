#include <catch2/catch_test_macros.hpp>

#include "../src/sim/simulator.hpp"
#include "../src/sim/occupancy_grid.hpp"
#include "../src/sim/robot.hpp"
#include "../src/planning/astar.hpp"
#include "../src/util/math.hpp"

TEST_CASE("Simulator drives robot to goal in empty world") {
    OccupancyGrid grid(20, 20, 0.1, Vec2{0.0, 0.0});

    double wheelbase = 0.2;

    RobotState init;
    init.position = Vec2{0.0, 0.0};
    init.theta = 0.0;

    MotionModel model{wheelbase};
    RangeSensor sensor{3, 1.5, 10};
    Robot robot(init, model, sensor);

    AStarPlanner planner;
    Vec2 goal{1.0, 0.0};
    Simulator sim(grid, robot, planner, goal, wheelbase);

    // --- Run ---
    double dt = 0.05;
    int max_steps = 500;
    sim.run(dt, max_steps);

    // --- Verify ---
    Vec2 final_pos = robot.state().position;
    double dist_to_goal = final_pos.distance(goal);

    REQUIRE(dist_to_goal < 0.05);
}
