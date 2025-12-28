#include <catch2/catch_test_macros.hpp>
#include "../src/planning/astar.hpp"

TEST_CASE("AStar straight path", "[astar]") {
    OccupancyGrid grid(5, 5, 1.0);
    AStarPlanner planner;

    Vec2 start{0, 0};
    Vec2 goal{4, 0};

    auto path = planner.plan(grid, start, goal);

    REQUIRE(!path.empty());
    REQUIRE(path.front().x == 0);
    REQUIRE(path.front().y == 0);
    REQUIRE(path.back().x == 4);
    REQUIRE(path.back().y == 0);
}

TEST_CASE("AStar obstacle avoidance", "[astar]") {
    OccupancyGrid grid(5, 5, 1.0);
    AStarPlanner planner;

    grid.setOccupied(0, 2, true);
    grid.setOccupied(1, 2, true);
    grid.setOccupied(2, 2, true);

    Vec2 start{0, 0};
    Vec2 goal{4, 0};

    auto path = planner.plan(grid, start, goal);
    REQUIRE(!path.empty());

    for (auto const& p : path) {
        int row = static_cast<int>(p.y);
        int col = static_cast<int>(p.x);
        REQUIRE_FALSE(grid.isOccupied(row, col));
    }
}

TEST_CASE("AStar no path", "[astar]") {
    OccupancyGrid grid(3, 3, 1.0);
    AStarPlanner planner;

    for (int j = 0; j < 3; ++j) {
        grid.setOccupied(1, j, true);
    }

    Vec2 start{0, 0};
    Vec2 goal{2, 2};

    auto path = planner.plan(grid, start, goal);
    REQUIRE(path.empty());
}
