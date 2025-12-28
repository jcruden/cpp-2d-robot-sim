#include "../src/planning/astar.hpp"
#include <cassert>
#include <iostream>

void test_astar_straight() {
    OccupancyGrid grid(5, 5, 1.0);
    AStarPlanner planner;

    Vec2 start{0, 0};
    Vec2 goal{4, 0};

    auto path = planner.plan(grid, start, goal);

    assert(!path.empty());
    assert(path.front().x == 0);
    assert(path.front().y == 0);
    assert(path.back().x == 4);
    assert(path.back().y == 0);
}

void test_astar_obstacle() {
    OccupancyGrid grid(5, 5, 1.0);
    AStarPlanner planner;

    // Block middle
    grid.setOccupied(0, 2, true);
    grid.setOccupied(1, 2, true);
    grid.setOccupied(2, 2, true);

    Vec2 start{0, 0};
    Vec2 goal{4, 0};

    auto path = planner.plan(grid, start, goal);
    assert(!path.empty());

    // Ensure path does not go through obstacles
    for (auto const& p : path) {
        int row = static_cast<int>(p.y);
        int col = static_cast<int>(p.x);
        assert(!grid.isOccupied(row, col));
    }
}

void test_astar_no_path() {
    OccupancyGrid grid(3, 3, 1.0);
    AStarPlanner planner;

    // Block row 1
    for (int j = 0; j < 3; ++j) {
        grid.setOccupied(1, j, true);
    }

    Vec2 start{0, 0};
    Vec2 goal{2, 2};

    auto path = planner.plan(grid, start, goal);
    assert(path.empty());
}

int main() {
    test_astar_straight();
    test_astar_obstacle();
    test_astar_no_path();
    std::cout << "\nAll astar tests passed.\n";
    return 0;
}