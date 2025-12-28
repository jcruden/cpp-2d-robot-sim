#pragma once

#include <vector>
#include <unordered_map>
#include "../sim/occupancy_grid.hpp"
#include "../util/math.hpp"

class AStarPlanner {
public:
    // Plan path from start to goal
    std::vector<Vec2> plan(
        OccupancyGrid const& grid,
        Vec2 const& start,
        Vec2 const& goal
    );
};

using Index = int;
// cost map of grid index, g cost
using CostMap = std::unordered_map<Index, int>;
// map current cell to parent cell by indices
using ParentMap = std::unordered_map<Index, Index>;
