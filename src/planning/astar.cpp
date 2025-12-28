#include "astar.hpp"
#include <cassert>
#include <cmath>
#include <algorithm>
#include <queue>

namespace {
    struct Node {
        int i; // row
        int j; // col
        int g; // cost from start
        int f; // g + heuristic
    };

    struct NodeCompare {
        bool operator()(Node const& a, Node const& b) const {
            // priority queue puts max f first, want smallest cost
            return a.f > b.f;
        }
    };

    using OpenSet = std::priority_queue<Node, std::vector<Node>, NodeCompare>;
}

static constexpr int di[4] = { -1, 1, 0, 0 };
static constexpr int dj[4] = { 0, 0, -1, 1 };

// manhattan distance for 4 connected graph
static int heuristic(int i0, int j0, int i1, int j1) {
    return std::abs(i0 - i1) + std::abs(j0 - j1);
}

// i = y = row, j = x = col
std::vector<Vec2> AStarPlanner::plan(
    OccupancyGrid const& grid,
    Vec2 const& start,
    Vec2 const& goal)
{
    // initialize containers
    OpenSet open;
    CostMap curr_cost;
    ParentMap parent;

    // get coordinates
    int height = grid.height();
    int width = grid.width();
    auto [starti, startj] = grid.worldToGrid(start);
    auto [goali, goalj] = grid.worldToGrid(goal);
    Index goal_ind = grid.index(goali, goalj);
    Index start_ind = grid.index(starti, startj);

    // initialize first node
    int g = 0;
    int f = heuristic(starti, startj, goali, goalj);
    curr_cost[start_ind] = g;
    Node start_node = {starti, startj, g, f};
    open.push(start_node);

    // A star algorithm
    while (!open.empty()) {
        Node curr = open.top();
        open.pop();
        int row = curr.i;
        int col = curr.j;
        Index curr_ind = grid.index(row, col);

        if (curr_ind == goal_ind) {
            break;
        }

        for (int k = 0; k < 4; k++) {
            int new_row = row + di[k];
            int new_col = col + dj[k];
            // check bounds
            if (new_row >= 0 && new_row < height && new_col >= 0 && new_col < width) {
                // check not occupied
                if (!grid.isOccupied(new_row, new_col)) {
                    Index neighbor_ind = grid.index(new_row, new_col);
                    int neighbor_g = curr.g + 1;
                    // check if node exists
                    auto it = curr_cost.find(neighbor_ind);
                    // update neighbor and add to open
                    if (it == curr_cost.end() || neighbor_g < curr_cost[neighbor_ind]) {
                        curr_cost[neighbor_ind] = neighbor_g;
                        parent[neighbor_ind] = curr_ind;
                        int f = neighbor_g + heuristic(new_row, new_col, goali, goalj);
                        Node neighbor{new_row, new_col, neighbor_g, f};
                        open.push(neighbor);
                    }
                }
            }
        }
    }

    std::vector<Vec2> path;

    Index curr = goal_ind;
    while (curr != start_ind) {
        auto [i, j] = grid.coord(curr);
        path.push_back(Vec2 {
            static_cast<double>(j),
            static_cast<double>(i)
        });
        if (parent.find(curr) == parent.end()) {
            return {};
        }
        curr = parent[curr];
    }
    auto [i, j] = grid.coord(start_ind);
    path.push_back(Vec2 {
        static_cast<double>(j),
        static_cast<double>(i)
    });

    std::reverse(path.begin(), path.end());

    return path;
}