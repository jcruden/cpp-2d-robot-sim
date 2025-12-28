#include "../src/sim/occupancy_grid.hpp"
#include <cassert>

void test_grid() {
    OccupancyGrid grid(4, 3, 1.0);

    for (int i = 0; i < grid.height(); ++i) {
        for (int j = 0; j < grid.width(); ++j) {
            auto idx = grid.index(i, j);
            auto [ii, jj] = grid.coord(idx);

            assert(i == ii);
            assert(j == jj);
        }
    }
}
