#include <catch2/catch_test_macros.hpp>
#include "../src/sim/occupancy_grid.hpp"

TEST_CASE("Grid coordinate conversion", "[grid]") {
    OccupancyGrid grid(4, 3, 1.0);

    for (int i = 0; i < grid.height(); ++i) {
        for (int j = 0; j < grid.width(); ++j) {
            auto idx = grid.index(i, j);
            auto [ii, jj] = grid.coord(idx);

            REQUIRE(i == ii);
            REQUIRE(j == jj);
        }
    }
}
