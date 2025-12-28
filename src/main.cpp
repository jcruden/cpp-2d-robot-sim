#include "sim/occupancy_grid.hpp"
#include "util/math.hpp"

#include <iostream>

int main() {
    // 50x50 grid, 0.1m res, default origin
    OccupancyGrid grid(50, 50, 0.1, Vec2{0.0, 0.0});

    // Add obstacles
    grid.addRectangle(Vec2{2.5, 2.5}, 2.0, 1.0);
    grid.addCircle(Vec2{1.5, 1.5}, 0.5);

    grid.writeSVG("map.svg");

    std::cout << "Wrote map.svg\n";
    return 0;
}