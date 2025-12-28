#include <iostream>
#include "util/math.hpp"
#include "sim/occupancy_grid.hpp"

int main() {
    Vec2 a{3.0, 4.0};
    Vec2 b{1.0, -2.0};

    Vec2 c = a + b;

    std::cout << "a = (" << a.x << ", " << a.y << ")\n";
    std::cout << "b = (" << b.x << ", " << b.y << ")\n";
    std::cout << "c = a + b = (" << c.x << ", " << c.y << ")\n";
    std::cout << "|a| = " << a.norm() << "\n";

    OccupancyGrid g{3, 3, 1.0};
    g.setOccupied(0, 1);
    bool first = g.isOccupied(0, 1);
    bool second = g.isOccupied(0, 2);

    std::cout << "first : " << first << "\n";
    std::cout << "second : " << second << "\n";

    return 0;
}