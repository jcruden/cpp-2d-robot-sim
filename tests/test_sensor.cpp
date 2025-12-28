#include <cassert>
#include <iostream>
#include "../src/sim/sensor.hpp"

void test_single_obstacle() {
    OccupancyGrid grid(10, 10, 1.0);
    grid.setOccupied(7, 5);

    RobotState s;
    s.position = {5.0, 2.0};
    s.theta = M_PI / 2.0; // facing +y

    RangeSensor sensor(M_PI / 2.0, 1, 10.0);
    auto readings = sensor.sense(grid, s);

    assert(readings.size() == 1);
    assert(std::abs(readings[0].range - 5.0) < 1e-3);

    std::cout << "test_single_obstacle OK\n";
}

int main() {
    test_single_obstacle();
    std::cout << "All sensor tests passed.\n";
}
