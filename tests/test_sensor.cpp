#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "../src/sim/sensor.hpp"
#include <cmath>

TEST_CASE("Sensor single obstacle", "[sensor]") {
    OccupancyGrid grid(10, 10, 1.0);
    grid.setOccupied(7, 5);

    RobotState s;
    s.position = {5.0, 2.0};
    s.theta = M_PI / 2.0;

    RangeSensor sensor(M_PI / 2.0, 1, 10.0);
    auto readings = sensor.sense(grid, s);

    REQUIRE(readings.size() == 1);
    REQUIRE(readings[0].range == Catch::Approx(5.0).margin(1e-3));
}

