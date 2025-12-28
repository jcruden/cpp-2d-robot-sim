#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "../src/sim/robot_state.hpp"
#include "../src/sim/motion_model.hpp"
#include <cmath>

TEST_CASE("Motion model straight line", "[motion]") {
    RobotState s;
    s.position = Vec2{0.0, 0.0};
    s.theta = 0.0;
    s.v_left = 1.0;
    s.v_right = 1.0;

    MotionModel model(0.5);
    model.step(s, s.v_left, s.v_right, 1.0);

    REQUIRE(s.position.x == Catch::Approx(1.0).margin(1e-6));
    REQUIRE(s.position.y == Catch::Approx(0.0).margin(1e-6));
    REQUIRE(s.theta == Catch::Approx(0.0).margin(1e-6));
}

static double angleDiff(double a, double b) {
    double d = std::fmod(a - b + M_PI, 2 * M_PI);
    if (d < 0) d += 2 * M_PI;
    return d - M_PI;
}

TEST_CASE("Motion model rotate in place", "[motion]") {
    RobotState s;
    s.position = Vec2{0.0, 0.0};
    s.theta = 0.0;
    s.v_left = -1.0;
    s.v_right = 1.0;

    double L = 0.5;
    MotionModel model(L);
    model.step(s, s.v_left, s.v_right, 1.0);

    double expected_omega = 2.0 / L;
    double expected_theta = expected_omega * 1.0;

    REQUIRE(s.position.x == Catch::Approx(0.0).margin(1e-6));
    REQUIRE(s.position.y == Catch::Approx(0.0).margin(1e-6));

    double err = angleDiff(s.theta, expected_theta);
    REQUIRE(std::abs(err) < 1e-6);
}

TEST_CASE("Motion model zero velocity", "[motion]") {
    RobotState s;
    s.position = Vec2{0.5, -0.2};
    s.theta = 1.23;
    s.v_left = 0.0;
    s.v_right = 0.0;

    MotionModel model(0.5);
    model.step(s, s.v_left, s.v_right, 0.5);

    REQUIRE(s.position.x == Catch::Approx(0.5).margin(1e-9));
    REQUIRE(s.position.y == Catch::Approx(-0.2).margin(1e-9));
    REQUIRE(s.theta == Catch::Approx(1.23).margin(1e-9));
}

