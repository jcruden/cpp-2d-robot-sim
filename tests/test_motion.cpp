#include "../src/sim/robot_state.hpp"
#include "../src/sim/motion_model.hpp"

#include <cmath>
#include <cassert>
#include <iostream>

void test_straight_line() {
    RobotState s;
    s.position = Vec2{0.0, 0.0};
    s.theta = 0.0;            // facing +x
    s.v_left = 1.0;          // m/s
    s.v_right = 1.0;         // m/s

    MotionModel model(0.5);   // wheel_base = 0.5 m
    model.step(s, 1.0);       // dt = 1s

    // Expect x = 1.0, y = 0.0, theta unchanged
    assert(std::abs(s.position.x - 1.0) < 1e-6);
    assert(std::abs(s.position.y - 0.0) < 1e-6);
    assert(std::abs(s.theta - 0.0) < 1e-6);

    std::cout << "test_straight_line OK\n";
}

double angleDiff(double a, double b) {
    double d = std::fmod(a - b + M_PI, 2 * M_PI);
    if (d < 0) d += 2 * M_PI;
    return d - M_PI;
}

void test_rotate_in_place() {
    RobotState s;
    s.position = Vec2{0.0, 0.0};
    s.theta = 0.0;
    s.v_left = -1.0;
    s.v_right = 1.0;

    double L = 0.5;
    MotionModel model(L);
    model.step(s, 1.0); // dt = 1s

    double expected_omega = 2.0 / L;
    double expected_theta = expected_omega * 1.0;

    assert(std::abs(s.position.x - 0.0) < 1e-6);
    assert(std::abs(s.position.y - 0.0) < 1e-6);

    double err = angleDiff(s.theta, expected_theta);
    assert(std::abs(err) < 1e-6);

    std::cout << "test_rotate_in_place OK\n";
}

void test_zero_velocity() {
    RobotState s;
    s.position = Vec2{0.5, -0.2};
    s.theta = 1.23;
    s.v_left = 0.0;
    s.v_right = 0.0;

    MotionModel model(0.5);
    model.step(s, 0.5);

    assert(std::abs(s.position.x - 0.5) < 1e-9);
    assert(std::abs(s.position.y - (-0.2)) < 1e-9);
    assert(std::abs(s.theta - 1.23) < 1e-9);

    std::cout << "test_zero_velocity OK\n";
}

int main() {
    test_straight_line();
    test_rotate_in_place();
    test_zero_velocity();
    std::cout << "\nAll motion tests passed.\n";
    return 0;
}
