#include <catch2/catch_test_macros.hpp>
#include "../src/sim/robot_state.hpp"
#include "../src/sim/motion_model.hpp"
#include "../src/control/path_follower.hpp"
#include "../src/util/math.hpp"
#include <cmath>

TEST_CASE("Closed loop path following", "[closed_loop]") {
    RobotState state;
    state.position = Vec2{0, 0};
    state.theta = 0;

    PathFollower follower(0.5);
    MotionModel model(0.5);

    Vec2 goal{2, 0};

    const double dt = 0.01;
    const double epsilon = 0.05;

    for (int i = 0; i < 2000; ++i) {
        auto [vl, vr] = follower.computeControl(state, goal);
        model.step(state, vl, vr, dt);
    }

    REQUIRE(state.position.distance(goal) < epsilon);
    REQUIRE(std::abs(wrapToPi(state.theta)) < 0.1);
}

