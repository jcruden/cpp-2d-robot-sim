#include "robot.hpp"

Robot::Robot(
    RobotState initial, 
    MotionModel model, 
    RangeSensor sensor
) : state_(initial),
    model_(model),
    sensor_(sensor) {}

void Robot::step(double vl, double vr, double dt) {
    model_.step(state_, vl, vr, dt);
}

RobotState const& Robot::state() const {
    return state_;
}

std::vector<Reading> Robot::sense(OccupancyGrid const& world) const {
    return sensor_.sense(world, state_);
}