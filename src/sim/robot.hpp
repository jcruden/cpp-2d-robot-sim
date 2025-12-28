#pragma once

#include "robot_state.hpp"
#include "motion_model.hpp"
#include "sensor.hpp"
#include "occupancy_grid.hpp"

class Robot {
public:
    Robot(
        RobotState initial, 
        MotionModel model, 
        RangeSensor sensor
    );

    void step(double vl, double vr, double dt);
    RobotState const& state() const;
    std::vector<Reading> sense(OccupancyGrid const& world) const;

private:
    RobotState state_;
    MotionModel model_;
    RangeSensor sensor_;
};