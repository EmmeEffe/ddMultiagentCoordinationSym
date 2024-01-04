#ifndef SECONDORDERDYNAMICS_H
#define SECONDORDERDYNAMICS_H

#pragma once

#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "utilities.h"

struct Point2d{
    double x = 0;
    double y = 0;
};

struct State{ // State of system
    Point2d pos;
    Point2d vel;
    int count_tick = 0;
};

class secondOrderDynamics
{
public:
    secondOrderDynamics(double timeInt, int index);
    ~secondOrderDynamics();

    nav_msgs::msg::Odometry returnNewTwistMessage(std_msgs::msg::Float64MultiArray::SharedPtr acc);
    int getTick(){return state.count_tick;};
    void newTwistMessage(std_msgs::msg::Float64MultiArray::SharedPtr acc);
    nav_msgs::msg::Odometry returnOdometry();

private:
    int tick(Point2d acc);
    void initializeRobotPosition(int index);
    State state;
    double timeInt;
};

#endif