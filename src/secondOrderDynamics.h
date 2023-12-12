#ifndef SECONDORDERDYNAMICS_H
#define SECONDORDERDYNAMICS_H

#pragma once

#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
    secondOrderDynamics(double timeInt);
    ~secondOrderDynamics();

    nav_msgs::msg::Odometry returnNewTwistMessage(std_msgs::msg::Float64MultiArray::SharedPtr acc);


private:
    int tick(Point2d acc);
    nav_msgs::msg::Odometry returnOdometry();
    State state;
    double timeInt;
};

#endif