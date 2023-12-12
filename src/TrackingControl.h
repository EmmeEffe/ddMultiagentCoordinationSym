#ifndef TRACKINGCONTROL_H
#define TRACKINGCONTROL_H

#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "utilities.h"

#define DERIV_FILTER_SIZE 100

struct Point2d{
    double x = 0;
    double y = 0;
};

struct State{
    Point2d pos;
    double theta = 0;
};



class TrackingControl
{
public:
    TrackingControl(double timeStep, double k1, double k2);
    ~TrackingControl();
    void updateVirtualUni(nav_msgs::msg::Odometry::SharedPtr msg);
    void updateRealUni(nav_msgs::msg::Odometry::SharedPtr);
    geometry_msgs::msg::Twist getCommandVel();

private:
    double calculateThetaDot();
    void addRecordForDerivative(double newRecord);
    
    State virtualUni, realUni;
    double desired_vel = 0; // Velocity of virtual unicycle
    struct Params{double _1 = 0;double _2 = 0;}k;

    std::vector<double> buffer; // Circular Array
    int currentIndex = 0;       // Used for circular array

    double timeStep;
};

#endif