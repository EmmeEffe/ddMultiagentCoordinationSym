#include "secondOrderDynamics.h"
#include <cmath> // Include cmath library for std::isnan
#include <iostream> // Include iostream library for printing

secondOrderDynamics::secondOrderDynamics(double _timeInt)
{
    timeInt = _timeInt;
}

secondOrderDynamics::~secondOrderDynamics()
{

}

int secondOrderDynamics::tick(Point2d acc) // evolve the state
{
    //Update Pos
    State oldState = state;
    state.pos.x = state.pos.x + state.vel.x * timeInt; // s(t1) = s(t0) + v(t0) * DELTA t
    state.pos.y = state.pos.y + state.vel.y * timeInt; // s(t1) = s(t0) + v(t0) * DELTA t

    //Update Vel
    state.vel.x = state.vel.x + acc.x * timeInt;
    state.vel.y = state.vel.y + acc.y * timeInt;

    // Check if any variable becomes NaN
    if (std::isnan(state.pos.x) || std::isnan(state.pos.y) || std::isnan(state.vel.x) || std::isnan(state.vel.y)) {
        throw std::runtime_error("One or more variables became NaN.\n"
                                 "state.pos.x: " + std::to_string(state.pos.x) + "\n"
                                 "state.pos.y: " + std::to_string(state.pos.y) + "\n"
                                 "state.vel.x: " + std::to_string(state.vel.x) + "\n"
                                 "state.vel.y: " + std::to_string(state.vel.y) + "\n"
                                 "oldState.pos.x: " + std::to_string(oldState.pos.x) + "\n"
                                 "oldState.pos.y: " + std::to_string(oldState.pos.y) + "\n"
                                 "oldState.vel.x: " + std::to_string(oldState.vel.x) + "\n"
                                 "oldState.vel.y: " + std::to_string(oldState.vel.y) + "\n"
                                 "acc.x: " + std::to_string(acc.x) + "\n"
                                 "acc.y: " + std::to_string(acc.y) + "\n"
                                 "timeInt: " + std::to_string(timeInt));
    }

    state.count_tick++; // add tick
    return state.count_tick;
}


nav_msgs::msg::Odometry secondOrderDynamics::returnNewTwistMessage(std_msgs::msg::Float64MultiArray::SharedPtr acc)
{
    Point2d ptAcc;
    ptAcc.x = acc->data[0];
    ptAcc.y = acc->data[1];

    tick(ptAcc); // Update the system

    return returnOdometry();
}

nav_msgs::msg::Odometry secondOrderDynamics::returnOdometry(){ // point mass, no angular data, just linear
    nav_msgs::msg::Odometry msg;
    msg.pose.pose.position.x = state.pos.x;
    msg.pose.pose.position.y = state.pos.y;
    msg.pose.pose.position.z = 0;

    msg.twist.twist.linear.x = state.vel.x;
    msg.twist.twist.linear.y = state.vel.y;
    msg.twist.twist.linear.z = 0;

    return msg;
}