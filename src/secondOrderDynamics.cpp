#include "secondOrderDynamics.h"

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
    state.pos.x = state.pos.x + state.vel.x * timeInt; // s(t1) = s(t0) + v(t0) * DELTA t
    state.pos.y = state.pos.y + state.vel.y * timeInt; // s(t1) = s(t0) + v(t0) * DELTA t

    //Update Vel
    state.vel.x = state.vel.x + acc.x * timeInt; 
    state.vel.y = state.vel.y + acc.y * timeInt; 

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