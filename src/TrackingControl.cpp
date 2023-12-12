#include "TrackingControl.h"

TrackingControl::TrackingControl(double timeStep, double k1, double k2)
{
    k._1 = k1;
    k._2 = k2;
    timeStep = timeStep;
    buffer = std::vector<double>(DERIV_FILTER_SIZE, 0.0); // Initialize the vector
}

TrackingControl::~TrackingControl()
{

}

void TrackingControl::updateVirtualUni(nav_msgs::msg::Odometry::SharedPtr msg)
{
    virtualUni.pos.x = msg->pose.pose.position.x;
    virtualUni.pos.y = msg->pose.pose.position.y;
    double vel_x = msg->twist.twist.linear.x;
    double vel_y = msg->twist.twist.linear.y;
    virtualUni.theta = std::atan2(vel_y, vel_x);
    desired_vel = std::sqrt((vel_x*vel_x)+(vel_y*vel_y));
    addRecordForDerivative(virtualUni.theta);
}

void TrackingControl::updateRealUni(nav_msgs::msg::Odometry::SharedPtr msg)
{
    realUni.pos.x = msg->pose.pose.position.x;
    realUni.pos.y = msg->pose.pose.position.y;
    realUni.theta = quaternionToTheta(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

geometry_msgs::msg::Twist TrackingControl::getCommandVel()
{
    geometry_msgs::msg::Twist cmd_vel;

    Vector3d errorPos = Vector3d(virtualUni.pos.x - realUni.pos.x, virtualUni.pos.y - realUni.pos.y, 0);

    double e1 = xVehicleVersor(realUni.theta).dot(errorPos);
    double e2 = yVehicleVersor(realUni.theta).dot(errorPos);
    double e3 = virtualUni.theta - realUni.theta;

    double zeroThreshold = 0.3; // 30cm
    if((e1*e1+e2*e2)<=zeroThreshold*zeroThreshold){ // If so close to target, stop
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        return cmd_vel;
    }

    double desired_thetadot = calculateThetaDot();

    double cmdUni_vel = desired_vel * std::cos(e3) + k._1 * e1;
    double cmdUni_rot = desired_thetadot + k._2 * desired_vel * e2 + std::sin(e3);

    cmd_vel.linear.x = saturate(cmdUni_vel, 1.0);
    cmd_vel.angular.z = saturate(cmdUni_rot, 2.0); // Saturate signal

    return cmd_vel;
}

double TrackingControl::calculateThetaDot()
{
    return (buffer[currentIndex] - buffer[(currentIndex+1)%DERIV_FILTER_SIZE])/((DERIV_FILTER_SIZE-1) * timeStep);
}

void TrackingControl::addRecordForDerivative(double newRecord) // Add new record in calculateThetaDot list
{
    buffer[currentIndex] = newRecord;
    currentIndex = (currentIndex+1) % DERIV_FILTER_SIZE;
}