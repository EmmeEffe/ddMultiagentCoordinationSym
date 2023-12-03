/*
 * Control the system with a proportional error loop 

  - If i'm far from the point let the acceleration be constant and in direction of the point
 - If i'm closer than a threshold, let the acceleration be zero
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "utilities.h"

struct xyObject{
  double x = 0;
  double y = 0;
};

double clockToSeconds(rosgraph_msgs::msg::Clock::SharedPtr msg){ // Return the current time in seconds
    double time = (double)msg->clock.sec;
    time+=(double) msg->clock.nanosec * 1.0e-9;
    return time;
}

class ControlSystemProportional : public rclcpp::Node {
public:
  ControlSystemProportional() : Node("control_system_proportional") {    
    // Subscribe to Clock topic
    clock_subscription = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&ControlSystemProportional::timeTick, this, std::placeholders::_1)); // Subscribe to clock data

    // Subscribe to the odometry topic
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "newpt_coordinates", 10, std::bind(&ControlSystemProportional::odomCallback, this, std::placeholders::_1)
        ); // Subscribe to odometer data

    target_pos_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "target_pos", 10, std::bind(&ControlSystemProportional::targetCallback, this, std::placeholders::_1)
        ); // Subscribe to odometer data


    // Subscribe also to target_pos

    acc_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_acc", 10); // Publish new point movement data

    // Declare parameters
    this->declare_parameter("vel_gain", 0.1); // Proportional gain
    this->declare_parameter("acc_gain", 1.0); // Proportional gain
    this->declare_parameter("max_vel", 5.0); // Maximum Velocity
    this->declare_parameter("max_acc", 10.0); // Maximum Acceleration

    // Get the parameter
    this->get_parameter("vel_gain", vel_gain);
    this->get_parameter("acc_gain", acc_gain);
    this->get_parameter("max_vel", max_vel);
    this->get_parameter("max_acc", max_acc);

    oldsec = this->now().seconds(); // Initialize the time
    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger, ("Init time in sec: " + oldsec).c_str());
    #endif
  }

private:
  void timeTick(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    // Time ticked
    //double tau = clockToSeconds(msg)-oldSec; // Tau in seconds
    oldSec = clockToSeconds(msg);

    if(oldSec<=5)
      return; // Inizia il controllo dopo 5 secondi

    xyObject errorPos, desiredVel, errorVel;
    errorPos.x = (targetPos.x - currentPos.x);
    errorPos.y = (targetPos.y - currentPos.y);

    desiredVel.x = std::min(vel_gain * errorPos.x, (double) max_vel);
    desiredVel.y = std::min(vel_gain * errorPos.y, (double) max_vel);

    errorVel.x = desiredVel.x - vel.x;
    errorVel.y = desiredVel.y - vel.y;

    //double theta = atan2(errorPos.y, errorPos.x);

    std_msgs::msg::Float64MultiArray accel;
    accel.data.resize(2);
    accel.data.at(0) = std::min(acc_gain * errorVel.x, (double) max_acc);
    accel.data.at(1) = std::min(acc_gain * errorVel.y, (double) max_acc);
    
    // Publish the message
    acc_publisher->publish(accel);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // IMPORTANT!!! Velocities data (twist) are in veichle reference frame
    // IMPORTANT!!! pose.orientation is a quaternion

    //double tau = this->now().seconds() - oldsec; // Calculate time step (timestep may vary a bit)
    currentPos.x = msg->pose.pose.position.x;
    currentPos.y = msg->pose.pose.position.y;
    
    vel.x = msg->twist.twist.linear.x;
    vel.y = msg->twist.twist.linear.y;
  }

  void targetCallback(const std_msgs::msg::Float64MultiArray msg){ // Update Target Position
    targetPos.x = msg.data.at(0);
    targetPos.y = msg.data.at(1);
  }

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription;
  double oldSec = 0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_subscription;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acc_publisher;

  float vel_gain, acc_gain, max_vel, max_acc; 
  double oldsec;
  xyObject targetPos, vel, currentPos;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlSystemProportional>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
