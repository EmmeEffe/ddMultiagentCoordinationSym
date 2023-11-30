/*
 * Control the system with a proportional error loop in position
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "utilities.h"

struct xyObject{
  double x = 0;
  double y = 0;
};

class ControlSystemProportional : public rclcpp::Node {
public:
  ControlSystemProportional() : Node("control_system_proprortional") {

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
    this->declare_parameter("gain", 0.5); // Proportional gain

    // Get the parameter
    this->get_parameter("gain", gain);

    oldsec = this->now().seconds(); // Initialize the time
    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger, ("Init time in sec: " + oldsec).c_str());
    #endif
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // IMPORTANT!!! Velocities data (twist) are in veichle reference frame
    // IMPORTANT!!! pose.orientation is a quaternion

    double tau = this->now().seconds() - oldsec; // Calculate time step (timestep may vary a bit)

    xyObject errorVel;
    errorVel.x = gain*(targetPos.x - msg->pose.pose.position.x);
    errorVel.y = gain*(targetPos.y - msg->pose.pose.position.y);

    // Now derive errorvel to get accel
    // I Know that this method of doing derivatives can lead to high frequency noise in the signal, but i want to keep this node as simple as possible, as it serve just as a simple "check"

    std_msgs::msg::Float64MultiArray accel;
    accel.data.resize(2);
    accel.data.at(0) = (errorVel.x-oldvel.x)/tau;
    accel.data.at(1) = (errorVel.y-oldvel.y)/tau;
    
    oldvel = errorVel;

    // Publish the message
    acc_publisher->publish(accel);
  }

  void targetCallback(const std_msgs::msg::Float64MultiArray msg){ // Update Target Position
    targetPos.x = msg.data.at(0);
    targetPos.y = msg.data.at(1);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_subscription;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acc_publisher;

  float gain; // Distance between p and p tilde
  double oldsec;
  xyObject targetPos, oldvel;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlSystemProportional>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
