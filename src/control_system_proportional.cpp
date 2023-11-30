/*
 * Control the system with a proportional error loop 

  - If i'm far from the point let the acceleration be constant and in direction of the point
 - If i'm closer than a threshold, let the acceleration be zero
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
  ControlSystemProportional() : Node("control_system_proportional") {

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
    this->declare_parameter("gain", 0.01); // Proportional gain

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

    //double tau = this->now().seconds() - oldsec; // Calculate time step (timestep may vary a bit)

    xyObject errorPos;
    errorPos.x = (targetPos.x - msg->pose.pose.position.x);
    errorPos.y = (targetPos.y - msg->pose.pose.position.y);

    double theta = atan2(errorPos.y, errorPos.x);

    float sqThresh = 0.3;
    float const_acc = 0.02;
    std_msgs::msg::Float64MultiArray accel;
    accel.data.resize(2);

    if(errorPos.x*errorPos.x + errorPos.y*errorPos.y<sqThresh){
      accel.data.at(0) = 0;
      accel.data.at(1) = 0;
    }else{
      accel.data.at(0) = const_acc * cos(theta);
      accel.data.at(1) = const_acc * sin(theta);
    }


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
