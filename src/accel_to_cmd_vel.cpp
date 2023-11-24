#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "utilities.h"

class AccelToCmdVel : public rclcpp::Node {
public:
  AccelToCmdVel() : Node("accel_to_cmd_vel") {
    // Subscribe to the odometry topic
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&AccelToCmdVel::odomCallback, this, std::placeholders::_1)
        ); // Subscribe to odometer data
    
    input_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_acc", 10, std::bind(&AccelToCmdVel::inputCallback, this, std::placeholders::_1)
    ); // Subscribe to input data

    // Advertise on the cmd_vel topic
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // Publish cmd_vel data

    // Declare parameters
    this->declare_parameter("dist_l", 0.5); // dist_l is the distance between p_tilde and p
    this->declare_parameter("publish_period", 0.1); // the frequency (period) in which data are published

    // Get the proportional gain parameter
    this->get_parameter("dist_l", dist_l);
    this->get_parameter("publish_period", tau);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract linear velocities from odometry message
    linear_velocity_x = msg->twist.twist.linear.x;
    linear_velocity_y = msg->twist.twist.linear.y;
    theta_orientation = msg->pose.pose.orientation.z;
  }

  void inputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    if (msg->data.size() == 2) {
      // Access the elements of the array
      double accel_x = msg->data[0];
      double accel_y = msg->data[1];

      // Estimate velocity. It is calculate by integration between the current vel and the input acceleration, so it should not accumulate errors
      double est_vel_x = linear_velocity_x + accel_x * tau;
      double est_vel_y = linear_velocity_y + accel_y * tau;

      // Remap Velocity of p tilde point to unicycle equivalent
      Eigen::Vector2d unicycle = velocityToUnicycle(est_vel_x, est_vel_y, dist_l, theta_orientation);

      // Create and publish the control command on cmd_vel topic
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel_msg->linear.x = unicycle(0);
      cmd_vel_msg->angular.z = unicycle(1);
      cmd_vel_publisher_->publish(std::move(cmd_vel_msg));

    } else {
      RCLCPP_WARN(this->get_logger(), "Received array with incorrect size");
    }
  }

  // Variables with parameters
  double linear_velocity_x = 0;
  double linear_velocity_y = 0;
  double theta_orientation = 0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_subscription;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  float dist_l; // Distance between p and p tilde
  double tau;   // Sample Time of the system
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AccelToCmdVel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}