/*
 * Convert the odom data from the center of mass to the position/velocity data of another point (forward at distance l)
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "utilities.h"

class ComToPtOdom : public rclcpp::Node {
public:
  ComToPtOdom() : Node("com_to_pt_odom") {

    // Subscribe to the odometry topic
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&ComToPtOdom::odomCallback, this, std::placeholders::_1)
        ); // Subscribe to odometer data


    // Advertise on the cmd_vel topic
    new_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("newpt_coordinates", 10); // Publish new point movement data

    // Declare parameters
    this->declare_parameter("dist_l", 0.5); // dist_l is the distance between p_tilde and p

    // Get the parameter
    this->get_parameter("dist_l", dist_l);

    oldsec = this->now().seconds(); // Initialize the time
    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger, ("Init time in sec: " + oldsec).c_str());
    #endif
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // IMPORTANT!!! Velocities data (twist) are in veichle reference frame
    // IMPORTANT!!! pose.orientation is a quaternion

    nav_msgs::msg::Odometry newMsg;
    double theta = quaternionToTheta(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    newMsg.pose.pose.orientation = msg->pose.pose.orientation; // Orientation stays the same
    newMsg.twist.twist.angular.z = msg->twist.twist.angular.z; //theta dot doesn't change

    // Remap Position in x, y
    newMsg.pose.pose.position.x = msg->pose.pose.position.x + dist_l * cos(theta);
    newMsg.pose.pose.position.y = msg->pose.pose.position.y + dist_l * sin(theta);

    // Remap Velocity in fixed frame
    Vector2d velocityTilde = unicycleToVelocity(msg->twist.twist.linear.x, msg->twist.twist.angular.z, theta, dist_l);
    newMsg.twist.twist.linear.x = velocityTilde(0);
    newMsg.twist.twist.linear.y = velocityTilde(1);

    // Publish the message
    new_odom_publisher->publish(newMsg);
  }


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr new_odom_publisher;

  float dist_l; // Distance between p and p tilde
  double oldsec;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ComToPtOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
