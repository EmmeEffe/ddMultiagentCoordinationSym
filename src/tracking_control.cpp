/*
 * Model a mass dynamics concentrated in a point
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "TrackingControl.h"
#include "utilities.h"


class TrackingControlNode : public rclcpp::Node {
public:
  TrackingControlNode() : Node("tracking_control") {

    _2ndOrder_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "point_nav_state", 10, std::bind(&TrackingControlNode::_2ndOrderCallback, this, std::placeholders::_1)
    );

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "point_nav_state", 10, std::bind(&TrackingControlNode::odomCallback, this, std::placeholders::_1)
    );

    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Declare parameters
    this->declare_parameter("publish_rate", 0.01);
    this->declare_parameter("k1", 1);
    this->declare_parameter("k2", 1);

    // Get the parameter
    this->get_parameter("publish_rate", publish_rate);
    this->get_parameter("k1", k1);
    this->get_parameter("k2", k2);

    track = new TrackingControl(publish_rate, k1, k2);
  }

private:
  void _2ndOrderCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    track->updateVirtualUni(msg);
    cmd_vel_publisher->publish(track->getCommandVel());
  }
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    track->updateRealUni(msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _2ndOrder_sub, odom_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

  TrackingControl *track;
  double publish_rate, k1, k2;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackingControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
