/*
 * Model a mass dynamics concentrated in a point
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "utilities.h"
#include "secondOrderDynamics.h"


class PointMassDynamics : public rclcpp::Node {
public:
  PointMassDynamics() : Node("point_mass_dynamics") {

    input_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_acc", 10, std::bind(&PointMassDynamics::inputCallback, this, std::placeholders::_1)
    ); // Subscribe to input data

    // Subscribe also to target_pos

    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("point_nav_state", 10); // Publish new point movement data

    // Declare parameters
    this->declare_parameter("publish_rate", 0.01); 

    // Get the parameter
    this->get_parameter("publish_rate", publish_rate);

    sys = new secondOrderDynamics(publish_rate);

    // Create a timer to call the timerCallback function every 0.01 seconds
    timer = this->create_wall_timer(std::chrono::duration<double>(publish_rate), std::bind(&PointMassDynamics::timerCallback, this));
  }

private:
  void inputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    if (msg->data.size() == 2) {
        if(std::isnan(msg->data[0]) || std::isnan(msg->data[1])){
            RCLCPP_WARN(this->get_logger(), "Received NaN acceleration, setting zero acceleration at iteration %d", sys->getTick());
            msg->data[0] = 0;
            msg->data[1] = 0;
            sys->newTwistMessage(msg);
            return;
        }
        sys->newTwistMessage(msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Received array with incorrect size");
    }
  }

  void timerCallback() {
    odom_publisher->publish(sys->returnOdometry()); // publish odometry at 100Hz
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_subscription;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::TimerBase::SharedPtr timer;

  secondOrderDynamics *sys;
  double publish_rate;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointMassDynamics>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
