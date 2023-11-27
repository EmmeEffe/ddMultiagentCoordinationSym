/*
 * Program used to create the target positions curves
 * In this case i create a formation of robots positioned on the vertices of a regular polygon
 * The polygon can rotate around it center at a certain speed
 * TODO: Edit this to accept also time function's targets
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "utilities.h"
#include "targetPositionGenerator.h"

class CreateTargetPositions : public rclcpp::Node {
public:
  CreateTargetPositions() : Node("create_target_positions") {
    // Declare parameters
    this->declare_parameter("num_robots", 2); // Number of robots
    this->declare_parameter("center_x", 0.0); // center of the formation
    this->declare_parameter("center_y", 0.0); // center of the formation
    this->declare_parameter("start_angle", 0.0); // start angle of the formation
    this->declare_parameter("radius", 1.5); // radius of the formation

    // Get the parameters
    this->get_parameter("num_robots", num_robots);
    this->get_parameter("center_x", center_x);
    this->get_parameter("center_y", center_y);
    this->get_parameter("start_angle", start_angle);
    this->get_parameter("radius", radius);

    // Debug prints
    RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
    RCLCPP_INFO(this->get_logger(), "num_robots: %d", num_robots);
    RCLCPP_INFO(this->get_logger(), "center_x: %f", center_x);
    RCLCPP_INFO(this->get_logger(), "center_y: %f", center_y);
    RCLCPP_INFO(this->get_logger(), "start_angle: %f", start_angle);
    RCLCPP_INFO(this->get_logger(), "radius: %f", radius);

    targetGen = new TargetPositionGenerator(TargetPositionGenerator::generationTypeConstantAroundACircle); // Initialize the target position generator
    targetGen->setRobotNumbers(num_robots);
    targetGen->setParam(param_center_x, center_x);
    targetGen->setParam(param_center_y, center_y);
    targetGen->setParam(param_formation_radius, radius);
    targetGen->setParam(param_start_angle, start_angle);

    // Resize the publisher array. Size has not been allocated since this moment, then create the publishers

    publishers.resize(num_robots);
    std::string topic_name;

    for(int i=0; i<num_robots; i++){
      topic_name = "robot" + std::to_string(i+1) + "/target_pos";
      publishers[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 10);
    }

    clock_subscription = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&CreateTargetPositions::timeTick, this, std::placeholders::_1)); // Subscribe to clock data
  }

private:
  void timeTick(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
      if(msg->clock.sec!=oldSec){ // Tick every second
        oldSec = msg->clock.sec;
        std::vector<Vector2d> Points = targetGen->getFormation(oldSec);
        auto pos_msg = std_msgs::msg::Float64MultiArray();
        pos_msg.data.resize(2);
        for(int i=0; i<num_robots; i++){ // Publish all the messages
          pos_msg.data[0] = Points[i](0);
          pos_msg.data[1] = Points[i](1);

          publishers[i]->publish(pos_msg); // Publish the message
        }
      }
    }

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> publishers; // Create the vector containing all the publishers' pointers
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription;

  // Parameters from YAML
  int num_robots;
  int32_t oldSec = 0;
  float center_x, center_y, start_angle, radius;
  TargetPositionGenerator * targetGen;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CreateTargetPositions>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
