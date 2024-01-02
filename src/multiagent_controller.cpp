/*
 * Multiagent Controller Following the article
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "utilities.h"
#include "MultiAgentControl.h"


class MultiAgentControlNode : public rclcpp::Node {
public:
  MultiAgentControlNode() : Node("multi_agent_control") {

    RCLCPP_WARN(this->get_logger(), "Starting MultiAgentControlNode:");

    this->declare_parameter("num_robots", 12); // Number of robots

    // Get the parameters
    this->get_parameter("num_robots", num_robots);

  // Add Subscribers to target_pos and odom, pubishers to cmd_acc
  odom_subscribers.resize(num_robots);
  cmd_acc_publishers.resize(num_robots);
  trajectories_publishers.resize(num_robots);
  errors_publishers.resize(num_robots);
  x.resize(num_robots);
  x_received.resize(num_robots);

  for(int i=0; i<num_robots; i++){
      std::string topic_name = "robot" + std::to_string(i+1) + "/point_nav_state";
      odom_subscribers[i] = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name,
        10,
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odom_callback(msg, i);
        }
    );
  }

  for(int i=0; i<num_robots; i++){
      std::string topic_name = "robot" + std::to_string(i+1) + "/cmd_acc";
      cmd_acc_publishers[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 10);
      trajectories_publishers[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>("robot" + std::to_string(i+1) + "/desired_trajectories", 10);
      errors_publishers[i] = this->create_publisher<std_msgs::msg::Float64MultiArray>("robot" + std::to_string(i+1) + "/errors", 10);
  }

  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, int robot_id) {
    Eigen::Vector4d robot_pos(msg->pose.pose.position.x, msg->twist.twist.linear.x, msg->pose.pose.position.y, msg->twist.twist.linear.y);
    x[robot_id] = robot_pos;
    x_received[robot_id] = true;
    publish_acc();
  }


  std::string missing_robots_str;

  void publish_acc(){
    for(int i=0; i<num_robots; i++){
        if(!x_received[i]){
          if(!doneOnce){
            for (int i = 0; i < num_robots; i++) {
              if(!x_received[i])
                missing_robots_str += "robot" + std::to_string(i) + " ";
            }
            RCLCPP_WARN(this->get_logger(), "Robots %s are missing", missing_robots_str.c_str());
            missing_robots_str = "";
            return;
          }
        }
    }
    // Continue if only all have received
    if(!doneOnce){
      doneOnce = true;
      RCLCPP_INFO(this->get_logger(), "All targets received, starting the controller");
      mac = new MultiAgentControl();
      
      Eigen::MatrixXd W = mac->getWeight(0);
      std::stringstream matrix_values;
      for (int i = 0; i < W.rows(); i++) {
        for (int j = 0; j < W.cols(); j++) {
          matrix_values << W(i, j) << " ";
        }
        matrix_values << "\n";
      }
      RCLCPP_WARN(this->get_logger(), "Matrix values: %s", matrix_values.str().c_str());

    }
    for(int i=0; i<num_robots; i++){ // Clear the vector
        x_received[i] = false;
    }
    std::vector<Eigen::Vector4d> errors;
    std::vector<Eigen::Vector4d> calc_trajectories;
    std::vector<Vector2d> accels = mac->getControl(x, currentTime, calc_trajectories, errors);
    currentTime += timeStep;
    for(int i=0; i<num_robots; i++){
        std_msgs::msg::Float64MultiArray msg_acc;
        msg_acc.data = {accels[i](0), accels[i](1)};
        cmd_acc_publishers[i]->publish(msg_acc);

        std_msgs::msg::Float64MultiArray msg_traj;
        msg_traj.data = {calc_trajectories[i](0), calc_trajectories[i](1), calc_trajectories[i](2), calc_trajectories[i](3)};
        trajectories_publishers[i]->publish(msg_traj);

        std_msgs::msg::Float64MultiArray msg_errors;
        msg_errors.data = {errors[i](0), errors[i](1), errors[i](2), errors[i](3)};
        errors_publishers[i]->publish(msg_errors);
    }
  }

  // Subscribe to odom data, publish cmd_acc, trajectories and errors
  std::vector <rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers;

  std::vector <rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> cmd_acc_publishers;
  std::vector <rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> trajectories_publishers;
  std::vector <rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> errors_publishers;

  MultiAgentControl *mac;
  double publish_rate;
  int num_robots;
  std::vector<Eigen::Vector4d> x; // h is the target position, x is the robot position
  std::vector<bool> x_received;
  bool doneOnce = false;

  double currentTime = 0;
  double timeStep = 0.01;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiAgentControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
