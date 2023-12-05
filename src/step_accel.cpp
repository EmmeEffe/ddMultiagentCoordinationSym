/*
Give a step
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "utilities.h"

struct xyObject{
  double x = 0;
  double y = 0;
};

class StepAccel : public rclcpp::Node {
public:
  StepAccel() : Node("step_accel") {

    acc_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/robot1/cmd_acc", 10); // Publish new point movement data

    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&StepAccel::publishMessage, this));
    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger, ("Init time in sec: " + oldsec).c_str());
    #endif
  }

private:

  void publishMessage() {
    count++;
    std_msgs::msg::Float64MultiArray accel;
    accel.data.resize(2);

    if((count < 1000)||(count > 18000)){
        accel.data.at(0) = 0;
        accel.data.at(1) = 0;
    }else{
        accel.data.at(0) = 0;
        accel.data.at(1) = 1;
    }
    // Publish the message
    acc_publisher->publish(accel);

  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acc_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  int count = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StepAccel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
