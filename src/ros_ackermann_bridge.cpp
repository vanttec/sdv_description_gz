#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <functional>
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace std::chrono_literals;

class RosAckermannBridge : public rclcpp::Node {
public:
  RosAckermannBridge() : Node("ros_ackermann_bridge"){
    using namespace std::placeholders;

    steerSub = this->create_subscription<std_msgs::msg::Float32>(
        "/sdc_control/control_signal/delta", 10,
        [this](const std_msgs::msg::Float32 &msg) {
          this->delta.data = std::clamp((double)msg.data, -1.28, 1.28);
        });
    velSub = this->create_subscription<std_msgs::msg::Float32>(
        "/sdc_control/setpoint/velocity", 10,
        [this](const std_msgs::msg::Float32 &msg) {this->velocity_setpoint.data = msg.data * 10 / 3;});


    leftDPub = this->create_publisher<std_msgs::msg::Float64>("/delta_l", 10);
    rightDPub = this->create_publisher<std_msgs::msg::Float64>("/delta_r", 10);
    wheelSpeedPub = this->create_publisher<std_msgs::msg::Float64>("/wheel_speed", 10);

    updateTimer = this->create_wall_timer(10ms, std::bind(&RosAckermannBridge::update, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leftDPub, rightDPub, wheelSpeedPub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steerSub, velSub;

  std_msgs::msg::Float64 left, right, velocity_setpoint;
  std_msgs::msg::Float32 delta;

  bool autonomous_on{false};

  rclcpp::TimerBase::SharedPtr updateTimer;

  float l{2.11}; // length of the car from the rear wheels to the front wheels (in gz's sdf)
  float r; // radius of the turn
  float w_2{0.625}; // half of the car's width (in gz's sdf)

  void update() {
    r = l / tan(this->delta.data);
    right.data = atan(l/(r - w_2));
    left.data = atan(l/(r + w_2));

    if(right.data > M_PI)
      right.data-=2*M_PI;
    if(left.data > M_PI)
      left.data-=2*M_PI;
    

    leftDPub->publish(left);
    rightDPub->publish(right);
    wheelSpeedPub->publish(velocity_setpoint);
  }
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosAckermannBridge>());
  rclcpp::shutdown();
  return 0;
}