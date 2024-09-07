#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
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

    steerSub = this->create_subscription<std_msgs::msg::Float64>(
        "/sdv/steering/delta_setpoint", 10,
        [this](const std_msgs::msg::Float64 &msg) {
          delta.data = std::clamp((double)msg.data, -1.28, 1.28);
        });
    velSub = this->create_subscription<std_msgs::msg::Float64>(
        "/sdv/velocity/throttle", 10,
        [this](const std_msgs::msg::Float64 &msg) {
          throttle_cmd = msg.data;
        });


    leftDPub = this->create_publisher<std_msgs::msg::Float64>("/delta_l", 10);
    rightDPub = this->create_publisher<std_msgs::msg::Float64>("/delta_r", 10);
    wheelSpeedPub = this->create_publisher<std_msgs::msg::Float64>("/wheel_speed", 10);

    updateTimer = this->create_wall_timer(10ms, std::bind(&RosAckermannBridge::update, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leftDPub, rightDPub, wheelSpeedPub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steerSub, velSub;

  std_msgs::msg::Float64 left, right, velocity_setpoint;
  std_msgs::msg::Float64 delta;

  bool autonomous_on{false};
  double dt{0.01};
  double throttle_cmd{0.}, last_throttle_cmd{0.};

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
    
    velocity_setpoint.data = calc_increment(velocity_setpoint.data, throttle_cmd, last_throttle_cmd);
    last_throttle_cmd = throttle_cmd;

    leftDPub->publish(left);
    rightDPub->publish(right);
    wheelSpeedPub->publish(velocity_setpoint);
  }

  double sign(double n){
    if(n >= 0)
      return 1;
    return -1;
  }

  double calc_increment(double current_speed, double throttle, double last_throttle){
    if(current_speed < 0.05 && throttle < 0.)
      return 0.;
    double x = std::clamp(-std::log(0.1*current_speed) / 0.3, 0.001, 14.);
    if(last_throttle_cmd > 0.){
      x = 14. - x;
    }
    double xd = 3*std::exp(-0.3 * x);
    if(throttle < 0.)
      xd *= 0.5;

    std::cout << "x: " << x << " || current speed: " << current_speed << " || incrementing by " << xd << std::endl;

    return std::clamp(current_speed + sign(throttle)*xd*0.1, 0., 10.);
  }
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosAckermannBridge>());
  rclcpp::shutdown();
  return 0;
}