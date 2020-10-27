#include <chrono>
#include <memory>
#include<ctype.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using namespace std::chrono_literals;
double period_msg;
double duty_cycle_msg;

class PWMPublisher : public rclcpp::Node
{
public:
  PWMPublisher()
  : Node("pwm_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pwm_control_msg", 10 );
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&PWMPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float64MultiArray();
    message.data.resize(2);
    message.data[0] = period_msg;
    message.data[1] = duty_cycle_msg/100;
    RCLCPP_INFO(this->get_logger(), "Publishing: Period = %lf, Duty Cycle = %lf", message.data[0],message.data[1]);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (argc!=3||isalpha(*argv[1])||isalpha(*argv[2]))
  {
    std::cout << "Usage: ros2 run nsdk_example_pwm pwm_publisher <Period of PWM> <Duty Cycle(in percentage)>" << std::endl;;
    return 0;
  }
  period_msg = atof(argv[1]);
  duty_cycle_msg = atof(argv[2]);
  rclcpp::spin(std::make_shared<PWMPublisher>());
  rclcpp::shutdown();
  return 0;
}
