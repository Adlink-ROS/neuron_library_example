#include <memory>
#include <stdio.h>
#include <string> 

/* ros2 header */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

/* mraa header */
#include "mraa/pwm.h"

/* PWM declaration */
#define PWM 22 //Since ROSCube-X has only one pwm pin, we set to that pin directly.
mraa_result_t status = MRAA_SUCCESS;
mraa_pwm_context pwm;

using std::placeholders::_1;
int pwm_init()
{
  mraa_init();
  pwm = mraa_pwm_init(PWM);
  if (pwm == NULL) 
  {
    fprintf(stderr, "Failed to initialize PWM\n");
    mraa_deinit();
    return EXIT_FAILURE;
  }
  return MRAA_SUCCESS;
}

void pwm_deinit()
{
  mraa_pwm_close(pwm);
  mraa_deinit();
}

class PWMSubscriber : public rclcpp::Node
{
public:
  PWMSubscriber()
  : Node("message_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "pwm_control_msg", 10, std::bind(&PWMSubscriber::topic_callback, this, _1));
  }

private:
  int topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
  {
      double output;
      double pwm_period = msg->data[0];
      double pwm_duty_cycle = msg->data[1];

      /* enable PWM */
      status = mraa_pwm_enable(pwm, 1);
      if (status != MRAA_SUCCESS) 
      {
        RCLCPP_ERROR(this->get_logger(), "Can't enable this pwm pin.");
        return 1;
      }
      status = mraa_pwm_period_us(pwm, pwm_period);
      if (status != MRAA_SUCCESS) 
      {
        RCLCPP_ERROR(this->get_logger(), "Can't set period value of pwm.");
        return 1;
      }

      /* write PWM duty cyle */
      status = mraa_pwm_write(pwm, pwm_duty_cycle);
      if (status != MRAA_SUCCESS) 
      {
        RCLCPP_ERROR(this->get_logger(), "Can't set duty cycle value of pwm.");
        return 1;
      }
      /* read PWM duty cyle */
      output = mraa_pwm_read(pwm);
      RCLCPP_INFO(this->get_logger(), "PWM value is %lf\n",output);
      return 1;
  }
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  pwm_init();
  rclcpp::spin(std::make_shared<PWMSubscriber>());
  pwm_deinit();
  rclcpp::shutdown();
  return 0;
}
