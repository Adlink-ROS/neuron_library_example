# PWM Port Example

## Package 'nsdk_example_pwm':

This package contains two executors: pwm_control_subscriber and pwm_publisher.

1. Executor `pwm_control_subscriber` controlls the PWM by using mraa library, when `pwm_control_subscriber` get the message from `pwm_publisher`, it will set the PWM signal including duty cycle and period.  
  - Run the pwm_control_subscriber of this package by using:
```bash
 ros2 run nsdk_example_pwm pwm_control_subscriber
```
2. Executor `pwm_publisher` publishes messages every 1 second to the topic `/pwm_control_msg`, and in the message contains 2 parameters,period and duty cycle. 
  - Run the pwn_publisher of this package by using.  
```bash
ros2 run nsdk_example_pwm pwm_publisher <Period of PWM(in microsecond)> <Duty Cycle(in percentage)>
#For example, you want to set the period of pwm to 50000us and duty cycle to 50%, you should run this node by using:
ros2 run nsdk_example_pwm pwm_publisher 50000 50
```
