# LED Port Example

## Package 'nsdk_example_led':

This package contains two executors: led_control_service and led_client.

1. Executor `led_control_service` controlls the LED by using mraa library, when `led_control_service` get the request from `led_control_service`, it will read or write the ROM connect through LED.  
  - Run the led_control_service of this package by using:
```bash
ros2 run nsdk_example_led led_control_service
```
2. Executor `led_client` calls a request to the `led_client`, and in the request contains 2 or 3 parameters. 
  - Run the gpio_server of this package by using.  
```bash
# Set the brightness of LED
# For ROSCube-I, LEDs can be set to on(value set 0) or off(value set 1).
ros2 run nsdk_example_led led_client S value
#Read the LED is on or off.
ros2 run nsdk_example_led led_client R <led_num>
```