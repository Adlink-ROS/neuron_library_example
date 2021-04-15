# LED Port Example

## Package 'nsdk_example_led':

This package contains two executors: led_control_service and led_client.

1. Executor `led_control_service` controls the LED by using mraa library, when `led_control_service` get the request from `led_control_service`, it will read or write the ROM connect through LED.  
  - Run the led_control_service of this package by using:
```bash
ros2 run nsdk_example_led led_control_service
```
2. Executor `led_client` calls a request to the `led_client`, and in the request contains 2 or 3 parameters. 
  - Run the led_client of this package by using.  
```bash
# Set the brightness of LED
# For ROSCube-I, LEDs can be set to on(value set 1) or off(value set 0).
# For ROSCube-X, LEDs can be set to on(value set from 1 to 255) or off(value set 0).
ros2 run nsdk_example_led led_client S <led_num> value
#Read the the value of LED.
ros2 run nsdk_example_led led_client R <led_num>
```
