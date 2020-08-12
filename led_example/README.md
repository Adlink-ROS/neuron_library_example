# I2C Port Example

## Package 'nsdk_example_i2c':

This package contains two executors: gpio_server and gpio_client.

1. Executor `led_control_service` controlls the i2c port by using mraa library, when `led_control_service` get the request from `led_control_service`, it will read or write the ROM connect through i2c port.  
  - Run the i2c_control_service of this package by using:
```bash
ros2 run nsdk_example_led led_control_service
```
2. Executor `led_client` calls a request to the `led_client`, and in the request contains 2 or 3 parameters. 
  - Run the gpio_server of this package by using.  
```bash
# Set the brightness of LED
# For ROSCube-I, LEDs can be set to on(value set 0) or off(value set 1).
ros2 run nsdk_example_led led_client S value
# Deinitialize the LED
ros2 run nsdk_example_led led_client C <led_num>
#Read the LED is on or off.
ros2 run nsdk_example_led led_client R <led_num>
```