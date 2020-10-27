# GPIO Port Example

## Package 'nsdk_example_gpio':

This package contains two executors: gpio_server and gpio_client.

1. Executor `gpio_server` controls the gpio port by using mraa library, when `gpio_server` get the request from `gpio_client`, it will read or write the GPIO port.  
  - Run the gpio_server of this package by using:
```bash
ros2 run nsdk_example_gpio gpio_server
```
2. Executor `gpio_client` calls a request to the `gpio_server`, and in the request contains 2 or 3 parameters. 
  - Run the gpio_client of this package by using.  
```bash
# Set input to GPIO pin
ros2 run nsdk_example_gpio gpio_client I <gpio_num>
# Set output to GPIO pin
ros2 run nsdk_example_gpio gpio_client O <gpio_num> <value> 
```
  - **Remarks:** value 0 for high (5V), value 1 for low (0V)  
    - In the example for reading the input of pin number 1, you should run the client using:
    ```
    ros2 run nsdk_example_gpio gpio_client I 1 
    ```
    - For setting low voltage(0V) to the output of pin number 34, you should run the client using:
    ```
    ros2 run nsdk_example_gpio gpio_client O 34 1
    ```
