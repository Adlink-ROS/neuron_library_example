# I2C Port Example

## Package 'nsdk_example_i2c':

This package contains two executors: i2c_control_service and i2c_client.

1. Executor `i2c_control_service` controls the i2c port by using mraa library, when `i2c_control_service` get the request from `i2c_control_service`, it will read or write the ROM connect through i2c port.  
  - Run the i2c_control_service of this package by using:
```bash
ros2 run nsdk_example_i2c i2c_control_service
```
2. Executor `i2c_client` calls a request to the `i2c_client`, and in the request contains 2 or 3 parameters. 
  - Run the i2c_server of this package by using.  
```bash
# Write the ROM through i2c
ros2 run nsdk_example_i2c i2c_client W <i2c_bus_num> <i2c_device_address> <i2c_rom_address> value
# Read the ROM through i2c
ros2 run nsdk_example_i2c i2c_client R <i2c_bus_num> <i2c_device_address> <i2c_rom_address> 
```
  - **Remarks:** Pleace check which i2c hub and address your device is connected to.  
    - If you want to write a value:12 to the ROM at ic2 number:0, device address 0x50, ROM address 0x01, pleace excute the client as below:
    ```
    ros2 run nsdk_example_i2c i2c_client W 0 0x50 0x01 12 
    ```
    - If you want to read the value on the ROM at ic2 number 0, device address 0x50, ROM address 0x02, pleace excute the client as below:
    ```
    ros2 run nsdk_example_i2c i2c_client R 0 0x50 0x02 
    ```