# PWM Port Example

## Package 'nsdk_example_spi':

This package contains two executors: spi_control_subscriber and spi_publisher. This example transforms hex code through SPI to control a chip MAX7219, and this chp contorls an 8x8 LED Matrix. You could translate your image to hex code using this tool on this wedsite: [DotMatrixTool](dotmatrixtool.com).

1. Executor `spi_control_subscriber` controlls the SPI by using mraa library, when `spi_control_subscriber` get the message from `spi_publisher`, it will send hex code through SPI to the MAX7219, and controls the LED Matrix.  
  - Run the pwm_control_subscriber of this package by using:
```bash
 ros2 run nsdk_example_spi spi_control_subscriber 
```
2. Executor `spi_publisher` publishes one message to the topic `/spi_control_msg` every time you execute it , and in the message contains every hex you send to the program. Composition of the hex data: 0xaabb, aa is for the value you want to set to the address, bb is for the address. Pleace refer to register map of MAX7219 for the mapping odf the address.
  - Run the pwn_publisher of this package by using.  
```bash
ros2 run nsdk_example_spi spi_publisher <LED setup message1(hex)> <LED setup message2(hex)>...
#*Remarks:you should initialize the LED matrix by setting up addresses 0x09, 0x0a, 0x0b, 0x0c, 0x0F before sending data of you image.
ros2 run nsdk_example_spi spi_publisher 0x0009 0x070a 0x070b 0x010c 0x000f  #Example of publishing initial message.
#For example, we could display an arrow by using:
ros2 run nsdk_example_spi spi_publisher 0x1801 0x3c02 0x7e03 0xff04 0x3c05 0x3c06 0x3c07 0x3c08
```
### Register Map Of MAX7219
* For controlling the display, several registers of MAX7219 need to be set correctly. In the example of this tutorial, six types of registers will baccessed:

  |Name / Functionality | Address |                       Description                       |
  | :-----------------: | :-----: | :-----------------------------------------------------: |
  |    Decode Mode      |  0x09   |              Turn on / off the decode mode              |
  |     Intensity       |  0x0A   |           Adjust the brightness of the display          |
  |    Scan Limit       |	 0x0B   |	         Determine how many digits are enabled          |
  |     Shutdown        |	 0x0C   |	           Turn on / off the display module             |
  |   Display Test      |	 0x0F   | Enter the test mode (all LED segments light on) or not  |
  |Digit 0~ Digit 7 data|0x01~0x08|	       The address to control digit 0 ~ digit 7         |
