# Neuron Library Example

## Serial Port

### Package 'examples_mraa_uart_controller':

This package will create a node named 'uart_mraa_controller', and this node has two functions:

1. This node will subscribe to a topic 'message_to_uart' and will write the message to uart port. 
2. This node will listen to the uart port every 500ms and will publish the message from the uart to the topic 'message_from_uart'.

```
ros2 run nsdk_example_serial mraa_uart_controller
```

### Package 'examples_data_publisher'

This package create a node named 'data_publisher', and this node publishes data every 500ms to a topic 'message_to_uart'.  

```
ros2 run nsdk_example_serial data_publisher 
```

### Package 'examples_uart_message_subscriber'

This package create a node named 'message_subscriber', and this node subscirbes to topic 'message_from_uart', and show the message from the uart on the screen.  

```
ros2 run nsdk_example_serial uart_message_subscriber 
```

### *Remarks*

1. Pleace install mraa for ROScube-I from [Adlink-ROS-mraa](https://github.com/Adlink-ROS/mraa.git).
2. You should change the directory in /examples_mraa_uart_controller/CMakeLists.txt line 18 & 19 to the directory where the mraa is installed. This will be fixed in the future.
