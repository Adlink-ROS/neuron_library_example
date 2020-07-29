# Neuron Library Example

## GPIO Port

### Package 'nsdk_example_gpio':

This package contains two executors: gpio_server and gpio_client.

1. Executor 'gpio_server' controlls the gpio port by using mraa library, when 'gpio_server' get the request from 
```
Run the gpio_server of this package by using *ros2 run neuronlib_gpio gpio_server *   
```
2.Executor 'gpio_client' calls a request to the 'gpio_server', and in the the request contains 2 or 3 parameters. 
```
Run the gpio_server of this package by using *ros2 run neuronlib_gpio gpio_client <PARAMETERS> *  
```
Replace the <PARAMETERS> using : I <gpio_num>  
       			       : O <gpio_num> value      _Remarks: value 0 for high (5V), value 1 for low (0V)_  
	-In the example for reading the input of pin number 1, you should run the client using:*ros2 run neuronlib_gpio gpio_client I 1* 
	-For setting low voltage(0V) to the output of pin number 34, you should run the client using:*ros2 run neuronlib_gpio gpio_client O 34 1*  
