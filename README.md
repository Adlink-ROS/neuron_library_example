# Neuron Library ROS 2 Example

This is a neuron library ROS 2 example for ADLINK's products.
You can easily rewrite your program based on these examples.

## Environment Requirement
* ROS version: 
  - ROS 2 Foxy
* Ubuntu version:
  - 18.04
  - 20.04
* Hardware support list:
  - ROSCube-I
  - ROSCube-X

## Setup
### Option 1: Install Neuron Library
If you use Neuron SDK, then Neuron Library should be built-in. Otherwise, you can download Neuron Library [here](https://github.com/Adlink-ROS/mraa/releases)

You can use the example directly.

Please contact ADLINK if you have any question about Neuorn Library.

1. Install ROS 2
2. Create workspace and git clone this package from github
```bash 
# You can name your workspace.
mkdir -p neuronlib_example_ws/src
cd neuronlib_example_ws/src
git clone https://github.com/Adlink-ROS/neuron_library_example.git
cd ..
```
3. Build this ROS 2 package.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
4. Source package
```bash 
#Source the package everytime you open a new terminal
source install/local_setup.bash
```

### Option 2: Build MRAA by yourself
If you don't have Neuron Library, then git clone MRAA Library and build by yourself.
The environmental variables need to be set before building (colcon build) this example.

1. Install ROS 2
2. Install and Build the mraa package
```bash
mkdir -p ~/mraa_ws/src
cd ~/mraa_ws/src
git clone https://github.com/Adlink-ROS/mraa.git -b roscube_series
cd mraa
mkdir build 
cd build
cmake ..
make 
```
3. Set environmental variables of MRAA Library.
```bash
# Your library path
export NLIB_MRAA_LIBRARY_PATH=~/mraa_ws/src/mraa/build/src
# Your header path
export NLIB_MRAA_INCLUDE_PATH=~/mraa_ws/src/mraa/api
```
4. Create workspace and git clone this package from github
```bash 
# You can name your workspace.
mkdir -p ~/neuronlib_example_ws/src
cd ~/neuronlib_example_ws/src
git clone https://github.com/Adlink-ROS/neuron_library_example.git
cd ..
```
5. Build this ROS 2 package.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
6. Source package
```bash 
# Source the package everytime you open a new terminal
source install/local_setup.bash
```

## Examples
To view the usage of examples more detailed, pleace click the links below. 

* [Uart control](/serial_example/README.md)
* [GPIO control](/gpio_example/README.md)
* [I2C control](/i2c_example/README.md)
* [LED control](/led_example/README.md)
* [PWM control](/pwm_example/README.md)
* [SPI control](/spi_example/README.md)
