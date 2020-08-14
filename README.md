# Neuron Library ROS 2 Example

This is a neuron library ROS 2 example for ADLINK's products.

## Environment Requirement
* ROS version: 
  - Foxy
* Ubuntu version:
  - 18.04
  - 20.04
* Hardware support list:
  - ROSCube-I
  - ROSCube-X


## Setup
### Option 1: Install Neuron Lib first.
1. Install ROS 2
2. Download and install Neuron Lib 
```bash
cd ~/Downlaods
sudo apt install ./neuron-library-1.0.0-amd64.deb 
```
3. Create workspace and git clone this package from github
```bash 
# You can name your workspace.
mkdir -p neuronlib_example_ws/src
cd neuronlib_example_ws/src
git clone https://github.com/Adlink-ROS/neuron_library_example.git
cd ..
```
4. Build this ROS 2 package.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
5. Source package
```bash 
#Source the package everytime you open a new terminal
source install/local_setup.bash
```
### Option 2: 
  -Git clone the Libmraa yourself, and set the environment path before building (colcon build) this example.
1. Install ROS 2
2. Install and Build the mraa package
```bash
mkdir -p mraa_ws/src
cd mraa_ws/src
git clone https://github.com/Adlink-ROS/mraa.git -b roscube_series
cd mraa
mkdir build 
cd build
cmake ..
make 
cd ..
```
3. Set environment path of libmraa.
```bash
export NLIB_MRAA_LIBRARY_PATH=~/mraa_ws/src/mraa/build/src
export NLIB_MRAA_INCLUDE_PATH=~/mraa_ws/src/mraa/api
```
4. Create workspace and git clone this package from github
```bash 
# You can name your workspace.
mkdir -p neuronlib_example_ws/src
cd neuronlib_example_ws/src
git clone https://github.com/Adlink-ROS/neuron_library_example.git
cd ..
```
5. Build this ROS 2 package.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
6. Source package
```bash 
#Source the package everytime you open a new terminal
source install/local_setup.bash
```

## Examples
To view usage details, pleace click the link below. 
* [Uart control](/serial_example/README.md)
* [GPIO control](/gpio_example/README.md)
* [I2C control](/i2c_example/README.md)
* [LED control](/led_example/README.md)
