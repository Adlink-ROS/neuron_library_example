// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <memory>
#include "neuronlib_i2c_interfaces/srv/i2c_service.hpp"
#include "rclcpp/rclcpp.hpp"

using i2cservice = neuronlib_i2c_interfaces::srv::I2cService;

int main(int argc, char * argv[])
{
    if (argc < 5) 
    {
        std::cout << "Usage: <i2c_client> R <i2c_bus_num> <i2c_device_address> <i2c_rom_address>" << std::endl;;
        std::cout << "       <i2c_client> W <i2c_bus_num> <i2c_device_address> <i2c_rom_address> value" << std::endl;
        return -1;
    }
    int32_t i2c_bus_num = std::stoi(argv[2],0,0);
    char i2c_act = argv[1][0];
    int32_t i2c_dev_addr = std::stoi(argv[3],0,0);
    int32_t i2c_rom_addr = std::stoi(argv[4],0,0);
    int8_t data_val = 0;
    if (i2c_act == 'W') 
    {
        if (argc < 6) 
        {
            std::cout << "Usage: <i2c_client> W <i2c_bus_num> <i2c_device_address> <i2c_rom_address> value" << std::endl;
            return -1;
        }
        else 
        {
            data_val = std::stoi(argv[5],0,0);
        }
    }

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("i2c_client");
    auto client = node->create_client<i2cservice>("i2c_msg");
    while (!client->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for i2c_control service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for i2c_control service to appear...");
    }
    auto request = std::make_shared<i2cservice::Request>();
    request-> i2c_action  = i2c_act;
    request-> i2c_bus_num = i2c_bus_num;
    request-> dev_addr    = i2c_dev_addr;
    request-> rom_addr    = i2c_rom_addr;
    request-> data_val    = data_val;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed.");
        return 1;
    }
    auto result = result_future.get();
    if (result->w_result == "SUCCESS")
    {
        RCLCPP_INFO(node->get_logger(),"Write to ROM Success!");
    }
    else if (result->w_result == "ERROR")
    {
        RCLCPP_ERROR(node->get_logger(),"Failed to write ROM!");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),"Read from ROM: %d" , result-> data_read);
    }

    rclcpp::shutdown();
    return 0;
}