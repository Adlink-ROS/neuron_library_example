// Copyright 2020 ADLINK Technology, Inc.
// Developer:  'chih-chieh.chang@adlinktech.com'

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
#include "neuronlib_led_interfaces/srv/led_service.hpp"
#include "rclcpp/rclcpp.hpp"

using ledservice = neuronlib_led_interfaces::srv::LedService;

int main(int argc, char * argv[])
{
    if (argc < 3) 
    {
        std::cout << "Usage: <led_client> S <led_num> <value>" << std::endl;;
        std::cout << "       <led_client> C <led_num> " << std::endl;
        std::cout << "       <led_client> R <led_num> " << std::endl;        
        return -1;
    }

    char led_act = argv[1][0];
    int8_t led_num = std::stoi(argv[2]);
    int8_t led_val;
    if (led_act == 'S') 
    {
        if (argc < 4) 
        {
            std::cout << "Usage: <led_client> S <led_num> <value>" << std::endl;
            return -1;
        }
        else 
        {
            led_val = std::stoi(argv[3]);
        }
    }

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("led_client");
    auto client = node->create_client<ledservice>("led_status");
    while (!client->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for led_control service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for led_control service to appear...");
    }
    auto request = std::make_shared<ledservice::Request>();
    request-> led_action  = led_act;
    request -> led_num = led_num;
    request-> led_val    = led_val;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed.");
        return 1;
    }
    auto result = result_future.get();
    if (result->r_status == "BRIGHT")
    {
        RCLCPP_INFO(node->get_logger(),"This LED is on.");
    }
    else if (result->r_status == "DARK")
    {
        RCLCPP_INFO(node->get_logger(),"This LED is off.");
    }
    else if (result->r_status == "not_read")
    {
        RCLCPP_INFO(node->get_logger(),"LED set successfully."  );
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(),"Failed! Read the message at service node.");
    }

    rclcpp::shutdown();
    return 0;
}