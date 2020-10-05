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

#include <inttypes.h>
#include <memory>
#include <string>
#include "neuronlib_led_interfaces/srv/led_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mraa/led.h"

using ledservice = neuronlib_led_interfaces::srv::LedService;
rclcpp::Node::SharedPtr g_node = nullptr;

int handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<ledservice::Request> request,
  const std::shared_ptr<ledservice::Response> response)
{
    (void)request_header;
    mraa_led_context led;
    mraa_result_t status = MRAA_SUCCESS;
    int val;
    led = mraa_led_init(request->led_num);
    RCLCPP_INFO(g_node->get_logger(),"LED num: %d",request->led_num);
    if (led == NULL) 
    {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to initialize LED%d",request->led_num);
        mraa_deinit();
        response->ret_status = 1;   //return status fail
        return 1;
    }

    if (request->led_action == "S" )
    {
        status = mraa_led_set_brightness(led,request->led_val);
        if (status != MRAA_SUCCESS) 
        {
            RCLCPP_ERROR(g_node->get_logger(), "Failed to set LED%d brightness.",request->led_num);
            response->ret_status= 1 ;   //return status fail
            return 1;
        }
        response->ret_status = 0;  //return status success
    }

    else
    {
        val = mraa_led_read_brightness(led);
        response->ret_status = 0;  //return status success
        response->ret_val = val;
    }
    mraa_led_close(led);
    return 1; 
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    mraa_init();
    g_node = rclcpp::Node::make_shared("led_control");
    auto server = g_node->create_service<ledservice>("led_status", handle_service);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}

