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
#include "neuronlib_i2c_interfaces/srv/i2c_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mraa/i2c.h"

using i2cservice = neuronlib_i2c_interfaces::srv::I2cService;
rclcpp::Node::SharedPtr g_node = nullptr;

int handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<i2cservice::Request> request,
  const std::shared_ptr<i2cservice::Response> response)
{
    (void)request_header;
    mraa_result_t status = MRAA_SUCCESS;
    mraa_i2c_context i2c;
    i2c = mraa_i2c_init(request->i2c_bus_num);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // add enough delay before access
    if (i2c == NULL) 
    {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to initialize I2C\n");
        mraa_deinit();
        return 1;
    }
    status = mraa_i2c_address(i2c, request->dev_addr);
    if (status != MRAA_SUCCESS) 
    {
        return 1;
    }
    if (request->i2c_action == "R" )
    {
        response->data_read  = mraa_i2c_read_byte_data(i2c,request->rom_addr);
        RCLCPP_INFO(g_node->get_logger(),"get %d", response->data_read);
        response->w_result = "not_write";
        return 1;
    }
    else
    {
        status = mraa_i2c_write_byte_data(i2c,request->data_val,request->rom_addr);
        if (status != MRAA_SUCCESS)
        {
            response->w_result = "ERROR";
            return 1;
        }
        response->w_result= "SUCCESS";
    }
    return 1; 
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    mraa_init();
    g_node = rclcpp::Node::make_shared("i2c_control");
    auto server = g_node->create_service<i2cservice>("i2c_msg", handle_service);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}

