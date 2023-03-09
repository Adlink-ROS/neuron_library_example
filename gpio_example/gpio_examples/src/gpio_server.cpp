// Copyright 2020 ADLINK
// Name: gpio_server.cpp
// Author: ChenYing Kuo

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "neuronlib_interfaces/srv/gpio_service.hpp"

// MRAA
#include "mraa/gpio.h"

#define GPIO_PIN_NUM 50

namespace gpio_server_node
{

class GpioServerNode : public rclcpp::Node
{
public:
/**/
    explicit GpioServerNode(const rclcpp::NodeOptions & options)
    : Node("gpio_server", options)
    {

        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto handle_gpio_service =
          [this](const std::shared_ptr<rmw_request_id_t> request_header,
           const std::shared_ptr<neuronlib_interfaces::srv::GpioService::Request> request,
            std::shared_ptr<neuronlib_interfaces::srv::GpioService::Response> response)->void         
        {

            (void)request_header;
            // Get initial value
            response->result = 0;
            response->ret_val = 0;
            // Print value received
	    if (request->direction == 'I')
	    {
		RCLCPP_INFO(this->get_logger(), "Incoming request\ndirection:%c, gpio_num:%d" ,
                        request->direction, request->gpio_num);
	    }
	    else
	    {
            	RCLCPP_INFO(this->get_logger(), "Incoming request\ndirection:%c, gpio_num:%d, set_value:%d" ,
                        request->direction, request->gpio_num, request->value);
	    }
            
            
            if (gpio_pin[request->gpio_num] == NULL)
            {
                gpio_pin[request->gpio_num] = mraa_gpio_init(request->gpio_num);
   	    	std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // add enough delay before controlling the GPIO pin
            }
            if (gpio_pin[request->gpio_num] == NULL)
            {
                RCLCPP_ERROR(this->get_logger(), "Fail to initialize GPIO %d", request->gpio_num);
            }
        
            // Direction
            if (mraa_gpio_dir(gpio_pin[request->gpio_num], (request->direction=='I')?MRAA_GPIO_IN:MRAA_GPIO_OUT) != MRAA_SUCCESS) //[request->gpio_num]
            {
                RCLCPP_ERROR(this->get_logger(), "Set GPIO %d direction error", request->gpio_num);
                response->result = 1;
                return;
            }
            if (request->direction == 'O') 
            {
                if (mraa_gpio_write(gpio_pin[request->gpio_num], (request->value == 0)?0:1) != MRAA_SUCCESS) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Set GPIO %d value error", request->gpio_num);
                    response->result = 1;
                    return;
                }
            } 
            else 
            {
                int pin_val = mraa_gpio_read(gpio_pin[request->gpio_num]);
                if (pin_val == -1)
                {
                    RCLCPP_ERROR(this->get_logger(), "Get GPIO %d value error", request->gpio_num);
                    response->result = 1;
                    return;
                }
                response->ret_val = pin_val;
            }

        };

        init_neuronlib();
         // Create a service that will use the callback function to handle requests. The service's name is neuronlib_gpio.
        srv_ = create_service<neuronlib_interfaces::srv::GpioService>("neuronlib_gpio", handle_gpio_service);
    }

private:
    void init_neuronlib() 
    {   
        mraa_init();
    }

    mraa_gpio_context gpio_pin[GPIO_PIN_NUM];
    rclcpp::Service<neuronlib_interfaces::srv::GpioService>::SharedPtr srv_;
};

}  // namespace gpio_server_node

RCLCPP_COMPONENTS_REGISTER_NODE(gpio_server_node::GpioServerNode)
