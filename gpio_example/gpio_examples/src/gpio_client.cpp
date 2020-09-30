// Copyright 2020 ADLINK
// Name: gpio_server.cpp
// Author: ChenYing Kuo

#include "rclcpp/rclcpp.hpp"

#include "neuronlib_interfaces/srv/gpio_service.hpp"

using namespace std::chrono_literals;  // used by 1s

neuronlib_interfaces::srv::GpioService::Response::SharedPtr send_request(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<neuronlib_interfaces::srv::GpioService>::SharedPtr client,
    neuronlib_interfaces::srv::GpioService::Request::SharedPtr request)
{
    auto result = client->async_send_request(request);
    // Waiting
    if (rclcpp::spin_until_future_complete(node, result) == 
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get();
    } else {
        return NULL;
    }
}

int main(int argc, char *argv[]) {
    // Read the parameter
    if (argc < 3) {
        std::cout << "Usage: <gpio_client> I <gpio_num>" << std::endl;;
        std::cout << "       <gpio_client> O <gpio_num> value" << std::endl;
        return -1;
    }
    int gpio_num = std::stoi(argv[2]);
    char gpio_dir = argv[1][0];
    int gpio_val = 0;
    if (gpio_dir == 'O') {
        if (argc < 4) {
            std::cout << "Usage: <gpio_client> O <gpio_num> value" << std::endl;
            return -1;
        } else {
            gpio_val = std::stoi(argv[3]);
        }
    }

    rclcpp::init(argc, argv);

    // Node name
    auto node = rclcpp::Node::make_shared("gpio_client");

    // Create a client to query service name "neuronlib_gpio"
    auto client = node->create_client<neuronlib_interfaces::srv::GpioService>("neuronlib_gpio");

    // Build the request
    auto request = std::make_shared<neuronlib_interfaces::srv::GpioService::Request>();
    request->direction = gpio_dir;
    request->gpio_num = gpio_num;
    request->value = gpio_val;

    // Waiting until the service is up
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Catch interrupt and stop the program!");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service...");
    }

    // Send the request
    auto result = send_request(node, client, request);
    if (result) {
        RCLCPP_INFO(node->get_logger(), "result=%d, get_val=%d", result->result, result->ret_val);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Catch interrupt and stop the program!");
    }
    rclcpp::shutdown();
    return 0;

}