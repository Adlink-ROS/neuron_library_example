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

#include <memory>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <chrono>
using namespace std::chrono_literals;

/*ROS header*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/*mraa header*/
#include "mraa/uart.h"

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif

/* UART port name */
const char* dev_path = "/dev/ttyS1";
mraa_uart_context uart;
volatile sig_atomic_t flag = 1;

char record[8] ;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    uart_init();
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic1", 11);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  
  int uart_init()
{
    mraa_result_t status = MRAA_SUCCESS;

    int baudrate = 9600, stopbits = 1, databits = 8;
    mraa_uart_parity_t parity = MRAA_UART_PARITY_NONE;
    unsigned int ctsrts = FALSE, xonxoff = FALSE;
    const char* name = NULL;

    /* install signal handler */
  //  signal(SIGINT, sig_handler);
    /* initialize mraa for the platform (not needed most of the time) */
    mraa_init();

    //! [Interesting]
    /* initialize uart*/ 
    uart = mraa_uart_init_raw(dev_path);
    if (uart == NULL) {
        RCLCPP_ERROR(this-> get_logger(), "Failed to initialize UART\n");
        return EXIT_FAILURE;
    } 

    /* set serial port parameters */
    status =
    mraa_uart_settings(-1, &dev_path, &name, &baudrate, &databits, &stopbits, &parity, &ctsrts, &xonxoff);
    if (status != MRAA_SUCCESS) {
   //     goto err_exit;
    }

    /* stop uart */
    //mraa_uart_stop(uart);

    //! [Interesting]
    /* deinitialize mraa for the platform (nuart_init();

    /* stop uart */
    //mraa_uart_stop(uart);

    /* deinitialize mraa for the platform (not needed most of the times) */
    //mraa_deinit();

    //return EXIT_FAILURE;
    return EXIT_SUCCESS;

  }
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I sended: '%s'", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "size: '%x'", sizeof(msg->data.c_str()));
    mraa_uart_write(uart, msg->data.c_str(), sizeof(msg->data.c_str()));
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void timer_callback()
  {

    //record = nullptr;
    
    if (mraa_uart_data_available(uart,50)){
    mraa_uart_read(uart,record,8);
    RCLCPP_INFO(this->get_logger(), "I heard form uart: '%s'", record);
    //publisher_->publish(*message);
    }

  }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}