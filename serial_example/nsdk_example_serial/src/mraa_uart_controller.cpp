// Copyright 2020 ADLINK Technology, Inc.
// Developer:  'chih-chieh.chang@adlinktech.com'

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
const char* dev_path = "/dev/ttyttyS1";
mraa_uart_context uart;
int baudrate = 9600, stopbits = 1, databits = 8;
mraa_uart_parity_t parity = MRAA_UART_PARITY_NONE;
unsigned int ctsrts = FALSE, xonxoff = FALSE;
const char* name = NULL;

char record[256] ;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("uart_mraa_controller")
  {
    uart_init();
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "message_to_uart", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("message_from_uart", 11);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  
  int uart_init()
  {
    mraa_result_t status = MRAA_SUCCESS;

    /* initialize mraa for the platform (not needed most of the time) */
    mraa_init();

    //! [Interesting]
    /* initialize uart*/ 
    uart = mraa_uart_init_raw(dev_path);
    if (uart == NULL) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize UART\n");
      return EXIT_FAILURE;
    } 
    /* set serial port parameters */
    status =
    mraa_uart_settings(-1, &dev_path, &name, &baudrate, &databits, &stopbits, &parity, &ctsrts, &xonxoff);
    if (status != MRAA_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set serial port parameters UART\n");
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I sended: '%s'", msg->data.c_str());
    mraa_uart_write(uart, msg->data.c_str(), sizeof(msg->data.c_str()));
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void timer_callback()
  { 
    if (mraa_uart_data_available(uart,50))
    {
      int len = mraa_uart_read(uart,record,sizeof(record));
      record[len] = 0;
      RCLCPP_INFO(this->get_logger(), "I heard form uart: '%s'", record);
      auto msg_ = std::make_unique<std_msgs::msg::String>();
      msg_->data = record;
      publisher_->publish(std::move(msg_));
    }

  }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
