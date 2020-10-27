#include <memory>
#include <stdio.h>
#include <string> 

/* ros2 header */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

/* mraa header */
#include "mraa/spi.h"

/* SPI declaration */
#define SPI_BUS 0 //Since ROSCube-X has only one spi, we set to that pin directly.
#define SPI_FREQ 100000  
mraa_result_t status = MRAA_SUCCESS;
mraa_spi_context spi;

using std::placeholders::_1;
int spi_init()
{
  mraa_init();
  spi = mraa_spi_init(SPI_BUS);
  if (spi == NULL) 
  {
    fprintf(stderr,  "Failed to initialize SPI\n");
    mraa_deinit();
    return EXIT_FAILURE;
  }
      /* set SPI frequency */
  status = mraa_spi_frequency(spi, SPI_FREQ);
  if (status != MRAA_SUCCESS)
  {
    fprintf(stderr,  "Failed to set SPI frequency.\n");
    mraa_deinit();
    return EXIT_FAILURE;
  }    
  /* MAX7219/21 chip needs the data in word size */
  status = mraa_spi_bit_per_word(spi, 16);
  if (status != MRAA_SUCCESS) {
    fprintf(stderr, "Failed to set SPI Device to 16Bit mode\n");
    mraa_deinit();
    return EXIT_FAILURE;
  }
  return MRAA_SUCCESS;
}

void spi_deinit()
{
  mraa_spi_stop(spi);
  mraa_deinit();
}

class SPISubscriber : public rclcpp::Node
{
public:
  SPISubscriber()
  : Node("message_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "spi_control_msg", rclcpp::QoS(10).reliable(), std::bind(&SPISubscriber::topic_callback, this, _1));
  }

private:
  int topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) const
  {
    uint8_t i;
    printf("Message from the publisher:");
    for (i=0;i<msg->data.size();i++)
    {
      mraa_spi_write_word(spi, msg->data[i]);
      printf("%#06x,", msg->data[i]);
    }
    printf("\n");
    return 1;
  }
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  spi_init();
  rclcpp::spin(std::make_shared<SPISubscriber>());
  spi_deinit();
  rclcpp::shutdown();
  return 0;
}
