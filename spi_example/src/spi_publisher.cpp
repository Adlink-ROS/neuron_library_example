#include <chrono>
#include <memory>
#include<ctype.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
using namespace std::chrono_literals;
auto message = std_msgs::msg::UInt16MultiArray();

class SPIPublisher : public rclcpp::Node
{
public:
  SPIPublisher()
  : Node("spi_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("spi_control_msg", rclcpp::QoS(10).reliable() );
    SPIPublisher::timer_callback();
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(),"Publishing data...|size:%d",message.data.size());
    publisher_->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  uint8_t i;
  rclcpp::init(argc, argv);
  if (argc<2)
  {
    std::cout << "Usage: ros2 run nsdk_example_spi spi_publisher <LED setup message1(hex)> <LED setup message2(hex)>..." << std::endl;
    return 0;
    
  }
  message.data.resize(argc-1);
  for(i=1;i<argc;i++)
  {
    if (isalpha(*argv[i]))
    {
      std::cout << "Usage: ros2 run nsdk_example_spi spi_publisher <LED setup message1(hex)> <LED setup message2(hex)>..." << std::endl;
      return 0;
    }
    message.data[i-1]=strtol(argv[i], NULL, 16);
  }
  rclcpp::spin_some(std::make_shared<SPIPublisher>());
  rclcpp::shutdown();
  return 0;
}
