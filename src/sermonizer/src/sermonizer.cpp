// Copyright 2021 David Beaudette

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stan-common/Base2Head.hpp"

#include <sermonizer/serialib.hpp>

using namespace std::chrono_literals;

class Sermonizer : public rclcpp::Node
{
  public:
    Sermonizer()
    : Node("sermonizer"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      
      char errorOpening = serial_.openDevice("/dev/ttyUSB0", 115200);
      if(errorOpening < 1) {
        RCLCPP_ERROR(this->get_logger(), "Error opening serial port (error code %d)\n", (int)errorOpening);
      }

      timer_ = this->create_wall_timer(500ms, 
        std::bind(&Sermonizer::poll_serial, this));
    }
    ~Sermonizer() {
      serial_.closeDevice();
    }
  private:
    void poll_serial()
    {
      auto message = std_msgs::msg::String();
      char serial_buf[1024] = {0};
      serial_.readString(serial_buf, '\n', 1024, 0U);
      message.data = "Pkt " + std::to_string(count_++) + ": " + serial_buf;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    serialib serial_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sermonizer>());
  rclcpp::shutdown();
  return 0;
}
