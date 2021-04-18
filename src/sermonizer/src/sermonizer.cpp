// Copyright 2021 David Beaudette

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stan-common/Head2Base.hpp"
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
    if (errorOpening < 1)
    {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port (error code %d)\n", (int)errorOpening);
    }
    serial_.flushReceiver();

    timer_ = this->create_wall_timer(10ms,
                                     std::bind(&Sermonizer::poll_serial, this));
  }
  ~Sermonizer()
  {
    serial_.closeDevice();
  }

private:
  void poll_serial()
  {
    auto message = std_msgs::msg::String();
    uint8_t serial_buf[1024] = {0};
    int bytes_read = serial_.readBytes(serial_buf,
                                       1024, 1, 50);
    if (bytes_read > 0)
    {
      int idx = 0;
      while (idx < bytes_read)
      {
        if (serial_buf[idx] == BASE2HEAD_FOREBYTE_SLOW)
        {
          // Start of slow packet found
          Base2HeadSlow *msg = (Base2HeadSlow *)&serial_buf[idx];
          message.data += "Slow pkt seq " + std::to_string(msg->seq) +
                          ", #bytes " + std::to_string(bytes_read) + "; ";
          idx += sizeof(Base2HeadSlow);
        }
        else if (serial_buf[idx] == BASE2HEAD_FOREBYTE_FAST)
        {
/*           if((bytes_read - idx) < sizeof(Base2HeadFast)) {
            // Read the remainder of bytes
            bytes_read += serial_.readBytes(&serial_buf[bytes_read],
                                            1024 - bytes_read, 1);
          } */
          // Start of fast packet found
          Base2HeadFast *msg = (Base2HeadFast *)&serial_buf[idx];
          message.data += "Fast pkt seq " + std::to_string(msg->seq) +
                          ", sizeof (avr) " + std::to_string(msg->cam_tilt_pct) +
                          ", sizeof (linux) " + std::to_string(sizeof(Base2HeadFast)) +
                          ", written " + std::to_string(msg->pitch_ref) +
                          ", #bytes " + std::to_string(bytes_read) + "; ";
          idx += sizeof(Base2HeadFast);
        }
        else
        {
          ++idx;
        }
      }
      if (message.data.length() == 0)
      {
        message.data = "No fore byte found #bytes " +
                       std::to_string(bytes_read) +
                       "; content is [";
        for (int i = 0; i < bytes_read; ++i)
        {
          message.data += std::to_string((int)serial_buf[i]) + ", ";
        }
        message.data += "]";
      }
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  serialib serial_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sermonizer>());
  rclcpp::shutdown();
  return 0;
}
