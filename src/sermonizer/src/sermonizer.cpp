// Copyright 2021 David Beaudette

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

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
    string_pub_ = this->create_publisher<std_msgs::msg::String>("serial_pkt", 10);
    pitch_ref_pub_ = this->create_publisher<std_msgs::msg::Float32>("pitch_ref", 10);
    pitch_est_pub_ = this->create_publisher<std_msgs::msg::Float32>("pitch_est", 10);

    pitch_filt_gain_f32_ = this->declare_parameter("pitch_filt_gain", static_cast<double>(0.025));


    auto par_cb =
        [this](const std::vector<rclcpp::Parameter> &parameters)
        -> rcl_interfaces::msg::SetParametersResult {

      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "pitch_filt_gain")
        {
          pitch_filt_gain_f32_ = static_cast<float>(parameter.as_double());
          result.successful = true;
          result.reason = "Widdit done";
          RCLCPP_INFO(this->get_logger(), "Pitch filter gain changed to %f.", pitch_filt_gain_f32_);
        }
      }
      if (!result.successful)
      {
        result.reason = "Unknown parameter";
      }
      return result;
    };
    par_cb_hdl_ = this->add_on_set_parameters_callback(par_cb);

    char ser_open_err = serial_.openDevice("/dev/ttyUSB0", 115200);
    if (ser_open_err < 1)
    {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port (error code %d)\n", (int)ser_open_err);
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
          // Start of fast packet found
          Base2HeadFast *pkt = (Base2HeadFast *)&serial_buf[idx];
          message.data += "Fast pkt seq " + std::to_string(pkt->seq) +
                          ", #bytes " + std::to_string(bytes_read) + "; ";
          idx += sizeof(Base2HeadFast);
          auto pitch_ref_msg = std_msgs::msg::Float32();
          auto pitch_est_msg = std_msgs::msg::Float32();
          pitch_ref_msg.data = pkt->pitch_ref;
          pitch_est_msg.data = pkt->pitch_est;
          pitch_ref_pub_->publish(pitch_ref_msg);
          pitch_est_pub_->publish(pitch_est_msg);
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
      RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      string_pub_->publish(message);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_ref_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_est_pub_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr par_cb_hdl_;
  size_t count_;
  serialib serial_;
  float pitch_filt_gain_f32_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sermonizer>());
  rclcpp::shutdown();
  return 0;
}
