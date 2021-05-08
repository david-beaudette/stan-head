// Copyright 2021 David Beaudette

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <stan_common/msg/stan_base_command.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyActionizer : public rclcpp::Node
{
public:
  JoyActionizer()
      : Node("joy_actionizer")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 50, std::bind(&JoyActionizer::joy_msg_cb, this, _1));

    auto par_cb =
        [this](const std::vector<rclcpp::Parameter> &parameters)
        -> rcl_interfaces::msg::SetParametersResult {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      bool par_found = false;
      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "tbd")
        {
          par_found = true;
          result.successful = true;
        }
      }
      if (!result.successful)
      {
        if (par_found)
        {
          result.reason = "Unknown parameter";
        }
      }
      return result;
    };
    par_cb_hdl_ = this->add_on_set_parameters_callback(par_cb);
  }
  ~JoyActionizer()
  {
    
  }

private:
  
  void joy_msg_cb(const sensor_msgs::msg::Joy::SharedPtr msg) const
  {
    printf("Got message %d.\n", msg->buttons[0]);
    return;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr par_cb_hdl_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyActionizer>());
  rclcpp::shutdown();
  return 0;
}
