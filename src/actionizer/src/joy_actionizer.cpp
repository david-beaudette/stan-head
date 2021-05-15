// Copyright 2021 David Beaudette

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <stan_common/msg/stan_base_command.hpp>

#include <stan_common/Head2Base.hpp>

// Axes definitions
#define AXIS_TURN 0
#define AXIS_MOVE 1
#define AXIS_PAN_CAM 2
#define AXIS_TILT_CAM 3
#define AXIS_INCREMENT_PAR 4
#define AXIS_CHANGE_PAR 5

// Button definitions
#define BUTTON_ZERO_PITCH 3
#define BUTTON_KP 4
#define BUTTON_KI 6
#define BUTTON_KD 7
#define BUTTON_MODE 8
#define BUTTON_MOTORS_ENA 9

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyActionizer : public rclcpp::Node
{
public:
  JoyActionizer()
      : Node("joy_actionizer"),
        joy_buttons_prev_(16, 0),
        joy_axes_prev_(12, 0.0f),
        param_increment_{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}
        
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 50, std::bind(&JoyActionizer::joy_msg_cb, this, _1));
    base_command_pub_ =
        this->create_publisher<stan_common::msg::StanBaseCommand>("base_command", 10);
    string_pub_ = this->create_publisher<std_msgs::msg::String>("increment_change", 10);

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
  void joy_msg_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    printf("Got message %d.\n", msg->buttons[0]);
    auto msg_time = rclcpp::Time(msg->header.stamp);
    // Manage parameter changes
    if (fabsf(msg->axes[AXIS_INCREMENT_PAR]) > 0.5 &&
        fabsf(joy_axes_prev_[AXIS_INCREMENT_PAR]) < 0.5)
    {
      auto increment_change_msg = std_msgs::msg::String();
      // Manage increment value for all pressed buttons related to 
      // parameters
      if (msg->buttons[BUTTON_KP] == 1)
      {
        param_increment_[BUTTON_KP] *= pow(2.0, msg->axes[AXIS_INCREMENT_PAR]);
      }
      if (msg->buttons[BUTTON_KI] == 1)
      {
        param_increment_[BUTTON_KI] *= pow(2.0, msg->axes[AXIS_INCREMENT_PAR]);
      }
      if (msg->buttons[BUTTON_KD] == 1)
      {
        param_increment_[BUTTON_KD] *= pow(2.0, msg->axes[AXIS_INCREMENT_PAR]);
      }
      if (msg->buttons[BUTTON_ZERO_PITCH] == 1)
      {
        param_increment_[BUTTON_ZERO_PITCH] *= pow(2.0, msg->axes[AXIS_INCREMENT_PAR]);
      }
      increment_change_msg.data = "Par tuning increments: Kp = " + std::to_string(param_increment_[BUTTON_KP]) +
                                  "; Ki = " + std::to_string(param_increment_[BUTTON_KI]) +
                                  "; Kd = " + std::to_string(param_increment_[BUTTON_KD]) +
                                  "; Zero pitch = " + std::to_string(param_increment_[BUTTON_ZERO_PITCH]);
      string_pub_->publish(increment_change_msg);
    }
    if (fabsf(msg->axes[AXIS_INCREMENT_PAR]) > 0.5 &&
        fabsf(joy_axes_prev_[AXIS_INCREMENT_PAR]) < 0.5)
    {
      // Change the parameter
      if (msg->buttons[BUTTON_KP] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TunePitchControl1;
        base_command_msg.val1 = param_increment_[BUTTON_KP] * msg->axes[AXIS_INCREMENT_PAR] * 0.01;
        base_command_msg.val2 = 0.0f;
        base_command_pub_->publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_KI] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TunePitchControl1;
        base_command_msg.val1 = 0.0f;
        base_command_msg.val2 = param_increment_[BUTTON_KI] * msg->axes[AXIS_INCREMENT_PAR] * 0.001;
        base_command_pub_->publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_KD] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TunePitchControl2;
        base_command_msg.val1 = param_increment_[BUTTON_KD] * msg->axes[AXIS_INCREMENT_PAR] * 0.001;
        base_command_msg.val2 = 0.0f;
        base_command_pub_->publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_ZERO_PITCH] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TuneZeroPitch;
        base_command_msg.val1 = param_increment_[BUTTON_ZERO_PITCH] * msg->axes[AXIS_INCREMENT_PAR] * 0.001;
        base_command_msg.val2 = 0.0f;
        base_command_pub_->publish(base_command_msg);
      }
    }
    joy_axes_prev_ = msg->axes;
    joy_buttons_prev_ = msg->buttons;

    return;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<stan_common::msg::StanBaseCommand>::SharedPtr base_command_pub_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr par_cb_hdl_;
  std::vector<int> joy_buttons_prev_;
  std::vector<float> joy_axes_prev_;
  float param_increment_[16];
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyActionizer>());
  rclcpp::shutdown();
  return 0;
}
