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
#define BUTTON_BLINK_DT 5
#define BUTTON_KI 6
#define BUTTON_KD 7
#define BUTTON_MODE 8
#define BUTTON_MOTORS_ENA 9

#define NUM_JOY_BUTTONS_INI 16
#define NUM_JOY_AXES_INI 16

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyActionizer : public rclcpp::Node
{
public:
  JoyActionizer()
      : Node("joy_actionizer"),
        joy_buttons_prev_(NUM_JOY_BUTTONS_INI, 0),
        joy_axes_prev_(NUM_JOY_AXES_INI, 0.0f),
        param_increment_(NUM_JOY_BUTTONS_INI, 1.0f),
        motor_enable_prev_(false)
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 50, std::bind(&JoyActionizer::joy_msg_cb, this, _1));
    base_command_pub_ =
        this->create_publisher<stan_common::msg::StanBaseCommand>("base_command", 10);
    string_pub_ = this->create_publisher<std_msgs::msg::String>("increment_change", 10);

    param_increment_[BUTTON_KP] = 0.01;
    param_increment_[BUTTON_KI] = 0.001;
    param_increment_[BUTTON_KD] = 0.001;
    param_increment_[BUTTON_ZERO_PITCH] = 0.1;
    param_increment_[BUTTON_BLINK_DT] = 0.1;
  }
  ~JoyActionizer()
  {
  }

private:
  void joy_msg_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
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
      if (msg->buttons[BUTTON_BLINK_DT] == 1)
      {
        param_increment_[BUTTON_BLINK_DT] *= pow(2.0, msg->axes[AXIS_INCREMENT_PAR]);
      }
      increment_change_msg.data = "Par tuning increments: Kp = " + std::to_string(param_increment_[BUTTON_KP]) +
                                  "; Ki = " + std::to_string(param_increment_[BUTTON_KI]) +
                                  "; Kd = " + std::to_string(param_increment_[BUTTON_KD]) +
                                  "; Zero pitch = " + std::to_string(param_increment_[BUTTON_ZERO_PITCH]) +
                                  "; Blink dt = " + std::to_string(param_increment_[BUTTON_BLINK_DT]);
      string_pub_->publish(increment_change_msg);
    }
    if (fabsf(msg->axes[AXIS_CHANGE_PAR]) > 0.5 &&
        fabsf(joy_axes_prev_[AXIS_CHANGE_PAR]) < 0.5)
    {
      // Change the parameter
      if (msg->buttons[BUTTON_KP] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TunePitchControl1;
        base_command_msg.val1 = param_increment_[BUTTON_KP] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_msg.val2 = 0.0f;
        base_command_pub_->publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_KI] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TunePitchControl1;
        base_command_msg.val1 = 0.0f;
        base_command_msg.val2 = param_increment_[BUTTON_KI] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_pub_->publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_KD] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TunePitchControl2;
        base_command_msg.val1 = param_increment_[BUTTON_KD] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_msg.val2 = 0.0f;
        base_command_pub_->publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_ZERO_PITCH] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = TuneZeroPitch;
        base_command_msg.val1 = param_increment_[BUTTON_ZERO_PITCH] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_msg.val2 = 0.0f;
        base_command_pub_->publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_BLINK_DT] == 1)
      {
        auto base_command_msg = stan_common::msg::StanBaseCommand();
        base_command_msg.type = LedBlinkRate;
        base_command_msg.val1 = param_increment_[BUTTON_BLINK_DT];
        base_command_msg.val2 = 0.0f;
        base_command_pub_->publish(base_command_msg);
      }
    }

    // Manage other button presses
    if (msg->buttons[BUTTON_MOTORS_ENA] == 1 &&
        joy_buttons_prev_[BUTTON_MOTORS_ENA] == 0)
    {
      // Toggle motor enable state
      motor_enable_prev_ = !motor_enable_prev_;
      auto base_command_msg = stan_common::msg::StanBaseCommand();
      base_command_msg.type = SetRunningState;
      base_command_msg.val1 = static_cast<float>(motor_enable_prev_);
      base_command_msg.val2 = 0.0f;
      base_command_pub_->publish(base_command_msg);
    }

    // Save last commands to detect changes
    joy_axes_prev_ = msg->axes;
    joy_buttons_prev_ = msg->buttons;

    return;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<stan_common::msg::StanBaseCommand>::SharedPtr base_command_pub_;

  std::vector<int> joy_buttons_prev_;
  std::vector<float> joy_axes_prev_;
  std::vector<float> param_increment_;
  bool motor_enable_prev_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyActionizer>());
  rclcpp::shutdown();
  return 0;
}
