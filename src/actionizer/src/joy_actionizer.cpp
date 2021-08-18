// Copyright 2021 David Beaudette

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include <stan_common/StanBaseCommand.h>
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

class JoyActionizer
{
public:
  JoyActionizer(const ros::NodeHandle &node_handle,
                const ros::NodeHandle &private_node_handle)
      : nh_(node_handle),
        pnh_(private_node_handle),
        joy_buttons_prev_(NUM_JOY_BUTTONS_INI, 0),
        joy_axes_prev_(NUM_JOY_AXES_INI, 0.0f),
        param_increment_(NUM_JOY_BUTTONS_INI, 1.0f),
        motor_enable_prev_(false)
  {

    base_command_pub_ = pnh_.advertise<stan_common::StanBaseCommand>("/base_command", 10);
    string_pub_ = pnh_.advertise<std_msgs::String>("/increment_change", 10);
    
    joy_sub_ = pnh_.subscribe("/joy",
                              50,
                              &JoyActionizer::joy_msg_cb,
                              this);

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
  void joy_msg_cb(const sensor_msgs::Joy::ConstPtr& msg)
  {
    auto msg_time = ros::Time(msg->header.stamp);

    // Manage parameter changes
    if (fabsf(msg->axes[AXIS_INCREMENT_PAR]) > 0.5 &&
        fabsf(joy_axes_prev_[AXIS_INCREMENT_PAR]) < 0.5)
    {
      auto increment_change_msg = std_msgs::String();
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
      string_pub_.publish(increment_change_msg);
    }
    if (fabsf(msg->axes[AXIS_CHANGE_PAR]) > 0.5 &&
        fabsf(joy_axes_prev_[AXIS_CHANGE_PAR]) < 0.5)
    {
      // Change the parameter
      if (msg->buttons[BUTTON_KP] == 1)
      {
        auto base_command_msg = stan_common::StanBaseCommand();
        base_command_msg.type = TunePitchControl1;
        base_command_msg.val1 = param_increment_[BUTTON_KP] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_msg.val2 = 0.0f;
        base_command_pub_.publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_KI] == 1)
      {
        auto base_command_msg = stan_common::StanBaseCommand();
        base_command_msg.type = TunePitchControl1;
        base_command_msg.val1 = 0.0f;
        base_command_msg.val2 = param_increment_[BUTTON_KI] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_pub_.publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_KD] == 1)
      {
        auto base_command_msg = stan_common::StanBaseCommand();
        base_command_msg.type = TunePitchControl2;
        base_command_msg.val1 = param_increment_[BUTTON_KD] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_msg.val2 = 0.0f;
        base_command_pub_.publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_ZERO_PITCH] == 1)
      {
        auto base_command_msg = stan_common::StanBaseCommand();
        base_command_msg.type = TuneZeroPitch;
        base_command_msg.val1 = param_increment_[BUTTON_ZERO_PITCH] *
                                msg->axes[AXIS_CHANGE_PAR];
        base_command_msg.val2 = 0.0f;
        base_command_pub_.publish(base_command_msg);
      }
      else if (msg->buttons[BUTTON_BLINK_DT] == 1)
      {
        auto base_command_msg = stan_common::StanBaseCommand();
        base_command_msg.type = LedBlinkRate;
        base_command_msg.val1 = param_increment_[BUTTON_BLINK_DT];
        base_command_msg.val2 = 0.0f;
        base_command_pub_.publish(base_command_msg);
      }
    }

    // Manage other button presses
    if (msg->buttons[BUTTON_MOTORS_ENA] == 1 &&
        joy_buttons_prev_[BUTTON_MOTORS_ENA] == 0)
    {
      // Toggle motor enable state
      motor_enable_prev_ = !motor_enable_prev_;
      auto base_command_msg = stan_common::StanBaseCommand();
      base_command_msg.type = SetRunningState;
      base_command_msg.val1 = static_cast<float>(motor_enable_prev_);
      base_command_msg.val2 = 0.0f;
      base_command_pub_.publish(base_command_msg);
    }

    // Save last commands to detect changes
    joy_axes_prev_ = msg->axes;
    joy_buttons_prev_ = msg->buttons;

    return;
  }
  // Public ros node handle
  ros::NodeHandle nh_;
  // Private ros node handle
  ros::NodeHandle pnh_;
  std::string node_name_{"joy_actionizer"};

  ros::Timer timer_;
  ros::Subscriber joy_sub_;
  ros::Publisher string_pub_;
  ros::Publisher base_command_pub_;

  std::vector<int> joy_buttons_prev_;
  std::vector<float> joy_axes_prev_;
  std::vector<float> param_increment_;
  bool motor_enable_prev_;
};

int main(int argc, char *argv[])
{
  std::string node_name = "joy_actionizer";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  JoyActionizer node(nh, nh_private);
  ROS_INFO("Initialized joystick command interpreter.");
  ros::spin();
}
