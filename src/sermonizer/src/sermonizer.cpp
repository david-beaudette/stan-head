// Copyright 2021 David Beaudette

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ros/ros.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

#include <stan_common/Head2Base.hpp>
#include <stan_common/Base2Head.hpp>
#include <stan_common/fletcher_impl.hpp>

#include <stan_common/StanBaseCommand.h>

#include <sermonizer/serialib.hpp>

#define SERIAL_BUF_LEN 1024

//using namespace std::chrono_literals; 
//using std::placeholders::_1;

class Sermonizer
{
public:
  Sermonizer(const ros::NodeHandle &node_handle,
            const ros::NodeHandle &private_node_handle)
      : nh_(node_handle),
        pnh_(private_node_handle),
        count_(0),
        cmd_seq_prev_i_(255U),
        serial_buf_idx_next_(0U),
        crc_error_count_fast_ui32_(0U),
        crc_error_count_slow_ui32_(0U)
  {
    this->init();
  }
  void init() {
    string_pub_ = pnh_.advertise<std_msgs::String>("/serial_pkt", 10);
    pitch_cmd_pub_ = pnh_.advertise<std_msgs::Float32>("/pitch_cmd", 10);
    pitch_est_pub_ = pnh_.advertise<std_msgs::Float32>("/pitch_est", 10);
    speed_cmd_pub_ = pnh_.advertise<std_msgs::Float32>("/speed_cmd", 10);
    
    pitch_filt_gain_f32_ = 0.025f;
    blink_period_f32_ = 0.5f;
    pitch_filt_avg_len_f32_ = 1.0f;
    serial_dev_ = std::string("/dev/ttyUSB0");

    base_cmd_sub_ = pnh_.subscribe("/base_command",
                                    50,
                                    &Sermonizer::base_cmd_msg_cb,
                                    this);

    diagnostic_.add("Stan Base Status", this, &Sermonizer::diagnostics);
    diagnostic_.setHardwareID("arduino_nano_serial");
    lastDiagTime_ = ros::Time::now();

    char ser_open_err = serial_.openDevice(serial_dev_.c_str(), 115200);
    if (ser_open_err < 1)
    {
      ROS_ERROR("Error opening serial port (error code %d)\n", (int)ser_open_err);
      opened_ = false;
    }
    else
    {
      serial_.flushReceiver();

      timer_ = pnh_.createTimer(ros::Duration(0.01),
                                &Sermonizer::poll_serial,
                                this);
      opened_ = true;
    }
  }
  ~Sermonizer()
  {
    serial_.closeDevice();
  }

private:
  void poll_serial(const ros::TimerEvent &event)
  {
    auto message = std_msgs::String();
    int bytes_read = serial_.readBytes(&serial_buf_[serial_buf_idx_next_],
                                       SERIAL_BUF_LEN - serial_buf_idx_next_, 1, 50);
    bytes_read += serial_buf_idx_next_;
    serial_buf_idx_next_ = 0;
    if (bytes_read > 0)
    {
      int idx = 0;
      while (idx < bytes_read)
      {
        if (serial_buf_[idx] == BASE2HEAD_FOREBYTE_SLOW)
        {
          if ((bytes_read - idx) < (int)sizeof(Base2HeadSlow))
          {
            serial_buf_idx_next_ = (bytes_read - idx);
            BufferFall(serial_buf_, &serial_buf_[idx], serial_buf_idx_next_);
            message.data += "--Slow-- remain " + std::to_string(serial_buf_idx_next_) +
                            "B @" + std::to_string(idx) +
                            ", read " + std::to_string(bytes_read) + "B; ";
            break;
          }
          if (!ParseSlowPacket(&serial_buf_[idx]))
          {
            message.data += "--Slow-- !CRC @" + std::to_string(idx) + " (count " +
                            std::to_string(crc_error_count_slow_ui32_) +
                            "), read " + std::to_string(bytes_read) + "B; ";
          }
          else
          {
            message.data += "--Slow-- #" + std::to_string(slow_pkt_last_.seq) +
                            " @" + std::to_string(idx) +
                            ", last cmd #" + std::to_string(slow_pkt_last_.seq_recv_last) +
                            ", read " + std::to_string(bytes_read) + "B; ";
          }
          idx += sizeof(Base2HeadSlow);
        }
        else if (serial_buf_[idx] == BASE2HEAD_FOREBYTE_FAST)
        {
          if ((bytes_read - idx) < (int)sizeof(Base2HeadFast))
          {
            serial_buf_idx_next_ = (bytes_read - idx);
            BufferFall(serial_buf_, &serial_buf_[idx], serial_buf_idx_next_);
            message.data += "Fast remain " + std::to_string(serial_buf_idx_next_) +
                            "B @" + std::to_string(idx) +
                            ", read " + std::to_string(bytes_read) + "B; ";
            break;
          }
          if (!ParseFastPacket(&serial_buf_[idx]))
          {
            message.data += "Fast !CRC @" + std::to_string(idx) + " (count " +
                            std::to_string(crc_error_count_fast_ui32_) +
                            "), read " + std::to_string(bytes_read) + "B; ";
          }
          else
          {
            message.data += "Fast #" + std::to_string(fast_pkt_last_.seq) +
                            " @" + std::to_string(idx) +
                            ", read " + std::to_string(bytes_read) + "B; ";
          }
          idx += sizeof(Base2HeadFast);
        }
        else
        {
          ++idx;
        }
      }
      if (message.data.length() == 0)
      {
        message.data = "Incomplete " +
                       std::to_string(bytes_read) +
                       "B";
      }
      ROS_DEBUG("Publishing: '%s'", message.data.c_str());
      string_pub_.publish(message);
    }
  }
  /** @brief Move buffer bytes to start of buffer **/
  void BufferFall(uint8_t *bottom, const uint8_t *top, size_t len)
  {
    for (size_t idx = 0; idx < len; ++idx)
    {
      bottom[idx] = top[idx];
    }
  }

  /** @brief Parse slow packet **/
  bool ParseSlowPacket(const uint8_t *buf)
  {
    // Start of slow packet found
    Base2HeadSlow *pkt = (Base2HeadSlow *)buf;
    uint16_t crc = compute_fletcher16((uint8_t *)pkt,
                                      sizeof(Base2HeadSlow) -
                                          sizeof(Base2HeadSlow::crc));
    if (crc != pkt->crc)
    {
      ++crc_error_count_slow_ui32_;
      return false;
    }
    else
    {
      memcpy(&slow_pkt_last_, buf, sizeof(Base2HeadSlow));
      return true;
    }
  }

  /** @brief Parse fast packet **/
  bool ParseFastPacket(const uint8_t *buf)
  {
    // Start of fast packet found
    Base2HeadFast *pkt = (Base2HeadFast *)buf;
    uint16_t crc = compute_fletcher16((uint8_t *)pkt,
                                      sizeof(Base2HeadFast) -
                                          sizeof(Base2HeadFast::crc));
    if (crc != pkt->crc)
    {
      ++crc_error_count_fast_ui32_;
      return false;
    }
    else
    {
      memcpy(&fast_pkt_last_, buf, sizeof(Base2HeadFast));
      auto pitch_cmd_msg = std_msgs::Float32();
      auto pitch_est_msg = std_msgs::Float32();
      auto speed_cmd_msg = std_msgs::Float32();
      pitch_cmd_msg.data = pkt->pitch_cmd;
      pitch_est_msg.data = pkt->pitch_est;
      speed_cmd_msg.data = pkt->speed_cmd[0];
      pitch_cmd_pub_.publish(pitch_cmd_msg);
      pitch_est_pub_.publish(pitch_est_msg);
      speed_cmd_pub_.publish(speed_cmd_msg);
      return true;
    }
  }

  /** @brief Send base command on serial port **/
  bool SendBaseCommand(const Head2BaseCommandType type, const float val1, const float val2)
  {
    Head2BaseCommand cmd;
    cmd.forebyte = HEAD2BASE_FOREBYTE_CMD;
    if (cmd_seq_prev_i_ == UINT8_MAX)
    {
      cmd_seq_prev_i_ = 0U;
    }
    else
    {
      ++cmd_seq_prev_i_;
    }
    cmd.seq = cmd_seq_prev_i_;
    cmd.type = (uint8_t)type;
    cmd.val1 = val1;
    cmd.val2 = val2;
    cmd.crc = compute_fletcher16((uint8_t *)&cmd,
                                 sizeof(Head2BaseCommand) -
                                     sizeof(Head2BaseCommand::crc));

    return (serial_.writeBytes((uint8_t *)&cmd, sizeof(Head2BaseCommand)) >= 0);
  }
  /** @brief Publishes diagnostics and status **/
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    ros::Time now = ros::Time::now();
    //double interval = now - lastDiagTime_;
    /*  byte OK=0
        byte WARN=1
        byte ERROR=2
        byte STALE=3 */
    if (opened_)
    {
      stat.summary(0, "OK");
    }
    else
    {
      stat.summary(2, "Unable to open serial port");
    }

    stat.add("battery level (%)", slow_pkt_last_.batt_soc);
    stat.add("equilibrium pitch (deg)", slow_pkt_last_.pitch_zero);
    stat.add("pitch filter gain", slow_pkt_last_.pitch_filter_gain);
    stat.add("pitch control P gain", slow_pkt_last_.pitch_ctl_gain_P);
    stat.add("pitch control I gain", slow_pkt_last_.pitch_ctl_gain_I);
    stat.add("pitch control D gain", slow_pkt_last_.pitch_ctl_gain_D);
    stat.add("system status", slow_pkt_last_.status & 0x0F);
    stat.add("system error code", (slow_pkt_last_.status & 0xF0) >> 4);
    stat.add("base error count", slow_pkt_last_.crc_error_count);
    stat.add("fast packet parsing error count", crc_error_count_fast_ui32_);
    stat.add("fast packet size", (int)sizeof(Base2HeadFast));
    stat.add("slow packet parsing error count", crc_error_count_slow_ui32_);
    stat.add("slow packet size", (int)sizeof(Base2HeadSlow));
    stat.add("device", serial_dev_);
    lastDiagTime_ = now;
  }

  void base_cmd_msg_cb(const stan_common::StanBaseCommand::ConstPtr& msg)
  {
    this->SendBaseCommand(static_cast<Head2BaseCommandType>(msg->type),
                          msg->val1,
                          msg->val2);
    return;
  }

  // Public ros node handle
  ros::NodeHandle nh_;
  // Private ros node handle
  ros::NodeHandle pnh_;
  std::string node_name_{"sermonizer"};

  ros::Timer timer_;
  ros::Publisher string_pub_;
  ros::Publisher pitch_cmd_pub_;
  ros::Publisher pitch_est_pub_;
  ros::Publisher speed_cmd_pub_;
  ros::Subscriber base_cmd_sub_;
 
  diagnostic_updater::Updater diagnostic_;

  size_t count_;
  std::string serial_dev_;
  bool opened_;
  ros::Time lastDiagTime_;
  serialib serial_;
  float pitch_filt_gain_f32_;
  float blink_period_f32_;
  float pitch_filt_avg_len_f32_;
  uint8_t cmd_seq_prev_i_;
  uint8_t serial_buf_[SERIAL_BUF_LEN];
  size_t serial_buf_idx_next_;
  uint32_t crc_error_count_fast_ui32_;
  uint32_t crc_error_count_slow_ui32_;
  Base2HeadSlow slow_pkt_last_;
  Base2HeadFast fast_pkt_last_;
};

int main(int argc, char** argv) {
  std::string node_name = "sermonizer";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  Sermonizer node(nh, nh_private);
  ROS_INFO("Initialized serial port reading node sermonizer.");
  ros::spin();
}
