#ifndef BASE2HEAD_HPP_
#define BASE2HEAD_HPP_

#define BASE2HEAD_FOREBYTE_SLOW 0xAD
#define BASE2HEAD_FOREBYTE_FAST 0xAA

const uint8_t BASE_STATUS_PREINIT = 0x00;
const uint8_t BASE_STATUS_FILTINIT = 0x01;
const uint8_t BASE_STATUS_MOTOR_OFF = 0x02;
const uint8_t BASE_STATUS_RUNNING = 0x04;

const uint8_t BASE_ERR_NONE = 0x00;
const uint8_t BASE_ERR_SERIAL = 0x01;
const uint8_t BASE_ERR_LOW_BAT = 0x02;


struct Base2HeadSlow {
  uint8_t forebyte;
  uint8_t seq;
  uint8_t status;
  uint8_t seq_recv_last;
  float batt_volt;
  float batt_soc;
  float pitch_zero;
  float pitch_filter_gain;
  float pitch_ctl_gain_P;
  float pitch_ctl_gain_I;
  float pitch_ctl_gain_D;
  uint16_t crc_error_count;
  uint16_t crc;
} __attribute__((packed));

struct Base2HeadFast {
  uint8_t forebyte;
  uint8_t seq;
  uint8_t status;
  uint8_t unused;
  uint32_t acc_count;
  float cam_pan_pct;
  float cam_tilt_pct;
  float pitch_cmd;
  float pitch_est;
  float speed_cmd[2];
  float acc_x_mes;
  float acc_y_mes;
  float acc_z_mes;
  float w_mes;
  uint16_t crc;
} __attribute__((packed));

const size_t BASE2HEAD_PKT_SIZE_MAX = sizeof(Base2HeadFast) > sizeof(Base2HeadSlow) ? 
                                      sizeof(Base2HeadFast) : sizeof(Base2HeadSlow);

#endif
