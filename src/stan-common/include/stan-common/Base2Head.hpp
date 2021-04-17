#ifndef BASE2HEAD_HPP_
#define BASE2HEAD_HPP_

#define BASE2HEAD_FOREBYTE_SLOW 0xAD
#define BASE2HEAD_FOREBYTE_FAST 0xAA

struct Base2HeadSlow {
  uint8_t forebyte;
  uint8_t status;
  float batt_volt;
  float batt_soc;
  float pitch_cmd;
} __attribute__((packed));

struct Base2HeadFast {
  uint8_t forebyte;
  uint8_t status;
  uint32_t acc_count;
  float cam_pan_pct;
  float cam_tilt_pct;
  float pitch_ref;
  float pitch_est;
  float speed_cmd[2];
  float acc_x_mes;
  float acc_y_mes;
  float acc_z_mes;
  float w_mes;
} __attribute__((packed));

#endif
