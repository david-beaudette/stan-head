#ifndef BASE2HEAD_HPP_
#define BASE2HEAD_HPP_


struct Base2HeadSlow {
  uint8_t headbyte;
  uint8_t status;
  float batt_volt;
  float batt_soc;
} __attribute__((packed));

struct Base2HeadFast {
  uint8_t headbyte;
  uint8_t status;
  uint32_t acc_count;
  float pitch_est;
  float pitch_cmd;
  float speed_cmd;
  float acc_x_mes;
  float acc_y_mes;
  float acc_z_mes;
  float w_mes;
} __attribute__((packed));

#endif
