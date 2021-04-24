#ifndef HEAD2BASE_HPP_
#define HEAD2BASE_HPP_

#define HEAD2BASE_FOREBYTE_CMD 0xAF

enum Head2BaseCommandType {
  InitialiseFilter, // val1 = acc_coeff (0-1), val2 = avg_len (s)
  PanTiltAbsCamera, // val1 = pan (deg), val2 = tilt (deg)
  PanTiltRelCamera, // val1 = pan (deg), val2 = tilt (deg)
  LevelCamera,      // val1 = pan (deg)
  SetSpeed,         // val1 = speed (m/s)
  SetTurnRate,      // val1 = turn rate (deg/s)
  LedBlinkRate,     // val1 = blink period (s)
};


struct Head2BaseCommand {
  uint8_t forebyte;
  uint8_t seq;
  uint8_t type;
  float val1;
  float val2;
} __attribute__((packed));

#endif
