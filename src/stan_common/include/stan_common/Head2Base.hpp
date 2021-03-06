#ifndef HEAD2BASE_HPP_
#define HEAD2BASE_HPP_

#define HEAD2BASE_FOREBYTE_CMD 0xAF

enum Head2BaseCommandType {
  InitialiseFilter,  // val1 = acc_coeff (0-1), val2 = avg_len (s)
  TunePitchControl1, // val1 = P gain increment, val2 = I gain increment
  TunePitchControl2, // val1 = D gain increment
  PanTiltAbsCamera,  // val1 = pan (deg), val2 = tilt (deg)
  PanTiltRelCamera,  // val1 = pan (deg), val2 = tilt (deg)
  LevelCamera,       // val1 = pan (deg)
  TuneZeroPitch,      // val1 = equilibrium pitch increment (deg)
  SetSpeed,          // val1 = speed (m/s)
  SetTurnRate,       // val1 = turn rate (deg/s)
  LedBlinkRate,      // val1 = blink period (s)
  SetRunningState,   // val1 = 0 to disable control and motors, 1 to enable
};


struct Head2BaseCommand {
  uint8_t forebyte;
  uint8_t seq;
  uint8_t type;
  float val1;
  float val2;
  uint16_t crc;
} __attribute__((packed));

#endif
