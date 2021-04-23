#ifndef HEAD2BASE_HPP_
#define HEAD2BASE_HPP_

#define HEAD2BASE_FOREBYTE_CMD 0xAF

enum Head2BaseCommandType {
  InitialiseFilter,
  PanTiltAbsCamera,
  PanTiltRelCamera,
  LevelCamera,
  SetSpeed,
  SetTurnRate,
};


struct Head2BaseCommand {
  uint8_t forebyte;
  uint8_t seq;
  uint8_t type;
  uint8_t idx;
  float val1;
  float val2;
} __attribute__((packed));

#endif
