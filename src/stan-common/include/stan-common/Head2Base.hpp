#ifndef HEAD2BASE_HPP_
#define HEAD2BASE_HPP_

#define HEAD2BASE_FOREBYTE_CMD 0xAF

enum Head2BaseCommandType {
  SetParameter,
  InitialiseFilter,
  PanTiltAbsCamera,
  PanTiltRelCamera,
  LevelCamera,
  SetSpeed,
  SetTurnRate,
};

struct Head2BaseCommand {
  uint8_t forebyte;
  Head2BaseCommandType type;
  float val1;
  float val2;
} __attribute__((packed));

#endif
