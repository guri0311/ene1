// SPI の模擬: SD カードの状態確認 (CMD13) にだけ使う。カードがあれば R1 = 0x00、無ければ 0xFF
#pragma once
#include "Arduino.h"

#define MSBFIRST 1
#define SPI_MODE0 0
extern bool g_sdCardPresent;

struct SPISettings {
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};
class SPIClass {
 public:
  void beginTransaction(SPISettings) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t) { return g_sdCardPresent ? 0x00 : 0xFF; }
};
extern SPIClass SPI;
