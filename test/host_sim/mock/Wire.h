// Wire (I2C) の模擬。sim.cpp のデバイスモデルに転送し、1 バイトごとに 100kHz 相当の時間を進める
#pragma once
#include "Arduino.h"

#define BUFFER_LENGTH 32

// sim.cpp で実装: 書き込み/読み出しをデバイスへ届ける。ACK なら true
bool simI2cWrite(uint8_t addr, const uint8_t *data, uint8_t len);
bool simI2cRead(uint8_t addr, uint8_t *data, uint8_t len);
extern int g_wireOverflow;  // 32 バイトを超えて write した回数

class TwoWire {
  uint8_t txAddr = 0, txBuf[BUFFER_LENGTH], txLen = 0;
  uint8_t rxBuf[BUFFER_LENGTH], rxLen = 0, rxPos = 0;
  uint32_t clockHz = 100000;
  uint64_t byteUs() const { return 9ULL * 1000000ULL / clockHz; }

 public:
  void begin() {}
  void setClock(uint32_t hz) { clockHz = hz; }
  void setWireTimeout(uint32_t, bool) {}
  void beginTransmission(uint8_t addr) { txAddr = addr; txLen = 0; }
  size_t write(uint8_t b) {
    if (txLen >= BUFFER_LENGTH) { g_wireOverflow++; return 0; }
    txBuf[txLen++] = b;
    return 1;
  }
  uint8_t endTransmission(bool = true) {
    simAdvanceUs(byteUs() * (1 + txLen));          // アドレス + データ
    return simI2cWrite(txAddr, txBuf, txLen) ? 0 : 2;  // 2 = アドレス NACK
  }
  uint8_t requestFrom(uint8_t addr, uint8_t n) {
    if (n > BUFFER_LENGTH) n = BUFFER_LENGTH;
    simAdvanceUs(byteUs() * (1 + n));
    rxPos = 0;
    rxLen = simI2cRead(addr, rxBuf, n) ? n : 0;
    return rxLen;
  }
  int available() { return rxLen - rxPos; }
  int read() { return rxPos < rxLen ? rxBuf[rxPos++] : -1; }
};
extern TwoWire Wire;
