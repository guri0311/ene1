// PC 上でスケッチを動かすための Arduino API の模擬 (検証専用)
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <string>

// ---- 仮想時間 (sim.cpp で実装) ----
extern uint64_t g_us;                 // 起動からの仮想マイクロ秒
void simAdvanceUs(uint64_t us);       // 時間を進める (デバイスモデルも更新)
inline unsigned long millis() { return (unsigned long)(uint32_t)(g_us / 1000); }
inline unsigned long micros() { return (unsigned long)(uint32_t)g_us; }
inline void delay(unsigned long ms) { simAdvanceUs((uint64_t)ms * 1000); }
inline void delayMicroseconds(unsigned int us) { simAdvanceUs(us); }

// ---- ピン ----
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
int simDigitalRead(uint8_t pin);
inline void pinMode(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t pin) { return simDigitalRead(pin); }
inline void digitalWrite(uint8_t, uint8_t) {}

// ---- PROGMEM / F() ----
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))
#define PSTR(s) (s)
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t *)(p))
#define strncpy_P strncpy
// avr-libc の %S (PROGMEM 文字列) を %s に読み替える
inline int snprintf_P(char *buf, size_t n, const char *fmt, ...) {
  std::string f(fmt);
  for (size_t i = 0; (i = f.find("%S", i)) != std::string::npos; i += 2) f[i + 1] = 's';
  va_list ap;
  va_start(ap, fmt);
  int r = vsnprintf(buf, n, f.c_str(), ap);
  va_end(ap);
  return r;
}

#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
inline char *dtostrf(double v, signed char width, unsigned char prec, char *buf) {
  sprintf(buf, "%*.*f", width, prec, v);
  return buf;
}

#define DEC 10
#define HEX 16

// ---- Print ----
class Print {
  int write_error = 0;
 protected:
  void setWriteError(int e = 1) { write_error = e; }
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t *b, size_t n) {
    size_t k = 0;
    while (n--) k += write(*b++);
    return k;
  }
  int getWriteError() { return write_error; }
  void clearWriteError() { write_error = 0; }
  size_t print(const char *s) { return write((const uint8_t *)s, strlen(s)); }
  size_t print(const __FlashStringHelper *s) { return print((const char *)s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned long v, int base = DEC) {
    char b[24];
    snprintf(b, sizeof b, base == HEX ? "%lX" : "%lu", v);
    return print(b);
  }
  size_t print(long v, int base = DEC) {
    if (base == HEX) return print((unsigned long)v, HEX);
    char b[24];
    snprintf(b, sizeof b, "%ld", v);
    return print(b);
  }
  size_t print(unsigned int v, int base = DEC) { return print((unsigned long)v, base); }
  size_t print(int v, int base = DEC) { return print((long)v, base); }
  size_t print(unsigned char v, int base = DEC) { return print((unsigned long)v, base); }
  size_t println() { return print("\r\n"); }
  template <class T> size_t println(T v) { size_t n = print(v); return n + println(); }
  template <class T> size_t println(T v, int base) { size_t n = print(v, base); return n + println(); }
};

class HardwareSerial : public Print {
 public:
  void begin(unsigned long) {}
  size_t write(uint8_t c) override;
  using Print::write;
};
extern HardwareSerial Serial;
