// EEPROM の模擬 (1KB)。中身は sim.cpp が状態ファイルとは別のファイルに保存し、
// 電源を入れ直しても (cold) 残る。1 バイトの書き込みには ATmega328P と同じく約 3.3ms かかり、
// その間に次の読み書きをすると終わるまで待つ (実機の eeprom_* と同じ)
#pragma once
#include <cstdint>
#include <cstddef>
#define E2END 0x3FF
extern uint8_t  g_eeprom[E2END + 1];
extern uint64_t g_eeReadyUs;        // この時刻まで書き込み中
extern uint32_t g_eeWrites[E2END + 1];
extern int      g_eeLastAddr;       // 最後に書いた番地
void simEeWait();
inline bool eeprom_is_ready() { extern uint64_t g_us; return g_us >= g_eeReadyUs; }
inline uint8_t eeprom_read_byte(const uint8_t *p) { simEeWait(); return g_eeprom[(uintptr_t)p & E2END]; }
inline void eeprom_read_block(void *dst, const void *src, size_t n) {
  simEeWait();
  for (size_t i = 0; i < n; i++) ((uint8_t *)dst)[i] = g_eeprom[((uintptr_t)src + i) & E2END];
}
inline void eeprom_update_byte(uint8_t *p, uint8_t v) {
  extern uint64_t g_us;
  simEeWait();
  uintptr_t a = (uintptr_t)p & E2END;
  if (g_eeprom[a] == v) return;
  g_eeprom[a] = v;
  g_eeWrites[a]++;
  g_eeLastAddr = (int)a;
  g_eeReadyUs = g_us + 3300;
}
