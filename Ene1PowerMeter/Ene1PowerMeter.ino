/*
 * Ene-1 GP 車載電力計
 *   マイコン   : Arduino Nano (ATmega328P / 5V / 16MHz)
 *   センサー   : INA228 (Strawberry Linux #11228, シャント 2mΩ, I2C 0x40)
 *   表示       : 秋月 I2C キャラクター液晶 20x4 (ACM2004D-FLW-FBW-IIC, 制御IC RW1063)
 *   記録       : microSD (SPI, CS = D10)
 *
 * ボタン (D2 - GND)
 *   1 秒長押し : 積算 mAh・経過時間を 0 にして新しいログファイルを作る
 *   短く押す   : SD が NO SD のときだけ、SD の再初期化を試す (積算はそのまま)
 *
 * レース中の故障に備えた動作
 *   - Nano だけがリセットされた場合 (ウォッチドッグ・書き込み・リセットボタン等)、
 *     INA228 の設定が残っていれば積算 mAh を消さずに続きから再開する (経過時間も可能なら復元)
 *   - 走行中に INA228 の設定が消えた (電源瞬断など) ことを検出したら、再設定して
 *     それまでの積算値を引き継ぐ
 *   - LCD をノイズで化けたままにしないよう、5 秒ごとに初期化コマンドを再送する
 *   - 処理が固まったら 8 秒でウォッチドッグがリセットする
 *
 * 必要なライブラリ: Wire, SPI (標準) / SD
 *   LCD は自作の軽量ドライバ (lcd* 関数) で直接制御する
 *
 * ピン配置
 *   A4 = SDA, A5 = SCL  (INA228 と LCD で共用。4.7kΩ で 5V へ外付けプルアップ)
 *   D11 = MOSI, D12 = MISO, D13 = SCK, D10 = CS (SD カード)
 *   D2  = ボタン (内部プルアップ、押すと LOW)
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/wdt.h>

// ============================================================================
//  ユーザー設定 (ここを変えるだけで調整できる)
// ============================================================================
// シャント抵抗 [Ω]。基準の電流計と比べて表示がずれる場合はこの値で校正する
// (例: 表示が 1% 大きい → 0.002 x 1.01 = 0.00202)。電流・電力・積算すべてに効く
constexpr float    R_SHUNT_OHM     = 0.002f;
constexpr float    MAX_CURRENT_A   = 70.0f;    // 想定最大電流 [A] (CURRENT_LSB の計算に使う)
constexpr uint8_t  INA228_ADDR     = 0x40;     // INA228 の I2C アドレス (A0=A1=GND)
constexpr uint8_t  LCD_ADDR        = 0x3C;     // LCD の I2C アドレス (応答がなければ 0x3C〜0x3F から自動で探す)
constexpr uint16_t LCD_INTERVAL_MS = 250;      // LCD 更新間隔 [ms]
constexpr uint16_t LCD_REINIT_MS   = 5000;     // LCD 初期化コマンドの再送間隔 [ms] (ノイズ対策)
constexpr uint16_t LOG_INTERVAL_MS = 1000;     // SD 記録間隔 [ms]
constexpr uint8_t  LOG_FLUSH_EVERY = 5;        // 何行書くごとに flush するか (1 = 毎行)
constexpr uint16_t RESET_HOLD_MS   = 1000;     // 積算リセットの長押し時間 [ms]
constexpr uint8_t  BTN_SHORT_MIN_MS = 50;      // 短押しと認める最短時間 [ms] (チャタリング除け)
constexpr uint8_t  PIN_SD_CS       = 10;
constexpr uint8_t  PIN_BTN         = 2;
#define SERIAL_DEBUG 1   // 1: 起動時の I2C スキャン結果と CSV を USB シリアル (115200bps) に出す
#define USE_WATCHDOG 1   // 1: 8 秒固まったら自動リセット (旧ブートローダの Nano 互換機は下の注意を参照)

// INA228 の ADC 設定 (データシート ADC_CONFIG レジスタ)
constexpr uint8_t INA_MODE   = 0xB;  // 0xB = シャント電圧 + バス電圧 連続測定
constexpr uint8_t INA_VBUSCT = 5;    // バス電圧 変換時間 5 = 1052us
constexpr uint8_t INA_VSHCT  = 5;    // シャント電圧 変換時間 5 = 1052us
constexpr uint8_t INA_VTCT   = 5;    // 温度 変換時間 (MODE=0xB では未使用)
constexpr uint8_t INA_AVG    = 3;    // 平均回数 3 = 64 回 → (1052+1052)us x 64 ≒ 135ms ごとに更新
constexpr uint8_t INA_ADCRANGE = 0;  // 0 = ±163.84mV (1 にする場合は SHUNT_CAL を 4 倍する必要あり)

// ============================================================================
//  LCD (RW1063) の I2C 仕様  (RW1063-0A-001 データシート Rev.0.5 で確認)
//  - 7bit アドレスは 0 1 1 1 1 SA1 SA0 → 0x3C〜0x3F (p.28)。SA1/SA0 はモジュール基板の配線で決まる
//  - 送り方は [アドレス][制御バイト][本体...] (p.28)。制御バイトは Co A0 0 0 0 0 0 0 で、
//    Co=0 にするとそれ以降 STOP まで同じ種類が続く。A0=0 → コマンド (0x00), A0=1 → 表示データ (0x40)
//  - I2C では Busy Flag を読めないので、コマンドごとに待ち時間を入れる (p.21)
//  - コマンド本体は HD44780 互換 (p.17)。実行時間は 540kHz で 18.5us、全消去 0.76ms。
//    発振周波数は最小 380kHz なので最悪 26us / 1.1ms (p.36-37)
// ============================================================================
constexpr uint8_t LCD_ADDR_FIRST = 0x3C;
constexpr uint8_t LCD_ADDR_LAST  = 0x3F;
constexpr uint8_t LCD_CTRL_CMD  = 0x00;
constexpr uint8_t LCD_CTRL_DATA = 0x40;
constexpr uint8_t LCD_CMD_FUNCTION = 0x38;  // Function set: DL=1 (I2C では必須), N=1 (2 行モード), F=0 (5x8)
constexpr uint8_t LCD_CMD_DISPLAY  = 0x0C;  // Display ON, カーソル OFF, 点滅 OFF
constexpr uint8_t LCD_CMD_CLEAR    = 0x01;  // 全消去
constexpr uint8_t LCD_CMD_ENTRY    = 0x06;  // Entry mode: アドレス自動インクリメント, シフトなし
constexpr uint8_t LCD_CMD_DDRAM    = 0x80;  // 0x80 | アドレス で書き込み位置を指定
// 20x4 の各行の先頭 DDRAM アドレス。RW1063 の 2 行モード (1 行目 00h〜27h, 2 行目 40h〜67h, p.13/p.20)
// の各 40 文字を 20 文字ずつ折り返して 4 行にしている (表示行 1→00h, 2→40h, 3→14h, 4→54h)
const uint8_t LCD_ROW_ADDR[4] PROGMEM = {0x00, 0x40, 0x14, 0x54};
constexpr uint8_t LCD_COLS = 20;
constexpr uint8_t LCD_ROWS = 4;

// ============================================================================
//  INA228 レジスタと換算係数
// ============================================================================
enum : uint8_t {
  REG_CONFIG     = 0x00,  // 16bit
  REG_ADC_CONFIG = 0x01,  // 16bit
  REG_SHUNT_CAL  = 0x02,  // 16bit (15bit 有効)
  REG_VBUS       = 0x05,  // 24bit (上位 20bit)
  REG_CURRENT    = 0x07,  // 24bit (上位 20bit, 符号付き)
  REG_POWER      = 0x08,  // 24bit (符号なし)
  REG_CHARGE     = 0x0A,  // 40bit (符号付き)
  REG_MFG_ID     = 0x3E,  // 16bit = 0x5449 ("TI")
};
constexpr uint16_t CONFIG_RST    = 0x8000;  // bit15: 全レジスタをリセット
constexpr uint16_t CONFIG_RSTACC = 0x4000;  // bit14: ENERGY / CHARGE を 0 に (自動で 0 に戻る)
constexpr uint16_t CONFIG_VALUE  = (uint16_t)INA_ADCRANGE << 4;  // CONVDLY=0, TEMPCOMP=0
constexpr uint16_t ADC_CONFIG_VALUE =
    ((uint16_t)INA_MODE << 12) | ((uint16_t)INA_VBUSCT << 9) |
    ((uint16_t)INA_VSHCT << 6) | ((uint16_t)INA_VTCT << 3) | INA_AVG;

// CURRENT_LSB = 最大電流 / 2^19
constexpr float CURRENT_LSB_NOMINAL = MAX_CURRENT_A / 524288.0f;
// SHUNT_CAL = 13107.2e6 x CURRENT_LSB x R_SHUNT  (ADCRANGE=0 の式)
constexpr float SHUNT_CAL_F = 13107.2e6f * CURRENT_LSB_NOMINAL * R_SHUNT_OHM;
static_assert(SHUNT_CAL_F >= 1.0f && SHUNT_CAL_F <= 32767.0f, "SHUNT_CAL が 15bit に収まらない");
static_assert(INA_ADCRANGE == 0, "ADCRANGE=1 の場合は SHUNT_CAL を 4 倍する処理を追加すること");
static_assert(MAX_CURRENT_A * R_SHUNT_OHM <= 0.16384f, "最大電流 x シャント抵抗が ±163.84mV を超える");
constexpr uint16_t SHUNT_CAL = (uint16_t)(SHUNT_CAL_F + 0.5f);
// 整数に丸めた SHUNT_CAL から逆算した実際の CURRENT_LSB (70A/2mΩ なら SHUNT_CAL=3500 で誤差なし)
constexpr float CURRENT_LSB   = (float)SHUNT_CAL / (13107.2e6f * R_SHUNT_OHM);
constexpr float VBUS_LSB_V    = 195.3125e-6f;                         // バス電圧 LSB
constexpr float POWER_LSB_W   = 3.2f * CURRENT_LSB;                    // 電力 LSB
constexpr float CHARGE_LSB_MAH = CURRENT_LSB / 3.6f;                   // 電荷 LSB [C]→[mAh]

// ============================================================================
//  グローバル
// ============================================================================
File logFile;
char logName[13] = "";          // 記録中のファイル名 (8.3 形式)

struct Measurement {
  float voltage;   // [V]
  float current;   // [A]
  float power;     // [W]
  float chargemAh; // [mAh] = CHARGE レジスタ + chargeOffsetmAh (回生は差し引かれる)
  bool  ok;        // 直近の読み出しが成功したか
};
Measurement meas = {0, 0, 0, 0, false};
bool haveMeas = false;            // 起動後に一度でも正しく読めたか

// INA228 が走行中に設定を失った (電源瞬断など) ときに、それまでの積算を引き継ぐための値。
// 積算そのものは INA228 の CHARGE レジスタだけで行い、ソフトでは足し込まない
float    chargeOffsetmAh = 0;
uint8_t  inaRecoverCount = 0;      // INA228 を再設定した回数 (LCD 3 行目に表示)

bool     sdOk            = false;
bool     heartbeat       = false;  // 記録するたびに反転 (4 行目末尾の * で動作確認)
uint16_t nextLogIndex    = 0;      // 次に探し始めるファイル番号
uint8_t  linesSinceFlush = 0;
uint32_t timerStartMs    = 0;      // 経過時間の基準 (起動時・リセット時)
uint32_t nextLogMs       = 0;
uint32_t nextLcdMs       = 0;
uint32_t lastLcdInitMs   = 0;
uint32_t btnDownMs       = 0;
bool     btnDown         = false;
bool     btnHandled      = false;

// Nano がリセットされても消えない RAM 領域 (.noinit)。電源を切ると中身は不定になるので
// チェック値で正しさを確かめてから使う
struct Persist {
  uint32_t magic;
  uint32_t elapsedMs;
  float    chargeOffsetmAh;
  uint32_t check;
};
Persist persist __attribute__((section(".noinit")));
constexpr uint32_t PERSIST_MAGIC = 0xE1E1C0DEUL;

// ============================================================================
//  ウォッチドッグ
//  注意: 旧ブートローダ (ATmegaBOOT) の Nano 互換機では、ウォッチドッグでリセットされると
//  再起動を繰り返すことがある (電源を入れ直せば戻る)。固まった状態と比べて悪化はしないが、
//  気になる場合は USE_WATCHDOG を 0 にするか、Optiboot を書き込む
// ============================================================================
inline void wdtKick() {
#if USE_WATCHDOG
  wdt_reset();
#endif
}

// ============================================================================
//  I2C スキャン (デバッグ用)
// ============================================================================
#if SERIAL_DEBUG
void i2cScan() {
  Serial.print(F("I2C scan:"));
  uint8_t found = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F(" 0x"));
      if (a < 0x10) Serial.print('0');
      Serial.print(a, HEX);
      if (a == INA228_ADDR) Serial.print(F("(INA228)"));
      if (a >= LCD_ADDR_FIRST && a <= LCD_ADDR_LAST) Serial.print(F("(LCD?)"));
      found++;
    }
  }
  if (!found) Serial.print(F(" none"));
  Serial.println();
}
#endif

// ============================================================================
//  LCD 軽量ドライバ (RW1063, I2C)
// ============================================================================
uint8_t lcdAddr = LCD_ADDR;  // 実際に使う LCD のアドレス (起動時に決める)

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// コマンドを 1 つ送り、waitUs だけ待つ (I2C では Busy Flag を読めないため)
bool lcdCommand(uint8_t cmd, uint16_t waitUs = 50) {
  Wire.beginTransmission(lcdAddr);
  Wire.write(LCD_CTRL_CMD);
  Wire.write(cmd);
  bool ok = (Wire.endTransmission() == 0);
  delayMicroseconds(waitUs);
  return ok;
}

// 表示を消さずに設定だけ送り直す (走行中のノイズ対策で定期的に呼ぶ)
bool lcdResendSettings() {
  bool ok = lcdCommand(LCD_CMD_FUNCTION, 120);
  ok &= lcdCommand(LCD_CMD_DISPLAY, 120);
  ok &= lcdCommand(LCD_CMD_ENTRY, 120);
  return ok;
}

// データシート p.32「Serial Interface」の初期化手順どおり
bool lcdInit() {
  delay(50);                                   // VDD が 4.5V に達してから 40ms 以上待つ
  if (!i2cPresent(lcdAddr)) {                  // 指定アドレスに居なければ 0x3C〜0x3F を探す
    for (uint8_t a = LCD_ADDR_FIRST; a <= LCD_ADDR_LAST; a++)
      if (i2cPresent(a)) { lcdAddr = a; break; }
  }
  bool ok = lcdCommand(LCD_CMD_FUNCTION, 120); // Function set → 100us 以上
  ok &= lcdCommand(LCD_CMD_DISPLAY, 120);      // Display ON/OFF → 100us 以上
  ok &= lcdCommand(LCD_CMD_CLEAR, 0);          // Clear Display → 10ms 以上
  delay(11);
  ok &= lcdCommand(LCD_CMD_ENTRY, 120);        // Entry mode set → 100us 以上
  return ok;
}

// 1 行を書く。20 文字に満たない分は空白で埋める (全消去しないのでちらつかない)
void lcdLine(uint8_t row, const char *s) {
  if (row >= LCD_ROWS) return;
  lcdCommand(LCD_CMD_DDRAM | pgm_read_byte(&LCD_ROW_ADDR[row]));
  Wire.beginTransmission(lcdAddr);      // 制御バイト + 20 バイト = 21 < Wire バッファ 32
  Wire.write(LCD_CTRL_DATA);
  uint8_t n = 0;
  while (n < LCD_COLS && s[n]) Wire.write((uint8_t)s[n++]);
  while (n++ < LCD_COLS) Wire.write(' ');
  Wire.endTransmission();
}

void lcdLineP(uint8_t row, const __FlashStringHelper *s) {
  char buf[LCD_COLS + 1];
  strncpy_P(buf, (const char *)s, LCD_COLS);
  buf[LCD_COLS] = '\0';
  lcdLine(row, buf);
}

// ============================================================================
//  INA228 低レベルアクセス
// ============================================================================
bool inaRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(INA228_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool inaRead16(uint8_t reg, uint16_t &val) {
  uint8_t b[2];
  if (!inaRead(reg, b, 2)) return false;
  val = ((uint16_t)b[0] << 8) | b[1];
  return true;
}

bool inaRead24(uint8_t reg, uint32_t &val) {
  uint8_t b[3];
  if (!inaRead(reg, b, 3)) return false;
  val = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
  return true;
}

bool inaWrite16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

// 24bit レジスタの上位 20bit を符号付き整数にする
int32_t toSigned20(uint32_t raw24) {
  return (int32_t)(raw24 << 8) >> 12;  // 上位に寄せてから算術シフトで符号拡張
}

// INA228 がこのスケッチの設定のまま動いているか (電源投入直後は SHUNT_CAL=0 などの既定値)
enum : uint8_t { INA_CONFIGURED, INA_NOT_CONFIGURED, INA_NO_RESPONSE };
uint8_t inaCheckConfig() {  // 戻り値は上の INA_* のどれか
  uint16_t cal, adc, cfg;
  if (!inaRead16(REG_SHUNT_CAL, cal) || !inaRead16(REG_ADC_CONFIG, adc) ||
      !inaRead16(REG_CONFIG, cfg))
    return INA_NO_RESPONSE;
  bool same = (cal == SHUNT_CAL) && (adc == ADC_CONFIG_VALUE) &&
              ((cfg & ~(CONFIG_RST | CONFIG_RSTACC)) == CONFIG_VALUE);
  return same ? INA_CONFIGURED : INA_NOT_CONFIGURED;
}

// 全レジスタを初期化してから設定する (CHARGE も 0 になる)
bool inaConfigure() {
  if (!inaWrite16(REG_CONFIG, CONFIG_RST)) return false;
  delay(2);
  if (!inaWrite16(REG_CONFIG, CONFIG_VALUE)) return false;
  if (!inaWrite16(REG_ADC_CONFIG, ADC_CONFIG_VALUE)) return false;
  if (!inaWrite16(REG_SHUNT_CAL, SHUNT_CAL)) return false;
  return inaCheckConfig() == INA_CONFIGURED;  // 書けたか読み戻して確認
}

bool inaResetCharge() {
  return inaWrite16(REG_CONFIG, CONFIG_VALUE | CONFIG_RSTACC);
}

// 走行中に INA228 の設定が消えていたら、それまでの積算を引き継いで再設定する
void inaRecover() {
  // 最後に正しく読めた積算値を引き継ぐ (起動後まだ読めていなければ今のオフセットのまま)
  if (haveMeas) chargeOffsetmAh = meas.chargemAh;
  bool ok = inaConfigure();
  if (inaRecoverCount < 9) inaRecoverCount++;
#if SERIAL_DEBUG
  Serial.print(F("# INA228 re-configured, ok="));
  Serial.println(ok);
#else
  (void)ok;
#endif
}

// 電圧・電流・電力・積算 mAh をまとめて読む。
// 読み出しの前後で SHUNT_CAL を確かめ、途中で INA228 がリセットされた値は採用しない
bool inaMeasure(Measurement &m) {
  uint16_t cal;
  uint32_t rv, ri, rp;
  uint8_t q[5];

  if (!inaRead16(REG_SHUNT_CAL, cal)) return m.ok = false;
  if (cal != SHUNT_CAL) {           // 設定が消えている (電源瞬断など)
    inaRecover();
    return m.ok = false;            // 今回は値を使わない (次回から正常に読める)
  }
  if (!inaRead24(REG_VBUS, rv) || !inaRead24(REG_CURRENT, ri) ||
      !inaRead24(REG_POWER, rp) || !inaRead(REG_CHARGE, q, 5))
    return m.ok = false;
  if (!inaRead16(REG_SHUNT_CAL, cal) || cal != SHUNT_CAL) return m.ok = false;

  m.voltage = toSigned20(rv) * VBUS_LSB_V;
  m.current = toSigned20(ri) * CURRENT_LSB;
  m.power   = rp * POWER_LSB_W;     // 24bit 全部が値 (符号なし)
  // CHARGE は 40bit の 2 の補数。float で上位と下位を足すと負の値で桁落ちするので、
  // 64bit 整数で組み立ててから float にする
  int64_t qraw = ((int64_t)(int8_t)q[0] << 32) | ((uint32_t)q[1] << 24) |
                 ((uint32_t)q[2] << 16) | ((uint32_t)q[3] << 8) | q[4];
  m.chargemAh = (float)qraw * CHARGE_LSB_MAH + chargeOffsetmAh;
  haveMeas = true;
  return m.ok = true;
}

// ============================================================================
//  リセットされても消えない値 (経過時間・積算オフセット)
// ============================================================================
uint32_t persistCheckValue(const Persist &p) {
  uint32_t qoff;
  memcpy(&qoff, &p.chargeOffsetmAh, sizeof(qoff));
  return p.magic ^ p.elapsedMs ^ qoff ^ 0x5A5AA5A5UL;
}

void persistSave(uint32_t now) {
  persist.magic = PERSIST_MAGIC;
  persist.elapsedMs = now - timerStartMs;
  persist.chargeOffsetmAh = chargeOffsetmAh;
  persist.check = persistCheckValue(persist);
}

bool persistValid() {
  return persist.magic == PERSIST_MAGIC && persist.check == persistCheckValue(persist);
}

// ============================================================================
//  画面表示 (20 文字 x 4 行)
// ============================================================================
void lcdShowMeasurement(uint32_t now) {
  char line[LCD_COLS + 1];

  if (meas.ok) {
    char a[10], b[12];
    // dtostrf のバッファあふれを防ぐため表示範囲に制限
    dtostrf(constrain(meas.voltage, -9.99f, 99.99f), 6, 2, a);   // " 28.00"
    dtostrf(constrain(meas.current, -99.99f, 99.99f), 6, 2, b);  // "-70.00"
    // 1行目: "V: 28.00V  I:-70.00A" (20 文字)
    snprintf_P(line, sizeof(line), PSTR("V:%sV  I:%sA"), a, b);
    lcdLine(0, line);

    long pw = lround(constrain(meas.power, 0.0f, 9999.0f));
    long mah = lround(constrain(meas.chargemAh, -99999.0f, 999999.0f));
    // 2行目: "P: 245W  Q:  1234mAh" (20 文字)
    snprintf_P(line, sizeof(line), PSTR("P:%4ldW  Q:%6ldmAh"), pw, mah);
    lcdLine(1, line);
  } else {
    lcdLineP(0, F("INA228 I2C ERROR"));
    lcdLineP(1, F("CHECK WIRING"));
  }

  // 3行目: 経過時間 (リセットから) "TIME  12:34"  (100 分以上は "123:45")
  //        INA228 を再設定したことがあれば末尾に "INA!回数" を出す
  uint32_t s = (now - timerStartMs) / 1000;
  int n = snprintf_P(line, sizeof(line), PSTR("TIME  %02lu:%02u"), s / 60, (unsigned)(s % 60));
  if (inaRecoverCount && n > 0 && n < 14)
    snprintf_P(line + n, sizeof(line) - n, PSTR("   INA!%u"), inaRecoverCount);
  lcdLine(2, line);

  // 4行目: SD 状態 "REC LOG003.CSV     *" / "NO SD  (PUSH=RETRY)"
  if (sdOk) {
    snprintf_P(line, sizeof(line), PSTR("REC %-15s%c"), logName, heartbeat ? '*' : ' ');
    lcdLine(3, line);
  } else {
    lcdLineP(3, F("NO SD  (PUSH=RETRY)"));
  }
}

// ============================================================================
//  SD ログ
// ============================================================================
// LOG000.CSV〜LOG999.CSV の空き番号で新規作成し、logName に名前を入れる
bool openNewLog() {
  char name[13];
  for (; nextLogIndex < 1000; nextLogIndex++) {
    snprintf_P(name, sizeof(name), PSTR("LOG%03u.CSV"), nextLogIndex);
    if (SD.exists(name)) continue;
    logFile = SD.open(name, FILE_WRITE);
    if (!logFile) return false;
    nextLogIndex++;
    logFile.println(F("time_s,voltage_V,current_A,power_W,charge_mAh"));
    logFile.flush();
    if (logFile.getWriteError()) {
      logFile.close();
      return false;
    }
    strcpy(logName, name);
    linesSinceFlush = 0;
    return true;
  }
  return false;  // 1000 ファイル使い切り
}

// SD を (再) 初期化してログを開く。カードが無いと最大 2 秒ほどかかる
void startSdLog() {
  if (logFile) logFile.close();
  wdtKick();
  sdOk = SD.begin(PIN_SD_CS) && openNewLog();
  wdtKick();
  if (!sdOk) logName[0] = '\0';
#if SERIAL_DEBUG
  Serial.print(F("SD: "));
  if (sdOk) Serial.println(logName); else Serial.println(F("NO SD"));
#endif
}

// 1 行出力: 経過時間[s], 電圧[V], 電流[A], 電力[W], 積算[mAh]
void printCsvRow(Print &out, uint32_t elapsedMs) {
  char buf[16];
  snprintf_P(buf, sizeof(buf), PSTR("%lu.%u,"), elapsedMs / 1000,
             (unsigned)((elapsedMs % 1000) / 100));
  out.print(buf);
  out.print(dtostrf(meas.voltage, 1, 3, buf));  out.print(',');
  out.print(dtostrf(meas.current, 1, 3, buf));  out.print(',');
  out.print(dtostrf(meas.power, 1, 2, buf));    out.print(',');
  out.println(dtostrf(meas.chargemAh, 1, 1, buf));
}

void writeLog(uint32_t now) {
  if (!meas.ok) return;  // センサー読み取り失敗時は行を書かない
  printCsvRow(logFile, now - timerStartMs);
  if (++linesSinceFlush >= LOG_FLUSH_EVERY) {
    logFile.flush();  // ディレクトリ情報まで書き込み → 電源断でもここまでは残る
    linesSinceFlush = 0;
  }
  if (logFile.getWriteError()) {  // カード抜け等
    logFile.close();
    sdOk = false;
    logName[0] = '\0';
  }
  heartbeat = !heartbeat;
}

// ============================================================================
//  ボタン操作
// ============================================================================
void restartTimer(uint32_t elapsedMs) {
  uint32_t now = millis();
  timerStartMs = now - elapsedMs;
  nextLogMs = now + LOG_INTERVAL_MS;
  nextLcdMs = now;
}

// 長押し: 積算 mAh・経過時間を 0 にして新しいログ
void doCounterReset() {
  bool ok = inaResetCharge();
  chargeOffsetmAh = 0;
  inaRecoverCount = 0;
  meas.chargemAh = 0;
  lcdLineP(0, ok ? F("RESET mAh") : F("RESET FAILED"));
  lcdLineP(1, F(""));
  lcdLineP(2, F(""));
  lcdLineP(3, F("SD ..."));
  startSdLog();  // 新しいログファイル (SD が無かった場合はここで再初期化を試す)
#if SERIAL_DEBUG
  Serial.println(F("# mAh reset"));
#endif
  delay(800);    // メッセージ表示
  restartTimer(0);
  persistSave(millis());
}

// 短押し (NO SD のときだけ): 積算と経過時間はそのままで SD を再初期化
void doSdRetry() {
  lcdLineP(3, F("SD RETRY ..."));
  startSdLog();
  nextLcdMs = millis();
}

void handleButton(uint32_t now) {
  bool pressed = (digitalRead(PIN_BTN) == LOW);
  if (pressed) {
    if (!btnDown) {          // 押し始め (チャタリングで離れたらタイマーはやり直し)
      btnDown = true;
      btnDownMs = now;
    } else if (!btnHandled && now - btnDownMs >= RESET_HOLD_MS) {
      btnHandled = true;     // 離すまで再実行しない
      doCounterReset();
    }
    return;
  }
  // 離したとき: 長押しに届かなかった短押しなら SD 再試行
  if (btnDown && !btnHandled && !sdOk && now - btnDownMs >= BTN_SHORT_MIN_MS)
    doSdRetry();
  btnDown = false;
  btnHandled = false;
}

// ============================================================================
//  setup / loop
// ============================================================================
void haltWithMessage() {
  while (true) wdtKick();  // 仕様どおり停止 (ウォッチドッグでの再起動もしない)
}

void setup() {
#if USE_WATCHDOG
  wdt_enable(WDTO_8S);
#endif
  pinMode(PIN_BTN, INPUT_PULLUP);
#if SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println(F("Ene-1 PowerMeter"));
#endif
  Wire.begin();
  Wire.setClock(100000);           // 配線が長いので 100kHz
  Wire.setWireTimeout(3000, true); // ノイズで I2C が固まっても 3ms で復帰

#if SERIAL_DEBUG
  i2cScan();
#endif

  bool lcdOk = lcdInit();
#if SERIAL_DEBUG
  Serial.print(F("LCD: 0x"));
  Serial.print(lcdAddr, HEX);
  Serial.println(lcdOk ? F(" OK") : F(" no ACK (check wiring)"));
#else
  (void)lcdOk;
#endif
  lastLcdInitMs = millis();
  lcdLineP(0, F("Ene-1 PowerMeter"));
  lcdLineP(1, F("starting..."));

  // メーカー ID 確認 (0x3E = 0x5449)
  uint16_t id = 0;
  if (!inaRead16(REG_MFG_ID, id) || id != 0x5449) {
    char line[LCD_COLS + 1];
    lcdLineP(0, F("INA228 NOT FOUND"));
    snprintf_P(line, sizeof(line), PSTR("ID=%04X"), id);
    lcdLine(1, line);
    lcdLineP(2, F("CHECK I2C WIRING"));
    lcdLineP(3, F("STOPPED"));
#if SERIAL_DEBUG
    Serial.println(F("INA228 not found"));
#endif
    haltWithMessage();
  }

  // INA228 の設定が残っている = INA228 は電源が切れておらず Nano だけがリセットされた。
  // このときは積算 mAh を消さずに続きから再開する
  uint32_t resumeElapsedMs = 0;
  bool resumed = (inaCheckConfig() == INA_CONFIGURED);
  if (resumed) {
    if (persistValid()) {
      resumeElapsedMs = persist.elapsedMs;
      chargeOffsetmAh = persist.chargeOffsetmAh;
    }
    lcdLineP(1, F("RESUMED (mAh KEPT)"));
  } else {
    if (!inaConfigure()) {
      lcdLineP(0, F("INA228 CONFIG ERROR"));
      lcdLineP(3, F("STOPPED"));
#if SERIAL_DEBUG
      Serial.println(F("INA228 config error"));
#endif
      haltWithMessage();
    }
    lcdLineP(1, F("INA228 OK"));
  }
#if SERIAL_DEBUG
  Serial.print(F("INA228: "));
  Serial.println(resumed ? F("resumed (mAh kept)") : F("configured"));
  Serial.print(F("SHUNT_CAL=")); Serial.println(SHUNT_CAL);
#endif

  startSdLog();
  lcdLineP(2, sdOk ? F("SD OK") : F("NO SD"));
  if (sdOk) lcdLine(3, logName);
#if SERIAL_DEBUG
  Serial.println(F("time_s,voltage_V,current_A,power_W,charge_mAh"));
#endif
  delay(1500);

  restartTimer(resumeElapsedMs);
  persistSave(millis());
}

void loop() {
  wdtKick();
  uint32_t now = millis();

  handleButton(now);

  if ((int32_t)(now - nextLcdMs) >= 0) {
    nextLcdMs += LCD_INTERVAL_MS;
    if ((int32_t)(now - nextLcdMs) >= 0) nextLcdMs = now + LCD_INTERVAL_MS;  // 遅れたら追いつかない
    if (now - lastLcdInitMs >= LCD_REINIT_MS) {  // ノイズで化けた LCD を立て直す
      lcdResendSettings();
      lastLcdInitMs = now;
    }
    inaMeasure(meas);
    lcdShowMeasurement(now);
    persistSave(now);
  }

  if ((int32_t)(now - nextLogMs) >= 0) {
    nextLogMs += LOG_INTERVAL_MS;    // 1 秒刻みを保つ (ずれが溜まらない)
    if ((int32_t)(now - nextLogMs) >= 0) nextLogMs = now + LOG_INTERVAL_MS;
    inaMeasure(meas);                // 記録用に最新値を読む
    if (sdOk) writeLog(now);
#if SERIAL_DEBUG
    printCsvRow(Serial, now - timerStartMs);
#endif
  }
}
