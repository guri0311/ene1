/*
 * Ene-1 GP 車載電力計
 *   マイコン   : Arduino Nano (ATmega328P / 5V / 16MHz)
 *   センサー   : INA228 (Strawberry Linux #11228, シャント 2mΩ, I2C 0x40)
 *   表示       : 秋月 I2C キャラクター液晶 20x4 (ACM2004D-FLW-FBW-IIC, 制御IC RW1063)
 *   記録       : microSD (SPI, CS = D10)
 *   リセット   : D2 - GND 間のボタンを 1 秒長押し → 積算 Wh・経過時間を 0 に、新しいログファイル
 *
 * 必要なライブラリ: Wire, SPI (標準) / SD
 *   LCD は自作の軽量ドライバ (lcd* 関数) で直接制御する
 *
 * ピン配置
 *   A4 = SDA, A5 = SCL  (INA228 と LCD で共用。4.7kΩ で 5V へ外付けプルアップ)
 *   D11 = MOSI, D12 = MISO, D13 = SCK, D10 = CS (SD カード)
 *   D2  = 積算リセットボタン (内部プルアップ、押すと LOW)
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ============================================================================
//  ユーザー設定 (ここを変えるだけで調整できる)
// ============================================================================
constexpr float    R_SHUNT_OHM     = 0.002f;   // シャント抵抗 [Ω]
constexpr float    MAX_CURRENT_A   = 70.0f;    // 想定最大電流 [A] (CURRENT_LSB の計算に使う)
constexpr uint8_t  INA228_ADDR     = 0x40;     // INA228 の I2C アドレス (A0=A1=GND)
constexpr uint8_t  LCD_ADDR        = 0x3C;     // ※要確認: 秋月サンプルで照合する (7bit アドレス)
constexpr uint16_t LCD_INTERVAL_MS = 250;      // LCD 更新間隔 [ms]
constexpr uint16_t LOG_INTERVAL_MS = 1000;     // SD 記録間隔 [ms]
constexpr uint8_t  LOG_FLUSH_EVERY = 5;        // 何行書くごとに flush するか (1 = 毎行)
constexpr uint16_t RESET_HOLD_MS   = 1000;     // リセットボタンの長押し時間 [ms]
constexpr uint8_t  PIN_SD_CS       = 10;
constexpr uint8_t  PIN_RESET_BTN   = 2;
#define SERIAL_DEBUG 1   // 1: 起動時の I2C スキャン結果と CSV を USB シリアル (115200bps) に出す

// INA228 の ADC 設定 (データシート ADC_CONFIG レジスタ)
constexpr uint8_t INA_MODE   = 0xB;  // 0xB = シャント電圧 + バス電圧 連続測定
constexpr uint8_t INA_VBUSCT = 5;    // バス電圧 変換時間 5 = 1052us
constexpr uint8_t INA_VSHCT  = 5;    // シャント電圧 変換時間 5 = 1052us
constexpr uint8_t INA_VTCT   = 5;    // 温度 変換時間 (MODE=0xB では未使用)
constexpr uint8_t INA_AVG    = 3;    // 平均回数 3 = 64 回 → (1052+1052)us x 64 ≒ 135ms ごとに更新
constexpr uint8_t INA_ADCRANGE = 0;  // 0 = ±163.84mV (1 にする場合は SHUNT_CAL を 4 倍する必要あり)

// ============================================================================
//  LCD (RW1063) のコマンド  ※要確認: 秋月サンプルで照合する
//    I2C では [制御バイト][本体] の順に送る。制御バイト 0x00 = コマンド, 0x40 = 表示データ
//    コマンド本体は HD44780 互換
// ============================================================================
constexpr uint8_t LCD_CTRL_CMD  = 0x00;
constexpr uint8_t LCD_CTRL_DATA = 0x40;
constexpr uint8_t LCD_CMD_CLEAR = 0x01;  // 全消去 (実行に約 1.5ms)
constexpr uint8_t LCD_CMD_DDRAM = 0x80;  // 0x80 | アドレス で書き込み位置を指定
// 初期化コマンド列 (順に送る)
const uint8_t LCD_INIT_SEQ[] PROGMEM = {
  0x38,  // Function set: 8bit, 2 行モード (20x4 は 2 行モードを折り返して使う)
  0x0C,  // Display ON, カーソル OFF, 点滅 OFF
  0x06,  // Entry mode: アドレス自動インクリメント, シフトなし
};
// 20x4 の各行の先頭 DDRAM アドレス
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
  REG_ENERGY     = 0x09,  // 40bit (符号なし)
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
constexpr float ENERGY_LSB_WH = 16.0f * 3.2f * CURRENT_LSB / 3600.0f;  // 積算 LSB [J]→[Wh]

// ============================================================================
//  グローバル
// ============================================================================
File logFile;
char logName[13] = "";          // 記録中のファイル名 (8.3 形式)

struct Measurement {
  float voltage;   // [V]
  float current;   // [A]
  float power;     // [W]
  float energyWh;  // [Wh]
  bool  ok;        // I2C 読み出し成功
};
Measurement meas = {0, 0, 0, 0, false};

bool     sdOk            = false;
bool     heartbeat       = false;  // 記録するたびに反転 (4 行目末尾の * で動作確認)
uint16_t nextLogIndex    = 0;      // 次に探し始めるファイル番号
uint8_t  linesSinceFlush = 0;
uint32_t timerStartMs    = 0;      // 経過時間の基準 (起動時・リセット時)
uint32_t nextLogMs       = 0;
uint32_t nextLcdMs       = 0;
uint32_t btnDownMs       = 0;
bool     btnDown         = false;
bool     btnHandled      = false;

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
      if (a == LCD_ADDR)    Serial.print(F("(LCD)"));
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
bool lcdCommand(uint8_t cmd) {
  Wire.beginTransmission(LCD_ADDR);
  Wire.write(LCD_CTRL_CMD);
  Wire.write(cmd);
  bool ok = (Wire.endTransmission() == 0);
  if (cmd == LCD_CMD_CLEAR) delay(2);   // 全消去は約 1.5ms
  else delayMicroseconds(50);           // 通常コマンドは約 40us
  return ok;
}

bool lcdInit() {
  delay(50);  // 電源投入後の待ち
  bool ok = true;
  for (uint8_t i = 0; i < sizeof(LCD_INIT_SEQ); i++)
    ok &= lcdCommand(pgm_read_byte(&LCD_INIT_SEQ[i]));
  ok &= lcdCommand(LCD_CMD_CLEAR);
  return ok;
}

// 1 行を書く。20 文字に満たない分は空白で埋める (全消去しないのでちらつかない)
void lcdLine(uint8_t row, const char *s) {
  if (row >= LCD_ROWS) return;
  lcdCommand(LCD_CMD_DDRAM | pgm_read_byte(&LCD_ROW_ADDR[row]));
  Wire.beginTransmission(LCD_ADDR);     // 制御バイト + 20 バイト = 21 < Wire バッファ 32
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

bool inaConfigure() {
  if (!inaWrite16(REG_CONFIG, CONFIG_RST)) return false;  // 全レジスタ初期化
  delay(2);
  if (!inaWrite16(REG_CONFIG, CONFIG_VALUE)) return false;
  if (!inaWrite16(REG_ADC_CONFIG, ADC_CONFIG_VALUE)) return false;
  if (!inaWrite16(REG_SHUNT_CAL, SHUNT_CAL)) return false;
  uint16_t chk;  // 書けたか読み戻して確認
  if (!inaRead16(REG_SHUNT_CAL, chk) || chk != SHUNT_CAL) return false;
  if (!inaRead16(REG_ADC_CONFIG, chk) || chk != ADC_CONFIG_VALUE) return false;
  return true;
}

bool inaResetEnergy() {
  return inaWrite16(REG_CONFIG, CONFIG_VALUE | CONFIG_RSTACC);
}

// 電圧・電流・電力・積算をまとめて読む
bool inaMeasure(Measurement &m) {
  uint32_t raw;
  uint8_t e[5];

  if (!inaRead24(REG_VBUS, raw)) return m.ok = false;
  m.voltage = toSigned20(raw) * VBUS_LSB_V;

  if (!inaRead24(REG_CURRENT, raw)) return m.ok = false;
  m.current = toSigned20(raw) * CURRENT_LSB;

  if (!inaRead24(REG_POWER, raw)) return m.ok = false;
  m.power = raw * POWER_LSB_W;  // 24bit 全部が値 (符号なし)

  if (!inaRead(REG_ENERGY, e, 5)) return m.ok = false;  // 40bit 符号なし
  uint32_t lo = ((uint32_t)e[1] << 24) | ((uint32_t)e[2] << 16) |
                ((uint32_t)e[3] << 8) | e[4];
  m.energyWh = ((float)e[0] * 4294967296.0f + (float)lo) * ENERGY_LSB_WH;

  return m.ok = true;
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
    dtostrf(constrain(meas.energyWh, 0.0f, 999.999f), 7, 3, b);  // "  1.234"
    // 2行目: "P: 245W  E:  1.234Wh" (20 文字)
    snprintf_P(line, sizeof(line), PSTR("P:%4ldW  E:%sWh"), pw, b);
    lcdLine(1, line);
  } else {
    lcdLineP(0, F("INA228 I2C ERROR"));
    lcdLineP(1, F("CHECK WIRING"));
  }

  // 3行目: 経過時間 (リセットから) "TIME  12:34"  (100 分以上は "123:45")
  uint32_t s = (now - timerStartMs) / 1000;
  snprintf_P(line, sizeof(line), PSTR("TIME  %02lu:%02u"), s / 60, (unsigned)(s % 60));
  lcdLine(2, line);

  // 4行目: SD 状態 "REC LOG003.CSV     *" / "NO SD  (HOLD=RETRY)"
  if (sdOk) {
    snprintf_P(line, sizeof(line), PSTR("REC %-15s%c"), logName, heartbeat ? '*' : ' ');
    lcdLine(3, line);
  } else {
    lcdLineP(3, F("NO SD  (HOLD=RETRY)"));
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
    logFile.println(F("time_s,voltage_V,current_A,power_W,energy_Wh"));
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

// SD を (再) 初期化してログを開く
void startSdLog() {
  if (logFile) logFile.close();
  sdOk = SD.begin(PIN_SD_CS) && openNewLog();
  if (!sdOk) logName[0] = '\0';
#if SERIAL_DEBUG
  Serial.print(F("SD: "));
  if (sdOk) Serial.println(logName); else Serial.println(F("NO SD"));
#endif
}

// 1 行出力: 経過時間[s], 電圧[V], 電流[A], 電力[W], 積算[Wh]
void printCsvRow(Print &out, uint32_t elapsedMs) {
  char buf[16];
  snprintf_P(buf, sizeof(buf), PSTR("%lu.%u,"), elapsedMs / 1000,
             (unsigned)((elapsedMs % 1000) / 100));
  out.print(buf);
  out.print(dtostrf(meas.voltage, 1, 3, buf));  out.print(',');
  out.print(dtostrf(meas.current, 1, 3, buf));  out.print(',');
  out.print(dtostrf(meas.power, 1, 2, buf));    out.print(',');
  out.println(dtostrf(meas.energyWh, 1, 4, buf));
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
//  リセット (積算 Wh・経過時間を 0、新しいログ)
// ============================================================================
void restartTimer() {
  timerStartMs = millis();
  nextLogMs = timerStartMs + LOG_INTERVAL_MS;
  nextLcdMs = timerStartMs;
}

void doEnergyReset() {
  bool ok = inaResetEnergy();
  lcdLineP(0, ok ? F("ENERGY RESET") : F("RESET FAILED"));
  lcdLineP(1, F(""));
  lcdLineP(2, F(""));
  lcdLineP(3, F("SD ..."));
  startSdLog();  // 新しいログファイル (SD が無かった場合はここで再初期化を試す)
#if SERIAL_DEBUG
  Serial.println(F("# energy reset"));
#endif
  delay(800);    // メッセージ表示
  restartTimer();
}

void handleButton(uint32_t now) {
  bool pressed = (digitalRead(PIN_RESET_BTN) == LOW);
  if (!pressed) {
    btnDown = false;
    btnHandled = false;
    return;
  }
  if (!btnDown) {          // 押し始め (チャタリングで離れたらタイマーはやり直し)
    btnDown = true;
    btnDownMs = now;
  } else if (!btnHandled && now - btnDownMs >= RESET_HOLD_MS) {
    btnHandled = true;     // 離すまで再実行しない
    doEnergyReset();
  }
}

// ============================================================================
//  setup / loop
// ============================================================================
void setup() {
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
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
  if (!lcdOk) Serial.println(F("LCD: no ACK (check LCD_ADDR)"));
#else
  (void)lcdOk;
#endif
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
    while (true) {}  // 停止
  }
  if (!inaConfigure()) {
    lcdLineP(0, F("INA228 CONFIG ERROR"));
    lcdLineP(3, F("STOPPED"));
#if SERIAL_DEBUG
    Serial.println(F("INA228 config error"));
#endif
    while (true) {}
  }
  lcdLineP(1, F("INA228 OK"));

  startSdLog();
  lcdLineP(2, sdOk ? F("SD OK") : F("NO SD"));
  if (sdOk) lcdLine(3, logName);
#if SERIAL_DEBUG
  Serial.print(F("SHUNT_CAL=")); Serial.println(SHUNT_CAL);
  Serial.println(F("time_s,voltage_V,current_A,power_W,energy_Wh"));
#endif
  delay(1500);

  restartTimer();
}

void loop() {
  uint32_t now = millis();

  handleButton(now);

  if ((int32_t)(now - nextLcdMs) >= 0) {
    nextLcdMs += LCD_INTERVAL_MS;
    if ((int32_t)(now - nextLcdMs) >= 0) nextLcdMs = now + LCD_INTERVAL_MS;  // 遅れたら追いつかない
    inaMeasure(meas);
    lcdShowMeasurement(now);
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
