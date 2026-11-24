// Ene1PowerMeter を PC 上で動かす検証用シミュレーター
//   - INA228 と RW1063 (LCD) をデータシートどおりにモデル化して I2C でつなぐ
//   - SD カードは PC のディレクトリで代用
//   - 速度センサーは、車輪の回転位置を時間とともに進め、磁石が来るたびに D3 の割り込みを起こす
//   - 標準入力から手順 (シナリオ) を読み、期待どおりかを確かめる
//
// 使い方: sim <状態ファイル> <SDディレクトリ> < シナリオ
//   状態ファイルには INA228・LCD・リセットで消えない RAM (.noinit) の中身を保存する。
//   同じ状態ファイルで続けて起動すると「Nano だけのリセット」を再現できる。
//   EEPROM の中身は <状態ファイル>.eeprom に保存し、電源投入 (cold) でも残す。
#include "Arduino.h"
#include "Wire.h"
#include "SD.h"
#include "SPI.h"
#include "avr/eeprom.h"
#include <vector>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <stdexcept>

// ---------------------------------------------------------------------------
//  シミュレーター共通の状態
// ---------------------------------------------------------------------------
uint64_t g_us = 0;
HardwareSerial Serial;
TwoWire Wire;
SDClass SD;
SPIClass SPI;
bool g_sdCardPresent = true;
std::string g_sdDir;
int g_sdFlushCount = 0;
int g_wireOverflow = 0;
uint8_t  g_eeprom[E2END + 1];
uint64_t g_eeReadyUs = 0;
uint32_t g_eeWrites[E2END + 1];
int      g_eeLastAddr = -1;
void simEeWait() { if (g_us < g_eeReadyUs) simAdvanceUs(g_eeReadyUs - g_us); }

#include <dirent.h>
int simSdFileCount() {
  DIR *d = opendir(g_sdDir.c_str());
  if (!d) return 0;
  int n = 0;
  while (struct dirent *e = readdir(d)) if (e->d_name[0] != '.') n++;
  closedir(d);
  return n;
}
static bool g_button = false;
static bool g_wdtEnabled = false;
static uint64_t g_lastWdtUs = 0, g_maxWdtGapUs = 0;
static uint64_t g_loopStartUs = 0;
static bool g_inSetup = false;
static int g_fail = 0;
struct HaltException {};

// ---------------------------------------------------------------------------
//  速度センサー (車輪の磁石とホール IC) のモデル
//    車輪の回転位置 [回転] を時間とともに進め、磁石の位置を通るたびに D3 を 1ms だけ LOW にして
//    INT1 の割り込み (立ち下がり) を起こす。チャタリングと、数 us で消えるノイズも作れる
// ---------------------------------------------------------------------------
static void (*g_isr1)() = nullptr;     // attachInterrupt(1, ..., FALLING) で登録された関数
static int  g_isr1Mode = -1;
static bool g_irqOn = true;            // interrupts() / noInterrupts()
static bool g_irqPending = false;      // 割り込み禁止中に立ち下がりが来た (INTF1)
static bool g_inIsr = false;
static uint64_t g_speedLowUntil = 0;   // この時刻まで D3 が LOW
static bool g_speedSpike = false;      // ノイズ: 割り込みの処理が始まったときには、もう HIGH に戻っている
static double g_revPerUs = 0;          // 車輪の回転の速さ [回転/us]
static double g_rev = 0;               // 車輪の回転位置 [回転]
static long   g_nextPulse = 0;         // 次に来る磁石の通し番号
static int    g_magnets = 1;           // 磁石の数 (スケッチの SPEED_MAGNETS に合わせる)
static bool   g_uneven = false;        // 磁石の間隔をわざとずらす
static int    g_bounce = 0;            // 1 パルスごとのチャタリング (余分な立ち下がり) の回数
static uint64_t g_noisePeriodUs = 0, g_noiseNextUs = 0;  // 短いノイズの間隔 (0 = なし)
static long   g_pulses = 0, g_spikes = 0;

void simAttachInterrupt(int num, void (*isr)(), int mode) {
  if (num == 1) { g_isr1 = isr; g_isr1Mode = mode; }
}
static void simIrq1() {  // D3 の立ち下がり
  if (!g_isr1 || g_isr1Mode != FALLING) return;
  if (!g_irqOn || g_inIsr) { g_irqPending = true; return; }
  g_inIsr = true;
  g_isr1();
  g_inIsr = false;
}
void simInterrupts(bool enable) {
  g_irqOn = enable;
  if (enable && g_irqPending) { g_irqPending = false; simIrq1(); }
}
// k 番目の磁石が来る回転位置 [回転]。g_uneven のときは磁石ごとに位置をずらす
static double pulsePos(long k) {
  static const double off[4] = {0, 0.12, -0.08, 0.05};
  long rev = k / g_magnets;
  int j = (int)(k % g_magnets);
  return rev + (j + (g_uneven ? off[j % 4] : 0.0)) / g_magnets;
}

int simDigitalRead(uint8_t pin) {
  if (pin == 2) return g_button ? LOW : HIGH;
  if (pin == 3) return (!g_speedSpike && g_us < g_speedLowUntil) ? LOW : HIGH;
  return HIGH;
}
void simAdvanceUs(uint64_t us);
void simWdtEnable(int) { g_wdtEnabled = true; g_lastWdtUs = g_us; }
void simWdtReset() {
  g_lastWdtUs = g_us;
  simAdvanceUs(1);  // 無限ループでも時間が進むように
  if (g_inSetup && g_us - g_loopStartUs > 60000000ULL) throw HaltException();
}

static std::string serialLine;
size_t HardwareSerial::write(uint8_t c) {
  if (c == '\r') return 1;
  if (c == '\n') {  // CSV 行 (数字で始まる) は量が多いので表示しない
    if (serialLine.empty() || !isdigit((unsigned char)serialLine[0]) || getenv("SIM_VERBOSE"))
      printf("    serial| %s\n", serialLine.c_str());
    serialLine.clear();
  }
  else serialLine += (char)c;
  return 1;
}

// ---------------------------------------------------------------------------
//  INA228 モデル (データシートのレジスタ仕様どおり)
// ---------------------------------------------------------------------------
struct Ina228 {
  bool present = true;
  // 電源投入時の既定値 (INA228 データシートの値として想定。SHUNT_CAL=0x1000 はテスト 17 の境界条件)
  uint16_t config = 0, adcConfig = 0xFB68, shuntCal = 0x1000;
  double chargeCounts = 0;   // CHARGE レジスタ (端数込み)
  double vTrue = 0, iTrue = 0, rTrue = 0.002;  // 実際の電池電圧・電流・シャント抵抗

  void powerOn() { config = 0; adcConfig = 0xFB68; shuntCal = 0x1000; chargeCounts = 0; }
  // シャント電圧を ADC 範囲 (±163.84mV) で切ったうえでの CURRENT レジスタ値
  double currentCounts() const {
    if (shuntCal == 0) return 0;
    bool low = config & 0x0010;                 // ADCRANGE=1: ±40.96mV
    double fs = low ? 0.04096 : 0.16384, k = low ? 4.0 : 1.0;
    double vsh = iTrue * rTrue;
    if (vsh > fs) vsh = fs;
    if (vsh < -fs) vsh = -fs;
    // CURRENT_LSB x R = SHUNT_CAL / (13107.2e6 x k) なので CURRENT = Vshunt x 13107.2e6 x k / SHUNT_CAL
    double c = vsh * 13107.2e6 * k / shuntCal;
    if (c > 524287) c = 524287;
    if (c < -524288) c = -524288;
    return std::round(c);
  }
  void advance(double dtSec) {
    if (!present) return;
    // RSTACC はデータシートに「自動で 0 に戻る」と書かれていないので、1 のまま残る厳しい側でモデル化する。
    // 1 の間は CHARGE を 0 に保ち続ける
    if (config & 0x4000) { chargeCounts = 0; return; }
    if ((adcConfig >> 12) & 0x1) chargeCounts += currentCounts() * dtSec;  // シャント測定が有効なら積算
  }
  static void put(uint8_t *b, uint64_t v, int n) { for (int i = 0; i < n; i++) b[i] = v >> (8 * (n - 1 - i)); }
  uint8_t ptr = 0;
  bool write(const uint8_t *d, uint8_t len) {
    if (!present) return false;
    if (len >= 1) ptr = d[0];
    if (len == 3) {
      uint16_t v = (d[1] << 8) | d[2];
      if (ptr == 0x00) {
        if (v & 0x8000) { powerOn(); return true; }  // RST
        if (v & 0x4000) chargeCounts = 0;           // RSTACC (ENERGY/CHARGE)
        config = v & 0x7FF0;                        // bit3-0 は予約 (常に 0)。RSTACC は残る
      } else if (ptr == 0x01) adcConfig = v;
      else if (ptr == 0x02) shuntCal = v & 0x7FFF;
    }
    return true;
  }
  bool read(uint8_t *b, uint8_t n) {
    if (!present) return false;
    uint8_t tmp[5] = {0};
    int size = 2;
    switch (ptr) {
      case 0x00: put(tmp, config, 2); break;
      case 0x01: put(tmp, adcConfig, 2); break;
      case 0x02: put(tmp, shuntCal, 2); break;
      case 0x05: size = 3; put(tmp, ((uint64_t)std::lround(vTrue / 195.3125e-6) & 0xFFFFF) << 4, 3); break;
      case 0x07: size = 3; put(tmp, ((uint64_t)(int64_t)currentCounts() & 0xFFFFF) << 4, 3); break;
      case 0x08: {
        size = 3;
        double p = vTrue / 195.3125e-6 * 195.3125e-6 * std::fabs(currentCounts()) / 3.2;
        if (p > 0xFFFFFF) p = 0xFFFFFF;
        put(tmp, (uint64_t)std::lround(p), 3);
        break;
      }
      case 0x0A: size = 5; put(tmp, (uint64_t)(int64_t)std::llround(chargeCounts) & 0xFFFFFFFFFFULL, 5); break;
      case 0x3E: put(tmp, 0x5449, 2); break;
      case 0x3F: put(tmp, 0x2281, 2); break;
      default: break;
    }
    for (int i = 0; i < n; i++) b[i] = i < size ? tmp[i] : 0xFF;
    return true;
  }
} ina;

// ---------------------------------------------------------------------------
//  RW1063 モデル (I2C: 制御バイト Co/A0、処理時間の違反を数える)
// ---------------------------------------------------------------------------
struct Rw1063 {
  bool present = true;
  uint8_t addr = 0x3C;
  uint8_t ddram[128];
  uint8_t ac = 0;
  bool dl = true, n = false, f = false, on = false;
  uint64_t busyUntil = 0;     // 前の命令の処理が終わる時刻
  int64_t powerOnUs = 0;      // 電源投入時刻 (起動ごとの仮想時間に対する相対値)
  int busyViolations = 0, earlyViolations = 0, protoErrors = 0;
  // 発振周波数が最小 380kHz のときの処理時間 (540kHz で 18.5us / 0.76ms)
  static constexpr double SCALE = 540.0 / 380.0;
  void powerOn() {
    memset(ddram, ' ', sizeof ddram);
    ac = 0; dl = true; n = false; f = false; on = false;  // 内部リセット後: 1 行モード・表示 OFF
    powerOnUs = (int64_t)g_us; busyUntil = 0;
  }
  void incAc() {
    if (n) ac = (ac == 0x27) ? 0x40 : (ac == 0x67) ? 0x00 : ac + 1;
    else ac = (ac + 1) % 80;
  }
  void exec(bool isData, uint8_t v, uint64_t t) {
    if ((int64_t)t < powerOnUs + 40000) earlyViolations++;   // 電源投入から 40ms 以内
    if (t < busyUntil) { busyViolations++; return; }         // 処理中に届いた命令は失われる
    double execUs = 18.5 * SCALE;
    if (isData) { ddram[ac & 0x7F] = v; incAc(); }
    else if (v == 0x01) { memset(ddram, ' ', sizeof ddram); ac = 0; execUs = 760 * SCALE; }
    else if ((v & 0xFE) == 0x02) { ac = 0; execUs = 760 * SCALE; }
    else if ((v & 0xFC) == 0x04) {}
    else if ((v & 0xF8) == 0x08) on = v & 0x04;
    else if ((v & 0xF0) == 0x10) {}
    else if ((v & 0xE0) == 0x20) { dl = v & 0x10; n = v & 0x08; f = v & 0x04; }
    else if ((v & 0xC0) == 0x40) {}
    else ac = v & 0x7F;
    busyUntil = t + (uint64_t)execUs;
  }
  bool write(const uint8_t *d, uint8_t len) {
    if (!present) return false;
    uint64_t byteUs = 90, t0 = g_us - byteUs * len;  // 各バイトの到着時刻を推定
    uint8_t i = 0;
    while (i < len) {
      uint8_t ctrl = d[i++];
      if (ctrl & 0x3F) protoErrors++;
      bool co = ctrl & 0x80, a0 = ctrl & 0x40;
      if (co) { if (i < len) { exec(a0, d[i], t0 + byteUs * (i + 1)); i++; } }
      else { while (i < len) { exec(a0, d[i], t0 + byteUs * (i + 1)); i++; } }
    }
    return true;
  }
  std::string row(int r) const {
    static const uint8_t base[4] = {0x00, 0x40, 0x14, 0x54};
    if (!on) return "(表示OFF)";
    if (!n) return "(1行モード: 4行表示にならない)";
    std::string s;
    for (int c = 0; c < 20; c++) s += (char)ddram[base[r] + c];
    return s;
  }
} lcd;

static void advanceDevices(uint64_t us) {
  ina.advance(us / 1e6);
  g_us += us;
  g_rev += g_revPerUs * us;
  if (g_wdtEnabled && g_us - g_lastWdtUs > g_maxWdtGapUs) g_maxWdtGapUs = g_us - g_lastWdtUs;
}
// 時間を進める。途中で磁石やノイズが来たら、その時刻で割り込みを起こす
void simAdvanceUs(uint64_t us) {
  const uint64_t end = g_us + us;
  for (;;) {
    uint64_t tPulse = UINT64_MAX, tNoise = g_noisePeriodUs ? g_noiseNextUs : UINT64_MAX;
    if (g_revPerUs > 0) tPulse = g_us + (uint64_t)std::ceil(std::max(0.0, (pulsePos(g_nextPulse) - g_rev) / g_revPerUs));
    uint64_t t = std::max(g_us, std::min(tPulse, tNoise));  // チャタリングで時刻が先に進んでいることがある
    if (t > end) break;
    advanceDevices(t - g_us);
    if (t == tPulse) {
      g_rev = pulsePos(g_nextPulse++);
      g_pulses++;
      g_speedLowUntil = g_us + 1000;
      simIrq1();
      for (int b = 0; b < g_bounce; b++) { advanceDevices(30); simIrq1(); }  // 30us ごとに跳ね返る
    } else {
      g_noiseNextUs += g_noisePeriodUs;
      g_spikes++;
      g_speedSpike = true;
      simIrq1();
      g_speedSpike = false;
    }
  }
  if (end > g_us) advanceDevices(end - g_us);
}
bool simI2cWrite(uint8_t a, const uint8_t *d, uint8_t len) {
  if (a == 0x40) return ina.write(d, len);
  if (lcd.present && a == lcd.addr) return lcd.write(d, len);
  return false;
}
bool simI2cRead(uint8_t a, uint8_t *d, uint8_t len) {
  if (a == 0x40) return ina.read(d, len);
  return false;  // RW1063 は I2C 読み出し非対応
}

// ---------------------------------------------------------------------------
//  スケッチ本体を取り込む
// ---------------------------------------------------------------------------
#ifndef SKETCH_PATH
#define SKETCH_PATH "../../Ene1PowerMeter/Ene1PowerMeter.ino"
#endif
#include SKETCH_PATH

// ---------------------------------------------------------------------------
//  状態ファイル (Nano リセットをまたいで残るもの)
// ---------------------------------------------------------------------------
// EEPROM は電源を切っても残るので、cold でも読み込む (ファイルがなければ消去済み = 0xFF)
static void loadEeprom(const std::string &path) {
  memset(g_eeprom, 0xFF, sizeof g_eeprom);
  FILE *fp = fopen(path.c_str(), "rb");
  if (!fp) return;
  if (fread(g_eeprom, 1, sizeof g_eeprom, fp) != sizeof g_eeprom || fread(g_eeWrites, sizeof g_eeWrites, 1, fp) != 1) {
    memset(g_eeprom, 0xFF, sizeof g_eeprom);
    memset(g_eeWrites, 0, sizeof g_eeWrites);
  }
  fclose(fp);
}
static void saveEeprom(const std::string &path) {
  FILE *fp = fopen(path.c_str(), "wb");
  fwrite(g_eeprom, 1, sizeof g_eeprom, fp);
  fwrite(g_eeWrites, sizeof g_eeWrites, 1, fp);
  fclose(fp);
}
static void saveState(const char *path) {
  FILE *fp = fopen(path, "wb");
  fwrite(&ina.config, sizeof ina.config, 1, fp);
  fwrite(&ina.adcConfig, sizeof ina.adcConfig, 1, fp);
  fwrite(&ina.shuntCal, sizeof ina.shuntCal, 1, fp);
  fwrite(&ina.chargeCounts, sizeof ina.chargeCounts, 1, fp);
  fwrite(&persist, sizeof persist, 1, fp);
  fclose(fp);
}
static bool loadState(const char *path) {
  FILE *fp = fopen(path, "rb");
  if (!fp) return false;
  bool ok = fread(&ina.config, sizeof ina.config, 1, fp) && fread(&ina.adcConfig, sizeof ina.adcConfig, 1, fp) &&
            fread(&ina.shuntCal, sizeof ina.shuntCal, 1, fp) && fread(&ina.chargeCounts, sizeof ina.chargeCounts, 1, fp) &&
            fread(&persist, sizeof persist, 1, fp);
  fclose(fp);
  return ok;
}

// ---------------------------------------------------------------------------
//  シナリオ実行
// ---------------------------------------------------------------------------
static uint64_t g_markUs = 0;
static void runFor(uint64_t ms) {
  uint64_t end = g_us + ms * 1000;
  while (g_us < end) {
    g_loopStartUs = g_us;
    loop();
    simAdvanceUs(100);  // loop 1 回あたりの CPU 時間
  }
}
static std::string firstLine(const std::string &name) {
  std::string p = g_sdDir + "/" + name;
  FILE *fp = fopen(p.c_str(), "rb");
  if (!fp) return "(ファイルなし)";
  char buf[256] = "";
  if (!fgets(buf, sizeof buf, fp)) buf[0] = '\0';
  fclose(fp);
  std::string l = buf;
  while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
  return l;
}
static std::string lastLine(const std::string &name) {
  std::string p = g_sdDir + "/" + name, line, last;
  FILE *fp = fopen(p.c_str(), "rb");
  if (!fp) return "(ファイルなし)";
  char buf[256];
  while (fgets(buf, sizeof buf, fp)) { line = buf; if (!line.empty()) last = line; }
  fclose(fp);
  while (!last.empty() && (last.back() == '\n' || last.back() == '\r')) last.pop_back();
  return last;
}
static int countLines(const std::string &name) {
  std::string p = g_sdDir + "/" + name;
  FILE *fp = fopen(p.c_str(), "rb");
  if (!fp) return -1;
  int n = 0, c;
  while ((c = fgetc(fp)) != EOF) if (c == '\n') n++;
  fclose(fp);
  return n;
}
static void check(bool ok, const std::string &what) {
  printf("  %s %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) g_fail++;
}
static std::string quoted(std::istringstream &is) {
  std::string s;
  std::getline(is, s);
  size_t a = s.find('"'), b = s.rfind('"');
  return (a != std::string::npos && b > a) ? s.substr(a + 1, b - a - 1) : s;
}
static double num(std::istringstream &is) {
  std::string s;
  is >> s;
  return strtod(s.c_str(), nullptr);
}

int main(int argc, char **argv) {
  if (argc < 4) { fprintf(stderr, "usage: sim <state> <sddir> <cold|warm>\n"); return 2; }
  g_sdDir = argv[2];
  bool warm = std::string(argv[3]) == "warm";
  g_magnets = SPEED_MAGNETS;
  lcd.powerOn();
  const std::string eePath = std::string(argv[1]) + ".eeprom";
  loadEeprom(eePath);
  if (warm && loadState(argv[1])) {
    printf("  [Nano だけリセット: INA228 と .noinit RAM は前回のまま]\n");
  } else {
    ina.powerOn();
    srand(1234);
    uint8_t *p = (uint8_t *)&persist;  // 電源投入直後の RAM は不定
    for (size_t k = 0; k < sizeof persist; k++) p[k] = rand();
    printf("  [電源投入: すべて初期状態]\n");
  }

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string cmd;
    if (!(is >> cmd) || cmd[0] == '#') continue;
    if (cmd == "say") { printf("\n== %s\n", quoted(is).c_str()); }
    else if (cmd == "v") ina.vTrue = num(is);
    else if (cmd == "i") ina.iTrue = num(is);
    else if (cmd == "rshunt") ina.rTrue = num(is);
    else if (cmd == "lcdaddr") lcd.addr = (uint8_t)strtol(quoted(is).c_str(), nullptr, 0);
    else if (cmd == "lcd_absent") lcd.present = false;
    else if (cmd == "ina") {
      std::string a; is >> a;
      if (a == "off") ina.present = false;
      else if (a == "on") ina.present = true;
      else if (a == "glitch") ina.powerOn();  // INA228 だけ電源瞬断
    }
    else if (cmd == "sdfiles") {  // SD に LOG000〜LOGnnn を作っておく
      int n = (int)num(is);
      for (int k = 0; k < n; k++) {
        char nm[32]; snprintf(nm, sizeof nm, "/LOG%03d.CSV", k);
        FILE *fp = fopen((g_sdDir + nm).c_str(), "wb"); if (fp) fclose(fp);
      }
    }
    else if (cmd == "sd") { std::string a; is >> a; g_sdCardPresent = (a == "in"); }
    else if (cmd == "setup") {
      g_inSetup = true;
      g_loopStartUs = g_us;
      try { setup(); } catch (HaltException &) { printf("  [setup() で停止した (仕様どおりの停止)]\n"); }
      g_inSetup = false;
    }
    else if (cmd == "run") runFor((uint64_t)num(is));
    else if (cmd == "mark") g_markUs = g_us;  // run_to の基準の時刻
    else if (cmd == "run_to") {               // mark から ms 後まで進める (run を繰り返すとずれがたまるので、長い走行に使う)
      uint64_t end = g_markUs + (uint64_t)(num(is) * 1000);
      if (end > g_us) runFor((end - g_us + 999) / 1000);
    }
    else if (cmd == "press") {
      uint64_t ms = (uint64_t)num(is);
      g_button = true; runFor(ms); g_button = false; runFor(60);
    }
    else if (cmd == "button") { std::string a; is >> a; g_button = (a == "down"); }  // 押しっぱなし・ショート
    else if (cmd == "noise") {  // LCD がノイズで化けた状態を作る
      lcd.on = false; lcd.n = false; for (auto &c : lcd.ddram) c = '#';
    }
    else if (cmd == "lcd") {
      printf("  +--------------------+\n");
      for (int r = 0; r < 4; r++) printf("  |%-20s|\n", lcd.row(r).c_str());
      printf("  +--------------------+\n");
    }
    else if (cmd == "expect_lcd") {
      int r = (int)num(is); std::string s = quoted(is);
      check(lcd.row(r).find(s) != std::string::npos, "LCD " + std::to_string(r + 1) + "行目に \"" + s + "\" (実際: \"" + lcd.row(r) + "\")");
    }
    else if (cmd == "expect_not_lcd") {
      int r = (int)num(is); std::string s = quoted(is);
      check(lcd.row(r).find(s) == std::string::npos, "LCD " + std::to_string(r + 1) + "行目に \"" + s + "\" が無い (実際: \"" + lcd.row(r) + "\")");
    }
    else if (cmd == "expect_last") {
      std::string f; is >> f; std::string s = quoted(is), l = lastLine(f);
      check(l.find(s) != std::string::npos, f + " の最終行に \"" + s + "\" (実際: \"" + l + "\")");
    }
    else if (cmd == "expect_first") {
      std::string f; is >> f; std::string s = quoted(is), l = firstLine(f);
      check(l == s, f + " の見出し行が \"" + s + "\" (実際: \"" + l + "\")");
    }
    else if (cmd == "expect_last_col") {  // 最終行の col 列目 (0 から) の値の範囲
      std::string f; is >> f; int col = (int)num(is); double lo = num(is), hi = num(is);
      std::string l = lastLine(f), cell;
      std::istringstream ls(l);
      for (int k = 0; k <= col && std::getline(ls, cell, ','); k++) {}
      double v = strtod(cell.c_str(), nullptr);
      char b[200]; snprintf(b, sizeof b, "%s の最終行の %d 列目 %s が %g〜%g", f.c_str(), col + 1, cell.c_str(), lo, hi);
      check(!cell.empty() && v >= lo && v <= hi, b);
    }
    else if (cmd == "expect_lines") {
      std::string f; is >> f; int lo = (int)num(is), hi = (int)num(is), n = countLines(f);
      check(n >= lo && n <= hi, f + " の行数 " + std::to_string(n) + " が " + std::to_string(lo) + "〜" + std::to_string(hi));
    }
    else if (cmd == "expect_charge") {  // 表示中の積算 mAh (sketch の meas) の範囲
      double lo = num(is), hi = num(is);
      char b[160]; snprintf(b, sizeof b, "積算 %.1f mAh が %.1f〜%.1f", meas.chargemAh, lo, hi);
      check(meas.chargemAh >= lo && meas.chargemAh <= hi, b);
    }
    else if (cmd == "wheel") g_revPerUs = num(is) / 3.6 / WHEEL_CIRCUMFERENCE_M / 1e6;  // 車輪の速さ [km/h]
    else if (cmd == "wheel_bounce") g_bounce = (int)num(is);
    else if (cmd == "wheel_uneven") { std::string a; is >> a; g_uneven = (a == "on"); }
    else if (cmd == "wheel_noise") {  // 数 us で消えるノイズを 1 秒に n 回 (0 = なし)
      double hz = num(is);
      g_noisePeriodUs = hz > 0 ? (uint64_t)(1e6 / hz) : 0;
      g_noiseNextUs = g_us + g_noisePeriodUs / 3;
    }
    else if (cmd == "expect_speed") {  // 表示・記録している速さ [km/h] の範囲
      double lo = num(is), hi = num(is);
      char b[160]; snprintf(b, sizeof b, "速さ %.2f km/h が %.2f〜%.2f", speedKmh, lo, hi);
      check(speedKmh >= lo && speedKmh <= hi, b);
    }
    else if (cmd == "expect_dist") {  // 走った距離 [m] の範囲
      double lo = num(is), hi = num(is), d = distanceNowM();
      char b[200]; snprintf(b, sizeof b, "距離 %.1f m が %.1f〜%.1f (磁石が来た回数 %ld、ノイズ %ld 回)", d, lo, hi, g_pulses, g_spikes);
      check(d >= lo && d <= hi, b);
    }
    else if (cmd == "expect_clean") {
      char b[200];
      snprintf(b, sizeof b, "LCD 処理中の命令 %d / 40ms 以内の命令 %d / 制御バイト誤り %d / Wire あふれ %d / ウォッチドッグ最大間隔 %.2f s",
               lcd.busyViolations, lcd.earlyViolations, lcd.protoErrors, g_wireOverflow, g_maxWdtGapUs / 1e6);
      check(lcd.busyViolations == 0 && lcd.earlyViolations == 0 && lcd.protoErrors == 0 && g_wireOverflow == 0 &&
            g_maxWdtGapUs < 8000000ULL, b);
    }
    else if (cmd == "eeprom_tear") {  // 書いている途中で電源が落ちた: 最後に書いたバイトを壊す
      if (g_eeLastAddr >= 0) g_eeprom[g_eeLastAddr] ^= 0x5A;
      printf("  [EEPROM の 0x%03X を壊した (書き込み途中の電源断)]\n", g_eeLastAddr);
    }
    else if (cmd == "eeprom_erase") { memset(g_eeprom, 0xFF, sizeof g_eeprom); memset(g_eeWrites, 0, sizeof g_eeWrites); }
    else if (cmd == "expect_ee_wear") {  // 1 バイトあたりの書き込み回数の最大
      uint32_t mx = 0, total = 0;
      for (uint32_t w : g_eeWrites) { if (w > mx) mx = w; total += w; }
      double lim = num(is);
      char b[160]; snprintf(b, sizeof b, "EEPROM の書き込み: 1 バイトあたり最大 %u 回 (≦ %.0f)、合計 %u バイト", mx, lim, total);
      check(mx <= lim, b);
    }
    else { printf("  ?? 不明なコマンド: %s\n", cmd.c_str()); g_fail++; }
  }
  saveState(argv[1]);
  saveEeprom(eePath);
  return g_fail ? 1 : 0;
}
