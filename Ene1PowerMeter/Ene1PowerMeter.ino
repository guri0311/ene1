/*
 * Ene-1 GP 車載電力計
 *   マイコン   : Arduino Nano (ATmega328P / 5V / 16MHz)
 *   センサー   : INA228 (Strawberry Linux #11228, シャント 2mΩ, I2C 0x40)
 *   表示       : 秋月 I2C キャラクター液晶 20x4 (ACM2004D-FLW-FBW-IIC, 制御IC RW1063)
 *   記録       : microSD (SPI, CS = D10)
 *   速度センサー : 車輪の磁石 + ホール IC (またはリードスイッチ) を D3 (INT1) に。車輪 1 周の距離は
 *                  WHEEL_CIRCUMFERENCE_M で設定する
 *
 * ボタン (D2 - GND)
 *   3 秒以上押して離す : 積算 mAh・経過時間を 0 にして新しいログファイルを作る
 *                        (押したままの間は LCD に RELEASE=RESET mAh と出る。
 *                         10 秒以上押されたままなら配線のショートとみなして何もしない)
 *   SD カードの初期化は起動時だけ行う (走行中に NO SD になったら、電源を入れ直すまで記録しない)
 *
 * レース中の故障に備えた動作 (何が起きても、積算 mAh と電流の表示だけは続ける)
 *   - Nano だけがリセットされた場合 (ウォッチドッグ・書き込み・リセットボタン等)、
 *     INA228 の設定が残っていれば積算 mAh を消さずに続きから再開する (経過時間も可能なら復元)
 *   - 走行中に INA228 の設定が消えた (電源瞬断など) ことを検出したら、再設定して
 *     それまでの積算値を引き継ぐ
 *   - 電力計の電源が全部落ちても、EEPROM に 2 秒ごとに保存した積算 mAh と経過時間から
 *     続きを数える (0 に戻すのはボタンの 3 秒長押しだけ)
 *   - 起動時に INA228 が見つからなくても止まらず、つながるまで探し続ける。
 *     INA228 が読めない間も、最後に読めた積算 mAh を表示し続ける
 *   - LCD をノイズで化けたままにしないよう、5 秒ごとに初期化コマンドを再送する
 *   - SD カードが抜けたら、flush のたびの状態確認 (CMD13) で数秒以内に NO SD にする
 *   - I2C がノイズで固まっても Wire のタイムアウト (3ms) で復帰する
 *   - 速度センサーのチャタリングや短いノイズは捨てる。線が切れても速さが 0 になるだけで、
 *     積算と記録は続く。走った距離も積算 mAh と同じように引き継ぐ
 *   - 処理が固まったら 8 秒でウォッチドッグがリセットする
 *     (Optiboot の Nano はそのまま再開する。旧ブートローダの Nano は再起動を繰り返すので、
 *      Optiboot を書き込むか USE_WATCHDOG を 0 にする)
 *
 * 動作確認: test/host_sim (PC 上のシミュレーション) と test/avr_emu (実機バイナリを
 *           ATmega328P エミュレーターで動かすテスト)
 *
 * 必要なライブラリ: Wire, SPI (標準) / SD
 *   LCD は自作の軽量ドライバ (lcd* 関数) で直接制御する
 *
 * ピン配置
 *   A4 = SDA, A5 = SCL  (INA228 と LCD で共用。4.7kΩ で 5V へ外付けプルアップ)
 *   D11 = MOSI, D12 = MISO, D13 = SCK, D10 = CS (SD カード)
 *   D2  = ボタン (内部プルアップ、押すと LOW)
 *   D3  = 速度センサー (内部プルアップ + 外付け 4.7kΩ を推奨、磁石が来ると LOW)
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

// ============================================================================
//  ユーザー設定 (ここを変えるだけで調整できる)
// ============================================================================
// シャント抵抗 [Ω]。基準の電流計と比べて表示がずれる場合はこの値で校正する
// (例: 表示が 1% 大きい → 0.002 x 1.01 = 0.00202)。電流・電力・積算すべてに効く
constexpr float    R_SHUNT_OHM     = 0.002f;
// 想定最大電流 [A] (CURRENT_LSB の計算に使う)。これを超えると電流が頭打ちになり LCD に OVER と出る。
// 2mΩ では ADCRANGE=1 のとき 20.48A、ADCRANGE=0 のとき 81.92A まで設定できる
constexpr float    MAX_CURRENT_A   = 20.0f;
constexpr uint8_t  INA228_ADDR     = 0x40;     // INA228 の I2C アドレス (A0=A1=GND)
constexpr uint8_t  LCD_ADDR        = 0x3C;     // LCD の I2C アドレス (応答がなければ 0x3C〜0x3F から自動で探す)
constexpr uint16_t LCD_INTERVAL_MS = 250;      // LCD 更新間隔 [ms]
constexpr uint16_t LCD_REINIT_MS   = 5000;     // LCD 初期化コマンドの再送間隔 [ms] (ノイズ対策)
// SD 記録間隔 [ms]。50ms = 1 秒に 20 行。
//   - INA228 の更新周期 (INA_CONV_PERIOD_US、下の ADC 設定で約 18ms) より短くしても同じ値が
//     並ぶだけなので、それ以上にする (static_assert で確かめる)
//   - 20ms にすると LCD 更新などの処理が間に合わず、刻みが 10〜30ms にばらつく (エミュレーターで実測)
//   - 100ms 以上にするなら INA_AVG を 3 (64 回) に戻すと、1 行ごとの値のばらつきが小さくなる
constexpr uint16_t LOG_INTERVAL_MS = 50;
// SD への書き込みを確定 (flush) する間隔 [ms]。電源が落ちたとき、最大でこの時間ぶんの行が残らない
constexpr uint16_t LOG_FLUSH_MS    = 1000;
constexpr uint16_t RESET_HOLD_MS   = 3000;     // 積算リセットの長押し時間 [ms]
// 積算 mAh と経過時間を EEPROM に保存する間隔 [ms]。電源が全部落ちたとき、最大でこの時間ぶん
// (と電源が落ちていた間) の積算が数えられない。短くすると EEPROM の書き換え寿命が縮む (下の EEPROM の節)
constexpr uint16_t EE_SAVE_MS      = 2000;
constexpr uint16_t BTN_STUCK_MS    = 10000;    // これ以上押されたままなら故障 (ショート) とみなす [ms]
constexpr uint8_t  PIN_SD_CS       = 10;
constexpr uint8_t  PIN_BTN         = 2;
constexpr uint8_t  PIN_SPEED       = 3;        // 速度センサー (INT1。外部割り込みが使えるのは D2 と D3 だけ)
#define SERIAL_DEBUG 1   // 1: 起動時の I2C スキャン結果と CSV を USB シリアル (115200bps) に出す
#define USE_WATCHDOG 1   // 1: 8 秒固まったら自動リセット (旧ブートローダの Nano 互換機は下の注意を参照)

// ---- 速度センサー (車輪に付けた磁石が、車体側のホール IC の前を通るたびに 1 パルス) ----
// 車輪 1 周で進む距離 [m]。速さと距離のすべてに効く。初期値はタイヤの外径 0.50m (IRC EC-R 20x1.75) x π。
// 実際はドライバーが乗るとタイヤがつぶれて 1〜3% 短くなるので、乗った状態で 1 周転がして測った値にする
constexpr float    WHEEL_CIRCUMFERENCE_M = 1.571f;
// 車輪 1 周あたりの磁石の数 (1〜8)。増やすと、低い速さでも表示が早く変わる。
// 速さは車輪 1 周ぶんのパルスの間隔から出すので、磁石の間隔がそろっていなくても誤差にならない
constexpr uint8_t  SPEED_MAGNETS   = 1;
// これより速くなるパルスは、チャタリング・ノイズとして捨てる [km/h]。実際の最高速より十分大きくする
constexpr float    SPEED_MAX_KMH   = 90.0f;
// これより遅くなったら 0km/h と表示する [km/h]。パルスが来なくなってから 0 にするまでの時間がこれで決まる
// (初期値では 1.9 秒)
constexpr float    SPEED_MIN_KMH   = 3.0f;

// INA228 の ADC 設定 (データシート ADC_CONFIG レジスタ)
constexpr uint8_t INA_MODE   = 0xB;  // 0xB = シャント電圧 + バス電圧 連続測定
constexpr uint8_t INA_VBUSCT = 1;    // バス電圧 変換時間 1 = 84us (電圧はゆっくり変わるので短くてよい)
constexpr uint8_t INA_VSHCT  = 5;    // シャント電圧 変換時間 5 = 1052us (電流の精度を保つため長め)
constexpr uint8_t INA_VTCT   = 5;    // 温度 変換時間 (MODE=0xB では未使用)
constexpr uint8_t INA_AVG    = 2;    // 平均回数 2 = 16 回 → (84+1052)us x 16 ≒ 18ms ごとに更新
                                     // (積算 mAh は平均回数に関係なく、すべての変換結果を INA228 が足し込む)
// シャント電圧の測定範囲。1 = ±40.96mV (ローレンジ、分解能 4 倍)、0 = ±163.84mV
// 10A ブレーカの後ろで使うので、2mΩ で 20.48A まで測れるローレンジにしている
constexpr uint8_t INA_ADCRANGE = 1;

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
constexpr uint16_t CONFIG_RSTACC = 0x4000;  // bit14: ENERGY / CHARGE を 0 に (自動で戻る保証がないので書き戻す)
constexpr uint16_t CONFIG_VALUE  = (uint16_t)INA_ADCRANGE << 4;  // CONVDLY=0, TEMPCOMP=0
constexpr uint16_t ADC_CONFIG_VALUE =
    ((uint16_t)INA_MODE << 12) | ((uint16_t)INA_VBUSCT << 9) |
    ((uint16_t)INA_VSHCT << 6) | ((uint16_t)INA_VTCT << 3) | INA_AVG;

// CURRENT_LSB = 最大電流 / 2^19
constexpr float CURRENT_LSB_NOMINAL = MAX_CURRENT_A / 524288.0f;
// SHUNT_CAL = 13107.2e6 x CURRENT_LSB x R_SHUNT。ADCRANGE=1 のときは 4 倍する (データシート)
constexpr float ADCRANGE_FACTOR = INA_ADCRANGE ? 4.0f : 1.0f;
constexpr float SHUNT_FULL_SCALE_V = INA_ADCRANGE ? 0.04096f : 0.16384f;  // シャント電圧の測定範囲
constexpr float SHUNT_CAL_F = 13107.2e6f * CURRENT_LSB_NOMINAL * R_SHUNT_OHM * ADCRANGE_FACTOR;
static_assert(INA_ADCRANGE <= 1, "INA_ADCRANGE は 0 か 1");
static_assert(SHUNT_CAL_F >= 1.0f && SHUNT_CAL_F <= 32767.0f, "SHUNT_CAL が 15bit に収まらない");
// 上限ちょうど (2mΩ で 81.92A) は float の丸めでわずかに超えるので、1e-4 の誤差を許す
static_assert(MAX_CURRENT_A * R_SHUNT_OHM <= SHUNT_FULL_SCALE_V * 1.0001f,
              "最大電流 x シャント抵抗がシャント電圧の測定範囲 (INA_ADCRANGE) を超える");
constexpr uint16_t SHUNT_CAL = (uint16_t)(SHUNT_CAL_F + 0.5f);
// 整数に丸めた SHUNT_CAL から逆算した実際の CURRENT_LSB (20A/2mΩ/ADCRANGE=1 なら SHUNT_CAL=4000 で誤差なし)
constexpr float CURRENT_LSB   = (float)SHUNT_CAL / (13107.2e6f * R_SHUNT_OHM * ADCRANGE_FACTOR);
// 電流がこの値以上なら頭打ち (測定範囲外) とみなす
constexpr float OVER_CURRENT_A = MAX_CURRENT_A - 2.0f * CURRENT_LSB;
constexpr float VBUS_LSB_V    = 195.3125e-6f;                         // バス電圧 LSB
constexpr float POWER_LSB_W   = 3.2f * CURRENT_LSB;                    // 電力 LSB
constexpr float CHARGE_LSB_MAH = CURRENT_LSB / 3.6f;                   // 電荷 LSB [C]→[mAh]

// INA228 の測定値が更新される周期 [us] (データシート ADC_CONFIG の変換時間と平均回数)
constexpr uint16_t INA_CT_US[8]  = {50, 84, 150, 280, 540, 1052, 2074, 4120};
constexpr uint16_t INA_AVG_N[8]  = {1, 4, 16, 64, 128, 256, 512, 1024};
constexpr uint32_t INA_CONV_PERIOD_US =
    ((uint32_t)INA_CT_US[INA_VBUSCT] + INA_CT_US[INA_VSHCT]) * INA_AVG_N[INA_AVG];
static_assert(INA_MODE == 0xB, "INA_CONV_PERIOD_US はシャント + バス電圧の連続測定 (MODE=0xB) を前提にしている");
static_assert((uint32_t)LOG_INTERVAL_MS * 1000UL >= INA_CONV_PERIOD_US,
              "LOG_INTERVAL_MS が INA228 の更新周期より短い (同じ値が並ぶだけ)。INA_AVG か変換時間を小さくする");
// 速度センサーの換算
constexpr float    PULSE_DIST_M = WHEEL_CIRCUMFERENCE_M / SPEED_MAGNETS;  // 1 パルスで進む距離 [m]
// パルスの間隔の下限 [us] (SPEED_MAX_KMH で走ったときの間隔)。これより短い間隔のパルスは捨てる
constexpr uint32_t SPEED_MIN_PULSE_US = (uint32_t)(PULSE_DIST_M / (SPEED_MAX_KMH / 3.6f) * 1e6f);
// パルスがこれだけ来なければ止まったとみなす [us] (SPEED_MIN_KMH で走ったときの間隔)
constexpr uint32_t SPEED_TIMEOUT_US   = (uint32_t)(PULSE_DIST_M / (SPEED_MIN_KMH / 3.6f) * 1e6f);
// パルスの間隔 [us] から速さ [0.01km/h] を出す定数 (= PULSE_DIST_M x 3.6e6 x 100)。
// 整数の割り算で済ませる (浮動小数点の割り算はプログラムが 200 バイトほど増える)
constexpr uint32_t SPEED_K = (uint32_t)(PULSE_DIST_M * 3.6e8f + 0.5f);
static_assert(SPEED_MAGNETS >= 1 && SPEED_MAGNETS <= 8, "SPEED_MAGNETS は 1〜8");
static_assert(WHEEL_CIRCUMFERENCE_M > 0.3f && WHEEL_CIRCUMFERENCE_M < 3.0f, "WHEEL_CIRCUMFERENCE_M は車輪 1 周の距離 [m]");
static_assert(SPEED_MIN_PULSE_US >= 1000, "パルスの間隔が 1ms を切る (SPEED_MAX_KMH が大きすぎるか、磁石が多すぎる)");
static_assert(SPEED_TIMEOUT_US > 2 * SPEED_MIN_PULSE_US && SPEED_TIMEOUT_US <= 10000000UL,
              "SPEED_MIN_KMH は SPEED_MAX_KMH より十分小さく、0 になるまで 10 秒以内にする");

// flush する行数 (LOG_FLUSH_MS ごと)
constexpr uint16_t LOG_FLUSH_EVERY = LOG_FLUSH_MS / LOG_INTERVAL_MS ? LOG_FLUSH_MS / LOG_INTERVAL_MS : 1;

// CSV の見出し (SD のファイルの先頭と、USB シリアルに出す)
const char CSV_HEADER[] PROGMEM = "time_s,voltage_V,current_A,power_W,charge_mAh,speed_kmh,distance_m";

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
// 最後に正しく読めた積算 mAh。INA228 が読めない間の表示、INA228 の再設定、EEPROM への保存に使う
float lastKnownTotalmAh = 0;

// INA228 が走行中に設定を失った (電源瞬断など) ときに、それまでの積算を引き継ぐための値。
// 積算そのものは INA228 の CHARGE レジスタだけで行い、ソフトでは足し込まない
float    chargeOffsetmAh = 0;
uint8_t  inaRecoverCount = 0;      // INA228 を再設定した回数 (LCD 4 行目に表示)

// 速さと距離 (speedUpdate で更新する)
float    speedKmh        = 0;      // [km/h]
float    distanceM       = 0;      // 走った距離 [m] = distOffsetM + パルス数 x PULSE_DIST_M
float    distOffsetM     = 0;      // Nano のリセット・電源断の前に走った距離 (起動時に戻す)

bool     sdOk            = false;
bool     heartbeat       = false;  // 記録するたびに反転 (4 行目末尾の * で動作確認)
uint16_t linesSinceFlush = 0;
uint32_t timerStartMs    = 0;      // 経過時間の基準 (起動時・リセット時)
uint32_t nextLogMs       = 0;
uint32_t nextLcdMs       = 0;
uint32_t lastLcdInitMs   = 0;
uint32_t btnDownMs       = 0;
bool     btnDown         = false;

// Nano がリセットされても消えない RAM 領域 (.noinit)。電源を切ると中身は不定になるので
// チェック値で正しさを確かめてから使う
struct Persist {
  uint32_t magic;
  uint32_t elapsedMs;
  float    chargeOffsetmAh;
  float    totalmAh;        // 最後に正しく読めた積算 (INA228 が設定を失っていたときに引き継ぐ)
  float    distM;           // 走った距離 [m]
  uint32_t check;
};
Persist persist __attribute__((section(".noinit")));
constexpr uint32_t PERSIST_MAGIC = 0xE1E1C0DEUL;

// EEPROM に保存する 1 回分 (下の「電源を切っても消えない保存」の節)。
// Arduino IDE が関数の宣言を先頭に自動で足すので、型はここで定義しておく
struct SavedRecord {
  uint32_t seq;        // 書いた順番 (大きいほど新しい)
  float    totalmAh;   // 積算 mAh
  uint32_t elapsedMs;  // 経過時間
  float    distM;      // 走った距離 [m]
  uint16_t magic;      // この形の記録であることの印
  uint16_t crc;        // ここより前の 18 バイトの CRC
};
static_assert(sizeof(SavedRecord) == 20, "SavedRecord は 20 バイト");

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
// (負の値のシフトは C++ では未定義・処理系依存なので、符号なしのまま計算してから引く)
int32_t toSigned20(uint32_t raw24) {
  int32_t v = (int32_t)((raw24 >> 4) & 0xFFFFFUL);
  return (v & 0x80000L) ? v - 0x100000L : v;
}

// INA228 がこのスケッチの設定のまま動いているか (電源投入直後は各レジスタが既定値に戻っている)
enum : uint8_t { INA_CONFIGURED, INA_NOT_CONFIGURED, INA_NO_RESPONSE };
uint8_t inaCheckConfig() {  // 戻り値は上の INA_* のどれか
  uint16_t cal, adc, cfg;
  if (!inaRead16(REG_SHUNT_CAL, cal) || !inaRead16(REG_ADC_CONFIG, adc) ||
      !inaRead16(REG_CONFIG, cfg))
    return INA_NO_RESPONSE;
  bool same = (cal == SHUNT_CAL) && (adc == ADC_CONFIG_VALUE) &&
              ((cfg & ~CONFIG_RST) == CONFIG_VALUE);  // RSTACC が 1 のまま (積算が止まる) も設定違いとみなす
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

// RSTACC はデータシートに「自動で 0 に戻る」と書かれていない (RST だけ "self-clears" とある)。
// 1 のまま残ると CHARGE が 0 に保たれて積算が止まるので、すぐに 0 を書き戻す
bool inaResetCharge() {
  bool ok = inaWrite16(REG_CONFIG, CONFIG_VALUE | CONFIG_RSTACC);
  ok = inaWrite16(REG_CONFIG, CONFIG_VALUE) && ok;   // 1 回目が失敗しても RSTACC=0 は必ず書く
  return ok;
}

// 走行中に INA228 の設定が消えていたら、それまでの積算を引き継いで再設定する
void inaRecover() {
  // 最後に正しく読めた積算値を引き継ぐ (起動後まだ読めていなければ、起動時に
  // リセットで消えない RAM か EEPROM から戻した値)
  chargeOffsetmAh = lastKnownTotalmAh;
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
  uint32_t rv, ri, rp;
  uint8_t q[5];

  // SHUNT_CAL だけで判定すると、設定値がたまたま INA228 の既定値と同じとき
  // (既定値が 0x1000 なら、2mΩ で ADCRANGE=0・81.92A や ADCRANGE=1・20.48A のとき一致する) にリセットを見逃すので、
  // ADC_CONFIG・CONFIG も確かめる
  uint8_t st = inaCheckConfig();
  if (st == INA_NO_RESPONSE) return m.ok = false;
  if (st == INA_NOT_CONFIGURED) {   // 設定が消えている (電源瞬断など)
    inaRecover();
    return m.ok = false;            // 今回は値を使わない (次回から正常に読める)
  }
  if (!inaRead24(REG_VBUS, rv) || !inaRead24(REG_CURRENT, ri) ||
      !inaRead24(REG_POWER, rp) || !inaRead(REG_CHARGE, q, 5))
    return m.ok = false;
  if (inaCheckConfig() != INA_CONFIGURED) return m.ok = false;  // 読み出し中にリセットされた

  m.voltage = toSigned20(rv) * VBUS_LSB_V;
  m.current = toSigned20(ri) * CURRENT_LSB;
  m.power   = rp * POWER_LSB_W;     // 24bit 全部が値 (符号なし)
  // CHARGE は 40bit の 2 の補数。float で上位と下位を足すと負の値で桁落ちするので、
  // 64bit 整数で組み立ててから float にする (負の値のシフトは避け、符号なしで組んでから引く)
  uint64_t qu = ((uint64_t)q[0] << 32) | ((uint32_t)q[1] << 24) |
                ((uint32_t)q[2] << 16) | ((uint32_t)q[3] << 8) | q[4];
  int64_t qraw = (qu & 0x8000000000ULL) ? (int64_t)qu - 0x10000000000LL : (int64_t)qu;
  m.chargemAh = (float)qraw * CHARGE_LSB_MAH + chargeOffsetmAh;
  lastKnownTotalmAh = m.chargemAh;
  return m.ok = true;
}

// ============================================================================
//  速度センサー
//  磁石がホール IC の前に来ると出力が LOW になる (オープンドレイン)。その立ち下がりを INT1 の
//  割り込みで受け、時刻を記録する。速さは車輪 1 周ぶん (SPEED_MAGNETS 個) のパルスの間隔から出す。
//  - 割り込みの中で D3 がまだ LOW かを確かめ、数 us で消えたノイズは数えない
//  - 前のパルスから SPEED_MIN_PULSE_US 以内のパルス (チャタリング・ノイズ) は数えない
//  - パルスが SPEED_TIMEOUT_US 来なければ止まったとみなし、速さを 0 にする
// ============================================================================
volatile uint32_t spdPulses = 0;   // 数えたパルスの数 (起動・積算リセットから) → 距離
volatile uint32_t spdLastUs = 0;   // 最後に数えたパルスの時刻 [us]
volatile uint32_t spdSpanUs = 0;   // 直近 spdSpanN 個ぶんのパルスの間隔の合計 [us]
volatile uint8_t  spdSpanN  = 0;   // 0 = 速さを出せない (止まっている、動き出して 1 パルス目)
volatile uint8_t  spdRun    = 0;   // 止まってから続けて数えたパルスの数 (SPEED_MAGNETS で頭打ち)
uint32_t spdRing[SPEED_MAGNETS];   // 直近のパルスの時刻 (割り込みの中だけで使う)
uint8_t  spdRingIdx = 0;           // 次に書く場所 (= いちばん古い時刻の場所)

void speedIsr() {
  uint32_t t = micros();
  if (digitalRead(PIN_SPEED) != LOW) return;   // もう HIGH に戻っている = 数 us のノイズ
  uint8_t k = spdRun;
  if (k && t - spdLastUs < SPEED_MIN_PULSE_US) return;  // チャタリング・ありえない速さ
  spdPulses++;
  spdLastUs = t;
  if (k) {  // k 個前のパルスからの時間 (k が SPEED_MAGNETS なら、ちょうど車輪 1 周)
    spdSpanUs = t - spdRing[(uint8_t)(spdRingIdx + SPEED_MAGNETS - k) % SPEED_MAGNETS];
    spdSpanN = k;
  }
  spdRing[spdRingIdx] = t;
  spdRingIdx = (uint8_t)(spdRingIdx + 1) % SPEED_MAGNETS;
  if (k < SPEED_MAGNETS) spdRun = k + 1;
}

void speedBegin() {
  pinMode(PIN_SPEED, INPUT_PULLUP);  // センサーが外れていても HIGH に保つ (パルスは来ない)
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), speedIsr, FALLING);
}

// 走った距離 [m] (いつ呼んでもよい)
float distanceNowM() {
  noInterrupts();
  uint32_t n = spdPulses;
  interrupts();
  return distOffsetM + n * PULSE_DIST_M;
}

// 速さと距離を最新にする (表示・記録の直前に呼ぶ)
void speedUpdate() {
  noInterrupts();
  uint32_t n = spdPulses, span = spdSpanUs;
  uint8_t  cnt = spdSpanN;
  uint32_t since = micros() - spdLastUs;
  if (spdRun && since > SPEED_TIMEOUT_US) {  // パルスが来なくなった = 止まった
    spdRun = 0;
    spdSpanN = 0;
    cnt = 0;
  }
  interrupts();
  distanceM = distOffsetM + n * PULSE_DIST_M;
  if (cnt == 0) { speedKmh = 0; return; }
  uint32_t per = span / cnt;     // パルス 1 つぶんの間隔 (車輪 1 周の平均)
  if (since > per) per = since;  // 次のパルスが遅れている = 減速中。いまの速さはこれより遅い
  speedKmh = ((SPEED_K + per / 2) / per) * 0.01f;
}

// 距離を 0 にする (積算リセット)
void speedResetDistance() {
  noInterrupts();
  spdPulses = 0;
  interrupts();
  distOffsetM = 0;
  distanceM = 0;
}

// ============================================================================
//  リセットされても消えない値 (経過時間・積算オフセット・距離)
// ============================================================================
uint32_t persistCheckValue(const Persist &p) {
  uint32_t qoff, qtot, dist;
  memcpy(&qoff, &p.chargeOffsetmAh, sizeof(qoff));
  memcpy(&qtot, &p.totalmAh, sizeof(qtot));
  memcpy(&dist, &p.distM, sizeof(dist));
  return p.magic ^ p.elapsedMs ^ qoff ^ (qtot * 3UL) ^ (dist * 5UL) ^ 0x5A5AA5A5UL;
}

void persistSave(uint32_t now) {
  persist.magic = PERSIST_MAGIC;
  persist.elapsedMs = now - timerStartMs;
  persist.chargeOffsetmAh = chargeOffsetmAh;
  persist.totalmAh = lastKnownTotalmAh;
  persist.distM = distanceNowM();
  persist.check = persistCheckValue(persist);
}

bool persistValid() {
  return persist.magic == PERSIST_MAGIC && persist.check == persistCheckValue(persist);
}

// ============================================================================
//  電源を切っても消えない保存 (EEPROM)
//  積算 mAh・経過時間・距離を EE_SAVE_MS ごとに保存し、電力計の電源が全部落ちて戻ったときは
//  続きから数える。
//  - 1 回分 20 バイトを、50 か所に順番に書く (同じ場所への書き込みを 50 分の 1 にする)。
//    2 秒ごとなら 1 か所は 100 秒に 1 回で、書き換え寿命 10 万回に届くのは約 2800 時間後
//  - 記録の形を変えたら EE_MAGIC も変える (古い形の記録は読まない)
//  - 書いている途中で電源が落ちた分は CRC で見分け、1 つ前の保存を使う
//  - 1 バイトの書き込みに約 3.3ms かかるので、loop のたびに 1 バイトずつ書いて、
//    表示と記録を止めない
// ============================================================================
constexpr uint16_t EE_MAGIC = 0xE1A2;  // 0xE1A1 = 距離のない 16 バイトの形 (読まない)
constexpr uint8_t  EE_SLOTS = 50;
static_assert((uint16_t)EE_SLOTS * sizeof(SavedRecord) <= E2END + 1, "EEPROM に収まらない");

SavedRecord eeBuf;                          // 書き込み中の 1 回分
uint32_t eeSeq        = 0;                  // 最後に書き始めた記録の順番
uint16_t eeAddr       = 0;                  // 書き込み中の記録の先頭番地
uint8_t  eeWriteIdx   = sizeof(SavedRecord); // 次に書くバイト (16 = 書き込み中でない)
uint32_t lastEeSaveMs = 0;

uint16_t crc16(const uint8_t *p, uint8_t n) {  // CRC-16/MODBUS
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= *p++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}

// 正しく書けている記録のうち一番新しいものを探す
bool eeLoad(SavedRecord &best) {
  bool found = false;
  for (uint8_t slot = 0; slot < EE_SLOTS; slot++) {
    SavedRecord r;
    eeprom_read_block(&r, (const void *)(uintptr_t)(slot * sizeof(SavedRecord)), sizeof(r));
    if (r.magic != EE_MAGIC || r.crc != crc16((const uint8_t *)&r, sizeof(r) - 2)) continue;
    if (!(fabs(r.totalmAh) < 1e7f)) continue;  // NaN・無限大・ありえない値は使わない
    if (!(r.distM >= 0.0f && r.distM < 1e8f)) continue;
    if (!found || (int32_t)(r.seq - best.seq) > 0) { best = r; found = true; }
  }
  if (found) eeSeq = best.seq;
  return found;
}

// 次の場所に書き始める (実際の書き込みは eeService が 1 バイトずつ行う)
void eeStart(float totalmAh, uint32_t elapsedMs, float distM) {
  eeBuf.seq = ++eeSeq;
  eeBuf.totalmAh = totalmAh;
  eeBuf.elapsedMs = elapsedMs;
  eeBuf.distM = distM;
  eeBuf.magic = EE_MAGIC;
  eeBuf.crc = crc16((const uint8_t *)&eeBuf, sizeof(eeBuf) - 2);
  eeAddr = (uint16_t)(eeBuf.seq % EE_SLOTS) * sizeof(SavedRecord);
  eeWriteIdx = 0;
}

// loop のたびに呼ぶ。前の書き込みが終わっていれば 1 バイト書く (待たない)
void eeService() {
  if (eeWriteIdx >= sizeof(eeBuf) || !eeprom_is_ready()) return;
  eeprom_update_byte((uint8_t *)(uintptr_t)(eeAddr + eeWriteIdx), ((const uint8_t *)&eeBuf)[eeWriteIdx]);
  eeWriteIdx++;
}

// 書きかけの記録を最後まで書く (待つ。積算リセットのときだけ使う)
void eeFinish() {
  while (eeWriteIdx < sizeof(eeBuf)) {
    wdtKick();
    eeprom_update_byte((uint8_t *)(uintptr_t)(eeAddr + eeWriteIdx), ((const uint8_t *)&eeBuf)[eeWriteIdx]);
    eeWriteIdx++;
  }
}

void eeSaveNow(float totalmAh, uint32_t elapsedMs, float distM) {
  eeFinish();
  eeStart(totalmAh, elapsedMs, distM);
  eeFinish();
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
    // 1行目: "V: 28.00V  I:-12.34A" (20 文字)。測定範囲を超えたら "V: 28.00V  I:  OVER"
    if (fabs(meas.current) >= OVER_CURRENT_A)
      snprintf_P(line, sizeof(line), PSTR("V:%sV  I:  OVER"), a);
    else
      snprintf_P(line, sizeof(line), PSTR("V:%sV  I:%sA"), a, b);
    lcdLine(0, line);

    long pw = lround(constrain(meas.power, 0.0f, 9999.0f));
    long mah = lround(constrain(meas.chargemAh, -99999.0f, 999999.0f));
    // 2行目: "P: 245W  Q:  1234mAh" (20 文字)
    snprintf_P(line, sizeof(line), PSTR("P:%4ldW  Q:%6ldmAh"), pw, mah);
    lcdLine(1, line);
  } else {
    // INA228 が読めない間も、最後に読めた積算を出す (INA228 の中では積算が続いているので、
    // 読めるようになれば正しい値に戻る)
    lcdLineP(0, F("INA228 I2C ERROR"));
    long mah = lround(constrain(lastKnownTotalmAh, -99999.0f, 999999.0f));
    snprintf_P(line, sizeof(line), PSTR("LAST    Q:%6ldmAh"), mah);
    lcdLine(1, line);
  }

  // 3行目: 経過時間 (リセットから) と速さ "TIME  12:34 32.5km/h" (100 分以上は "TIME 123:45 32.5km/h")
  uint32_t s = (now - timerStartMs) / 1000;
  char tm[12], sp[8];
  snprintf_P(tm, sizeof(tm), PSTR("%02lu:%02u"), s / 60, (unsigned)(s % 60));
  dtostrf(constrain(speedKmh, 0.0f, 99.9f), 4, 1, sp);   // " 5.0" / "32.5"
  snprintf_P(line, sizeof(line), PSTR("TIME %6s %skm/h"), tm, sp);
  lcdLine(2, line);

  // 4行目: SD 状態 "REC LOG003.CSV     *" (* は記録するたびに点滅) / "NO SD"
  //        INA228 を再設定したことがあれば "REC " を省いて "INA!回数" を出す: "LOG003.CSV   INA!1 *"
  //        ボタンを 3 秒以上押している間は、離すとリセットされることを知らせる
  uint32_t held = btnDown ? now - btnDownMs : 0;
  if (held >= BTN_STUCK_MS) {
    lcdLineP(3, F("BUTTON STUCK?"));
  } else if (held >= RESET_HOLD_MS) {
    lcdLineP(3, F("RELEASE=RESET mAh"));
  } else {
    if (!inaRecoverCount)
      snprintf_P(line, sizeof(line), sdOk ? PSTR("REC %s") : PSTR("NO SD"), logName);
    else
      snprintf_P(line, sizeof(line), PSTR("%-13sINA!%u"), sdOk ? logName : "NO SD", inaRecoverCount);
    if (sdOk) {
      size_t n = strlen(line);
      while (n < LCD_COLS - 1) line[n++] = ' ';
      line[LCD_COLS - 1] = heartbeat ? '*' : ' ';
      line[LCD_COLS] = '\0';
    }
    lcdLine(3, line);
  }
}

// ============================================================================
//  SD ログ
// ============================================================================
void logFileName(char *name, uint16_t index) {
  snprintf_P(name, 13, PSTR("LOG%03u.CSV"), index);
}

// LOG000.CSV〜LOG999.CSV の空き番号で新規作成し、logName に名前を入れる。
// SD.exists() は毎回ルートディレクトリを先頭から読むので、0 から 1 つずつ調べると
// ファイルが数百個あるときに数秒〜数十秒かかり、ウォッチドッグで再起動を繰り返してしまう。
// ファイルは番号順に作られるので、二分探索で「最後のファイルの次」を約 10 回の確認で見つける
// (途中の番号が抜けていても、見つかるのは必ず存在しない番号なので上書きはしない)
bool openNewLog() {
  char name[13];
  uint16_t lo = 0, hi = 1000;
  while (lo < hi) {
    uint16_t mid = (lo + hi) / 2;
    logFileName(name, mid);
    if (SD.exists(name)) lo = mid + 1; else hi = mid;
    wdtKick();
  }
  for (uint16_t idx = lo; idx < 1000; idx++) {
    logFileName(name, idx);
    wdtKick();
    if (SD.exists(name)) continue;
    logFile = SD.open(name, FILE_WRITE);
    if (!logFile) return false;
    logFile.println((const __FlashStringHelper *)CSV_HEADER);
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

void printSdStatus() {
#if SERIAL_DEBUG
  Serial.print(F("SD: "));
  if (sdOk) Serial.println(logName); else Serial.println(F("NO SD"));
#endif
}

// 起動時だけ SD カードを初期化してログを開く。カードが無いと最大 2 秒ほどかかる
void sdBegin() {
  wdtKick();
  sdOk = SD.begin(PIN_SD_CS) && openNewLog();
  wdtKick();
  if (!sdOk) logName[0] = '\0';
  printSdStatus();
}

// 新しいログファイルに切り替える (SD の初期化はしない。NO SD のときは何もしない)
void sdNextFile() {
  if (!sdOk) return;
  logFile.close();
  sdOk = openNewLog();
  if (!sdOk) logName[0] = '\0';
  printSdStatus();
}

// 1 行出力: 経過時間[s] (ミリ秒まで), 電圧[V], 電流[A], 電力[W], 積算[mAh], 速さ[km/h], 距離[m]
void printCsvRow(Print &out, uint32_t elapsedMs) {
  char buf[16];
  snprintf_P(buf, sizeof(buf), PSTR("%lu.%03u,"), elapsedMs / 1000,
             (unsigned)(elapsedMs % 1000));
  out.print(buf);
  out.print(dtostrf(meas.voltage, 1, 3, buf));  out.print(',');
  out.print(dtostrf(meas.current, 1, 3, buf));  out.print(',');
  out.print(dtostrf(meas.power, 1, 2, buf));    out.print(',');
  out.print(dtostrf(meas.chargemAh, 1, 2, buf)); out.print(',');
  out.print(dtostrf(speedKmh, 1, 2, buf));       out.print(',');
  out.println(dtostrf(distanceM, 1, 1, buf));
}

// SD カードがまだ応答するかを CMD13 (SEND_STATUS) で確かめる。カードの初期化はしない。
// SD ライブラリの flush() は書き込みに失敗しても知らせないので、抜けたことに気づくのが
// 512 バイト書き進んだとき (十数秒後) まで遅れる。これを flush のたびの確認で数秒以内にする。
// 抜けたカードは応答せず (0xFF のまま)、挿し直したカードは未初期化なので R1 が 0 にならない
bool sdCardResponds() {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_SD_CS, LOW);
  SPI.transfer(0xFF);
  const uint8_t cmd13[6] = {0x40 | 13, 0, 0, 0, 0, 0xFF};
  for (uint8_t i = 0; i < 6; i++) SPI.transfer(cmd13[i]);
  uint8_t r1 = 0xFF;
  for (uint8_t i = 0; i < 10 && (r1 & 0x80); i++) r1 = SPI.transfer(0xFF);  // R1 を待つ
  SPI.transfer(0xFF);  // R2
  digitalWrite(PIN_SD_CS, HIGH);
  SPI.transfer(0xFF);
  SPI.endTransaction();
  return r1 == 0x00;
}

void writeLog(uint32_t now) {
  if (!meas.ok) return;  // センサー読み取り失敗時は行を書かない
  printCsvRow(logFile, now - timerStartMs);
  bool cardLost = false;
  if (++linesSinceFlush >= LOG_FLUSH_EVERY) {
    logFile.flush();  // ディレクトリ情報まで書き込み → 電源断でもここまでは残る
    linesSinceFlush = 0;
    cardLost = !sdCardResponds();
  }
  if (cardLost || logFile.getWriteError()) {  // カード抜け等
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

// 長押し: 積算 mAh・経過時間・距離を 0 にして新しいログ
void doCounterReset() {
  bool ok = inaResetCharge();
  chargeOffsetmAh = 0;
  inaRecoverCount = 0;
  meas.chargemAh = 0;
  lastKnownTotalmAh = 0;
  speedResetDistance();
  eeSaveNow(0, 0, 0);  // 電源を切っても 0 から始まるように、すぐに保存する
  lcdLineP(0, ok ? F("RESET mAh") : F("RESET FAILED"));
  lcdLineP(1, F(""));
  lcdLineP(2, F(""));
  lcdLineP(3, F(""));
  sdNextFile();  // 新しいログファイル (SD の初期化はしない)
#if SERIAL_DEBUG
  Serial.println(F("# mAh reset"));
#endif
  delay(800);    // メッセージ表示
  restartTimer(0);
  persistSave(millis());
}

// 動作は離したときに決める。押した瞬間 (3 秒経過時) にリセットすると、
// 配線が挟まれて GND とショートしたときに走行中の積算が消えてしまうため
void handleButton(uint32_t now) {
  bool pressed = (digitalRead(PIN_BTN) == LOW);
  if (pressed) {
    if (!btnDown) {          // 押し始め (チャタリングで離れたらタイマーはやり直し)
      btnDown = true;
      btnDownMs = now;
    }
    return;
  }
  if (btnDown) {             // 離したとき
    uint32_t held = now - btnDownMs;
    btnDown = false;
    if (held >= BTN_STUCK_MS) {
      // 長すぎる: ショートや故障とみなして何もしない
    } else if (held >= RESET_HOLD_MS) {
      doCounterReset();
    }                        // 3 秒未満は何もしない
  }
}

// ============================================================================
//  RAM の実測 (実機・SERIAL_DEBUG 時のみ)
//  起動時に空き RAM を目印 (0xA5) で塗り、スタックが一番深く伸びたときに
//  上書きされずに残った量を「最小空き RAM」として 30 秒ごとにシリアルに出す
// ============================================================================
#if SERIAL_DEBUG && defined(__AVR__)
extern char __heap_start;
extern char *__brkval;
constexpr uint8_t RAM_CANARY = 0xA5;
uint32_t lastRamReportMs = 0;

uint8_t *ramHeapTop() { return (uint8_t *)(__brkval ? __brkval : &__heap_start); }

void ramPaint() {
  uint8_t *p = ramHeapTop(), *sp = (uint8_t *)SP;
  while (p < sp - 32) *p++ = RAM_CANARY;  // 今使っているスタックの手前 32 バイトは残す
}

uint16_t ramMinFree() {
  uint8_t *p = ramHeapTop(), *sp = (uint8_t *)SP;
  uint16_t n = 0;
  while (p < sp && *p == RAM_CANARY) { p++; n++; }
  return n;
}
#endif

// ============================================================================
//  setup / loop
// ============================================================================
void setup() {
#if USE_WATCHDOG
  wdt_enable(WDTO_8S);
#endif
#if SERIAL_DEBUG && defined(__AVR__)
  ramPaint();
#endif
  pinMode(PIN_BTN, INPUT_PULLUP);
  speedBegin();  // 起動中に走っても距離を数える
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

  // メーカー ID 確認 (0x3E = 0x5449)。配線の接触が一瞬悪いだけのこともあるので 1 秒ほど試す
  uint16_t id = 0;
  bool inaFound = false;
  for (uint8_t k = 0; k < 20 && !inaFound; k++) {
    inaFound = inaRead16(REG_MFG_ID, id) && id == 0x5449;
    if (!inaFound) { delay(50); wdtKick(); }
  }

  // 積算と経過時間の続きを探す。Nano だけのリセットならリセットで消えない RAM、
  // 電源が全部落ちたあとなら EEPROM に残っている
  SavedRecord saved;
  bool eeOk = eeLoad(saved);
  bool warmOk = persistValid();
  float knownTotal = warmOk ? persist.totalmAh : eeOk ? saved.totalmAh : 0;
  uint32_t knownElapsedMs = warmOk ? persist.elapsedMs : eeOk ? saved.elapsedMs : 0;
  float knownDistM = warmOk ? persist.distM : eeOk ? saved.distM : 0;
  lastKnownTotalmAh = knownTotal;

  uint8_t st = inaFound ? inaCheckConfig() : (uint8_t)INA_NO_RESPONSE;
  uint32_t resumeElapsedMs = knownElapsedMs;
  float resumeDistM = knownDistM;
  bool resumed = false;   // 積算を引き継いだ (表示を早く出すため、起動画面を短くする)
  if (st == INA_CONFIGURED) {
    // INA228 の設定が残っている = INA228 は電源が切れておらず Nano だけがリセットされた。
    // INA228 の CHARGE はそのまま続いているので、オフセットだけ戻す
    chargeOffsetmAh = warmOk ? persist.chargeOffsetmAh : 0;
    if (!warmOk) { resumeElapsedMs = 0; resumeDistM = 0; }  // 時間と距離は 0 から (積算は INA228 が続けている)
    resumed = true;
    lcdLineP(1, F("RESUMED (mAh KEPT)"));
  } else if (st == INA_NOT_CONFIGURED) {
    // INA228 も電源が入り直した (CHARGE は 0)。保存してあった積算から続ける
    bool ok = inaConfigure();
    chargeOffsetmAh = knownTotal;
    resumed = warmOk || eeOk;
    if (!ok) lcdLineP(0, F("INA228 CONFIG ERROR"));  // 止まらない。loop で設定し直す
    lcdLineP(1, resumed ? F("RESTORED (mAh KEPT)") : F("INA228 OK"));
  } else {
    // INA228 が応答しない。止まらずに loop で探し続け、つながったら続きから数える
    chargeOffsetmAh = warmOk ? persist.chargeOffsetmAh : knownTotal;
    char line[LCD_COLS + 1];
    lcdLineP(0, F("INA228 NOT FOUND"));
    snprintf_P(line, sizeof(line), PSTR("ID=%04X RETRYING"), id);
    lcdLine(1, line);
    lcdLineP(2, F("CHECK I2C WIRING"));
#if SERIAL_DEBUG
    Serial.println(F("INA228 not found (retrying)"));
#endif
  }
#if SERIAL_DEBUG
  Serial.print(F("INA228: "));
  Serial.println(st == INA_CONFIGURED ? F("resumed (mAh kept)") : st == INA_NO_RESPONSE ? F("no response")
                 : resumed ? F("restored (mAh kept)") : F("configured"));
  Serial.print(F("SHUNT_CAL=")); Serial.println(SHUNT_CAL);
#endif
  distOffsetM = resumeDistM;

  sdBegin();
  lcdLineP(2, sdOk ? F("SD OK") : F("NO SD"));
  if (sdOk) lcdLine(3, logName);
#if SERIAL_DEBUG
  Serial.println((const __FlashStringHelper *)CSV_HEADER);
#endif
  // 起動画面を見せる時間。積算を引き継いだときは、電流の表示を早く出すため短くする
  delay(resumed ? 500 : 1500);

  restartTimer(resumeElapsedMs);
  // 最新の積算を読んで、すぐに EEPROM へ保存し始める (起動直後に電源が落ちても、
  // リセット前の保存まで戻らないように)
  inaMeasure(meas);
  persistSave(millis());
  lastEeSaveMs = millis() - EE_SAVE_MS;
}

void loop() {
  wdtKick();
  uint32_t now = millis();
  eeService();  // EEPROM への保存を 1 バイト進める (待たない)

  handleButton(now);

  if ((int32_t)(now - nextLcdMs) >= 0) {
    nextLcdMs += LCD_INTERVAL_MS;
    if ((int32_t)(now - nextLcdMs) >= 0) nextLcdMs = now + LCD_INTERVAL_MS;  // 遅れたら追いつかない
    if (now - lastLcdInitMs >= LCD_REINIT_MS) {  // ノイズで化けた LCD を立て直す
      lcdResendSettings();
      lastLcdInitMs = now;
    }
    // 記録のほうが頻繁なら、記録のときに読んだ最新値を表示する (I2C の読み出しを減らす)
    if (LOG_INTERVAL_MS > LCD_INTERVAL_MS) inaMeasure(meas);
    speedUpdate();
    lcdShowMeasurement(now);
    persistSave(now);
  }

  // 積算 mAh・経過時間・距離を EEPROM に保存し始める (前の保存を書き終えていれば)
  if (now - lastEeSaveMs >= EE_SAVE_MS && eeWriteIdx >= sizeof(eeBuf)) {
    lastEeSaveMs = now;
    eeStart(lastKnownTotalmAh, now - timerStartMs, distanceNowM());
  }

  if ((int32_t)(now - nextLogMs) >= 0) {
    nextLogMs += LOG_INTERVAL_MS;    // 一定の刻みを保つ (ずれが溜まらない)
    if ((int32_t)(now - nextLogMs) >= 0) nextLogMs = now + LOG_INTERVAL_MS;
    inaMeasure(meas);                // 記録用に最新値を読む
    speedUpdate();
    if (sdOk) writeLog(now);
#if SERIAL_DEBUG
    printCsvRow(Serial, now - timerStartMs);
#endif
#if SERIAL_DEBUG && defined(__AVR__)
    if (now - lastRamReportMs >= 30000) {
      lastRamReportMs = now;
      Serial.print(F("# RAM min free: "));
      Serial.print(ramMinFree());
      Serial.println(F(" bytes"));
    }
#endif
  }
}
