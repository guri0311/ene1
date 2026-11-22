# Ene1PowerMeter

Ene-1 GP（電動カー競技）用の車載電力計。Arduino Nano で INA228 から電圧・電流・電力・積算 mAh を読み、20×4 I2C 液晶に表示して microSD に CSV で記録する。

## ハードウェア

| 部品 | 内容 |
|---|---|
| マイコン | Arduino Nano（ATmega328P / 5V） |
| 電流・電圧センサー | Strawberry Linux #11228（INA228、シャント 2mΩ、I2C 0x40）。ローレンジ（±40.96mV）で 20A まで測定 |
| 表示 | 秋月 ACM2004D-FLW-FBW-IIC（20×4、RW1063） |
| 記録 | microSD モジュール（SPI、CS = D10） |
| ボタン | D2–GND 間（3 秒以上押して離す＝積算リセット） |

| ピン | 用途 |
|---|---|
| A4 / A5 | I2C SDA / SCL（4.7kΩ で 5V へプルアップ） |
| D10 / D11 / D12 / D13 | SD の CS / MOSI / MISO / SCK |
| D2 | ボタン |

## 表示

2 行目の `Q:` はリセットからの積算 mAh（INA228 の CHARGE レジスタ）。

```
V: 24.53V  I: 12.34A
P: 303W  Q:  1234mAh
TIME  12:34
REC LOG003.CSV     *
```

## ボタン

| 操作 | 動作 |
|---|---|
| 3 秒以上押して離す | 積算 mAh と経過時間を 0 にして、新しいログファイルを作る。押している間は 4 行目に `RELEASE=RESET mAh` |
| 3 秒未満で離す | 何もしない |
| 10 秒以上押したまま | 配線のショートとみなし、離しても何もしない（4 行目に `BUTTON STUCK?`） |

リセットを「離したとき」に行うのは、ボタン線が挟まれて GND とショートしても走行中の積算が消えないようにするため。

SD カードの初期化は起動時だけ行う。走行中に `NO SD` になった場合は、電源を入れ直すまで記録しない（表示と積算は続く）。

## 故障への備え

- **Nano だけがリセットされたとき**（ウォッチドッグ、スケッチの書き込み、Nano のリセットボタンなど）は、INA228 に設定が残っていれば積算 mAh を消さずに再開する。経過時間も、リセットで消えない RAM 領域（`.noinit`）から復元する。新しいログファイルは作るが、時刻は続きから記録する。
- **走行中に INA228 の設定が消えたとき**（電源の瞬断など）は、それを検出して設定し直し、それまでの積算値を引き継ぐ。3 行目に `INA!回数` を表示する。
- **LCD がノイズで化けたとき**に備え、5 秒ごとに初期化コマンドを再送する（表示は消さない）。
- **SD カードが抜けたとき**は、5 行ごとの書き込み確定のあとにカードの状態を確かめ（CMD13、初期化はしない）、約 2 秒で `NO SD` にする。
- **I2C がノイズで固まったとき**は、Wire のタイムアウト（3ms）で復帰する。
- **処理が固まったとき**は、8 秒でウォッチドッグが Nano をリセットし、積算を引き継いで再開する（`USE_WATCHDOG`）。Optiboot の Nano（Arduino IDE で「ATmega328P」を選んで書き込める Nano）ではそのまま再開する。旧ブートローダの Nano（「ATmega328P (Old Bootloader)」）では再起動を繰り返して表示が止まるので、Optiboot を書き込むか `USE_WATCHDOG 0` にする。
- **SD にログが大量にたまっても**、空き番号を二分探索で探すので起動は遅くならない。

動かないときは [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) を参照。

## ログ

`LOG000.CSV`, `LOG001.CSV` … の空き番号に 1 秒ごとに記録する。

```
time_s,voltage_V,current_A,power_W,charge_mAh
```

- `charge_mAh`：INA228 の CHARGE レジスタ（符号付き。回生分は差し引かれる）

## ビルド

必要なライブラリ: SD（Wire / SPI は標準）

```bash
arduino-cli lib install SD
```

```bash
arduino-cli compile --fqbn arduino:avr:nano Ene1PowerMeter
```

## テスト

### 実機バイナリのエミュレーターテスト（test/avr_emu）

実機に書き込むのと同じ .hex を ATmega328P エミュレーター（[avr8js](https://github.com/wokwi/avr8js)）で動かす。I2C には INA228 と LCD（RW1063）のモデル、SPI には FAT32 イメージを持つ SD カードのモデルをつなぎ、Optiboot・旧ブートローダも一緒に動かして確認する。CSV はイメージから読み出し、FAT の整合性は `fsck.fat` で確かめる。

必要なもの: Node.js、arduino-cli（arduino:avr コアと SD ライブラリ）、dosfstools、mtools

```bash
cd test/avr_emu && npm install
```

```bash
cd test/avr_emu && node run.mjs
```

### PC 上のシミュレーション（test/host_sim）

スケッチを PC 用にコンパイルし、INA228・LCD・SD を模擬して動かす。速く、メモリ破壊・未定義動作の検査（`SIM_SANITIZE=1`）もできる。

```bash
bash test/host_sim/run_tests.sh
```

## 校正

基準の電流計と比べて電流がずれる場合は、`R_SHUNT_OHM` を調整する（表示が 1% 大きい → 0.002 × 1.01 = 0.00202）。電流・電力・積算 mAh のすべてに反映される。

## 状態

LCD の I2C 仕様と初期化手順は RW1063 データシート（Rev.0.5）で確認済み。

- I2C アドレスは 0x3C〜0x3F のどれか（モジュール基板の SA1/SA0 の配線で決まる）。起動時に `LCD_ADDR`（0x3C）から応答を確かめ、なければ 0x3C〜0x3F を自動で探す。見つかったアドレスはシリアルに出る。
- 初期化はデータシート p.32 の手順（Function set → Display ON → Clear → Entry mode）と待ち時間どおり。
- 20×4 の各行の先頭アドレス（00h / 40h / 14h / 54h）は、RW1063 の 2 行モードを 20 文字ずつ折り返す一般的な配置。実機で行の順番が正しいか確認すること。
