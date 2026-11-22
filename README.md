# Ene1PowerMeter

Ene-1 GP（電動カー競技）用の車載電力計。Arduino Nano で INA228 から電圧・電流・電力・積算 mAh を読み、20×4 I2C 液晶に表示して microSD に CSV で記録する。

## ハードウェア

| 部品 | 内容 |
|---|---|
| マイコン | Arduino Nano（ATmega328P / 5V） |
| 電流・電圧センサー | Strawberry Linux #11228（INA228、シャント 2mΩ、I2C 0x40） |
| 表示 | 秋月 ACM2004D-FLW-FBW-IIC（20×4、RW1063） |
| 記録 | microSD モジュール（SPI、CS = D10） |
| ボタン | D2–GND 間（長押し＝積算リセット、短押し＝SD 再試行） |

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
| 1 秒長押し | 積算 mAh と経過時間を 0 にして、新しいログファイルを作る |
| 短く押す（NO SD のときだけ） | 積算と経過時間はそのままで SD を再初期化する |

## 故障への備え

- **Nano だけがリセットされたとき**（ウォッチドッグ、スケッチの書き込み、Nano のリセットボタンなど）は、INA228 に設定が残っていれば積算 mAh を消さずに再開する。経過時間も、リセットで消えない RAM 領域（`.noinit`）から復元する。新しいログファイルは作るが、時刻は続きから記録する。
- **走行中に INA228 の設定が消えたとき**（電源の瞬断など）は、それを検出して設定し直し、それまでの積算値を引き継ぐ。3 行目に `INA!回数` を表示する。
- **LCD がノイズで化けたとき**に備え、5 秒ごとに初期化コマンドを再送する（表示は消さない）。
- **処理が固まったとき**は、8 秒でウォッチドッグが Nano をリセットする（`USE_WATCHDOG`）。旧ブートローダの Nano 互換機では、ウォッチドッグでのリセット後に再起動を繰り返すことがある。

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

## 校正

基準の電流計と比べて電流がずれる場合は、`R_SHUNT_OHM` を調整する（表示が 1% 大きい → 0.002 × 1.01 = 0.00202）。電流・電力・積算 mAh のすべてに反映される。

## 状態

LCD の I2C アドレスと初期化コマンド（`LCD_ADDR`, `LCD_INIT_SEQ`）は秋月サンプルとの照合待ちの仮の値。
