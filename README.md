# Ene1PowerMeter

Ene-1 GP（電動カー競技）用の車載電力計。Arduino Nano で INA228 から電圧・電流・電力・積算電力量を読み、20×4 I2C 液晶に表示して microSD に CSV で記録する。

## ハードウェア

| 部品 | 内容 |
|---|---|
| マイコン | Arduino Nano（ATmega328P / 5V） |
| 電流・電圧センサー | Strawberry Linux #11228（INA228、シャント 2mΩ、I2C 0x40） |
| 表示 | 秋月 ACM2004D-FLW-FBW-IIC（20×4、RW1063） |
| 記録 | microSD モジュール（SPI、CS = D10） |
| リセット | D2–GND 間のボタン（1 秒長押し） |

| ピン | 用途 |
|---|---|
| A4 / A5 | I2C SDA / SCL（4.7kΩ で 5V へプルアップ） |
| D10 / D11 / D12 / D13 | SD の CS / MOSI / MISO / SCK |
| D2 | 積算リセットボタン |

## 表示

```
V: 24.53V  I: 12.34A
P: 303W  E:  1.234Wh
TIME  12:34
REC LOG003.CSV     *
```

## ログ

`LOG000.CSV`, `LOG001.CSV` … の空き番号に 1 秒ごとに記録する。

```
time_s,voltage_V,current_A,power_W,energy_Wh
```

## ビルド

必要なライブラリ: SD（Wire / SPI は標準）

```bash
arduino-cli lib install SD
```

```bash
arduino-cli compile --fqbn arduino:avr:nano .
```

## 状態

LCD の I2C アドレスと初期化コマンド（`LCD_ADDR`, `LCD_INIT_SEQ`）は秋月サンプルとの照合待ちの仮の値。
