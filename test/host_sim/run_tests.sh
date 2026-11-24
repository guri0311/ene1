#!/usr/bin/env bash
# Ene1PowerMeter の PC 上シミュレーションテスト
#   bash test/host_sim/run_tests.sh
# INA228 と LCD (RW1063) をデータシートどおりにモデル化し、スケッチを実際に動かして確認する
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="${TMPDIR:-/tmp}/ene1_sim_$$"
mkdir -p "$WORK"
trap 'rm -rf "$WORK"' EXIT

CXX_FLAGS=(-std=gnu++17 -O1 -Wall -Wextra -Wno-unused-parameter -I "$HERE/mock")
# SIM_SANITIZE=1 でメモリ破壊・未定義動作の検査付きでビルドする
if [ "${SIM_SANITIZE:-0}" = 1 ]; then CXX_FLAGS+=(-g -fsanitize=address,undefined -fno-sanitize-recover=undefined); fi
g++ "${CXX_FLAGS[@]}" -o "$WORK/sim" "$HERE/sim.cpp" || exit 1
# 高レンジ (ADCRANGE=0)・最大電流 81.92A にした版 (SHUNT_CAL が 0x1000 になる境界条件)
mkdir -p "$WORK/v8192"
sed -e 's/MAX_CURRENT_A   = 20.0f/MAX_CURRENT_A   = 81.92f/' -e 's/INA_ADCRANGE = 1;/INA_ADCRANGE = 0;/' \
  "$HERE/../../Ene1PowerMeter/Ene1PowerMeter.ino" > "$WORK/v8192/sketch.ino"
grep -q "MAX_CURRENT_A   = 81.92f" "$WORK/v8192/sketch.ino" && grep -q "INA_ADCRANGE = 0;" "$WORK/v8192/sketch.ino" \
  || { echo "81.92A 版の生成に失敗"; exit 1; }
g++ "${CXX_FLAGS[@]}" -DSKETCH_PATH="\"$WORK/v8192/sketch.ino\"" -o "$WORK/sim8192" "$HERE/sim.cpp" || exit 1
# 車輪に磁石を 4 個付けた版 (速さは車輪 1 周ぶんのパルスから出す)
mkdir -p "$WORK/mag4"
sed -e 's/SPEED_MAGNETS   = 1;/SPEED_MAGNETS   = 4;/' "$HERE/../../Ene1PowerMeter/Ene1PowerMeter.ino" > "$WORK/mag4/sketch.ino"
grep -q "SPEED_MAGNETS   = 4;" "$WORK/mag4/sketch.ino" || { echo "磁石 4 個の版の生成に失敗"; exit 1; }
g++ "${CXX_FLAGS[@]}" -DSKETCH_PATH="\"$WORK/mag4/sketch.ino\"" -o "$WORK/simmag4" "$HERE/sim.cpp" || exit 1

FAILED=0
# boot <名前> <cold|warm> : 標準入力のシナリオを 1 回の起動として実行
# 同じ <名前> の続けての起動は、INA228 と .noinit RAM を引き継ぐ (Nano だけのリセット)
boot() {
  local name="$1" mode="$2" bin="${3:-sim}"
  mkdir -p "$WORK/$name/sd"
  echo "---- [$name] $mode boot ($bin)"
  if ! "$WORK/$bin" "$WORK/$name/state" "$WORK/$name/sd" "$mode"; then FAILED=$((FAILED + 1)); fi
}

boot basic cold <<'EOF'
say "1. 起動と表示・記録 (24V, 10A を 6 分 → 1000mAh)"
v 24
setup
expect_lcd 1 "INA228 OK"
expect_lcd 3 "LOG000.CSV"
run 2000
expect_lcd 0 "V: 24.00V  I:  0.00A"
expect_lcd 2 "TIME  00:02  0.0km/h"
expect_lcd 3 "REC LOG000.CSV"
expect_first LOG000.CSV "time_s,voltage_V,current_A,power_W,charge_mAh,speed_kmh,distance_m"
i 10
run 360000
lcd
expect_lcd 0 "V: 24.00V  I: 10.00A"
expect_lcd 1 "P: 240W  Q:  1000mAh"
expect_lcd 2 "TIME  06:02"
expect_charge 998 1002
expect_last LOG000.CSV ",24.000,10.000,240.00,"
# 速度センサーをつないでいない (車輪が回らない) ときは、速さも距離も 0
expect_last LOG000.CSV ",0.00,0.0"
# 50ms ごと (1 秒に 20 行) に 362 秒 → 見出しを含めて約 7241 行。
# 1 秒ごとに flush するので、最後の最大 20 行はまだカードに書かれていないことがある
expect_lines LOG000.CSV 7215 7245
expect_clean

say "2. 逆向きの電流 (回生 -2A を 3 分 → 100mAh 減る)"
i -2
run 180000
lcd
expect_lcd 0 "I: -2.00A"
expect_lcd 1 "P:  48W"
expect_charge 898 902
expect_clean

say "3. 3 秒未満で離すと何もしない (短押し・2.5 秒押し)"
press 300
press 2500
expect_charge 898 902
expect_lcd 2 "TIME  09:0"
expect_lcd 3 "REC LOG000.CSV"

say "4. 3 秒以上押している間は合図、離すと積算・時間を 0、新しいファイル"
button down
run 2500
expect_not_lcd 3 "RELEASE"
run 1000
expect_lcd 3 "RELEASE=RESET mAh"
expect_charge 893 902
button up
run 1000
lcd
expect_charge -1 0.5
expect_lcd 2 "TIME  00:00"
expect_lcd 3 "REC LOG001.CSV"
expect_last LOG001.CSV "charge_mAh"
run 6000
# リセットで時刻が 0 から数え直されている (1 秒ごとに確定するので最終行は 6 秒の行)
expect_last LOG001.CSV "6.000,"
say "4b. リセットのあとも積算が再開する (RSTACC が 1 のまま残ると 0 で止まる)"
i 10
run 36000
expect_charge 93 99
expect_clean
EOF

boot sdfail cold <<'EOF'
say "5. 走行中に SD が抜ける → NO SD、表示と積算は続く"
v 24
i 5
setup
run 60000
sd out
run 3000
lcd
expect_lcd 3 "NO SD"
expect_charge 85 92
say "5b. カードを挿し直しても、SD の初期化はしないので NO SD のまま (短押し・リセットでも)"
sd in
press 200
run 2000
expect_lcd 3 "NO SD"
expect_charge 90 97
press 3500
run 1500
lcd
expect_lcd 3 "NO SD"
expect_charge -1 5
expect_lcd 2 "TIME  00:0"
expect_clean
EOF

boot nosd cold <<'EOF'
say "6. SD なしで起動 → 表示は続く"
v 24
i 3
sd out
setup
run 5000
lcd
expect_lcd 0 "I:  3.00A"
expect_lcd 3 "NO SD"
expect_clean
EOF

boot inaglitch cold <<'EOF'
say "7. 走行中に INA228 だけ電源瞬断 → 再設定して積算を引き継ぐ"
v 24
i 10
setup
run 36000
expect_charge 100 106
ina glitch
run 36000
lcd
expect_lcd 3 "LOG000.CSV   INA!1"
expect_charge 198 210
expect_clean
EOF

boot inaoff cold <<'EOF'
say "8. INA228 の配線が外れる → エラー表示、記録しない。戻れば復帰"
v 24
i 10
setup
run 10000
ina off
run 5000
lcd
expect_lcd 0 "INA228 I2C ERROR"
ina on
run 2000
expect_lcd 0 "I: 10.00A"
expect_clean
EOF

boot lcdnoise cold <<'EOF'
say "9. LCD がノイズで化ける → 5 秒以内に元に戻る"
v 24
i 1
setup
run 3000
noise
run 5500
lcd
expect_lcd 0 "V: 24.00V"
expect_lcd 3 "REC"
expect_clean
EOF

boot warm cold <<'EOF'
say "10. Nano だけリセット (前半): 10A で 6 分"
v 24
i 10
setup
run 360000
expect_charge 1000 1008
EOF
boot warm warm <<'EOF'
say "10b. Nano だけリセット (後半): 積算と時間を引き継ぐ"
v 24
i 10
setup
expect_lcd 1 "RESUMED (mAh KEPT)"
run 2000
lcd
expect_charge 1008 1020
expect_lcd 2 "TIME  06:0"
expect_lcd 3 "REC LOG001.CSV"
expect_clean
EOF
boot warm cold <<'EOF'
say "10c. 電源が全部落ちて戻る → EEPROM に保存した積算と時間から続ける (保存は 2 秒ごと)"
v 24
i 10
setup
expect_lcd 1 "RESTORED (mAh KEPT)"
run 3000
lcd
# 6 分 + 起動中の 1.5 秒で約 1004mAh。最後の保存は最大 2 秒前なので、そこから 3 秒ぶん増える
expect_charge 1000 1018
expect_lcd 2 "TIME  06:0"
expect_clean
EOF
boot warm warm <<'EOF'
say "10d. 3 秒長押しで 0 にしてから電源を入れ直す → 0 から"
v 24
i 0
setup
run 1000
press 3500
run 1000
EOF
boot warm cold <<'EOF'
say "10e. (10d の続き) 電源を入れ直しても 0 のまま"
v 24
setup
run 2000
lcd
expect_charge -0.5 0.5
expect_lcd 2 "TIME  00:0"
expect_clean
EOF

boot noina cold <<'EOF'
say "11. 起動時に INA228 が見つからない → 止まらずに探し続け、つながったら表示する"
v 24
i 10
ina off
setup
lcd
expect_lcd 0 "INA228 NOT FOUND"
expect_lcd 1 "RETRYING"
run 5000
lcd
expect_lcd 0 "INA228 I2C ERROR"
expect_lcd 1 "LAST    Q:     0mAh"
ina on
run 36000
lcd
expect_lcd 0 "I: 10.00A"
expect_charge 98 101
expect_clean
EOF

boot powerloss cold <<'EOF'
say "11b. 走行中に電力計の電源が全部落ちる (前半): 10A で約 60 秒"
v 24
i 10
setup
run 60000
# 起動中の 1.5 秒も 10A が流れているので 61.5 秒ぶん = 約 171mAh
expect_charge 169 173
EOF
boot powerloss cold <<'EOF'
say "11c. (後半) 電源が戻る → 0 に戻らず続きから数え、すぐに電流を表示する"
v 24
i 10
setup
expect_lcd 1 "RESTORED (mAh KEPT)"
run 300
lcd
expect_lcd 0 "I: 10.00A"
# 最後の保存 (最大 2 秒前、約 5.6mAh 前) から続ける
expect_charge 165 175
run 36000
expect_charge 265 278
expect_clean
EOF

boot torn cold <<'EOF'
say "11d. EEPROM に書いている途中で電源が落ちる (前半): 10A で 61 秒"
v 24
i 10
setup
run 61000
eeprom_tear
EOF
boot torn cold <<'EOF'
say "11e. (後半) 壊れた記録は使わず、1 つ前の記録 (2 秒前) から続ける"
v 24
i 0
setup
expect_lcd 1 "RESTORED (mAh KEPT)"
run 1000
lcd
expect_charge 158 170
expect_clean
EOF

boot inalost cold <<'EOF'
say "11f. INA228 が読めない間も、最後に読めた積算を表示する"
v 24
i 10
setup
run 36000
ina off
run 3000
lcd
expect_lcd 0 "INA228 I2C ERROR"
expect_lcd 1 "LAST    Q:   10"
ina on
run 2000
expect_charge 104 112
expect_clean
EOF

boot lcd3d cold <<'EOF'
say "12. LCD のアドレスが 0x3D でも自動で見つける"
lcdaddr "0x3D"
v 24
setup
run 1000
lcd
expect_lcd 0 "V: 24.00V"
expect_clean
EOF

boot overrange cold <<'EOF'
say "13. ローレンジ (20A まで): 19.9A は正しく表示、25A は OVER 表示で 20A に頭打ち"
v 24
setup
i 19.9
run 2000
expect_lcd 0 "I: 19.90A"
i 25
run 36000
lcd
expect_lcd 0 "I:  OVER"
expect_charge 205 215
i -25
run 2000
expect_lcd 0 "I:  OVER"
expect_clean
EOF

boot longrun cold <<'EOF'
say "14. 長時間 (15A で 2 時間 → 30000mAh、100 分超の時間表示)"
v 24
i 15
setup
run 7200000
lcd
expect_charge 29990 30020
expect_lcd 1 "Q: 300"
expect_lcd 2 "TIME 120:0"
expect_lcd 2 "  0.0km/h"
# 2 秒ごとに 50 か所へ順番に書くので、2 時間で 1 バイトあたり約 72 回
expect_ee_wear 75
expect_clean
EOF

boot stuck cold <<'EOF'
say "15. 走行中にボタン線が GND とショート (押しっぱなし) → 積算は消えない"
v 24
setup
i 10
run 60000
button down
run 4000
expect_lcd 3 "RELEASE=RESET mAh"
run 8000
lcd
expect_lcd 3 "BUTTON STUCK?"
button up
run 2000
expect_lcd 3 "REC LOG000.CSV"
expect_charge 200 210
expect_clean
EOF

boot stuckboot cold <<'EOF'
say "16. ボタンがショートしたまま起動 → リセットは起きず、表示は続く"
button down
v 24
i 5
setup
run 15000
lcd
expect_lcd 0 "I:  5.00A"
expect_lcd 3 "BUTTON STUCK?"
expect_charge 20 25
EOF

boot glitch8192 cold sim8192 <<'EOF'
say "17. 高レンジ・81.92A (SHUNT_CAL = 0x1000 = 既定値と同じ) でも INA228 の瞬断を検出する"
v 24
i 90
setup
run 2000
expect_lcd 0 "I:  OVER"
i 10
run 36000
ina glitch
run 36000
lcd
expect_lcd 3 "INA!1"
expect_charge 270 290
expect_clean
EOF

boot sdmany cold <<'EOF'
say "18. SD にログが 450 個たまっている → 起動が遅れず、ウォッチドッグで再起動しない"
sdfiles 450
v 24
setup
run 2000
expect_lcd 3 "REC LOG450.CSV"
expect_clean
EOF

# ---------------------------------------------------------------------------
#  速度センサー (車輪 1 周 1.571m、磁石 1 個。30km/h なら 0.189 秒ごとにパルス)
# ---------------------------------------------------------------------------
boot speed cold <<'EOF'
say "19. 速度センサー: 30km/h で 60 秒 → 表示 30.0km/h、距離 500m、CSV にも記録"
v 24
i 8
setup
run 2000
wheel 30
run 60000
lcd
expect_lcd 2 "TIME  01:0"
expect_lcd 2 "30.0km/h"
expect_speed 29.95 30.05
# 30km/h x 60 秒 = 500m (パルスは 1.571m ごと)
expect_dist 497 503
expect_last_col LOG000.CSV 5 29.95 30.05
expect_last_col LOG000.CSV 6 495 503
expect_clean

say "19b. 加速 (30 → 45km/h) と減速 (45 → 15km/h) に 1 秒以内でついていく"
wheel 45
run 1000
expect_speed 44.9 45.1
wheel 15
run 1000
expect_speed 14.9 15.1
lcd
expect_lcd 2 "15.0km/h"

say "19c. 止まる → パルスが来なくなって 2 秒ほどで 0km/h。距離はそのまま"
expect_dist 516 524
wheel 0
run 600
# 減速中の扱い: 次のパルスが遅れている間は、その遅れから出した速さ (今の速さ以下) を出す
expect_speed 0 12
run 1800
lcd
expect_speed 0 0
expect_lcd 2 " 0.0km/h"
expect_dist 516 524
expect_last_col LOG000.CSV 5 0 0
run 5000
expect_dist 516 524

say "19d. また走り出す: 1 パルス目では速さを出さず、2 パルス目から正しい速さ"
wheel 20
run 150
expect_speed 0 0
run 1000
expect_speed 19.95 20.05
say "19e. 下りの高速 (95km/h) でもパルスを捨てずに正しく測れる (SPEED_MAX_KMH=110)"
wheel 95
run 1000
expect_speed 94.8 95.2
lcd
expect_lcd 2 "95.0km/h"
expect_clean
EOF

boot speednoise cold <<'EOF'
say "20. チャタリング (1 パルスに 4 回の跳ね返り) と、数 us で消えるノイズ (1 秒に 50 回) は数えない"
v 24
i 6
setup
wheel_bounce 4
wheel_noise 50
wheel 36
run 50000
lcd
expect_lcd 2 "36.0km/h"
expect_speed 35.95 36.05
# 36km/h = 10m/s で 50 秒 → 500m
expect_dist 497 503
expect_clean

say "20b. 速度センサーの線が外れて止まったまま (パルスが来ない) → 速さ 0、ほかの表示と記録は続く"
wheel_bounce 0
wheel_noise 0
wheel 0
run 3000
lcd
expect_speed 0 0
expect_lcd 0 "V: 24.00V  I:  6.00A"
expect_lcd 3 "REC LOG000.CSV"
EOF

boot speedina cold <<'EOF'
say "21. INA228 が読めない間も、速さは表示し続ける"
v 24
i 5
setup
wheel 25
run 5000
ina off
run 3000
lcd
expect_lcd 0 "INA228 I2C ERROR"
expect_lcd 2 "25.0km/h"
ina on
run 2000
expect_lcd 0 "I:  5.00A"
expect_clean
EOF

boot dist cold <<'EOF'
say "22. 距離の引き継ぎ (前半): 36km/h で 60 秒 → 約 600m"
v 24
i 8
setup
wheel 36
run 60000
expect_dist 597 604
EOF
boot dist warm <<'EOF'
say "22b. Nano だけリセット → 距離を引き継ぐ (止まっている間)"
v 24
i 0
setup
expect_lcd 1 "RESUMED (mAh KEPT)"
run 2000
# .noinit RAM への保存は 0.25 秒ごと (36km/h で 2.5m)
expect_dist 596 604
wheel 36
run 10000
expect_dist 696 704
expect_last_col LOG001.CSV 6 692 704
EOF
boot dist cold <<'EOF'
say "22c. 電源が全部落ちて戻る → EEPROM に保存した距離から続ける (保存は 2 秒ごと)"
v 24
i 0
setup
expect_lcd 1 "RESTORED (mAh KEPT)"
run 2000
# 最後の保存は最大 2 秒前 (36km/h で 20m) まで戻る
expect_dist 676 704
press 3500
run 1000
say "22d. 3 秒長押しで距離も 0"
expect_dist 0 0
wheel 18
run 10000
expect_dist 46 52
EOF
boot dist cold <<'EOF'
say "22e. (22d の続き) 電源を入れ直す → 0 にしたあとの距離から続ける"
v 24
setup
run 1000
expect_dist 42 52
EOF

boot magnets4 cold simmag4 <<'EOF'
say "23. 磁石 4 個 (間隔をわざとずらす): 車輪 1 周ぶんで平均するので、速さはずれない"
v 24
i 8
wheel_uneven on
setup
wheel 30
run 20000
lcd
expect_lcd 2 "30.0km/h"
expect_speed 29.95 30.05
# 30km/h x 20 秒 = 167m。パルスは 0.39m ごと
expect_dist 164 169
wheel 8
run 3000
expect_speed 7.95 8.05
wheel 0
run 1000
expect_speed 0 0
expect_clean
EOF

echo
if [ "$FAILED" -eq 0 ]; then echo "ALL PASS"; else echo "FAILED: $FAILED boot(s)"; fi
exit "$FAILED"
