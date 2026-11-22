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
expect_lcd 3 "REC LOG000.CSV"
i 10
run 360000
lcd
expect_lcd 0 "V: 24.00V  I: 10.00A"
expect_lcd 1 "P: 240W  Q:  1000mAh"
expect_lcd 2 "TIME  06:02"
expect_charge 998 1002
expect_last LOG000.CSV ",24.000,10.000,240.00,"
# 5 行ごとに flush するので、最後の数行はまだカードに書かれていない
expect_lines LOG000.CSV 357 364
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
expect_last LOG001.CSV "5.0,"
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
expect_lcd 2 "INA!1"
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
say "10c. 電源を入れ直す → 0 から"
v 24
setup
run 2000
expect_charge -0.5 0.5
expect_lcd 2 "TIME  00:0"
EOF

boot noina cold <<'EOF'
say "11. INA228 が見つからない → エラー表示で停止"
ina off
setup
lcd
expect_lcd 0 "INA228 NOT FOUND"
expect_lcd 3 "STOPPED"
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
expect_lcd 2 "TIME  120:0"
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
expect_lcd 2 "INA!1"
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

echo
if [ "$FAILED" -eq 0 ]; then echo "ALL PASS"; else echo "FAILED: $FAILED boot(s)"; fi
exit "$FAILED"
