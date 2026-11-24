#!/usr/bin/env bash
# ログビューア (tools/log_viewer) の計算部分のテスト
#   bash test/log_viewer/run_tests.sh
# PC 上のシミュレーション (test/host_sim) でスケッチを実際に動かして CSV を作り、
# それをビューアの計算部分で読んで、走行のつなぎ方と集計が正しいかを確かめる。
# スケッチの CSV の形が変わってビューアが読めなくなったら、ここで気づける。
# 必要なもの: g++、Node.js
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SIM="$HERE/../host_sim"
WORK="${TMPDIR:-/tmp}/ene1_viewer_$$"
mkdir -p "$WORK/sd"
trap 'rm -rf "$WORK"' EXIT

g++ -std=gnu++17 -O1 -Wall -Wno-unused-parameter -I "$SIM/mock" -o "$WORK/sim" "$SIM/sim.cpp" || exit 1

# 1 回目の起動: 10A・30km/h を 120 秒 → 20A を超える電流 (OVER)・36km/h を 5 秒 → 5A・18km/h を 60 秒
"$WORK/sim" "$WORK/state" "$WORK/sd" cold > "$WORK/boot1.txt" <<'S' || { cat "$WORK/boot1.txt"; exit 1; }
v 24
setup
i 10
wheel 30
run 120000
i 25
wheel 36
run 5000
i 5
wheel 18
run 60000
S
# 2 回目: Nano だけのリセット (時刻・mAh・距離は続きから、ファイルは LOG001) → 8A・27km/h を 60 秒
#         止まって 3 秒長押しで積算リセット (LOG002、時刻と距離は 0 から) → 4A・24km/h を 30 秒
"$WORK/sim" "$WORK/state" "$WORK/sd" warm > "$WORK/boot2.txt" <<'S' || { cat "$WORK/boot2.txt"; exit 1; }
v 24
setup
i 8
wheel 27
run 60000
i 0
wheel 0
press 3500
i 4
wheel 24
run 30000
S

node "$HERE/test.mjs" "$WORK/sd"
