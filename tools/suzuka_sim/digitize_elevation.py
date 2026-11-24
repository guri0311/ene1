#!/usr/bin/env python3
"""鈴鹿フルコースの標高図 (大会の特別規則 付則1 ELEVATION) を読み取り、50m ごとの標高の表にする。

suzuka-sim.js の ELEVATION_RAW は、2026 Ene-1 SUZUKA Challenge KV-40 特別規則 (PDF 32 ページ) の
26 ページ (図の印刷ページ番号は 25) を、このやり方で読み取った値。いまのスクリプトで作り直すと、
コードの値との差は最大 0.3m (読み取りの細かい違い。シミュレーションの結果はほぼ変わらない)。
PDF はリポジトリに入れていない (大会主催者の資料なので)。

使い方:
    pdftoppm -r 300 -f 26 -l 26 -png 規則.pdf elev      # elev-26.png ができる (poppler-utils)
    python3 tools/suzuka_sim/digitize_elevation.py elev-26.png

必要なもの: Python 3、Pillow、NumPy

やっていること:
  1. 図は 1km ごとに 6 段。各段の横の格子線 (60, 50, 40, 30, 20, 10m) を、行の 6 割以上が黒い行として見つける
  2. 各段の左右の km の破線の x 座標 (MARKS) で、横の位置を距離に直す。MARKS は 300dpi の画像を見て
     手で合わせた値なので、解像度や図が変わったら見直す (最後の段の右端は Finish = 5.807km)
  3. 縦の列ごとに、格子線の上を除いた太い線 (曲線) の中心を探す。前の列から 12px 以上飛んだ点は捨てる
     (Under pass / Over pass の箱や文字を拾わないため)。段の始まりは、前の段の終わりの標高から探す
  4. 50m ごとに平均する。曲線が格子線とちょうど重なって読めない区間 (4.3〜4.7km 付近) は前後をつなぐ
  5. 最後の点は図の読み取り値のまま出す。1 周で同じ高さに戻すずれの補正は suzuka-sim.js が行う

カーブの半径 (CORNERS) は、図の下の破線の範囲と R の数字を、同じ画像から目で読んで書いた
(画像の x 座標 → 距離の換算は 2 と同じ)。
"""
import json
import sys

import numpy as np
from PIL import Image

LAP_M = 5807
DARK = 110          # これより暗い画素を線とみなす
# 各段の左右の km の破線の x 座標 [px] (300dpi)。最後の段の右は Finish
MARKS = [(151, 2118), (127, 2112), (127, 2115), (126, 2121), (127, 2120), (121, 1779)]


def find_gridlines(dark):
    w = dark.shape[1]
    frac = dark[:, int(w * 0.1):int(w * 0.9)].mean(axis=1)
    groups = []
    for y in np.where(frac > 0.6)[0]:
        if groups and y - groups[-1][-1] <= 3:
            groups[-1].append(y)
        else:
            groups.append([y])
    ys = [int(np.mean(g)) for g in groups]
    if len(ys) != 36:
        sys.exit(f"横の格子線が {len(ys)} 本見つかった (6 段 x 6 本 = 36 本のはず)。画像を確かめる")
    return [ys[i * 6:(i + 1) * 6] for i in range(6)]


def trace(dark, rows):
    pts, last_elev = [], 30.5        # 図のスタートの標高 (目で読んだ値) から探し始める
    for k, (r, (xl, xr)) in enumerate(zip(rows, MARKS)):
        top, bot = r[0], r[5]
        grid = {g + d for g in r for d in range(-3, 4)}
        prev = top + (60 - last_elev) / 50 * (bot - top)
        km_len = 1.0 if k < 5 else (LAP_M - 5000) / 1000
        for x in range(xl + 8, xr - 6, 3):
            col = dark[top - 10:bot, x]
            runs, start = [], None
            for i, v in enumerate(col):
                y = top - 10 + i
                v = bool(v) and y not in grid
                if v and start is None:
                    start = y
                if not v and start is not None:
                    runs.append((start, y - 1))
                    start = None
            if start is not None:
                runs.append((start, top - 10 + len(col) - 1))
            runs = [(a, b) for a, b in runs if 3 <= b - a <= 14]
            if not runs:
                continue
            a, b = min(runs, key=lambda ab: abs((ab[0] + ab[1]) / 2 - prev))
            yc = (a + b) / 2
            if abs(yc - prev) > 12:
                continue
            prev = yc
            elev = 60 - (yc - top) / (bot - top) * 50
            pts.append((k * 1000 + (x - xl) / (xr - xl) * km_len * 1000, elev))
        last_elev = pts[-1][1]
    return pts


def table(pts):
    d = np.array([p[0] for p in pts])
    e = np.array([p[1] for p in pts])
    out = []
    for m in range(0, LAP_M + 1, 50):
        sel = e[np.abs(d - m) <= 25]
        out.append([m, round(float(sel.mean()), 1) if len(sel) else None])
    known = [(m, v) for m, v in out if v is not None]
    km = np.array([m for m, _ in known])
    kv = np.array([v for _, v in known])
    out = [[m, v if v is not None else round(float(np.interp(m, km, kv)), 1)] for m, v in out]
    # 最後の点 (Finish) は、いちばん近い読み取り値から
    end = e[d >= LAP_M - 40]
    out.append([LAP_M, round(float(end.mean()), 1) if len(end) else out[-1][1]])
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    img = np.array(Image.open(sys.argv[1]).convert("L"))
    dark = img < DARK
    rows = find_gridlines(dark)
    pts = trace(dark, rows)
    tab = table(pts)
    print(f"読み取った点 {len(pts)} 個、表 {len(tab)} 行。スタート {tab[0][1]}m、最後 {tab[-1][1]}m", file=sys.stderr)
    print(json.dumps(tab))


if __name__ == "__main__":
    main()
