/*
 * Ene1PowerMeter ログビューアの計算部分 (画面に依存しない)
 *   - ブラウザでは index.html から <script> で読み込む (window.LogCore)
 *   - Node.js では require / import して test/log_viewer/test.mjs から確かめる
 *
 * CSV の形 (Ene1PowerMeter.ino の printCsvRow):
 *   time_s,voltage_V,current_A,power_W,charge_mAh,speed_kmh,distance_m
 *   12.350,24.012,10.034,240.93,34.41,32.51,1234.5
 * 速度センサーに対応する前のファームウェアの CSV (最後の 2 列がない) も読める。
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.LogCore = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const HEADER = ['time_s', 'voltage_V', 'current_A', 'power_W', 'charge_mAh'];
  // 速度センサーの列 (HEADER の後ろに続く)。これがない CSV は、速さと距離を持たない
  const SPEED_HEADER = ['speed_kmh', 'distance_m'];
  // ファームウェアの MAX_CURRENT_A (20A) に張り付いた行を OVER とみなすしきい値 [A]
  const DEFAULT_OVER_A = 19.99;
  // 行の間隔がこれより長ければ「記録の抜け」とみなす [s] (通常は 0.05 秒)
  const DEFAULT_GAP_S = 0.5;

  // ---------------------------------------------------------------------------
  //  CSV の読み込み

  // 1 ファイル分の CSV を数値の列に変換する。
  // 電源断で途中までしか書かれていない行や、数値でない行は数えて飛ばす。
  // 列の数 (5 = 速度センサーなし、7 = あり) は見出し行で決める (見出しがなければ最初の行で)
  function parseCsv(text, name) {
    const lines = String(text).split(/\r?\n/);
    const cols = { t: [], v: [], i: [], p: [], q: [], s: [], d: [] };
    let skipped = 0;
    let headerOk = false;
    let ncol = 0;
    for (let k = 0; k < lines.length; k++) {
      const line = lines[k].trim();
      if (!line) continue;
      if (line[0] === '#') continue;
      if (/^time_s\s*,/.test(line)) {
        const h = line.split(',').map((s) => s.trim());
        headerOk = HEADER.every((c, n) => h[n] === c) &&
          (h.length === HEADER.length || (h.length === HEADER.length + SPEED_HEADER.length && SPEED_HEADER.every((c, n) => h[HEADER.length + n] === c)));
        if (headerOk) ncol = h.length;
        continue;
      }
      const f = line.split(',');
      if (!ncol && (f.length === 5 || f.length === 7)) ncol = f.length;
      if (f.length !== ncol) { skipped++; continue; }
      const nums = f.map(Number);
      if (!f.every((s) => /^-?\d+(\.\d+)?$/.test(s.trim()))) { skipped++; continue; }
      const prevT = cols.t.length ? cols.t[cols.t.length - 1] : -Infinity;
      if (nums[0] < prevT) { skipped++; continue; }  // 時刻が戻る行は壊れているとみなす
      cols.t.push(nums[0]); cols.v.push(nums[1]); cols.i.push(nums[2]);
      cols.p.push(nums[3]); cols.q.push(nums[4]);
      if (ncol === 7) { cols.s.push(nums[5]); cols.d.push(nums[6]); }
    }
    return {
      name: name || '',
      headerOk,
      skipped,
      t: Float64Array.from(cols.t),
      v: Float64Array.from(cols.v),
      i: Float64Array.from(cols.i),
      p: Float64Array.from(cols.p),
      q: Float64Array.from(cols.q),
      // 速さ [km/h] と距離 [m]。速度センサーの列がない CSV では null
      s: ncol === 7 ? Float64Array.from(cols.s) : null,
      d: ncol === 7 ? Float64Array.from(cols.d) : null,
    };
  }

  // LOG012.CSV → 12。番号がなければ大きな値 (名前順で後ろ)
  function logNumber(name) {
    const m = /LOG(\d+)\.CSV$/i.exec(name || '');
    return m ? parseInt(m[1], 10) : Number.MAX_SAFE_INTEGER;
  }

  function compareFiles(a, b) {
    const d = logNumber(a.name) - logNumber(b.name);
    return d !== 0 ? d : String(a.name).localeCompare(String(b.name));
  }

  // ---------------------------------------------------------------------------
  //  走行 (run) へのまとめ
  //
  // ファームウェアは次のときに新しいファイルを作る:
  //   - Nano だけがリセットされた (ウォッチドッグ等) : 時刻と mAh は続きから → 同じ走行につなぐ
  //   - ボタンで積算リセット / 電源の入れ直し       : 時刻が 0 から        → 新しい走行
  // そこで、ファイル番号の順に並べ、先頭の時刻が前のファイルの最後以上なら同じ走行とみなす。
  function buildRuns(files) {
    const sorted = files.filter((f) => f.t.length > 0).sort(compareFiles);
    const groups = [];
    for (const f of sorted) {
      const g = groups[groups.length - 1];
      if (g) {
        const last = g[g.length - 1];
        if (f.t[0] >= last.t[last.t.length - 1]) { g.push(f); continue; }
      }
      groups.push([f]);
    }
    return groups.map((g, n) => joinFiles(g, n));
  }

  function joinFiles(group, index) {
    const len = group.reduce((s, f) => s + f.t.length, 0);
    // 速さと距離は、つなぐファイルのすべてにあるときだけ持つ
    const speed = group.every((f) => f.s && f.d);
    const run = {
      index,
      name: group.map((f) => f.name).join(' + '),
      files: group.map((f) => f.name),
      t: new Float64Array(len), v: new Float64Array(len), i: new Float64Array(len),
      p: new Float64Array(len), q: new Float64Array(len),
      s: speed ? new Float64Array(len) : null, d: speed ? new Float64Array(len) : null,
      // ファイルの継ぎ目 (Nano のリセット): 新しいファイルの先頭の行番号と時刻
      joins: [],
      skipped: group.reduce((s, f) => s + f.skipped, 0),
    };
    let o = 0;
    group.forEach((f, n) => {
      if (n > 0) run.joins.push({ index: o, t: f.t[0], file: f.name });
      run.t.set(f.t, o); run.v.set(f.v, o); run.i.set(f.i, o);
      run.p.set(f.p, o); run.q.set(f.q, o);
      if (speed) { run.s.set(f.s, o); run.d.set(f.d, o); }
      o += f.t.length;
    });
    return run;
  }

  // ---------------------------------------------------------------------------
  //  集計

  // t (昇順) の中で値 x 以上の最初の位置
  function lowerBound(t, x) {
    let lo = 0, hi = t.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (t[m] < x) lo = m + 1; else hi = m; }
    return lo;
  }

  // 行番号 i0〜i1 (両端を含む) の集計
  //   mAh は INA228 の CHARGE (記録の抜けの間も INA228 は積算を続けている) の差。
  //   Wh と電流の 2 乗平均は、行の間を台形でつないで積分する。抜けの区間は積分しない。
  //   距離は記録の距離の列の差 (抜けの間もファームウェアは数えている)。電費 (Wh/km) は
  //   Wh と同じく抜けの区間を除いた距離で割る
  function stats(run, i0, i1, opt) {
    const o = Object.assign({ overA: DEFAULT_OVER_A, gapS: DEFAULT_GAP_S }, opt || {});
    if (i0 === undefined) i0 = 0;
    if (i1 === undefined) i1 = run.t.length - 1;
    i0 = Math.max(0, i0); i1 = Math.min(run.t.length - 1, i1);
    const r = {
      rows: Math.max(0, i1 - i0 + 1),
      t0: run.t[i0], t1: run.t[i1], duration: 0,
      mAh: 0, Wh: 0, avgA: 0, avgW: 0, rmsA: 0, smooth: NaN,
      peakA: -Infinity, peakAt: NaN, minA: Infinity, minV: Infinity, minVAt: NaN, maxV: -Infinity,
      overRows: 0, overS: 0, gaps: [], gapS: 0, joins: 0,
      // 速度センサーの列があるときだけ数値になる
      hasSpeed: !!run.s, distM: NaN, avgKmh: NaN, maxKmh: NaN, maxKmhAt: NaN, whPerKm: NaN, mAhPerKm: NaN,
    };
    if (r.rows === 0) return r;
    r.duration = run.t[i1] - run.t[i0];
    r.mAh = run.q[i1] - run.q[i0];
    let e = 0, ii = 0, tt = 0, dd = 0;
    if (run.s) r.maxKmh = -Infinity;
    for (let k = i0; k <= i1; k++) {
      const a = run.i[k], v = run.v[k];
      if (run.s && run.s[k] > r.maxKmh) { r.maxKmh = run.s[k]; r.maxKmhAt = run.t[k]; }
      if (a > r.peakA) { r.peakA = a; r.peakAt = run.t[k]; }
      if (a < r.minA) r.minA = a;
      if (v < r.minV) { r.minV = v; r.minVAt = run.t[k]; }
      if (v > r.maxV) r.maxV = v;
      if (a >= o.overA) r.overRows++;
      if (k === i1) break;
      const dt = run.t[k + 1] - run.t[k];
      if (dt > o.gapS) {
        r.gaps.push({ t0: run.t[k], t1: run.t[k + 1] });
        r.gapS += dt;
        continue;
      }
      const b = run.i[k + 1];
      if (run.d) dd += run.d[k + 1] - run.d[k];
      e += (run.p[k] + run.p[k + 1]) / 2 * dt;
      ii += (a * a + b * b) / 2 * dt;
      tt += dt;
      if (a >= o.overA) r.overS += dt;
    }
    for (const j of run.joins) if (j.index > i0 && j.index <= i1) r.joins++;
    r.Wh = e / 3600;
    // 平均電流は INA228 の積算から出す (抜けの間も正しい)
    r.avgA = r.duration > 0 ? r.mAh * 3.6 / r.duration : 0;
    r.avgW = tt > 0 ? e / tt : 0;
    r.rmsA = tt > 0 ? Math.sqrt(ii / tt) : 0;
    // 電流の 2 乗平均の平方根 / 平均。1.00 = 一定の電流。電池の中の損失 (I²R) は平均の電流で
    // 走ったときの smooth² 倍になる
    r.smooth = r.avgA > 0.05 ? r.rmsA / r.avgA : NaN;
    if (run.d) {
      r.distM = run.d[i1] - run.d[i0];
      r.avgKmh = r.duration > 0 ? r.distM / r.duration * 3.6 : 0;
      // 10m 未満 (ほとんど止まっている) では割り算の値が大きくぶれるので出さない
      r.mAhPerKm = r.distM >= 10 ? r.mAh / (r.distM / 1000) : NaN;
      r.whPerKm = dd >= 10 ? r.Wh / (dd / 1000) : NaN;
    }
    return r;
  }

  // 時間に重みを付けた電流の分布 (binA ごと)。抜けの区間は数えない
  function histogram(run, i0, i1, binA, opt) {
    const o = Object.assign({ gapS: DEFAULT_GAP_S }, opt || {});
    binA = binA || 1;
    if (i0 === undefined) i0 = 0;
    if (i1 === undefined) i1 = run.t.length - 1;
    const map = new Map();
    for (let k = i0; k < i1; k++) {
      const dt = run.t[k + 1] - run.t[k];
      if (dt > o.gapS) continue;
      const b = Math.floor(run.i[k] / binA);
      map.set(b, (map.get(b) || 0) + dt);
    }
    if (map.size === 0) return [];
    const keys = [...map.keys()];
    const lo = Math.min(...keys), hi = Math.max(...keys);
    const out = [];
    for (let b = lo; b <= hi; b++) out.push({ a0: b * binA, a1: (b + 1) * binA, s: map.get(b) || 0 });
    return out;
  }

  // ---------------------------------------------------------------------------
  //  ペース配分
  //   budgetMah を raceS 秒で均等に使い切る線と比べる。plus = 使いすぎ
  function pace(run, budgetMah, raceS, atIndex) {
    if (!(budgetMah > 0) || !(raceS > 0) || run.t.length === 0) return null;
    const k = atIndex === undefined ? run.t.length - 1 : atIndex;
    const el = run.t[k] - run.t[0];
    const used = run.q[k] - run.q[0];
    const ideal = budgetMah * Math.min(el, raceS) / raceS;
    const remainS = raceS - el;
    const remainMah = budgetMah - used;
    return {
      elapsed: el, used, ideal, diff: used - ideal,
      remainS, remainMah,
      // 残りを均等に使うための平均電流 [A]
      needA: remainS > 0 ? remainMah * 3.6 / remainS : NaN,
      finished: remainS <= 0,
    };
  }

  // ---------------------------------------------------------------------------
  //  描画用の間引き
  //   画面の横幅 (buckets 個) ごとに最小・最大を残す。50ms 刻みで 2 時間 (14 万行) でも、
  //   短い電流のピークを落とさずに描ける。抜けの位置では線を切るために gap を付ける
  function decimate(t, y, i0, i1, buckets, gapS) {
    gapS = gapS || DEFAULT_GAP_S;
    const out = [];
    if (i1 < i0) return out;
    const n = i1 - i0 + 1;
    if (n <= buckets * 2) {
      for (let k = i0; k <= i1; k++) {
        out.push({ t: t[k], y: y[k], gap: k > i0 && t[k] - t[k - 1] > gapS });
      }
      return out;
    }
    const tA = t[i0], tB = t[i1], w = (tB - tA) / buckets || 1;
    let k = i0;
    for (let b = 0; b < buckets && k <= i1; b++) {
      const edge = tA + (b + 1) * w;
      let mn = Infinity, mx = -Infinity, kMn = k, kMx = k;
      const start = k;
      let gap = k > i0 && t[k] - t[k - 1] > gapS;
      while (k <= i1 && (t[k] < edge || b === buckets - 1)) {
        if (k > start && t[k] - t[k - 1] > gapS) gap = true;
        if (y[k] < mn) { mn = y[k]; kMn = k; }
        if (y[k] > mx) { mx = y[k]; kMx = k; }
        k++;
      }
      if (k === start) continue;
      const first = Math.min(kMn, kMx), second = Math.max(kMn, kMx);
      out.push({ t: t[first], y: y[first], gap });
      if (second !== first) out.push({ t: t[second], y: y[second], gap: false });
    }
    return out;
  }

  // 目盛り: 範囲 [lo, hi] に 4〜8 本程度のきりのよい値
  function niceTicks(lo, hi, target) {
    target = target || 5;
    if (!(hi > lo)) { hi = lo + 1; }
    const raw = (hi - lo) / target;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => (hi - lo) / s <= target + 1) || 10 * mag;
    const out = [];
    for (let x = Math.ceil(lo / step) * step; x <= hi + step * 1e-9; x += step) out.push(+x.toFixed(10));
    return { ticks: out, step };
  }

  // 秒 → "mm:ss" (1 時間以上なら "h:mm:ss")
  function fmtTime(s) {
    if (!isFinite(s)) return '--:--';
    const neg = s < 0; s = Math.abs(s);
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), sec = Math.floor(s % 60);
    const p2 = (x) => String(x).padStart(2, '0');
    return (neg ? '-' : '') + (h > 0 ? `${h}:${p2(m)}:${p2(sec)}` : `${p2(m)}:${p2(sec)}`);
  }

  return {
    HEADER, SPEED_HEADER, DEFAULT_OVER_A, DEFAULT_GAP_S,
    parseCsv, buildRuns, logNumber, lowerBound, stats, histogram, pace, decimate, niceTicks, fmtTime,
  };
});
