/*
 * 鈴鹿サーキットでの Ene-1 GP 走行シミュレーション (物理モデル)
 *   - ブラウザでは <script> で読み込む (window.SuzukaSim)。ログビューアの「鈴鹿シミュ」が使う
 *   - Node.js では require して tools/suzuka_sim/cli.mjs から使う
 *
 * 大会の形式: 充電式単 3 電池 40 本で、フルコース 5.807km を 1 周 x 3 回タイムアタックする
 * (途中で充電しない)。
 *
 * コースは公開情報からの近似: 全長 5807m、高低差 約 40m、いちばん低いのはターン 2 付近、
 * いちばん高いのはスプーンカーブ付近、ホームストレートは下り。区間ごとの標高とカーブの位置は
 * 実測ではないので、結果は「傾向をつかむためのもの」として使う。
 *
 * 電池は 20 本直列 x 2 並列 (公称 24V)、回路には 10A のサーキットブレーカー (温度補正あり) が入る。
 * 車・電池・ドライバーの値はすべて params で変えられる (DEFAULTS を参照)。
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.SuzukaSim = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const G = 9.81;
  const LAP_M = 5807;

  // スタートラインからの距離 [m] と標高 [m] (いちばん低い所を 0 とした近似)
  const ELEVATION = [
    [0, 14], [300, 9], [550, 3], [650, 0], [900, 6], [1250, 15], [1550, 24], [1800, 25],
    [2050, 23], [2300, 24], [2600, 28], [2900, 34], [3200, 39], [3350, 40], [3500, 39],
    [3800, 36], [4200, 32], [4550, 28], [4900, 21], [5000, 20], [5250, 18], [5500, 16], [5807, 14],
  ];

  // 区間の名前 (表示用)
  const SECTIONS = [
    [0, 'ホームストレート'], [550, '1・2コーナー'], [750, 'S字'], [1250, '逆バンク・ダンロップ'],
    [1700, 'デグナー'], [2000, '立体交差の下'], [2200, 'ヘアピン'], [2400, '200R'], [3200, 'スプーン'],
    [3450, 'バックストレート'], [4450, '130R'], [4850, 'シケイン'], [5050, '最終コーナー'], [5350, 'ホームストレート'],
  ];

  // カーブ: 区間と半径 [m] (近似)。曲がれる速さは √(横のグリップ x g x 半径)
  const CORNERS = [
    { from: 600, to: 700, r: 45, name: '1・2コーナー' },
    { from: 1700, to: 1760, r: 50, name: 'デグナー 1 つ目' },
    { from: 1850, to: 1910, r: 30, name: 'デグナー 2 つ目' },
    { from: 2240, to: 2330, r: 18, name: 'ヘアピン' },
    { from: 3300, to: 3450, r: 70, name: 'スプーン' },
    { from: 4500, to: 4600, r: 130, name: '130R' },
    { from: 4940, to: 5030, r: 15, name: 'シケイン' },
    { from: 5150, to: 5300, r: 90, name: '最終コーナー' },
  ];
  const cornerKmh = (c, p) => Math.sqrt(p.latG * G * c.r) * 3.6;

  // ニッケル水素電池 1 本の開放電圧 [V] (放電した割合 → 電圧)。エネループの放電特性の形に合わせた近似
  // 容量を超えて使うと電圧が急に落ちる (1.0 より先)
const OCV = [[0, 1.36], [0.05, 1.30], [0.2, 1.27], [0.5, 1.24], [0.8, 1.20], [0.9, 1.16], [0.95, 1.10], [1, 1.0], [1.04, 0.8], [1.08, 0.5]];

  const DEFAULTS = {
    // 車とドライバー (チームの車: 総重量 70kg、CFRP のモノコックとカウル、IRC EC-R 4.9bar、
    // モーター M1048 24V 仕様、スプロケット 25T → 60T)。CdA・Crr・効率は実測でないので見積もりの中ほど
    massKg: 70,            // 総重量 (ドライバー込み)
    crr: 0.0035,           // 転がり抵抗係数 (エコラン用タイヤ。サーキットの舗装で 0.0025〜0.0045 の見積もり)
    cdA: 0.07,             // 空気抵抗係数 x 前面投影面積 [m^2] (CFRP カウル。0.05〜0.09 の見積もり)
    airDensity: 1.2,       // [kg/m^3]
    efficiency: 0.82,      // 電池からタイヤまでの効率 (モーター・コントローラー・チェーン。0.82〜0.86 の見積もり)
    gearRatio: 60 / 25,    // ドリブン 60T / ドライブ 25T (いまは記録用。モーターの特性が分かれば使う)
    maxForceN: 140,        // 発進・上りでタイヤを押す力の上限 [N]
    // 電池 (充電式単 3 x 40 本 = 20 本直列 x 2 並列、公称 24V)
    series: 20, parallel: 2,
    cellMah: 1900,         // 1 本の容量 [mAh] (エネループ スタンダードの最小容量)
    cellOhm: 0.020,        // 1 本の内部抵抗 [Ω] (20 本直列 x 2 並列で組電池は 0.2Ω)
    cutoffCellV: 0.95,     // これより電圧が下がるとコントローラーが電流を絞る [V/本]
    maxCurrentA: 10,       // コントローラーの電流制限 [A]。ブレーカーの定格に合わせる
    // サーキットブレーカー (熱動式、温度補正あり = 周りの温度で定格が変わらない)。
    // 定格を超えた分 ((I/定格)^2 - 1) を時間で積み、breakerHeatS に達すると切れる。
    // 2 倍の電流で約 20 秒、1.5 倍で約 50 秒、1.2 倍で約 2 分で切れる (一般的な熱動式の形に合わせた近似。
    // 使うブレーカーの動作特性曲線があれば合わせる)。定格以下では少しずつ冷える
    breakerA: 10,
    breakerHeatS: 60,
    breakerCoolS: 30,      // 定格以下で冷えて元に戻るまでの時間の目安
    breakerResetS: 60,     // 切れたら止まって、冷えてから入れ直すまでの時間 [s]
    // ドライバー
    cruiseKmh: 45,         // 平らな所で保つ速さ
    accel: 0.6,            // 目標の速さまでの加速 [m/s^2]
    brakeDecel: 2.5,       // カーブ手前のブレーキ [m/s^2]
    maxCoastKmh: 80,       // 下りで惰性で出してよい速さ
    latG: 0.7,             // カーブで出せる横の加速度 [g] (細いタイヤの軽い車)
    holdOnClimb: true,     // 上りでも速さを保とうとする (false なら平らな所と同じ力まで)
    // 大会
    attacks: 3,            // タイムアタックの回数
    flyingStartKmh: 0,     // スタートラインを通るときの速さ (0 = 止まった状態から。走りながら計測するなら入れる)
    waitS: 120,            // アタックの間の待ち時間 (電流 0) [s]。実際より短くしている
    dt: 0.05,              // 計算と記録の刻み [s] (電力計の記録間隔と同じ)
  };

  function interp(table, x) {
    if (x <= table[0][0]) return table[0][1];
    for (let k = 1; k < table.length; k++) {
      if (x <= table[k][0]) {
        const [x0, y0] = table[k - 1], [x1, y1] = table[k];
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
      }
    }
    return table[table.length - 1][1];
  }
  const elevationAt = (d) => interp(ELEVATION, ((d % LAP_M) + LAP_M) % LAP_M);
  const gradeAt = (d) => (elevationAt(d + 1) - elevationAt(d - 1)) / 2;
  function sectionAt(d) {
    let name = SECTIONS[0][1];
    for (const [from, n] of SECTIONS) if (d >= from) name = n;
    return name;
  }

  // 距離 d で出してよい速さ [m/s] (この先のカーブに間に合うブレーキも考える)
  function speedLimit(d, p) {
    let lim = p.maxCoastKmh / 3.6;
    for (const c of CORNERS) {
      const cap = cornerKmh(c, p) / 3.6;
      if (d >= c.from && d <= c.to) lim = Math.min(lim, cap);
      else if (d < c.from) lim = Math.min(lim, Math.sqrt(cap * cap + 2 * p.brakeDecel * (c.from - d)));
    }
    return lim;
  }

  // 電池: 放電した量 [Ah] → 端子電圧と電流
  function battery(p) {
    const capAh = p.cellMah * p.parallel / 1000;
    const R = p.cellOhm * p.series / p.parallel;
    return {
      capAh, R,
      ocv(usedAh) { return p.series * interp(OCV, Math.min(1, usedAh / capAh)); },
      // 電力 pW を取り出すときの電流 (取り出せなければ取り出せる最大)
      currentFor(pW, usedAh) {
        const E = this.ocv(usedAh);
        const disc = E * E - 4 * R * pW;
        let i = disc >= 0 ? (E - Math.sqrt(disc)) / (2 * R) : E / (2 * R);
        i = Math.min(i, p.maxCurrentA);
        const vCut = p.cutoffCellV * p.series;
        if (E - i * R < vCut) i = Math.max(0, (E - vCut) / R);
        return i;
      },
    };
  }

  // 走行を計算する。戻り値は 50ms ごとの記録と、アタックごとのまとめ
  function simulate(params) {
    const p = Object.assign({}, DEFAULTS, params || {});
    const bat = battery(p);
    const rec = { t: [], d: [], v: [], elev: [], i: [], volt: [], q: [], section: [], heat: [] };
    const attacks = [], events = [];
    let heat = 0, tripped = false, trips = 0;
    const loss = { aero: 0, roll: 0, brake: 0, battery: 0, drive: 0, climb: 0 };
    let t = 0, usedAh = 0, dnf = null;
    const dt = p.dt;
    const record = (dAbs, v, i, volt) => {
      rec.t.push(+t.toFixed(3)); rec.d.push(dAbs); rec.v.push(v); rec.elev.push(elevationAt(dAbs));
      rec.i.push(i); rec.volt.push(volt); rec.q.push(usedAh * 1000); rec.section.push(sectionAt(dAbs % LAP_M));
      rec.heat.push(heat / p.breakerHeatS);
    };
    // ブレーカーの熱 (0〜1 で記録。1 で切れる)
    const breakerStep = (i) => {
      const x = (i / p.breakerA) ** 2 - 1;
      heat = x > 0 ? heat + x * dt : Math.max(0, heat - p.breakerHeatS / p.breakerCoolS * dt * Math.min(1, -x));
      if (heat >= p.breakerHeatS) { tripped = true; heat = p.breakerHeatS; }
    };
    const idle = (seconds, dAbs) => {
      for (let s = 0; s < seconds; s += dt) {
        t += dt;
        breakerStep(0);
        record(dAbs, 0, 0, bat.ocv(usedAh));
      }
    };

    for (let a = 0; a < p.attacks && !dnf; a++) {
      let d = 0, v = p.flyingStartKmh / 3.6, stuck = 0, peakA = 0, minV = Infinity;
      const t0 = t, q0 = usedAh;
      while (d < LAP_M) {
        const grade = gradeAt(d);
        const fGrav = p.massKg * G * grade;
        const fRoll = p.crr * p.massKg * G;
        const fAero = 0.5 * p.airDensity * p.cdA * v * v;
        const fRes = fGrav + fRoll + fAero;
        const lim = speedLimit(d, p);
        let fMotor = 0, brake = 0;
        if (tripped) {
          // ブレーカーが切れた: 惰性で止まり、冷えるのを待って入れ直す
          if (v < 0.05) {
            events.push({ t, attack: a + 1, d, section: sectionAt(d), kind: 'reset' });
            for (let s = 0; s < p.breakerResetS; s += dt) { t += dt; heat = Math.max(0, heat - p.breakerHeatS / p.breakerCoolS * dt); record(a * LAP_M + d, 0, 0, bat.ocv(usedAh)); }
            tripped = false; heat = 0;
            continue;
          }
          brake = 0.5 * p.massKg;                    // 安全のため軽くブレーキ
        } else if (v > lim + 0.05) {
          brake = p.brakeDecel * p.massKg;          // カーブの手前でブレーキ
        } else {
          const target = Math.min(p.cruiseKmh / 3.6, lim);
          const climbing = grade > 0.005;
          let want;
          if (v < target - 0.05) want = p.massKg * Math.min(p.accel, (target - v) / 1.0) + fRes;
          else if (v > target + 0.05) want = 0;      // 下りでついた速さは、惰性で目標まで落とす
          else want = fRes;                          // 速さを保つ
          if (!p.holdOnClimb && climbing) want = Math.min(want, fRoll + fAero + p.massKg * Math.min(p.accel, Math.max(0, target - v)));
          fMotor = Math.max(0, Math.min(want, p.maxForceN));  // 下りでは力を出さず惰性
        }
        // 電池から取り出す電力と電流 (電流制限・電圧低下で絞られる)
        let i = 0;
        const vv = Math.max(v, 0.3);
        if (fMotor > 0) {
          const pElec = fMotor * vv / p.efficiency;
          i = bat.currentFor(pElec, usedAh);
          const volt = bat.ocv(usedAh) - i * bat.R;
          fMotor = Math.min(fMotor, volt * i * p.efficiency / vv);
        }
        const volt = bat.ocv(usedAh) - i * bat.R;
        const wasTripped = tripped;
        breakerStep(i);
        if (tripped && !wasTripped) { trips++; events.push({ t, attack: a + 1, d, section: sectionAt(d), kind: 'trip', current: i }); }
        const acc = (fMotor - fRes - brake) / p.massKg;
        const vNew = Math.max(0, v + acc * dt);
        const ds = (v + vNew) / 2 * dt;
        // エネルギーの行き先 [J]
        loss.aero += fAero * ds; loss.roll += fRoll * ds; loss.climb += fGrav * ds;
        loss.brake += brake * ds;
        loss.battery += i * i * bat.R * dt;
        loss.drive += volt * i * (1 - p.efficiency) * dt;
        v = vNew; d += ds; t += dt;
        usedAh += i * dt / 3600;
        peakA = Math.max(peakA, i); minV = Math.min(minV, volt);
        record(a * LAP_M + d, v, i, volt);
        stuck = v < 0.3 && !tripped ? stuck + dt : 0;
        if (stuck > 20) { dnf = { attack: a + 1, d, section: sectionAt(d), reason: '電池の電圧が下がり、上りで止まった' }; break; }
      }
      attacks.push({
        attack: a + 1, finished: !dnf, startT: t0, lapS: t - t0, avgKmh: (Math.min(d, LAP_M) / (t - t0)) * 3.6,
        mAh: (usedAh - q0) * 1000, peakA, minV,
      });
      if (dnf) break;
      // ゴール後: ブレーキで止まって待つ
      while (v > 0) { v = Math.max(0, v - 1.5 * dt); d += v * dt; t += dt; record(a * LAP_M + Math.min(d, LAP_M + 200), v, 0, bat.ocv(usedAh)); }
      if (a < p.attacks - 1) idle(p.waitS, (a + 1) * LAP_M);
    }
    const toWh = (j) => j / 3600;
    return {
      params: p, rec, attacks, dnf, events, trips,
      usedMah: usedAh * 1000, capacityMah: bat.capAh * 1000,
      totalLapS: attacks.reduce((s, x) => s + (x.finished ? x.lapS : 0), 0),
      energyWh: {
        aero: toWh(loss.aero), roll: toWh(loss.roll), brake: toWh(loss.brake),
        battery: toWh(loss.battery), drive: toWh(loss.drive), climbNet: toWh(loss.climb),
      },
    };
  }

  // 3 回のアタックで、容量の share (0〜1) をちょうど使い切る平らな所の速さ [km/h] を探す。
  // 探す間は刻みを粗くして速くし (ブラウザでも 1 秒かからない)、最後に細かい刻みで確かめて詰める
  function findCruise(params, share) {
    const fits = (kmh, dt) => {
      const r = simulate(Object.assign({}, params, { cruiseKmh: kmh, dt }));
      return !r.dnf && r.usedMah <= share * r.capacityMah;
    };
    let lo = 15, hi = 80;
    while (hi - lo > 0.1) { const mid = (lo + hi) / 2; if (fits(mid, 0.2)) lo = mid; else hi = mid; }
    const dt = (params && params.dt) || DEFAULTS.dt;
    while (lo > 15 && !fits(lo, dt)) lo -= 0.1;
    return Math.round(lo * 10) / 10;
  }

  // 1 周の目標タイム lapS [s] で 3 回走れるかを調べる。
  // 平らな所の速さを上げていき、いちばん遅い周が目標に届く速さを探す。届かなければ、届く最速の値を返す
  function forLapTime(params, lapS) {
    const run = (kmh, dt) => simulate(Object.assign({}, params, { cruiseKmh: kmh, dt }));
    const slowest = (r) => (r.dnf ? Infinity : Math.max(...r.attacks.map((a) => a.lapS)));
    let lo = 15, hi = 120;
    while (hi - lo > 0.1) { const mid = (lo + hi) / 2; if (slowest(run(mid, 0.2)) <= lapS) hi = mid; else lo = mid; }
    const r = run(hi, (params && params.dt) || DEFAULTS.dt);
    const lapOk = slowest(r) <= lapS + 0.5;
    const shareUsed = r.usedMah / r.capacityMah;
    return { cruiseKmh: hi, result: r, lapOk, shareUsed, bestLapS: slowest(r), trips: r.trips };
  }

  // 目標タイムを 95% の電池で、ブレーカーを落とさずに出すには、空気抵抗 (CdA) がいくつ以下ならよいか
  function maxCdAFor(params, lapS, share) {
    share = share || 0.95;
    const ok = (cdA) => {
      const f = forLapTime(Object.assign({}, params, { cdA, dt: 0.2 }), lapS);
      return f.lapOk && f.shareUsed <= share && f.trips === 0;
    };
    let lo = 0.02, hi = 0.3;
    if (!ok(lo)) return null;
    while (hi - lo > 0.002) { const mid = (lo + hi) / 2; if (ok(mid)) lo = mid; else hi = mid; }
    return lo;
  }

  // ログビューア用: 走り方を 4 つ比べる (ちょうど使い切る・控えめ・攻めすぎ・電流制限 14A)
  function strategies(params, share) {
    share = share || 0.95;
    const base = Object.assign({}, params || {});
    const p = Object.assign({}, DEFAULTS, base);
    const c = findCruise(base, share);
    const c14 = findCruise(Object.assign({}, base, { maxCurrentA: 14 }), share);
    return [
      { key: 'just', name: 'ちょうど使い切る', note: `電流制限 ${p.maxCurrentA}A、容量の ${Math.round(share * 100)}% を使う速さ`, params: { cruiseKmh: c } },
      { key: 'eco', name: '控えめ', note: '平らな所を 5km/h 遅く', params: { cruiseKmh: c - 5 } },
      { key: 'over', name: '攻めすぎ', note: '平らな所を 5km/h 速く', params: { cruiseKmh: c + 5 } },
      { key: 'limit14', name: '電流制限 14A', note: 'ブレーカー 10A より上に設定', params: { cruiseKmh: c14, maxCurrentA: 14 } },
    ].map((st) => Object.assign(st, { result: simulate(Object.assign({}, base, st.params)) }));
  }

  // ログビューアや電力計と同じ形の CSV (電力計の記録と同じ列)
  function toCsv(result, from, to) {
    const r = result.rec;
    const out = ['time_s,voltage_V,current_A,power_W,charge_mAh'];
    for (let k = from || 0; k < (to || r.t.length); k++) {
      const i = r.i[k], v = r.volt[k];
      out.push(`${r.t[k].toFixed(3)},${v.toFixed(3)},${i.toFixed(3)},${(v * i).toFixed(2)},${r.q[k].toFixed(2)}`);
    }
    return out.join('\n') + '\n';
  }

  return { LAP_M, ELEVATION, SECTIONS, CORNERS, DEFAULTS, elevationAt, gradeAt, sectionAt, speedLimit, simulate, findCruise, forLapTime, maxCdAFor, strategies, toCsv, cornerKmh };
});
