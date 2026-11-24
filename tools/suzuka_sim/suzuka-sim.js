/*
 * 鈴鹿サーキットでの Ene-1 走行シミュレーション (物理モデル)
 *   - ブラウザでは <script> で読み込む (window.SuzukaSim)。ログビューアの「鈴鹿シミュ」が使う
 *   - Node.js では require して tools/suzuka_sim/cli.mjs から使う
 *
 * 大会の形式 (2026 Ene-1 SUZUKA Challenge KV-40 特別規則): 充電式単 3 電池 40 本で、フルコース 5.807km を
 * 1st・2nd・3rd アタックの順に各 1 周走り、3 回の合計タイムで順位を決める。止まった状態からスタートする。
 *
 * コース: 特別規則 付則1 の ELEVATION 図 (標高と Curve Radius) を読み取った値。
 *   標高は図の曲線を 50m ごとに読み取り (±1m 程度)、1 周で元の高さに戻るように少しだけ補正している。
 *   カーブの位置と半径は図の破線の範囲。曲がれる速さは √(横のグリップ x g x 半径) とする。
 *
 * 車: チームの車 (総重量 70kg ドライバー込み、CFRP のモノコックとカウル、IRC EC-R 20x1.75 4.9bar、
 *   モーター M1048 24V 仕様 (自動進角・可変界磁付き)、スプロケット 25T → 60T)。モーターはメーカーの
 *   特性図 (24V、電流ごとの回転数と効率) に合わせたモデルで、回転が上がって電圧が足りなくなると界磁を
 *   弱めて回す (弱め方の上限と損失は見積もり)。空気抵抗と転がり抵抗は実測がないので見積もり (DEFAULTS)。
 * 電池: 単 3 を 20 本直列 x 2 並列 (公称 24V)、1 本 20mΩ。サーキットブレーカー 10A (熱動式)。
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.SuzukaSim = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const G = 9.81;
  const LAP_M = 5807;

  // ---------------------------------------------------------------------------
  //  コース (特別規則 付則1 ELEVATION 図から)
  // スタートからの距離 [m] と標高 [m]。最後の点は図の読み取り値 (補正前)
  const ELEVATION_RAW = [
    [0, 29.2], [50, 28.9], [100, 27.7], [150, 26.3], [200, 24.9], [250, 23.6], [300, 22.2], [350, 21.0],
    [400, 18.9], [450, 17.9], [500, 17.8], [550, 18.8], [600, 20.3], [650, 20.9], [700, 21.7], [750, 22.5],
    [800, 23.4], [850, 24.6], [900, 26.4], [950, 28.4], [1000, 31.3], [1050, 33.0], [1100, 35.3], [1150, 37.6],
    [1200, 39.2], [1250, 39.4], [1300, 38.9], [1350, 37.8], [1400, 38.1], [1450, 41.0], [1500, 44.3], [1550, 47.7],
    [1600, 50.7], [1650, 51.8], [1700, 51.8], [1750, 51.1], [1800, 50.8], [1850, 50.0], [1900, 49.2], [1950, 48.7],
    [2000, 48.0], [2050, 46.8], [2100, 45.4], [2150, 44.2], [2200, 43.8], [2250, 43.8], [2300, 43.8], [2350, 44.2],
    [2400, 45.6], [2450, 47.9], [2500, 50.6], [2550, 52.7], [2600, 54.8], [2650, 55.7], [2700, 56.1], [2750, 56.5],
    [2800, 56.8], [2850, 57.2], [2900, 57.5], [2950, 57.6], [3000, 56.9], [3050, 55.2], [3100, 53.1], [3150, 52.4],
    [3200, 53.3], [3250, 54.9], [3300, 56.1], [3350, 57.0], [3400, 57.6], [3450, 58.0], [3500, 58.0], [3550, 58.0],
    [3600, 58.0], [3650, 58.0], [3700, 57.8], [3750, 56.2], [3800, 53.1], [3850, 48.9], [3900, 46.1], [3950, 44.6],
    [4000, 44.5], [4050, 45.3], [4100, 46.5], [4150, 47.7], [4200, 48.8], [4250, 49.2], [4300, 49.4], [4350, 49.5],
    [4400, 49.7], [4450, 49.8], [4500, 50.0], [4550, 50.1], [4600, 50.2], [4650, 50.4], [4700, 50.6], [4750, 50.7],
    [4800, 50.9], [4850, 51.6], [4900, 52.4], [4950, 53.2], [5000, 54.0], [5050, 54.8], [5100, 55.2], [5150, 54.2],
    [5200, 53.1], [5250, 51.2], [5300, 47.5], [5350, 44.4], [5400, 42.0], [5450, 40.8], [5500, 38.8], [5550, 37.4],
    [5600, 36.0], [5650, 34.6], [5700, 33.2], [5750, 31.7], [5800, 30.6], [5807, 30.5],
  ];
  // 読み取りのずれ (1 周で 1.3m) を距離に比例して戻し、1 周で同じ高さにする
  const drift = ELEVATION_RAW[ELEVATION_RAW.length - 1][1] - ELEVATION_RAW[0][1];
  const ELEVATION = ELEVATION_RAW.map(([m, z]) => [m, +(z - drift * m / LAP_M).toFixed(2)]);

  // 区間の名前 (表示用)
  const SECTIONS = [
    [0, 'ホームストレート'], [350, '1コーナー'], [480, '2コーナー'], [700, 'S字'], [1400, '逆バンク'],
    [1550, 'ダンロップ'], [1950, 'デグナー'], [2220, '立体交差の下'], [2420, 'ヘアピン'], [2700, '200R'],
    [3130, 'スプーン'], [3840, 'バックストレート'], [4640, '130R'], [4950, 'シケイン'], [5180, '最終コーナー'],
    [5460, 'ホームストレート'],
  ];

  // 公式セクター (2026 KV-40 特別規則のコース図: 区間 1 = 1,658m、区間 2 = 936m、区間 3 = 2,130m、区間 4 = 1,084m。
  // 4 つの合計は 5,808m なので、区間 4 の終点を 1 周 5,807m に合わせる)
  const SECTORS = [
    { id: 1, name: '区間1', from: 0,    to: 1658,  lengthM: 1658 },
    { id: 2, name: '区間2', from: 1658, to: 2594,  lengthM: 936 },
    { id: 3, name: '区間3', from: 2594, to: 4724,  lengthM: 2130 },
    { id: 4, name: '区間4', from: 4724, to: LAP_M, lengthM: 1084 },
  ];

  // カーブ: 図の Curve Radius の範囲 [m] と半径 [m]
  const CORNERS = [
    { from: 353, to: 454, r: 100, name: '1コーナー' },
    { from: 487, to: 621, r: 60, name: '2コーナー' },
    { from: 783, to: 852, r: 70, name: 'S字' },
    { from: 880, to: 971, r: 70, name: 'S字' },
    { from: 1044, to: 1139, r: 75, name: 'S字' },
    { from: 1190, to: 1295, r: 85, name: 'S字' },
    { from: 1295, to: 1341, r: 70, name: 'S字' },
    { from: 1410, to: 1521, r: 77.5, name: '逆バンク' },
    { from: 1556, to: 1661, r: 210, name: 'ダンロップ' },
    { from: 1678, to: 1728, r: 130, name: 'ダンロップ' },
    { from: 1748, to: 1842, r: 200, name: 'ダンロップ' },
    { from: 1995, to: 2012, r: 15, name: 'デグナー 1 つ目' },
    { from: 2147, to: 2184, r: 25, name: 'デグナー 2 つ目' },
    { from: 2453, to: 2519, r: 110, name: 'ヘアピン手前' },
    { from: 2601, to: 2659, r: 20, name: 'ヘアピン' },
    { from: 2795, to: 2869, r: 200, name: '200R' },
    { from: 2871, to: 2933, r: 120, name: '200R' },
    { from: 2965, to: 3109, r: 300, name: '200R' },
    { from: 3150, to: 3345, r: 250, name: 'スプーン手前' },
    { from: 3483, to: 3547, r: 60, name: 'スプーン' },
    { from: 3547, to: 3634, r: 200, name: 'スプーン' },
    { from: 3634, to: 3725, r: 60, name: 'スプーン' },
    { from: 3725, to: 3757, r: 100, name: 'スプーン' },
    { from: 3757, to: 3814, r: 200, name: 'スプーン' },
    { from: 3862, to: 4186, r: 1000, name: 'バックストレート' },
    { from: 4653, to: 4704, r: 85, name: '130R' },
    { from: 4713, to: 4876, r: 340, name: '130R' },
    { from: 5033, to: 5095, r: 250, name: 'シケイン手前' },
    { from: 5095, to: 5107, r: 9.5, name: 'シケイン' },
    { from: 5140, to: 5167, r: 20, name: 'シケイン' },
    { from: 5198, to: 5313, r: 115, name: '最終コーナー' },
    { from: 5317, to: 5452, r: 250, name: '最終コーナー' },
  ];
  const cornerKmh = (c, p) => Math.sqrt(p.latG * G * c.r) * 3.6;

  // ---------------------------------------------------------------------------
  //  モーター M1048 (24V 仕様) — メーカーの特性図から
  //   回転数: 無負荷 1780rpm、電流 1A ごとに 9.18rpm 下がる (24V)
  //   効率: 1A で約 55%、2A で約 77%、4A で約 87%、13A 付近で最大 約 91.5%、25A で約 89.5%
  //  これを、逆起電力 + 内部の電圧降下 (回転数) と、銅損 (電流の 2 乗) + 回転に比例する損失 (効率) に分けて表す
  const MOTOR = (() => {
    const vRef = 24, nlRpm = 1780, droopRpmPerA = 9.18;
    const wRef = nlRpm * 2 * Math.PI / 60;          // 無負荷の角速度 [rad/s]
    const ke = vRef / wRef;                          // 逆起電力定数 = トルク定数 [V·s/rad = N·m/A]
    const rDroop = droopRpmPerA * 2 * Math.PI / 60 * ke;  // 回転数の下がり方から求めた内部の電圧降下 [Ω]
    const rCopper = 0.08;                            // 銅損 [Ω] (効率の曲線に合わせた値)
    const pNoLoad = 10;                              // 無負荷の損失 [W] (24V・無負荷の回転数で)
    return { name: 'M1048 (24V)', vRef, nlRpm, wRef, ke, rDroop, rCopper, pNoLoad, iFric: pNoLoad / (wRef * ke) };
  })();
  // 電流 i [A]・角速度 w [rad/s] のとき: 軸の出力 [W] と、電気の入力 [W]
  const motorOut = (i, w) => Math.max(0, MOTOR.ke * (i - MOTOR.iFric) * w);
  const motorIn = (i, w) => MOTOR.ke * w * i + MOTOR.rCopper * i * i;
  // 特性図との照合用: 24V で全開 (デューティ 100%) のときの回転数 [rpm] と効率
  function motorAt24V(i) {
    const w = (MOTOR.vRef - MOTOR.rDroop * i) / MOTOR.ke;
    return { rpm: w * 60 / (2 * Math.PI), eff: motorOut(i, w) / (MOTOR.vRef * i) };
  }

  // ニッケル水素電池 1 本の開放電圧 [V] (放電した割合 → 電圧)。エネループの放電特性の形に合わせた近似。
  // 容量を超えて使うと電圧が急に落ちる (1.0 より先)
  const OCV = [[0, 1.36], [0.05, 1.30], [0.2, 1.27], [0.5, 1.24], [0.8, 1.20], [0.9, 1.16], [0.95, 1.10], [1, 1.0], [1.04, 0.8], [1.08, 0.5]];

  const DEFAULTS = {
    // 車とドライバー
    massKg: 70,            // 総重量 (ドライバー込み)
    crr: 0.003,            // 転がり抵抗係数 (エコラン用 IRC EC-R。上位チーム並みの見積もり)
    cdA: 0.05,             // 空気抵抗係数 x 前面投影面積 [m^2] (CFRP カウル。上位チーム並みの見積もり)
    airDensity: 1.2,       // [kg/m^3]
    latG: 0.7,             // カーブで出せる横の加速度 [g]
    tireDiameterM: 0.50,   // IRC EC-R 20x1.75 (ETRTO 47-406) の外径
    gearRatio: 60 / 25,    // ドリブン 60T / ドライブ 25T
    chainEff: 0.97,        // チェーンの効率
    ctrlEff: 0.97,         // モーターコントローラーの効率
    motorMaxA: 30,         // コントローラーがモーターに流す電流の上限 [A] (発進の力を決める)
    maxDuty: 0.98,         // コントローラーが出せる電圧の割合
    fieldMin: 0.75,        // 可変界磁で弱められる下限 (特性図の界磁に対する割合。0.75 なら回転数は約 1.33 倍まで)
    fieldLoss: 2.0,        // 界磁を弱めたときに増える銅損の割合 (弱めた分 x この値だけ銅損が増える。見積もり)
    // 惰性のとき (電流 0) もモーターがチェーンで回されると、鉄損と軸受けの摩擦がブレーキになる。
    // 特性図の無負荷の損失 (24V・1780rpm で約 10W) から、モーター軸で約 0.054N·m の引きずりとする。
    // ワンウェイクラッチで切り離すなら false。コギングは 1 回転で打ち消し合うので、平均の抵抗には入れない
    motorDragWhenCoasting: true,
    // 電池 (充電式単 3 x 40 本 = 20 本直列 x 2 並列、公称 24V)
    series: 20, parallel: 2,
    cellMah: 1900,         // 1 本の容量 [mAh] (エネループ スタンダードの最小容量)
    cellOhm: 0.020,        // 1 本の内部抵抗 [Ω] (20 本直列 x 2 並列で組電池は 0.2Ω)
    cutoffCellV: 0.95,     // これより電圧が下がるとコントローラーが電流を絞る [V/本]
    maxCurrentA: 10,       // 電池から取る電流の上限 [A]。ブレーカーの定格に合わせる
    // サーキットブレーカー (熱動式)。定格を超えた分 ((I/定格)^2 - 1) を時間で積み、breakerHeatS に達すると
    // 切れる。チームの見立て「12A なら 1 分くらい耐える」に合わせ、12A で 60 秒 (15A で約 21 秒、20A で約 9 秒)。
    // 定格より下では冷える (0A なら breakerCoolS で元に戻る)
    breakerA: 10,
    breakerHeatS: 26.4,    // = ((12/10)^2 - 1) x 60 秒
    breakerCoolS: 30,      // 定格以下で冷えて元に戻るまでの時間の目安
    breakerResetS: 60,     // 切れたら止まって、冷えてから入れ直すまでの時間 [s]
    // 上りだけ電流を増やす走り方: 勾配が boostGrade より急な上りで、ブレーカーの熱が boostHeatMax (0〜1) より
    // 少ない間だけ、電池の電流を boostA まで流す (null なら増やさない)
    boostA: null,
    boostGrade: 0.02,
    boostHeatMax: 0.8,
    // 下りで冷ます: 勾配が downhillGrade より急な下りでは、電池の電流を downhillA までに絞る (null なら絞らない)
    downhillA: null,
    downhillGrade: -0.01,
    // ドライバー
    cruiseKmh: 120,        // 平らな所で保つ速さ (大きくすると、出せる限りの全開)
    accel: 1.0,            // 目標の速さまでの加速 [m/s^2] (モーターの力で頭打ちになる)
    brakeDecel: 2.5,       // シケイン手前のブレーキ [m/s^2]
    // チームの運転: ブレーキは最後のシケインだけで、シケインは 40km/h で曲がる。ほかのカーブと下りでは
    // ブレーキを使わない。デグナーなどは図の半径 (R15 など) より大きく回る走行ラインなので減速しない。
    // false にすると、すべてのカーブを図の半径と latG から決まる速さまで落とす
    brakeOnlyAtChicane: true,
    chicaneKmh: 40,
    maxCoastKmh: 85,       // brakeOnlyAtChicane が false のとき、下りで出してよい速さ
    holdOnClimb: true,     // 上りでも速さを保とうとする
    coastBelowGrade: null, // この勾配より急な下り (例 -0.01 = -1%) ではアクセルを戻して惰性で走る (null = 戻さない)
    // 大会
    attacks: 3,            // タイムアタックの回数
    flyingStartKmh: 0,     // スタートラインを通るときの速さ (0 = 止まった状態から)
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
  const wrap = (d) => ((d % LAP_M) + LAP_M) % LAP_M;
  const elevationAt = (d) => interp(ELEVATION, wrap(d));
  const gradeAt = (d) => (elevationAt(d + 2) - elevationAt(d - 2)) / 4;
  function sectionAt(d) {
    let name = SECTIONS[0][1];
    for (const [from, n] of SECTIONS) if (wrap(d) >= from) name = n;
    return name;
  }
  function sectorAt(d) {
    const m = wrap(d);
    for (const s of SECTORS) if (m < s.to) return s.id;
    return SECTORS[SECTORS.length - 1].id;
  }

  // シケイン (R9.5 から R20 まで) の範囲
  const CHICANE = { from: 5095, to: 5167 };

  // 距離 d (1 周の中) で出してよい速さ [m/s] (この先のカーブに間に合うブレーキも考える)
  function speedLimit(d, p) {
    if (p.brakeOnlyAtChicane) {
      const cap = p.chicaneKmh / 3.6;
      if (d >= CHICANE.from && d <= CHICANE.to) return cap;
      if (d < CHICANE.from) return Math.sqrt(cap * cap + 2 * p.brakeDecel * (CHICANE.from - d));
      return Infinity;
    }
    let lim = p.maxCoastKmh / 3.6;
    for (const c of CORNERS) {
      const cap = cornerKmh(c, p) / 3.6;
      if (cap >= lim) continue;
      if (d >= c.from && d <= c.to) lim = Math.min(lim, cap);
      else if (d < c.from) lim = Math.min(lim, Math.sqrt(cap * cap + 2 * p.brakeDecel * (c.from - d)));
    }
    return lim;
  }

  function battery(p) {
    const capAh = p.cellMah * p.parallel / 1000;
    const R = p.cellOhm * p.series / p.parallel;
    return { capAh, R, ocv: (usedAh) => p.series * interp(OCV, Math.min(1.08, usedAh / capAh)) };
  }

  // 駆動: 欲しいタイヤの力 fWant [N]、速さ v [m/s] のとき、実際の力・電池の電流・モーター電流を求める。
  // 界磁の強さ k (1 = 特性図のまま) を、いちばん大きな力が出るものに選ぶ (可変界磁・自動進角)
  function drive(fWant, v, usedAh, bat, p, capA) {
    const rw = p.tireDiameterM / 2;
    const w = Math.max(v / rw * p.gearRatio, 1e-3);          // モーターの角速度
    const ocv = bat.ocv(usedAh), vCut = p.cutoffCellV * p.series;
    const toTorque = rw / (p.gearRatio * p.chainEff);
    const ibCap = Math.min(capA, Math.max(0, (ocv - vCut) / bat.R));
    let best = { force: 0, ib: 0, vb: ocv, im: 0, k: 1 };
    const steps = 6;
    for (let s = 0; s <= steps; s++) {
      const k = 1 - (1 - p.fieldMin) * s / steps;
      const ke = MOTOR.ke * k, rc = MOTOR.rCopper * (1 + p.fieldLoss * (1 - k)), iF = MOTOR.iFric / k;
      let im = Math.min(fWant * toTorque / ke + iF, p.motorMaxA), ib = 0, vb = ocv;
      for (let it = 0; it < 4; it++) {
        // 全開 (デューティ上限) で流せる電流: 逆起電力 + 内部の電圧降下 ≦ 電池の電圧
        const imFull = (vb * p.maxDuty - ke * w) / MOTOR.rDroop;
        im = Math.max(0, Math.min(im, imFull));
        ib = (ke * w * im + rc * im * im) / (vb * p.ctrlEff);
        if (ib > ibCap) {
          // 電池の電流が ibCap になるモーター電流 (rc·i^2 + ke·w·i = ibCap·vb·ctrlEff を解く)
          const c = ibCap * vb * p.ctrlEff;
          im = (-ke * w + Math.sqrt(ke * ke * w * w + 4 * rc * c)) / (2 * rc);
          ib = ibCap;
        }
        vb = ocv - ib * bat.R;
      }
      const force = im > iF ? ke * (im - iF) / toTorque : 0;
      if (force > best.force + 1e-6) best = { force, ib, vb, im, k };
      if (force >= fWant - 1e-6) break;          // 欲しい力が出るなら、界磁は弱めない (効率がよい)
    }
    return best;
  }

  // 走行を計算する。戻り値は 50ms ごとの記録と、アタックごとのまとめ
  function simulate(params) {
    const p = Object.assign({}, DEFAULTS, params || {});
    const bat = battery(p);
    const rec = { t: [], d: [], v: [], elev: [], i: [], volt: [], q: [], sector: [], section: [], heat: [], im: [], field: [], attack: [], phase: [] };
    let curAttack = 1, phase = '走行';
    const attacks = [], events = [], brakeBySection = {};
    const loss = { aero: 0, roll: 0, brake: 0, battery: 0, motor: 0, chain: 0, ctrl: 0, climb: 0 };
    let t = 0, usedAh = 0, dnf = null, heat = 0, tripped = false, trips = 0;
    const dt = p.dt;
    const record = (dAbs, v, i, volt, im, k) => {
      rec.t.push(+t.toFixed(3)); rec.d.push(dAbs); rec.v.push(v); rec.elev.push(elevationAt(dAbs));
      rec.i.push(i); rec.volt.push(volt); rec.q.push(usedAh * 1000); rec.sector.push(sectorAt(dAbs)); rec.section.push(sectionAt(dAbs));
      rec.heat.push(heat / p.breakerHeatS); rec.im.push(im); rec.field.push(k === undefined ? 1 : k); rec.attack.push(curAttack); rec.phase.push(phase);
    };
    const breakerStep = (i) => {
      const x = (i / p.breakerA) ** 2 - 1;
      heat = x > 0 ? heat + x * dt : Math.max(0, heat - p.breakerHeatS / p.breakerCoolS * dt * Math.min(1, -x));
      if (heat >= p.breakerHeatS) { tripped = true; heat = p.breakerHeatS; }
    };
    const idle = (seconds, dAbs) => {
      for (let s = 0; s < seconds; s += dt) { t += dt; breakerStep(0); record(dAbs, 0, 0, bat.ocv(usedAh), 0); }
    };

    for (let a = 0; a < p.attacks && !dnf; a++) {
      curAttack = a + 1; phase = '走行';
      let d = 0, v = p.flyingStartKmh / 3.6, stuck = 0, peakA = 0, minV = Infinity;
      const t0 = t, q0 = usedAh;
      const sectors = [];
      let secIdx = 0, secT0 = t0, secQ0 = q0;
      while (d < LAP_M) {
        const grade = gradeAt(d);
        const fGrav = p.massKg * G * grade;
        const fRoll = p.crr * p.massKg * G;
        const fAero = 0.5 * p.airDensity * p.cdA * v * v;
        const fRes = fGrav + fRoll + fAero;
        const lim = speedLimit(d, p);
        let fWant = 0, brake = 0;
        if (tripped) {
          // ブレーカーが切れた: 惰性で止まり、冷えるのを待って入れ直す
          if (v < 0.05) {
            events.push({ t, attack: a + 1, d, section: sectionAt(d), kind: 'reset' });
            phase = 'ブレーカー復帰待ち';
            for (let s = 0; s < p.breakerResetS; s += dt) { t += dt; heat = Math.max(0, heat - p.breakerHeatS / p.breakerCoolS * dt); record(a * LAP_M + d, 0, 0, bat.ocv(usedAh), 0); }
            phase = '走行';
            tripped = false; heat = 0;
            continue;
          }
          brake = 0.5 * p.massKg;
        } else if (v > lim + 0.05) {
          brake = p.brakeDecel * p.massKg;                  // カーブの手前でブレーキ
        } else {
          const target = Math.min(p.cruiseKmh / 3.6, lim);
          if (v < target - 0.05) fWant = p.massKg * Math.min(p.accel, (target - v) / 1.0) + fRes;
          else if (v > target + 0.05) fWant = 0;            // 下りでついた速さは、惰性で目標まで落とす
          else fWant = fRes;                                // 速さを保つ
          if (!p.holdOnClimb && grade > 0.005) fWant = Math.min(fWant, fRoll + fAero);
          if (p.coastBelowGrade !== null && grade < p.coastBelowGrade) fWant = 0;
          fWant = Math.max(0, fWant);
        }
        const boosting = p.boostA !== null && grade > p.boostGrade && heat < p.boostHeatMax * p.breakerHeatS;
        let capA = boosting ? Math.max(p.boostA, p.maxCurrentA) : p.maxCurrentA;
        if (p.downhillA !== null && grade < p.downhillGrade) capA = Math.min(capA, p.downhillA);
        const dr = fWant > 0 ? drive(fWant, v, usedAh, bat, p, capA) : { force: 0, ib: 0, vb: bat.ocv(usedAh), im: 0, k: 1 };
        let fMotor = Math.min(dr.force, fWant);
        // 電流を流していないときの、モーターの引きずり (惰性・ブレーキ中)
        if (dr.ib <= 0 && p.motorDragWhenCoasting && v > 0.05) {
          fMotor = -MOTOR.ke * MOTOR.iFric * p.gearRatio / (p.tireDiameterM / 2) / p.chainEff;
        }
        const i = dr.ib, volt = dr.vb;
        const wasTripped = tripped;
        breakerStep(i);
        if (tripped && !wasTripped) { trips++; events.push({ t, attack: a + 1, d, section: sectionAt(d), kind: 'trip', current: i }); }
        const acc = (fMotor - fRes - brake) / p.massKg;
        const vNew = Math.max(0, v + acc * dt);
        const ds = (v + vNew) / 2 * dt;
        // エネルギーの行き先 [J]
        loss.aero += fAero * ds; loss.roll += fRoll * ds; loss.climb += fGrav * ds; loss.brake += brake * ds;
        if (brake > 0) { const sec = sectionAt(d); brakeBySection[sec] = (brakeBySection[sec] || 0) + brake * ds / 3600; }
        loss.battery += i * i * bat.R * dt;
        const pBat = volt * i, pMotorIn = pBat * p.ctrlEff, pWheel = fMotor * (v + vNew) / 2;
        loss.ctrl += (pBat - pMotorIn) * dt;
        if (fMotor >= 0) {
          loss.chain += (pWheel / p.chainEff - pWheel) * dt;
          loss.motor += Math.max(0, pMotorIn - pWheel / p.chainEff) * dt;
        } else {
          loss.motor += -pWheel * dt;                     // 惰性のときのモーターの引きずり
        }
        const dPrev = d, tPrev = t, qPrev = usedAh;
        v = vNew; d += ds; t += dt;
        usedAh += i * dt / 3600;
        while (secIdx < SECTORS.length && d >= SECTORS[secIdx].to) {
          const sec = SECTORS[secIdx];
          const frac = d > dPrev ? (sec.to - dPrev) / (d - dPrev) : 1;
          const tCross = tPrev + (t - tPrev) * frac;
          const qCross = qPrev + (usedAh - qPrev) * frac;
          const timeS = tCross - secT0;
          sectors.push({
            id: sec.id, name: sec.name, startT: secT0, endT: tCross, timeS,
            avgKmh: ((sec.to - sec.from) / timeS) * 3.6, mAh: (qCross - secQ0) * 1000,
          });
          secT0 = tCross; secQ0 = qCross; secIdx++;
        }
        peakA = Math.max(peakA, i); minV = Math.min(minV, volt);
        record(a * LAP_M + d, v, i, volt, dr.im, dr.k);
        stuck = v < 0.3 && !tripped ? stuck + dt : 0;
        if (stuck > 20) { dnf = { attack: a + 1, d, section: sectionAt(d), reason: '電池の電圧が下がり、上りで止まった' }; break; }
      }
      attacks.push({
        attack: a + 1, finished: !dnf, startT: t0, lapS: t - t0, avgKmh: (Math.min(d, LAP_M) / (t - t0)) * 3.6,
        mAh: (usedAh - q0) * 1000, peakA, minV, sectors,
      });
      if (dnf) break;
      // ゴール後: ブレーキで止まって待つ
      phase = 'ゴール後';
      while (v > 0) { v = Math.max(0, v - 1.5 * dt); d += v * dt; t += dt; breakerStep(0); record(a * LAP_M + Math.min(d, LAP_M + 200), v, 0, bat.ocv(usedAh), 0); }
      phase = 'アタックの間';
      if (a < p.attacks - 1) idle(p.waitS, (a + 1) * LAP_M);
    }
    const toWh = (j) => j / 3600;
    return {
      params: p, rec, attacks, dnf, events, trips, brakeBySection,
      usedMah: usedAh * 1000, capacityMah: bat.capAh * 1000,
      totalLapS: attacks.reduce((s, x) => s + (x.finished ? x.lapS : 0), 0),
      energyWh: {
        aero: toWh(loss.aero), roll: toWh(loss.roll), brake: toWh(loss.brake), battery: toWh(loss.battery),
        motor: toWh(loss.motor), drive: toWh(loss.chain + loss.ctrl), climbNet: toWh(loss.climb),
      },
    };
  }

  // 3 回のアタックで、容量の share (0〜1) をちょうど使い切る平らな所の速さ [km/h] を探す。
  // 探す間は刻みを粗くして速くし、最後に細かい刻みで確かめて詰める
  function findCruise(params, share) {
    const fits = (kmh, dt) => {
      const r = simulate(Object.assign({}, params, { cruiseKmh: kmh, dt }));
      return !r.dnf && r.usedMah <= share * r.capacityMah;
    };
    let lo = 15, hi = 120;
    if (fits(hi, 0.2)) return hi;
    while (hi - lo > 0.1) { const mid = (lo + hi) / 2; if (fits(mid, 0.2)) lo = mid; else hi = mid; }
    const dt = (params && params.dt) || DEFAULTS.dt;
    while (lo > 15 && !fits(lo, dt)) lo -= 0.1;
    return Math.round(lo * 10) / 10;
  }

  // 1 周の目標タイム lapS [s] で 3 回走れるかを調べる (いちばん遅い周が目標に届く速さを探す)
  function forLapTime(params, lapS) {
    const run = (kmh, dt) => simulate(Object.assign({}, params, { cruiseKmh: kmh, dt }));
    const slowest = (r) => (r.dnf ? Infinity : Math.max(...r.attacks.map((a) => a.lapS)));
    let lo = 15, hi = 120;
    while (hi - lo > 0.1) { const mid = (lo + hi) / 2; if (slowest(run(mid, 0.2)) <= lapS) hi = mid; else lo = mid; }
    const r = run(hi, (params && params.dt) || DEFAULTS.dt);
    return { cruiseKmh: hi, result: r, lapOk: slowest(r) <= lapS + 0.5, shareUsed: r.usedMah / r.capacityMah, bestLapS: slowest(r), trips: r.trips };
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

  // 上りだけ電流を増やす走り方の組み合わせを探す。ブレーカーが切れず、電池が share 以下で、3 回の合計が
  // いちばん短いもの。戻り値は { params, result, tried }
  function optimizeBoost(params, share) {
    share = share || 0.95;
    const base = Object.assign({}, params || {});
    const ok = (r) => !r.dnf && r.trips === 0 && r.usedMah <= share * r.capacityMah;
    let best = null, tried = 0;
    const consider = (x) => {
      const r = simulate(Object.assign({}, base, x, { dt: 0.2 }));
      tried++;
      if (ok(r) && (!best || r.totalLapS < best.coarse)) best = { x, coarse: r.totalLapS };
    };
    consider({});
    for (const boostA of [11, 12, 13, 14, 16, 18]) for (const boostGrade of [0.01, 0.02, 0.03, 0.045])
      for (const boostHeatMax of [0.6, 0.85]) for (const downhillA of [null, 8, 5])
        consider({ boostA, boostGrade, boostHeatMax, downhillA });
    const result = simulate(Object.assign({}, base, best.x));
    return { params: best.x, result, tried, ok: ok(result) };
  }

  // ログビューア用: 走り方を比べる (全開・上りだけ増やす・控えめ・12A 全開)
  function strategies(params, share) {
    share = share || 0.95;
    const base = Object.assign({}, params || {});
    const p = Object.assign({}, DEFAULTS, base);
    const cEco = findCruise(base, 0.8);
    const boost = { cruiseKmh: 120, boostA: 12, boostGrade: 0.02, boostHeatMax: 0.85 };
    const list = [
      { key: 'full', name: `${p.maxCurrentA}A 全開`, note: `電池から ${p.maxCurrentA}A まで、出せる限り`, params: { cruiseKmh: 120 } },
      { key: 'boost', name: '上りだけ 12A', note: '勾配 2% より急な上りで、ブレーカーの熱が 85% になるまで 12A', params: boost },
    ];
    if (cEco < 120) list.push({ key: 'eco', name: '控えめ', note: `容量の 80% を使う速さ (${cEco} km/h まで)`, params: { cruiseKmh: cEco } });
    list.push({ key: 'over12', name: '12A 全開', note: `ずっと 12A (ブレーカー ${p.breakerA}A は 12A で約 1 分で切れる)`, params: { cruiseKmh: 120, maxCurrentA: 12 } });
    return list.map((st) => Object.assign(st, { result: simulate(Object.assign({}, base, st.params)) }));
  }

  // 車輪が回った距離 [m] (速さの積分)。電力計の速度センサーが数える距離と同じもの。
  // rec.d はコース上の位置なので、ゴール後に止まるまでの分を数えず、アタックごとにスタート位置へ戻る
  function odometer(rec) {
    const out = new Float64Array(rec.t.length);
    for (let k = 1; k < rec.t.length; k++) out[k] = out[k - 1] + (rec.v[k] + rec.v[k - 1]) / 2 * (rec.t[k] - rec.t[k - 1]);
    return out;
  }

  // ログビューアや電力計と同じ形の CSV (電力計の記録と同じ列。速さと距離は速度センサーの列)
  function toCsv(result, from, to) {
    const r = result.rec, odo = odometer(r);
    const out = ['time_s,voltage_V,current_A,power_W,charge_mAh,speed_kmh,distance_m'];
    for (let k = from || 0; k < (to || r.t.length); k++) {
      const i = r.i[k], v = r.volt[k];
      out.push(`${r.t[k].toFixed(3)},${v.toFixed(3)},${i.toFixed(3)},${(v * i).toFixed(2)},${r.q[k].toFixed(2)},${(r.v[k] * 3.6).toFixed(2)},${odo[k].toFixed(1)}`);
    }
    return out.join('\n') + '\n';
  }

  // 詳しい CSV (表計算ソフト用)。電力計の列に、シミュレーションだけにある値を足す。
  // Excel で日本語が化けないよう、先頭に BOM を付ける
  function toDetailCsv(result) {
    const r = result.rec;
    const head = ['time_s', 'attack', 'phase', 'lap_distance_m', 'sector', 'section', 'elevation_m', 'grade_pct', 'speed_kmh',
      'voltage_V', 'current_A', 'power_W', 'charge_mAh', 'motor_current_A', 'field_pct', 'breaker_heat_pct'];
    const out = ['\ufeff' + head.join(',')];
    for (let k = 0; k < r.t.length; k++) {
      const lapD = Math.max(0, Math.min(r.d[k] - (r.attack[k] - 1) * LAP_M, LAP_M));
      const running = r.phase[k] === '走行';
      out.push([
        r.t[k].toFixed(3), r.attack[k], r.phase[k], lapD.toFixed(1), running ? r.sector[k] : '', r.section[k], r.elev[k].toFixed(2),
        running ? (gradeAt(r.d[k]) * 100).toFixed(2) : '', (r.v[k] * 3.6).toFixed(2), r.volt[k].toFixed(3),
        r.i[k].toFixed(3), (r.volt[k] * r.i[k]).toFixed(2), r.q[k].toFixed(2), r.im[k].toFixed(2), (r.field[k] * 100).toFixed(0), (r.heat[k] * 100).toFixed(1),
      ].join(','));
    }
    return out.join('\n') + '\n';
  }

  return {
    LAP_M, ELEVATION, SECTIONS, SECTORS, CORNERS, MOTOR, DEFAULTS, elevationAt, gradeAt, sectionAt, sectorAt, speedLimit, cornerKmh,
    motorAt24V, simulate, findCruise, forLapTime, maxCdAFor, optimizeBoost, strategies, odometer, toCsv, toDetailCsv,
  };
});
