// 鈴鹿サーキットの走行シミュレーションを、走り方を変えて比べる
//   node tools/suzuka_sim/cli.mjs                 … 比べた結果を表で出し、CSV を out/ に書く
//   node tools/suzuka_sim/cli.mjs --firmware      … さらに「ちょうど使い切る」走り方の電流と電圧を
//                                                  実際のファームウェア (test/host_sim) に流し、
//                                                  電力計が SD に書く CSV と LCD の表示を確かめる
//   node tools/suzuka_sim/cli.mjs --set massKg=85 --set cdA=0.11   … 車の値を変える
import fs from 'fs';
import os from 'os';
import path from 'path';
import { execFileSync, spawnSync } from 'child_process';
import { createRequire } from 'module';
import { fileURLToPath } from 'url';

const require = createRequire(import.meta.url);
const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(HERE, '../..');
const S = require(path.join(HERE, 'suzuka-sim.js'));

const args = process.argv.slice(2);
const opt = { out: path.join(HERE, 'out'), firmware: false, set: {} };
for (let k = 0; k < args.length; k++) {
  if (args[k] === '--out') opt.out = path.resolve(args[++k]);
  else if (args[k] === '--firmware') opt.firmware = true;
  else if (args[k] === '--set') {
    const [key, val] = args[++k].split('=');
    if (!(key in S.DEFAULTS)) { console.error(`知らない値: ${key} (使えるのは ${Object.keys(S.DEFAULTS).join(', ')})`); process.exit(2); }
    opt.set[key] = typeof S.DEFAULTS[key] === 'boolean' ? val === 'true' : Number(val);
  } else { console.error(`知らない引数: ${args[k]}`); process.exit(2); }
}
const base = Object.assign({}, opt.set);
const p0 = Object.assign({}, S.DEFAULTS, base);

// ---------------------------------------------------------------------------
//  走り方
const SHARE = 0.95;   // 容量の 95% を使い切る目標 (残り 5% は個体差と見積もりの誤差のため)
const mmss = (s) => `${Math.floor(s / 60)}:${String(Math.round(s % 60)).padStart(2, '0')}`;
console.log('# 鈴鹿サーキット フルコース 5.807km × 3 回タイムアタックのシミュレーション\n');
console.log(`車: ${p0.massKg}kg、Crr ${p0.crr}、CdA ${p0.cdA}m²、モーター ${S.MOTOR.name} (可変界磁 ${p0.fieldMin} まで)、ギア比 ${p0.gearRatio.toFixed(2)}、タイヤ外径 ${p0.tireDiameterM}m`);
console.log(`電池: 単3 ${p0.series}直列×${p0.parallel}並列 (${p0.cellMah * p0.parallel}mAh、内部抵抗 ${p0.cellOhm * 1000}mΩ/本 → ${(p0.cellOhm * p0.series / p0.parallel).toFixed(2)}Ω)`);
console.log(`ブレーカー: ${p0.breakerA}A (2 倍で約 ${Math.round(p0.breakerHeatS / 3)} 秒で切れる近似)、アタックの間の待ち ${p0.waitS} 秒\n`);
console.log('| 走り方 | 平らな所の速さ | 1回目 | 2回目 | 3回目 | 合計 | 使った量 | 最大電流 | 最低電圧 | ブレーカー |');
console.log('|---|---|---|---|---|---|---|---|---|---|');
const results = S.strategies(base, SHARE).map((st) => ({ st, r: st.result }));
for (const { st, r } of results) {
  const laps = [0, 1, 2].map((k) => { const a = r.attacks[k]; return a ? (a.finished ? mmss(a.lapS) : `途中で停止 (${r.dnf.section})`) : '—'; });
  const peak = Math.max(...r.attacks.map((a) => a.peakA)), minV = Math.min(...r.attacks.map((a) => a.minV));
  const total = r.dnf ? '完走できず' : mmss(r.totalLapS);
  console.log(`| ${st.name} | ${st.params.cruiseKmh.toFixed(1)} km/h | ${laps.join(' | ')} | ${total} | ${r.usedMah.toFixed(0)} mAh (${(r.usedMah / r.capacityMah * 100).toFixed(0)}%) | ${peak.toFixed(1)} A | ${minV.toFixed(1)} V | ${r.trips ? `${r.trips} 回切れた` : '切れない'} |`);
}
console.log('\n## エネルギーの行き先 (3 回分) [Wh]\n');
console.log('| 走り方 | 空気抵抗 | 転がり抵抗 | ブレーキ | 電池の内部抵抗 | モーター | チェーン・コントローラー | 電池から出した合計 |');
console.log('|---|---|---|---|---|---|---|---|');
for (const { st, r } of results) {
  const e = r.energyWh;
  const totalWh = e.aero + e.roll + e.brake + e.battery + e.motor + e.drive + e.climbNet;
  console.log(`| ${st.name} | ${e.aero.toFixed(1)} | ${e.roll.toFixed(1)} | ${e.brake.toFixed(1)} | ${e.battery.toFixed(1)} | ${e.motor.toFixed(1)} | ${e.drive.toFixed(1)} | ${totalWh.toFixed(1)} |`);
}
for (const { st, r } of results) {
  if (!r.events.length) continue;
  console.log(`\n${st.name}: ブレーカーが切れた場所`);
  for (const e of r.events.filter((x) => x.kind === 'trip')) console.log(`  - ${e.attack} 回目 ${e.section} (${Math.round(e.d)}m 地点)`);
}

// ---------------------------------------------------------------------------
//  CSV (電力計と同じ形) を書く
fs.mkdirSync(opt.out, { recursive: true });
for (const { st, r } of results) {
  const dir = path.join(opt.out, st.key);
  fs.mkdirSync(dir, { recursive: true });
  fs.writeFileSync(path.join(dir, 'LOG000.CSV'), S.toCsv(r));
}
console.log(`\nCSV: ${path.relative(process.cwd(), opt.out)}/<走り方>/LOG000.CSV (ログビューアで開ける)`);

// ---------------------------------------------------------------------------
//  ファームウェアに流す
if (opt.firmware) {
  const { r } = results[0];
  const work = fs.mkdtempSync(path.join(os.tmpdir(), 'ene1_suzuka_'));
  const bin = path.join(work, 'sim');
  execFileSync('g++', ['-std=gnu++17', '-O2', '-I', path.join(REPO, 'test/host_sim/mock'), '-o', bin, path.join(REPO, 'test/host_sim/sim.cpp')], { stdio: 'inherit' });
  const sd = path.join(opt.out, 'firmware');
  fs.rmSync(sd, { recursive: true, force: true });
  fs.mkdirSync(sd, { recursive: true });
  const lines = [`v ${r.rec.volt[0].toFixed(3)}`, 'i 0', 'setup', 'mark'];
  let lastAttack = 0;   // 各アタックのゴールの直後に LCD を表示する
  for (let k = 1; k < r.rec.t.length; k++) {
    lines.push(`v ${r.rec.volt[k].toFixed(3)}`, `i ${r.rec.i[k].toFixed(3)}`, `run_to ${Math.round(r.rec.t[k] * 1000)}`);
    const attackNow = Math.floor(r.rec.d[k] / S.LAP_M + 1e-9);
    if (attackNow > lastAttack && attackNow <= r.attacks.length) { lines.push(`say "${attackNow} 回目のゴール"`, 'lcd'); lastAttack = attackNow; }
  }
  lines.push('say "最後"', 'lcd');
  const res = spawnSync(bin, [path.join(work, 'state'), sd, 'cold'], { input: lines.join('\n') + '\n', encoding: 'utf8', maxBuffer: 1 << 26 });
  const shown = res.stdout.split('\n').filter((l) => /^==|^\s+\|/.test(l));
  console.log('\n## ファームウェアに流した結果 (LCD の表示)\n');
  console.log(shown.join('\n'));
  const logs = fs.readdirSync(sd).filter((n) => /\.CSV$/i.test(n));
  const last = fs.readFileSync(path.join(sd, logs[logs.length - 1]), 'utf8').trim().split('\n').pop().split(',');
  console.log(`\n電力計の記録: ${logs.join(', ')}、最後の行の積算 ${Number(last[4]).toFixed(1)} mAh (物理モデル ${r.usedMah.toFixed(1)} mAh)`);
  console.log(`CSV: ${path.relative(process.cwd(), sd)}/`);
  fs.rmSync(work, { recursive: true, force: true });
  if (res.status !== 0) { console.error(res.stdout.slice(-2000)); process.exit(1); }
}
