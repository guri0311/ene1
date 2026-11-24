// ログビューアの計算部分 (tools/log_viewer/logviewer-core.js) のテスト
//   node test/log_viewer/test.mjs <シミュレーションが作った SD のディレクトリ>
// 通常は run_tests.sh から呼ぶ
import fs from 'fs';
import path from 'path';
import { createRequire } from 'module';
import { fileURLToPath } from 'url';

const require = createRequire(import.meta.url);
const HERE = path.dirname(fileURLToPath(import.meta.url));
const C = require(path.join(HERE, '../../tools/log_viewer/logviewer-core.js'));

let passed = 0, failed = 0;
function check(ok, msg) {
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${msg}`);
  if (ok) passed++; else failed++;
}
const near = (x, want, tol) => Math.abs(x - want) <= tol;
const f2 = (x) => (Number.isFinite(x) ? x.toFixed(2) : String(x));

const sdDir = process.argv[2];
if (!sdDir) { console.error('使い方: node test.mjs <SD のディレクトリ>'); process.exit(2); }

// ---------------------------------------------------------------------------
console.log('\n== 1. シミュレーションが作った実際の CSV を読む');
const names = fs.readdirSync(sdDir).filter((n) => /\.CSV$/i.test(n)).sort();
check(names.join(' ') === 'LOG000.CSV LOG001.CSV LOG002.CSV', `ファイルは LOG000〜LOG002 (実際: ${names.join(' ')})`);
// 読み込む順番がばらばらでも、ファイル番号で並べ直すことを確かめるため逆順で渡す
const files = names.reverse().map((n) => C.parseCsv(fs.readFileSync(path.join(sdDir, n), 'utf8'), n));
for (const f of files) {
  check(f.headerOk, `${f.name}: 見出し行が ${C.HEADER.concat(C.SPEED_HEADER).join(',')}`);
  check(f.s && f.d && f.s.length === f.t.length, `${f.name}: 速さと距離の列も読めた`);
  check(f.skipped === 0, `${f.name}: 読めない行なし (${f.skipped} 行)`);
  const dt = [];
  for (let k = 1; k < f.t.length; k++) dt.push(f.t[k] - f.t[k - 1]);
  const med = dt.sort((a, b) => a - b)[dt.length >> 1];
  check(near(med, 0.05, 0.002), `${f.name}: 記録間隔は 50ms (中央値 ${(med * 1000).toFixed(1)}ms, ${f.t.length} 行)`);
}

console.log('\n== 2. 走行へのまとめ (Nano のリセットはつなぐ、積算リセットは分ける)');
const runs = C.buildRuns(files);
check(runs.length === 2, `走行は 2 つ (実際: ${runs.length})`);
const [r0, r1] = runs;
check(r0.files.join(',') === 'LOG000.CSV,LOG001.CSV', `1 つ目の走行 = LOG000 + LOG001 (実際: ${r0.files.join(',')})`);
check(r0.joins.length === 1 && r0.joins[0].file === 'LOG001.CSV', `継ぎ目 (Nano のリセット) は LOG001 の先頭の 1 か所`);
check(r1.files.join(',') === 'LOG002.CSV', `2 つ目の走行 = LOG002 (実際: ${r1.files.join(',')})`);
check(r1.t[0] < 1, `2 つ目の走行は時刻 0 から (先頭 ${f2(r1.t[0])} 秒)`);
let mono = true;
for (let k = 1; k < r0.t.length; k++) if (r0.t[k] < r0.t[k - 1]) mono = false;
check(mono, 'つないだ走行の時刻は増える一方');

console.log('\n== 3. 1 つ目の走行の集計');
const s0 = C.stats(r0);
// 10A×120s + 20A(頭打ち)×5s + 5A×60s + 8A×60s = 1200+100+300+480 = 2080 As = 577.8 mAh
const wantMah = 2080 / 3.6;
check(near(s0.mAh, wantMah, wantMah * 0.01), `使用量 ${f2(s0.mAh)} mAh ≒ ${f2(wantMah)} mAh (±1%)`);
check(near(s0.Wh, 24 * wantMah / 1000, 24 * wantMah / 1000 * 0.015), `電力量 ${f2(s0.Wh)} Wh ≒ 24V × ${f2(wantMah)} mAh = ${f2(24 * wantMah / 1000)} Wh (±1.5%)`);
// 1 回目 185 秒 + 2 回目 60 秒 + 長押しの間 (0A) 3.5 秒
check(near(s0.duration, 248.5, 0.3), `走行時間 ${f2(s0.duration)} 秒 ≒ 248.5 秒`);
check(near(s0.avgA, 2080 / 248.5, 0.05), `平均電流 ${f2(s0.avgA)} A ≒ ${f2(2080 / 248.5)} A`);
check(s0.peakA >= C.DEFAULT_OVER_A && s0.peakA <= 20.01, `最大電流 ${f2(s0.peakA)} A (20A で頭打ち)`);
check(near(s0.overS, 5, 0.3), `OVER (20A に張り付き) の時間 ${f2(s0.overS)} 秒 ≒ 5 秒`);
check(near(s0.minV, 24, 0.01) && near(s0.maxV, 24, 0.01), `電圧 ${f2(s0.minV)}〜${f2(s0.maxV)} V (24V 一定)`);
check(s0.joins === 1, `範囲内の継ぎ目 1 か所 (実際: ${s0.joins})`);
check(s0.smooth > 1.0 && s0.smooth < 1.5, `電流のばらつき (2 乗平均 / 平均) ${f2(s0.smooth)} は 1 より大きい`);

console.log('\n== 3b. 1 つ目の走行の速さと距離 (Nano のリセットをまたいでも距離は続く)');
// 30km/h×120s + 36km/h×5s + 18km/h×60s + 27km/h×60s = 1000+50+300+450 = 1800m
check(s0.hasSpeed, '速度センサーの列がある');
check(near(s0.distM, 1800, 1800 * 0.01), `距離 ${f2(s0.distM)} m ≒ 1800 m (±1%)`);
check(near(s0.maxKmh, 36, 0.05), `最高速 ${f2(s0.maxKmh)} km/h ≒ 36`);
check(near(s0.avgKmh, 1800 / 248.5 * 3.6, 0.3), `平均速度 ${f2(s0.avgKmh)} km/h ≒ ${f2(1800 / 248.5 * 3.6)}`);
check(near(s0.whPerKm, s0.Wh / (s0.distM / 1000), 0.05), `電費 ${f2(s0.whPerKm)} Wh/km = 電力量 ÷ 距離`);
check(near(s0.mAhPerKm, s0.mAh / (s0.distM / 1000), 0.5), `${f2(s0.mAhPerKm)} mAh/km = 使用量 ÷ 距離`);
const jd = r0.joins[0].index;
check(r0.d[jd] >= r0.d[jd - 1] && r0.d[jd] - r0.d[jd - 1] < 3, `Nano のリセットの前後で距離が続く (${f2(r0.d[jd - 1])} → ${f2(r0.d[jd])} m)`);

console.log('\n== 4. 範囲を指定した集計 (10A 一定の区間)');
const a = C.lowerBound(r0.t, 20), b = C.lowerBound(r0.t, 110);
const sr = C.stats(r0, a, b);
check(near(sr.avgA, 10, 0.05), `20〜110 秒の平均電流 ${f2(sr.avgA)} A ≒ 10 A`);
check(near(sr.mAh, 250, 2), `20〜110 秒の使用量 ${f2(sr.mAh)} mAh ≒ 250 mAh`);
check(near(sr.smooth, 1, 0.01), `一定の電流ならばらつき ${f2(sr.smooth)} ≒ 1.00`);
check(near(sr.avgKmh, 30, 0.3) && near(sr.maxKmh, 30, 0.05), `20〜110 秒の平均 ${f2(sr.avgKmh)} km/h・最高 ${f2(sr.maxKmh)} km/h ≒ 30`);
check(near(sr.distM, 750, 3), `20〜110 秒の距離 ${f2(sr.distM)} m ≒ 750 m`);

console.log('\n== 5. 2 つ目の走行 (積算リセットのあと)');
const s1 = C.stats(r1);
check(near(s1.avgA, 4, 0.1), `平均電流 ${f2(s1.avgA)} A ≒ 4 A`);
check(near(s1.mAh, 4 * s1.duration / 3.6, 2), `使用量 ${f2(s1.mAh)} mAh ≒ 4A × ${f2(s1.duration)} 秒`);
check(r1.d[0] < 1, `積算リセットで距離も 0 から (先頭 ${f2(r1.d[0])} m)`);
check(near(s1.distM, 200, 4), `距離 ${f2(s1.distM)} m ≒ 24km/h × 30 秒 = 200 m`);

console.log('\n== 6. 電流の分布 (時間の重み付き)');
const h = C.histogram(r0, undefined, undefined, 1);
const total = h.reduce((s, x) => s + x.s, 0);
check(near(total, s0.duration - s0.gapS, 0.2), `分布の合計 ${f2(total)} 秒 = 走行時間 - 抜け`);
const at10 = h.find((x) => x.a0 === 10);
check(at10 && near(at10.s, 120, 1.5), `10〜11A の時間 ${at10 ? f2(at10.s) : '-'} 秒 ≒ 120 秒`);

console.log('\n== 7. ペース配分');
// 248.4 秒で 1000mAh を使い切る計画なら、終わりの時点の理想は 1000mAh
const p = C.pace(r0, 1000, s0.duration);
check(near(p.ideal, 1000, 5), `終わりの時点の理想 ${f2(p.ideal)} mAh ≒ 1000`);
check(near(p.diff, s0.mAh - 1000, 5), `差 ${f2(p.diff)} mAh (マイナス = 余裕あり)`);
const pm = C.pace(r0, 1000, 600, C.lowerBound(r0.t, 120));
check(near(pm.ideal, 1000 * 120 / 600, 1), `120 秒の時点の理想 ${f2(pm.ideal)} mAh ≒ 200 (600 秒で 1000mAh)`);
check(near(pm.needA, pm.remainMah * 3.6 / pm.remainS, 1e-9) && pm.needA > 0, `残りを均等に使う電流 ${f2(pm.needA)} A`);
check(C.pace(r0, 0, 600) === null, '予算が 0 なら計算しない');

console.log('\n== 8. 壊れた CSV');
const broken = 'time_s,voltage_V,current_A,power_W,charge_mAh\n0.050,24.000,1.000,24.00,0.01\n0.100,24.000,1.000,24.00,0.03\n0.150,24.0';
const fb = C.parseCsv(broken, 'LOG009.CSV');
check(fb.t.length === 2 && fb.skipped === 1, `電源断で途中までの最後の行は飛ばす (読めた ${fb.t.length} 行, 飛ばした ${fb.skipped} 行)`);
const fw = C.parseCsv('time_s,voltage_V,current_A\n1,2,3\n', 'x.csv');
check(!fw.headerOk, '見出しの列が違うファイルは headerOk = false');
const fr = C.parseCsv('time_s,voltage_V,current_A,power_W,charge_mAh\r\n1.000,24.000,1.000,24.00,0.30\r\n', 'LOG010.CSV');
check(fr.t.length === 1 && fr.skipped === 0, 'Windows の改行 (CRLF) でも読める');
const fs7 = C.parseCsv('time_s,voltage_V,current_A,power_W,charge_mAh,speed_kmh,distance_m\n0.050,24.000,1.000,24.00,0.01,30.00,0.0\n0.100,24.000,1.000,24.00,0.03,30.00,1.6\n0.150,24.000,1.000,24.00,0.04', 'LOG011.CSV');
check(fs7.headerOk && fs7.t.length === 2 && fs7.skipped === 1 && fs7.s[1] === 30 && fs7.d[1] === 1.6, '速さ・距離の列がある CSV: 電源断で列が足りない最後の行は飛ばす');

console.log('\n== 8b. 速度センサーに対応する前の CSV (5 列) も読める');
const old = C.parseCsv('time_s,voltage_V,current_A,power_W,charge_mAh\n0.050,24.000,1.000,24.00,0.01\n0.100,24.000,1.000,24.00,0.03\n', 'LOG020.CSV');
check(old.headerOk && old.t.length === 2 && old.s === null && old.d === null, '5 列の CSV は速さ・距離なし (s, d が null)');
const so = C.stats(C.buildRuns([old])[0]);
check(!so.hasSpeed && Number.isNaN(so.distM) && Number.isNaN(so.whPerKm) && near(so.mAh, 0.02, 1e-9), '5 列の CSV の集計: 距離・電費は NaN、ほかは今までどおり');
const mixed = C.buildRuns([old, C.parseCsv('time_s,voltage_V,current_A,power_W,charge_mAh,speed_kmh,distance_m\n0.150,24.000,1.000,24.00,0.04,30.00,0.0\n', 'LOG021.CSV')]);
check(mixed.length === 1 && mixed[0].s === null && mixed[0].t.length === 3, '5 列と 7 列のファイルをつないだ走行は、速さ・距離なしで扱う');

console.log('\n== 9. 描画用の間引き (2 時間分 = 14 万行)');
const N = 144000;
const T = new Float64Array(N), Y = new Float64Array(N);
for (let k = 0; k < N; k++) { T[k] = (k + 1) * 0.05; Y[k] = 5 + Math.sin(k / 500); }
Y[77777] = 19.5;            // 50ms だけのピーク
for (let k = 100000; k < N; k++) T[k] += 30;  // 30 秒の記録の抜け
const t0 = Date.now();
const d = C.decimate(T, Y, 0, N - 1, 800);
const ms = Date.now() - t0;
check(d.length <= 1600, `点の数 ${d.length} (≦ 横 800 × 2)`);
check(d.some((x) => x.y === 19.5), '50ms だけのピークも残る');
check(d.filter((x) => x.gap).length === 1, '記録の抜けの位置で線を切る (1 か所)');
check(ms < 200, `間引きにかかった時間 ${ms}ms`);

console.log('\n== 10. 表示用の小物');
check(C.fmtTime(0) === '00:00' && C.fmtTime(754.9) === '12:34' && C.fmtTime(3725) === '1:02:05', `時刻の表示 (${C.fmtTime(754.9)}, ${C.fmtTime(3725)})`);
const tk = C.niceTicks(0, 23.7, 5).ticks;
check(tk[0] === 0 && tk.length >= 4 && tk.length <= 7, `目盛り 0〜23.7 → ${tk.join(', ')}`);
check(C.logNumber('LOG012.CSV') === 12 && C.logNumber('foo.csv') > 1e9, 'ファイル番号の読み取り');

console.log(`\n${passed} PASS / ${failed} FAIL`);
process.exit(failed ? 1 : 0);
