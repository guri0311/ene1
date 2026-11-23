// Ene1PowerMeter の実機バイナリ (.hex) を ATmega328P エミュレーターで動かすテスト
//   cd test/avr_emu && npm install && node run.mjs [シナリオ名の一部]
// 必要なもの: arduino-cli (arduino:avr コアと SD ライブラリ入り)、mkfs.fat / fsck.fat (dosfstools)、mtools
import fs from 'fs';
import os from 'os';
import path from 'path';
import { execFileSync, spawnSync } from 'child_process';
import { fileURLToPath } from 'url';
import { Board, loadHex } from './board.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(HERE, '../..');
const WORK = fs.mkdtempSync(path.join(process.env.TMPDIR || os.tmpdir(), 'ene1_avr_'));
const FILTER = process.argv[2] || '';
process.env.MTOOLS_SKIP_CHECK = '1';

function findTool(name, candidates) {
  for (const c of candidates) if (c && fs.existsSync(c)) return c;
  return name;
}
const ARDUINO_CLI = findTool('arduino-cli', [process.env.ARDUINO_CLI, path.join(os.homedir(), 'bin/arduino-cli')]);
function avrNm() {
  const base = path.join(os.homedir(), '.arduino15/packages/arduino/tools/avr-gcc');
  for (const v of fs.existsSync(base) ? fs.readdirSync(base) : []) {
    const p = path.join(base, v, 'bin/avr-nm');
    if (fs.existsSync(p)) return p;
  }
  return 'avr-nm';
}

// ---------------------------------------------------------------------------
//  ファームウェアのビルド (実機に書き込むのと同じ .hex)
// ---------------------------------------------------------------------------
function build(sketchDir, buildDir, fqbn = 'arduino:avr:nano', withBootloader = false) {
  const r = spawnSync(ARDUINO_CLI, ['compile', '--fqbn', fqbn, '--build-path', buildDir, sketchDir], { encoding: 'utf8' });
  if (r.status !== 0) { console.error(r.stdout, r.stderr); process.exit(1); }
  const summary = r.stdout.split('\n').filter((l) => /Sketch uses|Global variables/.test(l)).join('\n');
  const elf = path.join(buildDir, 'Ene1PowerMeter.ino.elf');
  const syms = {};
  for (const line of execFileSync(avrNm(), [elf], { encoding: 'utf8' }).split('\n')) {
    const [addr, , name] = line.trim().split(/\s+/);
    if (name) syms[name] = parseInt(addr, 16);
  }
  return {
    program: loadHex(fs.readFileSync(path.join(buildDir, withBootloader ? 'Ene1PowerMeter.ino.with_bootloader.hex' : 'Ene1PowerMeter.ino.hex'), 'utf8')),
    symbols: { brkval: syms.__brkval & 0xffff, heapStart: syms.__heap_start & 0xffff },
    summary,
  };
}

// ---------------------------------------------------------------------------
//  SD カードのイメージ (FAT32、実際のカードと同じ形式)
// ---------------------------------------------------------------------------
let imgSeq = 0;
function newImage() {
  const img = path.join(WORK, `sd${imgSeq++}.img`);
  execFileSync('mkfs.fat', ['-C', '-F', '32', '-s', '8', '-n', 'ENE1', img, String(300 * 1024)], { stdio: 'ignore' });
  return img;
}
function sdFiles(img) {
  const out = spawnSync('mdir', ['-b', '-i', img, '::'], { encoding: 'utf8' }).stdout || '';
  return out.split('\n').map((l) => l.replace(/^::\/?/, '').trim()).filter(Boolean).sort();
}
function sdRead(img, name) {
  const r = spawnSync('mtype', ['-i', img, `::${name}`], { encoding: 'utf8' });
  return r.status === 0 ? r.stdout : null;
}
function sdAddFiles(img, n) {
  const dir = fs.mkdtempSync(path.join(WORK, 'files_'));
  const names = [];
  for (let k = 0; k < n; k++) {
    const nm = `LOG${String(k).padStart(3, '0')}.CSV`;
    fs.writeFileSync(path.join(dir, nm), 'time_s\n');
    names.push(path.join(dir, nm));
  }
  for (let k = 0; k < names.length; k += 100) execFileSync('mcopy', ['-i', img, ...names.slice(k, k + 100), '::'], { stdio: 'ignore' });
}
// Arduino の SD ライブラリは FAT32 の FSInfo (空きクラスタ数の目安) を更新しないので、
// fsck はいつも "Free cluster summary wrong" を出す。これは目安の値だけの差で中身には影響しないため除外し、
// それ以外の指摘 (ファイルサイズとクラスタの不一致、交差リンクなど) があれば問題とする
function fsck(img) {
  const r = spawnSync('fsck.fat', ['-n', img], { encoding: 'utf8' });
  const text = (r.stdout + r.stderr).trim();
  const issues = text.split('\n').filter((l) => !/^fsck\.fat|Free cluster summary wrong|Auto-correcting|Leaving filesystem unchanged|files, .* clusters$|^\s*$/.test(l));
  return { ok: r.status === 0 || issues.length === 0, text: issues.length ? issues.join(' / ') : 'OK', fsinfoOnly: r.status !== 0 && issues.length === 0 };
}

// ---------------------------------------------------------------------------
//  テストの道具
// ---------------------------------------------------------------------------
let passed = 0, failed = 0;
const failures = [];
function check(ok, msg) {
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${msg}`);
  if (ok) passed++; else { failed++; failures.push(msg); }
}
function showLcd(b) {
  console.log('  +--------------------+');
  for (const r of b.lcdRows()) console.log(`  |${r.padEnd(20)}|`);
  console.log('  +--------------------+');
}
function expectLcd(b, row, text) {
  const r = b.lcd.row(row);
  check(r.includes(text), `LCD ${row + 1}行目に "${text}" (実際: "${r}")`);
}
function expectNoLcd(b, row, text) {
  const r = b.lcd.row(row);
  check(!r.includes(text), `LCD ${row + 1}行目に "${text}" が無い (実際: "${r}")`);
}
function lcdCharge(b) {
  const m = b.lcd.row(1).match(/Q:\s*(-?\d+)mAh/);
  return m ? Number(m[1]) : NaN;
}
function expectCharge(b, lo, hi) {
  const q = lcdCharge(b);
  check(q >= lo && q <= hi, `LCD の積算 ${q} mAh が ${lo}〜${hi}`);
}
function expectSerial(b, re, what) {
  check(b.serial.some((l) => re.test(l)), `シリアルに ${what}`);
}
function csvRows(img, name) {
  const t = sdRead(img, name);
  if (t === null) return null;
  return t.split(/\r?\n/).filter(Boolean);
}
function expectClean(b) {
  const v = b.lcd.violations;
  check(v.busy === 0 && v.early === 0 && v.proto === 0,
    `LCD のタイミング違反なし (処理中 ${v.busy} / 40ms 以内 ${v.early} / 制御バイト ${v.proto})`);
  check(b.maxWdtGapUs < 8e6, `ウォッチドッグのリセット間隔 最大 ${(b.maxWdtGapUs / 1e6).toFixed(2)} 秒 (< 8 秒)`);
  check(b.resets.watchdog === 0, `ウォッチドッグによる再起動なし (${b.resets.watchdog} 回)`);
}

const scenarios = [];
function scenario(name, fn) { scenarios.push({ name, fn }); }

// スケッチの記録間隔・flush 間隔 (テストの期待値をこれに合わせる)
const SKETCH_SRC = fs.readFileSync(path.join(REPO, 'Ene1PowerMeter', 'Ene1PowerMeter.ino'), 'utf8');
const LOG_MS = Number(SKETCH_SRC.match(/LOG_INTERVAL_MS\s*=\s*(\d+)/)[1]);
const FLUSH_MS = Number(SKETCH_SRC.match(/LOG_FLUSH_MS\s*=\s*(\d+)/)[1]);
const PER_SEC = 1000 / LOG_MS;  // 1 秒あたりの行数
// CSV の時刻列の刻みを調べる (ms 単位)
function timeSteps(rows) {
  const t = rows.map((r) => Math.round(Number(r.split(',')[0]) * 1000));
  return t.slice(1).map((v, k) => v - t[k]);
}

let fw, fwOptiboot, fwOldBoot;
// ブートローダの開始番地 (ワード単位)。ヒューズ 0xDA の Nano: Optiboot は 0x7E00、旧ブートローダは 0x7800
const BOOT = { optiboot: 0x7e00 / 2, old: 0x7800 / 2 };
function newBoard(opts = {}) {
  const img = opts.img || newImage();
  if (opts.files) sdAddFiles(img, opts.files);
  const f = opts.boot === 'optiboot' ? fwOptiboot : opts.boot === 'old' ? fwOldBoot : fw;
  const b = new Board({ program: f.program, symbols: f.symbols, sdImage: img, lcdAddr: opts.lcdAddr, echoSerial: true,
    bootWord: opts.boot ? BOOT[opts.boot] : null });
  b.img = img;
  if (opts.noIna) b.ina.present = false;
  if (opts.noCard) b.sd.remove();
  b.ina.setVoltage(opts.v ?? 24);
  if (opts.i !== undefined) b.ina.setCurrent(opts.i);
  if (opts.button) b.setButton(true);
  return b;
}
function hold(b, ms) { b.setButton(true); b.run(ms); b.setButton(false); }

// ---------------------------------------------------------------------------
//  シナリオ
// ---------------------------------------------------------------------------
scenario('起動・表示・記録 (24V、10A を 36 秒 → 100mAh)', () => {
  const b = newBoard();
  b.run(1200);
  showLcd(b);
  expectLcd(b, 0, 'Ene-1 PowerMeter');
  expectLcd(b, 1, 'INA228 OK');
  expectLcd(b, 2, 'SD OK');
  expectLcd(b, 3, 'LOG000.CSV');
  b.run(2000);
  expectSerial(b, /I2C scan: 0x3C\(LCD\?\) 0x40\(INA228\)/, 'I2C スキャン結果 (0x3C と 0x40)');
  expectSerial(b, /^LCD: 0x3C OK/, '"LCD: 0x3C OK"');
  expectSerial(b, /^INA228: configured/, '"INA228: configured"');
  expectSerial(b, /^SHUNT_CAL=4000$/, '"SHUNT_CAL=4000"');
  expectSerial(b, /^SD: LOG000.CSV/, '"SD: LOG000.CSV"');
  expectLcd(b, 0, 'V: 24.00V  I:  0.00A');
  expectLcd(b, 3, 'REC LOG000.CSV');
  b.ina.setCurrent(10);
  b.run(36000);
  showLcd(b);
  expectLcd(b, 0, 'V: 24.00V  I: 10.00A');
  expectLcd(b, 1, 'P: 240W');
  expectCharge(b, 99, 101);
  expectLcd(b, 2, 'TIME  00:3');
  const rows = csvRows(b.img, 'LOG000.CSV');
  check(rows && rows[0] === 'time_s,voltage_V,current_A,power_W,charge_mAh', `CSV の見出し行 (実際: "${rows && rows[0]}")`);
  const data = rows ? rows.slice(1) : [];
  const lo = Math.floor((36000 - FLUSH_MS) / LOG_MS) - 2, hi = Math.ceil(37200 / LOG_MS) + 2;
  check(data.length >= lo && data.length <= hi, `CSV のデータ行数 ${data.length} が ${lo}〜${hi} (${LOG_MS}ms ごと、flush は ${FLUSH_MS}ms ごと)`);
  const steps = timeSteps(data);
  const sMin = Math.min(...steps), sMax = Math.max(...steps);
  const avg = steps.reduce((a, v) => a + v, 0) / steps.length;
  console.log(`  [記録間隔] 平均 ${avg.toFixed(2)}ms / 最短 ${sMin}ms / 最長 ${sMax}ms (設定 ${LOG_MS}ms)`);
  check(Math.abs(avg - LOG_MS) < 1, `CSV の時刻の刻みが平均 ${LOG_MS}ms (${avg.toFixed(2)}ms)`);
  check(sMax < LOG_MS * 1.5 && sMin > LOG_MS * 0.5, `行の抜けなし (最短 ${sMin}ms / 最長 ${sMax}ms)`);
  const vals = new Set(data.slice(-20).map((r) => r.split(',')[4]));
  check(vals.size >= 15, `どの行も新しい測定値 (最後の 20 行で積算値が ${vals.size} 種類)`);
  const last = data[data.length - 1] || '';
  check(/^\d+\.\d{3},24\.000,10\.000,240\.00,\d+\.\d{2}$/.test(last), `CSV の最終行の形式 (実際: "${last}")`);
  const fk = fsck(b.img);
  check(fk.ok, `FAT の整合性 (fsck.fat -n): ${fk.text}`);
  expectClean(b);
  const wdt = b.cpu.data[0x60];
  check((wdt & 0x29) === 0x29, `ウォッチドッグが 8 秒で有効 (WDTCSR=0x${wdt.toString(16)})`);
  const margin = b.minSp - b.heapTop();
  console.log(`  [RAM] スタック最深 0x${b.minSp.toString(16)} / ヒープ上端 0x${b.heapTop().toString(16)} → 余裕 ${margin} バイト`);
  check(margin > 200, `スタックとヒープの間に 200 バイト以上の余裕 (${margin} バイト)`);
});

scenario('逆向きの電流と 20A 超え (OVER)', () => {
  const b = newBoard({ i: -2 });
  b.run(5000);
  expectLcd(b, 0, 'I: -2.00A');
  expectLcd(b, 1, 'P:  48W');
  b.ina.setCurrent(19.9);
  b.run(1000);
  expectLcd(b, 0, 'I: 19.90A');
  b.ina.setCurrent(25);
  b.run(1000);
  showLcd(b);
  expectLcd(b, 0, 'I:  OVER');
  b.ina.setCurrent(-25);
  b.run(1000);
  expectLcd(b, 0, 'I:  OVER');
  expectClean(b);
});

scenario('ボタン: 3 秒未満は何もしない、3 秒以上で離すとリセット、押しっぱなしは無視', () => {
  const b = newBoard({ i: 10 });
  b.run(20000);
  const q0 = lcdCharge(b);
  hold(b, 300); b.run(500);
  hold(b, 2500); b.run(500);
  check(lcdCharge(b) >= q0, `3 秒未満の押下では積算が消えない (${q0} → ${lcdCharge(b)} mAh)`);
  expectLcd(b, 3, 'REC LOG000.CSV');
  b.setButton(true);
  b.run(2500);
  expectNoLcd(b, 3, 'RELEASE');
  b.run(1000);
  expectLcd(b, 3, 'RELEASE=RESET mAh');
  b.setButton(false);
  b.run(1500);
  showLcd(b);
  expectCharge(b, 0, 5);
  expectLcd(b, 2, 'TIME  00:00');
  expectLcd(b, 3, 'REC LOG001.CSV');
  expectSerial(b, /^# mAh reset/, '"# mAh reset"');
  b.run(20000);
  const q1 = lcdCharge(b);
  // リセットのあとも積算が再開する (RSTACC が 1 のまま残ると 0 で止まる)。10A x 20 秒 ≒ 56mAh
  check(q1 >= 50 && q1 <= 63, `リセット後に積算が再開する (${q1} mAh が 50〜63)`);
  b.setButton(true);
  b.run(12000);
  expectLcd(b, 3, 'BUTTON STUCK?');
  b.setButton(false);
  b.run(1500);
  check(lcdCharge(b) >= q1, `押しっぱなし (12 秒) ではリセットしない (${q1} → ${lcdCharge(b)} mAh)`);
  expectLcd(b, 3, 'REC LOG001.CSV');
  const rows = csvRows(b.img, 'LOG001.CSV');
  check(rows && rows.length > 20 * PER_SEC, `LOG001.CSV に記録が続いている (${rows && rows.length} 行)`);
  check(JSON.stringify(sdFiles(b.img)) === JSON.stringify(['LOG000.CSV', 'LOG001.CSV']), `SD のファイル: ${sdFiles(b.img).join(' ')}`);
  expectClean(b);
});

scenario('走行中に SD が抜ける → NO SD、挿し直しても初期化しない、表示と積算は続く', () => {
  const b = newBoard({ i: 5 });
  b.run(20000);
  b.sd.remove();
  let tDetect = 0;
  while (!b.lcd.row(3).includes('NO SD') && tDetect < 30000) { b.run(500); tDetect += 500; }
  showLcd(b);
  expectLcd(b, 3, 'NO SD');
  check(tDetect <= 6000, `抜けてから NO SD 表示まで ${(tDetect / 1000).toFixed(1)} 秒 (6 秒以内)`);
  expectLcd(b, 0, 'I:  5.00A');
  const q = lcdCharge(b);
  b.sd.insert();
  b.run(3000);
  hold(b, 300);
  b.run(2000);
  expectLcd(b, 3, 'NO SD');
  check(lcdCharge(b) > q, `積算は続く (${q} → ${lcdCharge(b)} mAh)`);
  const rows = csvRows(b.img, 'LOG000.CSV');
  check(rows && rows.length >= 15 * PER_SEC, `抜く前までの記録が残っている (${rows && rows.length} 行)`);
  const fk = fsck(b.img);
  console.log(`  [fsck] ${fk.text}`);
  expectClean(b);
});

scenario('SD なしで起動 → 表示は続く', () => {
  const b = newBoard({ i: 3, noCard: true });
  b.run(6000);
  showLcd(b);
  expectLcd(b, 0, 'I:  3.00A');
  expectLcd(b, 3, 'NO SD');
  expectSerial(b, /^SD: NO SD/, '"SD: NO SD"');
  expectClean(b);
});

scenario('INA228 だけ電源瞬断 → 再設定して積算を引き継ぐ', () => {
  const b = newBoard({ i: 10 });
  b.run(20000);
  const q = lcdCharge(b);
  b.ina.powerOn();
  b.run(18000);
  showLcd(b);
  expectLcd(b, 2, 'INA!1');
  expectSerial(b, /INA228 re-configured, ok=1/, 'INA228 の再設定');
  expectCharge(b, q + 45, q + 56);
  expectClean(b);
});

scenario('INA228 の配線が外れる → エラー表示、戻れば復帰', () => {
  const b = newBoard({ i: 10 });
  b.run(8000);
  b.ina.present = false;
  b.run(3000);
  showLcd(b);
  expectLcd(b, 0, 'INA228 I2C ERROR');
  b.ina.present = true;
  b.run(2000);
  expectLcd(b, 0, 'I: 10.00A');
  expectClean(b);
});

scenario('LCD がノイズで化ける → 5 秒以内に戻る', () => {
  const b = newBoard({ i: 1 });
  b.run(5000);
  b.lcd.noise();
  b.run(5500);
  showLcd(b);
  expectLcd(b, 0, 'V: 24.00V  I:  1.00A');
  expectLcd(b, 3, 'REC LOG000.CSV');
  expectClean(b);
});

scenario('Nano だけリセット → 積算と時間を引き継ぐ / 電源を入れ直すと 0', () => {
  const b = newBoard({ i: 10 });
  b.run(40000);
  const q = lcdCharge(b);
  const t = b.lcd.row(2);
  b.resetMcu();
  b.run(1200);
  showLcd(b);
  expectLcd(b, 1, 'RESUMED (mAh KEPT)');
  b.run(2500);
  showLcd(b);
  expectSerial(b, /^INA228: resumed/, '"INA228: resumed (mAh kept)"');
  check(lcdCharge(b) >= q && lcdCharge(b) <= q + 15, `積算を引き継ぐ (${q} → ${lcdCharge(b)} mAh)`);
  expectLcd(b, 2, 'TIME  00:4');
  console.log(`  [時間] リセット前 "${t.trim()}" → 後 "${b.lcd.row(2).trim()}"`);
  expectLcd(b, 3, 'REC LOG001.CSV');
  b.ina.setCurrent(0);
  b.powerCycle();
  b.run(3500);
  showLcd(b);
  expectCharge(b, 0, 2);
  expectLcd(b, 2, 'TIME  00:0');
  expectLcd(b, 3, 'REC LOG002.CSV');
  expectClean(b);
});

scenario('INA228 が見つからない → エラー表示で停止 (再起動を繰り返さない)', () => {
  const b = newBoard({ noIna: true });
  b.run(20000);
  showLcd(b);
  expectLcd(b, 0, 'INA228 NOT FOUND');
  expectLcd(b, 3, 'STOPPED');
  check(b.resets.watchdog === 0, `ウォッチドッグによる再起動なし (${b.resets.watchdog} 回)`);
});

scenario('LCD のアドレスが 0x3D でも自動で見つける', () => {
  const b = newBoard({ lcdAddr: 0x3d });
  b.run(3000);
  showLcd(b);
  expectSerial(b, /^LCD: 0x3D OK/, '"LCD: 0x3D OK"');
  expectLcd(b, 0, 'V: 24.00V');
  expectClean(b);
});

scenario('SD にログが 300 個ある → 起動が遅れず、次の番号で記録', () => {
  const b = newBoard({ files: 300 });
  b.run(4000);
  showLcd(b);
  expectLcd(b, 3, 'REC LOG300.CSV');
  check(b.sd.stats.reads < 2000, `起動中の SD 読み込み ${b.sd.stats.reads} ブロック`);
  expectClean(b);
});

scenario('記録中に電源が落ちる → FAT は壊れず、flush 済みの行は読める', () => {
  const b = newBoard({ i: 7 });
  b.run(23300);
  // ここで電源断 (エミュレーターを止める)
  const fk = fsck(b.img);
  console.log(`  [fsck] ${fk.text}`);
  check(fk.ok, `FAT が壊れていない (${fk.text})`);
  const rows = csvRows(b.img, 'LOG000.CSV');
  check(rows && rows.length - 1 >= 15 * PER_SEC, `flush 済みのデータ行が残っている (${rows ? rows.length - 1 : 0} 行)`);
});

scenario('長時間 (15A で 10 分 → 2500mAh) と RAM の実測', () => {
  const b = newBoard({ i: 15 });
  b.run(600000);
  showLcd(b);
  expectCharge(b, 2495, 2505);
  check(/TIME  (09:5[6-9]|10:0)/.test(b.lcd.row(2)), `LCD 3行目が約 10 分 (起動中の約 2 秒を除く) (実際: "${b.lcd.row(2)}")`);
  const ram = b.serial.filter((l) => l.startsWith('# RAM min free:'));
  console.log(`  [RAM] ファームウェアの自己計測: ${ram[ram.length - 1] || '(なし)'}`);
  const m = ram.length ? Number(ram[ram.length - 1].match(/(\d+) bytes/)[1]) : 0;
  check(m > 200, `ファームウェアが測った最小空き RAM ${m} バイト (> 200)`);
  const margin = b.minSp - b.heapTop();
  console.log(`  [RAM] エミュレーターが測ったスタックとヒープの余裕 ${margin} バイト`);
  const rows = csvRows(b.img, 'LOG000.CSV');
  check(rows && rows.length >= 595 * PER_SEC, `10 分ぶんの記録 (${rows && rows.length} 行)`);
  const fk = fsck(b.img);
  check(fk.ok, `FAT の整合性 (fsck.fat -n): ${fk.text}`);
  expectClean(b);
});

scenario('I2C が固まる (ノイズ) → Wire のタイムアウトで復帰し、再起動しない', () => {
  const b = newBoard({ i: 10 });
  b.run(8000);
  for (let k = 0; k < 5; k++) { b.i2cHangCount = 1; b.run(700); }
  b.run(3000);
  showLcd(b);
  expectLcd(b, 0, 'I: 10.00A');
  expectLcd(b, 3, 'REC LOG000.CSV');
  expectClean(b);
});

// 想定外のハング (SPI 転送が終わらない) を起こし、ウォッチドッグからの復帰を確かめる
function hangAndRecover(b, label) {
  b.run(20000);
  const q = lcdCharge(b);
  b.spiHang = true;
  let t = 0, sawResume = false;
  while (t < 20000) {
    b.run(250); t += 250;
    if (b.lcd.row(1).includes('RESUMED')) sawResume = true;
    if (b.resets.watchdog > 0 && sawResume && t > 2000 && b.lcd.row(3).includes('REC')) break;
  }
  b.run(3000);
  showLcd(b);
  console.log(`  [${label}] ハングから ${(t / 1000).toFixed(1)} 秒で復帰、ウォッチドッグ再起動 ${b.resets.watchdog} 回`);
  check(b.resets.watchdog === 1, `ウォッチドッグが 1 回だけ働いた (${b.resets.watchdog} 回)`);
  check(sawResume, '"RESUMED (mAh KEPT)" を表示した');
  check(lcdCharge(b) >= q, `積算を引き継いだ (${q} → ${lcdCharge(b)} mAh)`);
  expectLcd(b, 3, 'REC LOG001.CSV');
  check(b.lcd.violations.busy === 0 && b.lcd.violations.early === 0, 'LCD のタイミング違反なし');
}

scenario('ハング → 8 秒でウォッチドッグ → 再開 (ブートローダなし、ISP 書き込み相当)', () => {
  hangAndRecover(newBoard({ i: 10 }), 'ブートローダなし');
});

scenario('ハング → ウォッチドッグ → Optiboot を通って再開 (Nano の標準)', () => {
  const b = newBoard({ i: 10, boot: 'optiboot' });
  hangAndRecover(b, 'Optiboot');
});

scenario('Optiboot の Nano でリセットボタン → ブートローダ待ちのあと再開、積算を引き継ぐ', () => {
  const b = newBoard({ i: 10, boot: 'optiboot' });
  b.run(20000);
  const q = lcdCharge(b);
  b.resetMcu();
  let t = 0;
  while (t < 10000 && !b.lcd.row(1).includes('RESUMED')) { b.run(100); t += 100; }
  console.log(`  [Optiboot] リセットから RESUMED 表示まで ${(t / 1000).toFixed(1)} 秒`);
  b.run(3000);
  showLcd(b);
  check(lcdCharge(b) >= q, `積算を引き継いだ (${q} → ${lcdCharge(b)} mAh)`);
  expectLcd(b, 3, 'REC LOG001.CSV');
});

scenario('旧ブートローダの Nano でハング → ウォッチドッグ後の動き', () => {
  const b = newBoard({ i: 10, boot: 'old' });
  b.run(20000);
  expectLcd(b, 0, 'I: 10.00A');
  b.spiHang = true;
  b.run(15000);
  const n1 = b.resets.watchdog;
  b.run(5000);
  const n2 = b.resets.watchdog;
  showLcd(b);
  const loop = n2 > n1;
  console.log(`  [旧ブートローダ] ウォッチドッグ再起動 ${n2} 回 (最後の 5 秒で ${n2 - n1} 回) → ${loop ? '再起動を繰り返している' : '再起動ループなし'}`);
  check(true, `旧ブートローダでの挙動を記録: ${loop ? '再起動ループになる (電源の入れ直しが必要)' : '1 回の再起動で復帰'}`);
  b.oldBootLoop = loop;
  globalThis.oldBootLoop = loop;
});

scenario('ボタンのチャタリング: 押す・離す瞬間のばたつきでもリセットは 1 回、短いノイズは無視', () => {
  const b = newBoard({ i: 10 });
  b.run(15000);
  const chatter = (endPressed) => { for (let k = 0; k < 8; k++) { b.setButton(k % 2 === 0); b.run(2); } b.setButton(endPressed); };
  // 5ms 程度の LOW ノイズを 20 回
  const q0 = lcdCharge(b);
  for (let k = 0; k < 20; k++) { b.setButton(true); b.run(5); b.setButton(false); b.run(300); }
  check(lcdCharge(b) >= q0, `短いノイズではリセットしない (${q0} → ${lcdCharge(b)} mAh)`);
  chatter(true); b.run(3500); chatter(false); b.run(1500);
  showLcd(b);
  expectCharge(b, 0, 6);
  expectLcd(b, 3, 'REC LOG001.CSV');
  b.run(5000);
  check(JSON.stringify(sdFiles(b.img)) === JSON.stringify(['LOG000.CSV', 'LOG001.CSV']), `リセットは 1 回だけ (SD のファイル: ${sdFiles(b.img).join(' ')})`);
  expectClean(b);
});

scenario('SD カードの書き込みが遅い (毎回 250ms のビジー) → 記録も表示も止まらない', () => {
  const b = newBoard({ i: 6 });
  // ビジー 1 バイトの確認に約 4.9us かかる (SPI 4MHz + ループ) ので 51000 バイトで約 250ms。
  // SD 規格の書き込みビジー上限は 250ms。SD ライブラリのタイムアウトは 600ms (約 123000 バイト)
  b.sd.writeBusyBytes = 51000;
  b.run(40000);
  showLcd(b);
  expectLcd(b, 3, 'REC LOG000.CSV');
  expectCharge(b, 60, 70);
  const rows = csvRows(b.img, 'LOG000.CSV');
  const data = rows ? rows.slice(1) : [];
  const steps = timeSteps(data);
  const sMax = Math.max(...steps);
  const late = steps.filter((v) => v > LOG_MS * 1.5).length;
  console.log(`  [遅いカード] ${data.length} 行 / 最長の間隔 ${sMax}ms / 間隔が ${LOG_MS * 1.5}ms を超えた所 ${late} か所`);
  check(data.length - 1 >= 33, `記録が続いている (${data.length} 行)`);
  check(sMax < 1000, `書き込みが遅くても 1 秒以上の抜けはない (最長 ${sMax}ms)`);
  const fk = fsck(b.img);
  check(fk.ok, `FAT の整合性: ${fk.text}`);
  expectClean(b);
});

scenario('LCD が外れている → 記録は続く', () => {
  const b = newBoard({ i: 4 });
  b.lcd.present = false;
  b.run(15000);
  expectSerial(b, /^LCD: 0x3C no ACK/, '"LCD: 0x3C no ACK"');
  const rows = csvRows(b.img, 'LOG000.CSV');
  check(rows && rows.length - 1 >= 8 * PER_SEC, `LCD がなくても記録している (${rows ? rows.length - 1 : 0} 行)`);
  check(b.resets.watchdog === 0, 'ウォッチドッグによる再起動なし');
});

// ---------------------------------------------------------------------------
console.log('ファームウェアをビルド中...');
fw = build(path.join(REPO, 'Ene1PowerMeter'), path.join(WORK, 'build'));
console.log(fw.summary);
fwOptiboot = build(path.join(REPO, 'Ene1PowerMeter'), path.join(WORK, 'build_opti'), 'arduino:avr:nano:cpu=atmega328', true);
fwOldBoot = build(path.join(REPO, 'Ene1PowerMeter'), path.join(WORK, 'build_old'), 'arduino:avr:nano:cpu=atmega328old', true);
let fwNoSerial = null;
if (!FILTER || FILTER === 'release') {
  const dir = path.join(WORK, 'release', 'Ene1PowerMeter');
  fs.mkdirSync(dir, { recursive: true });
  const src = fs.readFileSync(path.join(REPO, 'Ene1PowerMeter/Ene1PowerMeter.ino'), 'utf8').replace(/^#define SERIAL_DEBUG 1/m, '#define SERIAL_DEBUG 0');
  fs.writeFileSync(path.join(dir, 'Ene1PowerMeter.ino'), src);
  fwNoSerial = build(dir, path.join(WORK, 'build_release'));
  console.log('SERIAL_DEBUG 0 版: ' + fwNoSerial.summary.replace(/\n/g, ' / '));
}
const t0 = Date.now();
for (const s of scenarios) {
  if (FILTER && !s.name.includes(FILTER)) continue;
  console.log(`\n== ${s.name}`);
  try { s.fn(); } catch (e) { check(false, `例外: ${e.stack}`); }
}
if (fwNoSerial) {
  console.log('\n== SERIAL_DEBUG 0 (シリアル出力なし) の版: 表示・記録・リセット・SD 抜け');
  const b = new Board({ program: fwNoSerial.program, symbols: fwNoSerial.symbols, sdImage: newImage(), echoSerial: false });
  b.img = b.sd ? b.img : null;
  b.ina.setVoltage(24); b.ina.setCurrent(10);
  b.run(20000);
  showLcd(b);
  expectLcd(b, 0, 'V: 24.00V  I: 10.00A');
  check(b.serial.length === 0, `シリアル出力なし (${b.serial.length} 行)`);
  hold(b, 3500); b.run(1500);
  expectCharge(b, 0, 6);
  expectLcd(b, 3, 'REC LOG001.CSV');
  b.sd.remove(); b.run(7000);
  expectLcd(b, 3, 'NO SD');
  expectClean(b);
}
console.log(`\n${passed} PASS / ${failed} FAIL  (${((Date.now() - t0) / 1000).toFixed(0)} 秒)`);
if (failed) { console.log('失敗:'); for (const f of failures) console.log('  - ' + f); }
fs.rmSync(WORK, { recursive: true, force: true });
process.exit(failed ? 1 : 0);
