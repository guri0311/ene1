// 仮想ボードにつなぐ部品のモデル (データシートの仕様どおりに応答する)
//   INA228 (I2C 0x40) / RW1063 LCD (I2C 0x3C〜0x3F) / microSD カード (SPI モード)
import fs from 'fs';

// ---------------------------------------------------------------------------
//  I2C バス: avr8js の TWI イベントを部品に配り、100kHz 相当の時間をかけて応答する
// ---------------------------------------------------------------------------
export class I2CBus {
  constructor(board, cpu, twi) {
    this.board = board;
    this.cpu = cpu;
    this.twi = twi;
    this.target = null;
  }
  get bitCycles() { return Math.max(1, Math.round(16e6 / this.twi.sclFrequency)); }
  later(fn, bits) {
    if (this.board.i2cHangCount > 0) { this.board.i2cHangCount--; return; }  // 故障注入: 応答しない
    this.cpu.addClockEvent(fn, this.bitCycles * bits);
  }
  start() {
    this.target = null;
    this.later(() => this.twi.completeStart(), 1);
  }
  stop() {
    if (this.target && this.target.stop) this.target.stop();
    this.target = null;
    this.later(() => this.twi.completeStop(), 1);
  }
  connectToSlave(addr, write) {
    const dev = this.board.i2cDevices.find((d) => d.present && d.addr === addr && d.accepts(write));
    this.target = dev || null;
    if (dev) dev.begin(write);
    this.later(() => this.twi.completeConnect(!!dev), 9);
  }
  writeByte(v) {
    const ack = this.target ? this.target.writeByte(v) !== false : false;
    this.later(() => this.twi.completeWrite(ack), 9);
  }
  readByte() {
    const v = this.target ? this.target.readByte() : 0xff;
    this.later(() => this.twi.completeRead(v), 9);
  }
}

// ---------------------------------------------------------------------------
//  INA228
// ---------------------------------------------------------------------------
const CT_US = [50, 84, 150, 280, 540, 1052, 2074, 4120];
const AVG_N = [1, 4, 16, 64, 128, 256, 512, 1024];

export class INA228 {
  constructor(board) {
    this.board = board;
    this.addr = 0x40;
    this.present = true;
    this.v = 0;          // 実際の電池電圧 [V]
    this.i = 0;          // 実際の電流 [A]
    this.rShunt = 0.002; // 実際のシャント抵抗 [Ω]
    this.powerOn();
  }
  powerOn() {
    this.config = 0x0000;
    this.adcConfig = 0xfb68;
    this.shuntCal = 0x1000;
    this.charge = 0;         // CHARGE レジスタ (端数込み)
    this.latched = { vbus: 0, cur: 0 };
    this.lastConvUs = this.board.timeUs;
    this.ptr = 0;
  }
  accepts() { return true; }
  convPeriodUs() {
    const mode = this.adcConfig >> 12;
    const vbusct = CT_US[(this.adcConfig >> 9) & 7], vshct = CT_US[(this.adcConfig >> 6) & 7], vtct = CT_US[(this.adcConfig >> 3) & 7];
    const n = AVG_N[this.adcConfig & 7];
    let t = 0;
    if (mode & 1) t += vbusct;
    if (mode & 2) t += vshct;
    if (mode & 4) t += vtct;
    return Math.max(1, t) * n;
  }
  currentCounts() {
    if (this.shuntCal === 0) return 0;
    const low = (this.config & 0x0010) !== 0;
    const fs = low ? 0.04096 : 0.16384, k = low ? 4 : 1;
    let vsh = this.i * this.rShunt;
    vsh = Math.max(-fs, Math.min(fs, vsh));
    const c = Math.round((vsh * 13107.2e6 * k) / this.shuntCal);
    return Math.max(-524288, Math.min(524287, c));
  }
  // 変換周期ごとに結果を確定し、CHARGE を積算する (連続測定モード)
  update() {
    const now = this.board.timeUs;
    const mode = this.adcConfig >> 12;
    if (mode < 8) { this.lastConvUs = now; return; }  // 連続測定以外は扱わない
    const period = this.convPeriodUs();
    const n = Math.floor((now - this.lastConvUs) / period);
    if (n <= 0) return;
    const cur = (mode & 2) ? this.currentCounts() : 0;
    this.latched = { vbus: Math.round(this.v / 195.3125e-6), cur };
    this.charge += cur * n * (period / 1e6);
    this.lastConvUs += n * period;
  }
  setCurrent(a) { this.update(); this.i = a; }
  setVoltage(v) { this.update(); this.v = v; }
  regBytes(reg) {
    this.update();
    const be = (val, n) => { const b = []; for (let k = n - 1; k >= 0; k--) b.push(Number((BigInt(val) >> BigInt(8 * k)) & 0xffn)); return b; };
    switch (reg) {
      case 0x00: return be(this.config, 2);
      case 0x01: return be(this.adcConfig, 2);
      case 0x02: return be(this.shuntCal, 2);
      case 0x05: return be((this.latched.vbus & 0xfffff) << 4, 3);
      case 0x07: return be(((this.latched.cur & 0xfffff) << 4) >>> 0, 3);
      case 0x08: {
        const vbusV = this.latched.vbus * 195.3125e-6;
        let p = Math.round((vbusV * Math.abs(this.latched.cur)) / 3.2);
        return be(Math.min(p, 0xffffff), 3);
      }
      case 0x0a: {
        let q = BigInt(Math.round(this.charge));
        if (q < 0n) q += 1n << 40n;
        return be(q & 0xffffffffffn, 5);
      }
      case 0x3e: return be(0x5449, 2);
      case 0x3f: return be(0x2281, 2);
      default: return be(0, 2);
    }
  }
  begin(write) {
    if (write) { this.wbuf = []; }
    else { this.rbuf = this.regBytes(this.ptr); }
  }
  writeByte(v) { this.wbuf.push(v); return true; }
  readByte() { return this.rbuf.length ? this.rbuf.shift() : 0xff; }
  stop() {
    if (!this.wbuf || this.wbuf.length === 0) return;
    this.ptr = this.wbuf[0];
    if (this.wbuf.length >= 3) {
      this.update();
      const v = (this.wbuf[1] << 8) | this.wbuf[2];
      if (this.ptr === 0x00) {
        if (v & 0x8000) { this.powerOn(); this.ptr = 0; }
        else { if (v & 0x4000) this.charge = 0; this.config = v & 0x3fff; }
      } else if (this.ptr === 0x01) { this.adcConfig = v; this.lastConvUs = this.board.timeUs; }
      else if (this.ptr === 0x02) this.shuntCal = v & 0x7fff;
    }
    this.wbuf = [];
  }
}

// ---------------------------------------------------------------------------
//  RW1063 (ACM2004D-FLW-FBW-IIC)
//    制御バイト Co/A0 を解釈し、処理中に届いた命令や電源投入直後の命令を違反として数える
// ---------------------------------------------------------------------------
export class RW1063 {
  constructor(board, addr = 0x3c) {
    this.board = board;
    this.addr = addr;
    this.present = true;
    this.violations = { busy: 0, early: 0, proto: 0 };
    this.powerOn();
  }
  powerOn() {
    this.ddram = new Uint8Array(128).fill(0x20);
    this.ac = 0;
    this.dl = true; this.n = false; this.f = false; this.on = false;  // 内部リセット後
    this.busyUntil = 0;
    this.powerOnUs = this.board.timeUs;
  }
  accepts(write) { return write; }  // R/W は常に 0 (読み出しは NACK)
  begin() { this.expectCtrl = true; this.co = false; this.a0 = false; }
  writeByte(v) {
    if (this.expectCtrl) {
      if (v & 0x3f) this.violations.proto++;
      this.co = !!(v & 0x80); this.a0 = !!(v & 0x40);
      this.expectCtrl = false;
      return true;
    }
    this.exec(this.a0, v);
    if (this.co) this.expectCtrl = true;
    return true;
  }
  readByte() { return 0xff; }
  incAc() {
    if (this.n) this.ac = this.ac === 0x27 ? 0x40 : this.ac === 0x67 ? 0x00 : this.ac + 1;
    else this.ac = (this.ac + 1) % 80;
  }
  exec(isData, v) {
    const t = this.board.timeUs;
    if (t < this.powerOnUs + 40000) this.violations.early++;
    if (t < this.busyUntil) { this.violations.busy++; return; }  // 処理中に届いた命令は失われる
    const scale = 540 / 380;  // 発振周波数が最小 380kHz のとき
    let execUs = 18.5 * scale;
    if (isData) { this.ddram[this.ac & 0x7f] = v; this.incAc(); }
    else if (v === 0x01) { this.ddram.fill(0x20); this.ac = 0; execUs = 760 * scale; }
    else if ((v & 0xfe) === 0x02) { this.ac = 0; execUs = 760 * scale; }
    else if ((v & 0xfc) === 0x04) { /* entry mode */ }
    else if ((v & 0xf8) === 0x08) { this.on = !!(v & 0x04); }
    else if ((v & 0xf0) === 0x10) { /* shift */ }
    else if ((v & 0xe0) === 0x20) { this.dl = !!(v & 0x10); this.n = !!(v & 0x08); this.f = !!(v & 0x04); }
    else if ((v & 0xc0) === 0x40) { /* CGRAM */ }
    else { this.ac = v & 0x7f; }
    this.busyUntil = t + execUs;
  }
  noise() { this.on = false; this.n = false; this.ddram.fill(0x23); }
  row(r) {
    if (!this.on) return '(表示OFF)';
    if (!this.n) return '(1行モード)';
    const base = [0x00, 0x40, 0x14, 0x54][r];
    let s = '';
    for (let c = 0; c < 20; c++) s += String.fromCharCode(this.ddram[base + c]);
    return s;
  }
}

// ---------------------------------------------------------------------------
//  microSD カード (SPI モード、SDHC)。中身は FAT イメージファイル
// ---------------------------------------------------------------------------
export class SdCard {
  constructor(board, imagePath) {
    this.board = board;
    this.fd = fs.openSync(imagePath, 'r+');
    this.blocks = Math.floor(fs.fstatSync(this.fd).size / 512);
    this.inserted = true;
    this.writeBusyBytes = 200;  // 書き込み後のビジー (0x00) を返すバイト数
    this.stats = { reads: 0, writes: 0, cmds: 0 };
    this.powerUp();
  }
  powerUp() { this.state = 'idle'; this.acmdCount = 0; this.appCmd = false; this.busy = 0; this.deselect(); }
  deselect() { this.mode = 'cmd'; this.cmd = []; this.out = []; }  // ビジーは CS と関係なく続く
  remove() { this.inserted = false; this.deselect(); }
  insert() { this.inserted = true; this.powerUp(); }  // 挿し直すとカードは初期化前の状態に戻る
  close() { fs.closeSync(this.fd); }
  r1() { return this.state === 'ready' ? 0x00 : 0x01; }
  transfer(mosi, selected) {
    if (!this.inserted) return 0xff;
    if (!selected) { this.deselect(); return 0xff; }
    let miso = 0xff;
    if (this.out.length) miso = this.out.shift();
    else if (this.busy > 0) { this.busy--; miso = 0x00; }  // 書き込み中 (ビジー)
    if (this.mode === 'wtoken') {
      if (mosi === 0xfe) { this.mode = 'wdata'; this.wbuf = []; }
    } else if (this.mode === 'wdata') {
      this.wbuf.push(mosi);
      if (this.wbuf.length === 514) {  // データ 512 + CRC 2
        const buf = Buffer.from(this.wbuf.slice(0, 512));
        fs.writeSync(this.fd, buf, 0, 512, this.wblock * 512);
        this.stats.writes++;
        this.out = [0xe5];                 // データ受理
        this.busy = this.writeBusyBytes;   // そのあとビジー (0x00) を返す
        this.mode = 'cmd';
      }
    } else if (this.cmd.length > 0 || (mosi & 0xc0) === 0x40) {
      this.cmd.push(mosi);
      if (this.cmd.length === 6) { this.command(this.cmd[0] & 0x3f, ((this.cmd[1] << 24) | (this.cmd[2] << 16) | (this.cmd[3] << 8) | this.cmd[4]) >>> 0); this.cmd = []; }
    }
    return miso;
  }
  command(cmd, arg) {
    this.stats.cmds++;
    const app = this.appCmd;
    this.appCmd = false;
    const resp = (...b) => { this.out = [0xff, ...b]; };
    if (cmd === 0) { this.state = 'idle'; this.acmdCount = 0; return resp(0x01); }
    if (cmd === 8) return resp(this.r1(), 0x00, 0x00, 0x01, arg & 0xff);
    if (cmd === 55) { this.appCmd = true; return resp(this.r1()); }
    if (app && cmd === 41) {
      if (this.state === 'idle' && ++this.acmdCount >= 3) this.state = 'ready';
      return resp(this.r1());
    }
    if (cmd === 58) return resp(this.r1(), 0xc0, 0xff, 0x80, 0x00);  // SDHC・電源投入完了
    if (this.state !== 'ready') return resp(0x05);                  // 初期化前: 不正コマンド
    if (cmd === 13) return resp(0x00, 0x00);
    if (cmd === 17) {
      if (arg >= this.blocks) return resp(0x40);  // アドレスエラー
      const buf = Buffer.alloc(512);
      fs.readSync(this.fd, buf, 0, 512, arg * 512);
      this.stats.reads++;
      return resp(0x00, 0xff, 0xfe, ...buf, 0xff, 0xff);
    }
    if (cmd === 24) {
      if (arg >= this.blocks) return resp(0x40);
      this.wblock = arg;
      this.mode = 'wtoken';
      return resp(0x00);
    }
    return resp(0x04);  // 未対応のコマンド
  }
}
