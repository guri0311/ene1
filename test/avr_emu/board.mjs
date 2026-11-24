// 仮想ボード: ATmega328P (avr8js) に INA228・LCD・SD カード・ボタン・速度センサー・USB シリアルをつなぐ
import { createRequire } from 'module';
import { I2CBus, INA228, RW1063, SdCard } from './devices.mjs';

const require = createRequire(import.meta.url);
const avr = require('avr8js');

const F_CPU = 16e6;
const MCUSR = 0x54;
const SRAM_START = 0x100;

export function loadHex(text) {
  const prog = new Uint8Array(0x8000).fill(0xff);
  let base = 0;
  for (const line of text.split(/\r?\n/)) {
    if (!line.startsWith(':')) continue;
    const n = parseInt(line.substr(1, 2), 16);
    const addr = parseInt(line.substr(3, 4), 16);
    const type = parseInt(line.substr(7, 2), 16);
    if (type === 0) for (let i = 0; i < n; i++) prog[base + addr + i] = parseInt(line.substr(9 + i * 2, 2), 16);
    else if (type === 2) base = parseInt(line.substr(9, 4), 16) << 4;
    else if (type === 4) base = parseInt(line.substr(9, 4), 16) << 16;
  }
  return new Uint16Array(prog.buffer);
}

export class Board {
  // symbols: { brkval, heapStart } (ELF のシンボルアドレス)
  // bootWord: ブートローダの開始番地 (ワード単位)。null ならスケッチの 0 番地から起動する
  constructor({ program, sdImage, symbols, lcdAddr = 0x3c, echoSerial = false, bootWord = null }) {
    this.program = program;
    this.bootWord = bootWord;
    this.spiHang = false;      // 故障注入: SPI 転送が終わらない
    this.i2cHangCount = 0;     // 故障注入: I2C の応答をこの回数だけ返さない
    this.symbols = symbols;
    this.echoSerial = echoSerial;
    this.baseCycles = 0;
    this.cpu = null;
    this.ina = new INA228(this);
    this.lcd = new RW1063(this, lcdAddr);
    this.sd = new SdCard(this, sdImage);
    // EEPROM は電源を切っても残るので、チップを作り直しても同じ中身を使う
    this.eeprom = new avr.EEPROMMemoryBackend(1024);
    this.i2cDevices = [this.ina, this.lcd];
    this.serial = [];
    this.serialLine = '';
    this.button = false;
    // 速度センサー (D3): 車輪の磁石が来るたびに 1ms だけ LOW。チャタリングと数 us のノイズも作れる。
    // 時刻はボードの通しのサイクル数で持つので、Nano がリセットされても車輪は回り続ける
    this.wheel = { kmh: 0, circ: 1.571, magnets: 1, bounce: 0, noiseHz: 0 };
    this.wheelArmed = false;
    this.noiseArmed = false;
    this.d3Pulse = false;      // 磁石による LOW
    this.d3Spike = false;      // ノイズによる LOW
    this.pulses = 0;
    this.spikes = 0;
    this.events = [];          // { cyc, fn } (時刻順)
    this.nextEventCyc = Infinity;
    this.resets = { watchdog: 0, external: 0, power: 0 };
    this.minSp = 0xffff;
    this.maxWdtGapUs = 0;
    this.powerOn();
  }

  get timeUs() { return ((this.baseCycles + (this.cpu ? this.cpu.cycles : 0)) * 1e6) / F_CPU; }
  get cycles() { return this.baseCycles + (this.cpu ? this.cpu.cycles : 0); }

  // チップを組み立てる。sram を渡すとその内容を引き継ぐ (リセットでは SRAM は消えない)
  buildChip(sram, mcusr) {
    if (this.cpu) this.baseCycles += this.cpu.cycles;
    const cpu = new avr.CPU(this.program);
    if (sram) cpu.data.set(sram.subarray(SRAM_START), SRAM_START);
    cpu.data[MCUSR] = mcusr;
    const clock = new avr.AVRClock(cpu, F_CPU, avr.clockConfig);
    new avr.AVRTimer(cpu, avr.timer0Config);
    new avr.AVRTimer(cpu, avr.timer1Config);
    new avr.AVRTimer(cpu, avr.timer2Config);
    const usart = new avr.AVRUSART(cpu, avr.usart0Config, F_CPU);
    usart.onByteTransmit = (b) => this.onSerialByte(b);
    const twi = new avr.AVRTWI(cpu, avr.twiConfig, F_CPU);
    twi.eventHandler = new I2CBus(this, cpu, twi);
    // 実機の TWI は TWEN=0 で転送が中断される (Wire のタイムアウト処理はこれで復帰する)
    const twcrHook = cpu.writeHooks[avr.twiConfig.TWCR];
    cpu.writeHooks[avr.twiConfig.TWCR] = (v, old, addr, mask) => {
      if (!(v & 0x04)) twi.busy = false;
      return twcrHook(v, old, addr, mask);
    };
    this.portB = new avr.AVRIOPort(cpu, avr.portBConfig);
    this.portC = new avr.AVRIOPort(cpu, avr.portCConfig);
    this.portD = new avr.AVRIOPort(cpu, avr.portDConfig);
    const spi = new avr.AVRSPI(cpu, avr.spiConfig, F_CPU);
    spi.onByte = (v) => {
      if (this.spiHang) return;  // 転送完了を返さない
      const selected = this.portB.pinState(2) === avr.PinState.Low;  // D10 = CS
      const r = this.sd.transfer(v, selected);
      cpu.addClockEvent(() => spi.completeTransfer(r), spi.transferCycles);
    };
    this.portB.addListener(() => { if (this.portB.pinState(2) !== avr.PinState.Low) this.sd.deselect(); });
    new avr.AVRWatchdog(cpu, avr.watchdogConfig, clock);
    new avr.AVREEPROM(cpu, this.eeprom);
    const origReset = cpu.reset.bind(cpu);
    cpu.reset = () => { origReset(); this.pendingReset = 'watchdog'; };
    const origWdr = cpu.onWatchdogReset;
    cpu.onWatchdogReset = () => { this.noteWdr(); origWdr(); };
    this.cpu = cpu;
    this.lastWdrUs = this.timeUs;
    this.wdtActive = false;
    this.spiHang = false;      // 一時的な故障とみなし、リセットで消える
    if (this.bootWord !== null) cpu.pc = this.bootWord;
    // ウォッチドッグでリセットされた直後は WDRF により WDE が強制的に有効 (16ms)
    if (mcusr & 0x08) { cpu.writeData(0x60, 0x18); cpu.writeData(0x60, 0x08); }
    this.applyButton();
    this.applyD3();
  }

  noteWdr() {
    const now = this.timeUs;
    if (this.wdtActive) this.maxWdtGapUs = Math.max(this.maxWdtGapUs, now - this.lastWdrUs);
    this.wdtActive = true;
    this.lastWdrUs = now;
  }

  powerOn() {
    const sram = new Uint8Array(0x900);
    for (let i = 0; i < sram.length; i++) sram[i] = (Math.random() * 256) | 0;  // 電源投入直後の RAM は不定
    this.buildChip(sram, 0x01);  // PORF
    this.ina.powerOn();
    this.lcd.powerOn();
    this.sd.powerUp();
    this.resets.power++;
  }
  // Nano のリセットボタン・書き込みなど (INA228・LCD・SD の電源はそのまま)
  resetMcu() { this.buildChip(this.cpu.data, 0x02); this.resets.external++; }
  powerCycle() { this.powerOn(); }

  setButton(pressed) { this.button = pressed; this.applyButton(); }
  applyButton() { if (this.portD) this.portD.setPin(2, !this.button); }

  // ---- 速度センサー ----
  applyD3() { if (this.portD) this.portD.setPin(3, !(this.d3Pulse || this.d3Spike)); }
  schedule(cyc, fn) {
    let k = this.events.length;
    while (k > 0 && this.events[k - 1].cyc > cyc) k--;
    this.events.splice(k, 0, { cyc, fn });
    this.nextEventCyc = this.events[0].cyc;
  }
  runEvents(now) {
    while (this.events.length && this.events[0].cyc <= now) this.events.shift().fn();
    this.nextEventCyc = this.events.length ? this.events[0].cyc : Infinity;
  }
  pulsePeriodCyc() { return Math.round((this.wheel.circ / this.wheel.magnets / (this.wheel.kmh / 3.6)) * F_CPU); }
  // 車輪の速さ [km/h] と、チャタリングの回数 (bounce)・ノイズの回数 [1/s] (noiseHz) など
  setWheel(kmh, opts = {}) {
    Object.assign(this.wheel, opts, { kmh });
    if (kmh > 0 && !this.wheelArmed) { this.wheelArmed = true; this.schedule(this.cycles + this.pulsePeriodCyc(), () => this.pulse()); }
    if (this.wheel.noiseHz > 0 && !this.noiseArmed) { this.noiseArmed = true; this.schedule(this.cycles + F_CPU / this.wheel.noiseHz / 3, () => this.spike()); }
  }
  pulse() {
    if (!(this.wheel.kmh > 0)) { this.wheelArmed = false; return; }
    const now = this.cycles, us = F_CPU / 1e6;
    this.pulses++;
    this.d3Pulse = true; this.applyD3();
    for (let b = 0; b < this.wheel.bounce; b++) {  // 30us ごとに HIGH と LOW を繰り返す
      this.schedule(now + (60 * b + 30) * us, () => { this.d3Pulse = false; this.applyD3(); });
      this.schedule(now + (60 * b + 60) * us, () => { this.d3Pulse = true; this.applyD3(); });
    }
    this.schedule(now + 1000 * us, () => { this.d3Pulse = false; this.applyD3(); });
    this.schedule(now + this.pulsePeriodCyc(), () => this.pulse());
  }
  spike() {  // 2us だけ LOW になるノイズ
    if (!(this.wheel.noiseHz > 0)) { this.noiseArmed = false; return; }
    const now = this.cycles;
    this.spikes++;
    this.d3Spike = true; this.applyD3();
    this.schedule(now + 2 * (F_CPU / 1e6), () => { this.d3Spike = false; this.applyD3(); });
    this.schedule(now + Math.round(F_CPU / this.wheel.noiseHz), () => this.spike());
  }

  onSerialByte(b) {
    if (b === 13) return;
    if (b === 10) {
      this.serial.push(this.serialLine);
      if (this.echoSerial && !/^\d/.test(this.serialLine)) console.log('    serial| ' + this.serialLine);
      this.serialLine = '';
    } else this.serialLine += String.fromCharCode(b);
  }

  run(ms) {
    const end = this.timeUs + ms * 1000;
    const endCycles = this.baseCycles + this.cpu.cycles + (ms * F_CPU) / 1000;
    const { avrInstruction } = avr;
    while (this.baseCycles + this.cpu.cycles < endCycles) {
      const cpu = this.cpu;
      avrInstruction(cpu);
      cpu.tick();
      if (this.baseCycles + cpu.cycles >= this.nextEventCyc) this.runEvents(this.baseCycles + cpu.cycles);
      const sp = cpu.SP;
      if (sp < this.minSp) this.minSp = sp;
      if (this.pendingReset) {
        this.pendingReset = null;
        this.resets.watchdog++;
        this.buildChip(cpu.data, 0x08);  // WDRF
      }
    }
    if (this.wdtActive) this.maxWdtGapUs = Math.max(this.maxWdtGapUs, this.timeUs - this.lastWdrUs);
    return end;
  }

  // ヒープの上端 (__brkval、未使用なら __heap_start)
  heapTop() {
    const d = this.cpu.data;
    const brk = d[this.symbols.brkval] | (d[this.symbols.brkval + 1] << 8);
    return brk || this.symbols.heapStart;
  }
  lcdRows() { return [0, 1, 2, 3].map((r) => this.lcd.row(r)); }
}
