/*
 * c3iq.js: browser driver for the ESP32-C3 I/Q firmware (c3_iq.ino / c3_iq_core.cpp).
 * Talks to the unmodified sketch over Web Serial (Chrome / Edge) at 115200 baud.
 * Classic script (no modules) so it also works when opened from file://.
 *
 *   const dev = await C3IQ.connectSerial();        // or: await C3IQ.connectDemo();
 *   dev.on('reading', e => console.log(e.detail)); // streaming readings at the tuned frequency
 *   await dev.tune(2437);
 *   await dev.sampleRate(16);
 *   await dev.gain('auto');                        // or a number: manual gain index
 *   const pts = await dev.scan(2400, 2500, 1);     // [{mhz, rms, dbfs, dcI, dcQ, peak, gain, ...}]
 */
(function (root) {
  const READ = /^(\d+) MHz: rms=([-\d.na]+) \(([-\d.naif]+) dBFS\) dcI=([-\d.na]+) dcQ=([-\d.na]+) peak=(\d+) gain=(agc|manual) (\d+)/i;
  const FAIL = /^(\d+) MHz: capture failed/;

  // "2437 MHz: rms=102.8 (-13.9 dBFS) dcI=-14.7 dcQ=1.9 peak=265 gain=agc 76" -> object, or null
  function parseReading(line) {
    let m = READ.exec(line);
    if (m) return { mhz: +m[1], rms: +m[2], dbfs: +m[3], dcI: +m[4], dcQ: +m[5], peak: +m[6],
                    gainMode: m[7].toLowerCase(), gain: +m[8], failed: false, t: Date.now() };
    m = FAIL.exec(line);
    return m ? { mhz: +m[1], failed: true, t: Date.now() } : null;
  }

  const int = (v, lo, hi, what) => {
    v = Math.round(+v);
    if (!(v >= lo && v <= hi)) throw new RangeError(`${what} must be ${lo}..${hi}`);
    return v;
  };

  class C3IQ extends EventTarget {
    constructor(transport) {
      super();
      this.t = transport; this._buf = ''; this._q = Promise.resolve();
      this._wait = null; this._scan = null; this.connected = true; this.ready = false;
      this._frame = null; this._iqWait = null; this.baud = 115200;
      transport.onData(chunk => chunk === null ? this._closed() : this._feed(chunk));
    }
    on(name, fn) { this.addEventListener(name, fn); return this; }
    _emit(name, detail) { this.dispatchEvent(new CustomEvent(name, { detail })); }
    _closed() { this.connected = false; this._emit('close'); }

    _feed(c) {                                    // text lines, with binary I/Q frames (start byte 0x00) mixed in
      const u = typeof c === 'string' ? new TextEncoder().encode(c) : c;
      let i = 0;
      while (i < u.length) {
        const f = this._frame;
        if (f) {
          const k = Math.min(f.buf.length - f.got, u.length - i);
          f.buf.set(u.subarray(i, i + k), f.got); f.got += k; i += k;
          if (f.got === f.buf.length) { this._frame = null; this._frameDone(f); }
        } else if (u[i] === 0) this._frame = { hdr: true, buf: new Uint8Array(12), got: 0 };
        else {
          const ch = u[i++];
          if (ch === 10) { const l = this._buf.trim(); this._buf = ''; if (l) this._line(l); } else this._buf += String.fromCharCode(ch);
        }
      }
    }
    _frameDone(f) {
      const dv = new DataView(f.buf.buffer);
      if (f.hdr) {
        const b = f.buf;
        if (b[1] === 0xA5 && b[2] === 73 && b[3] === 81) {                     // raw I/Q frame
          const n = dv.getUint32(4, true);
          if (n >= 1 && n <= 16380) this._frame = { hdr: false, buf: new Uint8Array(n * 4), got: 0, n, rate: dv.getUint16(8, true), mhz: dv.getUint16(10, true) };
        } else if (b[1] === 0xA6 && b[2] === 83 && b[3] === 80) {              // spectrum frame
          const n = dv.getUint16(4, true);
          if (n >= 1) this._frame = { hdr: false, sp: true, buf: new Uint8Array(n), got: 0, n, rate: dv.getUint16(6, true), mhz: dv.getUint16(8, true), gain: b[10], avg: b[11] };
        }
        return;
      }
      if (f.sp) {
        const db = new Float32Array(f.n), freqs = new Float64Array(f.n);
        for (let k = 0; k < f.n; k++) { db[k] = -f.buf[k] / 2; freqs[k] = f.mhz + (k - f.n / 2) * f.rate / f.n; }
        const r = { center: f.mhz, rate: f.rate, N: f.n, gain: f.gain, avg: f.avg, binMHz: f.rate / f.n, freqs, db, t: Date.now() };
        this._emit('spectrum', r);
        if (this._iqWait) { const w = this._iqWait; this._iqWait = null; w.res(r); }
        return;
      }
      const I = new Float32Array(f.n), Q = new Float32Array(f.n); let g = 0;
      for (let k = 0; k < f.n; k++) {
        const w = dv.getUint32(4 * k, true);
        let i = w & 0x3ff, nq = (w >> 10) & 0x3ff;
        if (i & 0x200) i -= 1024; if (nq & 0x200) nq -= 1024;
        I[k] = i; Q[k] = -nq; g = (w >> 20) & 0xff;
      }
      const r = { mhz: f.mhz, rate: f.rate, n: f.n, gain: g, i: I, q: Q, t: Date.now() };
      this._emit('iq', r);
      if (this._iqWait) { const w = this._iqWait; this._iqWait = null; w.res(r); }
    }
    _line(l) {
      this._emit('line', l);
      if (l.startsWith('ready')) { this.ready = true; this._emit('ready'); }
      const r = parseReading(l);
      if (r) {
        const s = this._scan;
        if (s) {                                  // scan lines arrive back to back, in order
          if (r.mhz === s.fr[s.pts.length]) {
            s.pts.push(r); this._emit('scanpoint', r);
            if (s.pts.length === s.fr.length) s.done();
          }
        } else this._emit('reading', r);
        return;
      }
      if (this._iqWait && l.startsWith('ERR')) { const w = this._iqWait; this._iqWait = null; return w.rej(new Error(l)); }
      if (this._scan && l.startsWith('ERR')) return this._scan.fail(new Error(l));
      if (this._wait && /^(OK|ERR|GAIN )/.test(l)) { const w = this._wait; this._wait = null; w(l); }
    }

    _enqueue(fn) { const p = this._q.then(fn); this._q = p.catch(() => {}); return p; }
    _cmd(text, timeout = 2000) {
      return this._enqueue(() => new Promise((res, rej) => {
        const to = setTimeout(() => { this._wait = null; rej(new Error('timeout: ' + text)); }, timeout);
        this._wait = l => { clearTimeout(to); l.startsWith('ERR') ? rej(new Error(l)) : res(l); };
        this.t.write(text + '\n');
      }));
    }

    /** Wait for the boot banner (opening the port may reset the board). */
    async waitReady(ms = 3000) {
      if (this.ready) return true;
      return new Promise(res => {
        const to = setTimeout(() => res(false), ms);
        this.on('ready', () => { clearTimeout(to); res(true); });
      });
    }

    tune(mhz)       { return this._cmd(`F ${int(mhz, 100, 6000, 'frequency (MHz)')}`); }
    bandwidth(mhz)  { const b = +mhz; return this._cmd(`B ${b === 0 ? 0 : int(b, 14, 62, 'bandwidth (MHz), or 0 = auto')}`); }
    sampleRate(msps){ if (![80, 40, 16].includes(+msps)) throw new RangeError('rate must be 80, 40 or 16'); return this._cmd(`R ${+msps}`); }
    samples(n)      { return this._cmd(`N ${int(n, 1, 16380, 'samples')}`); }
    gain(v)         { return this._cmd(v === 'auto' ? 'G H' : `G ${int(v, 0, 127, 'gain index')}`); }
    /** -> { mode: 'HARDWARE'|'MANUAL', index, max, latched } (tune first, max is only known then) */
    async gainInfo() {
      const m = /GAIN (\w+) (\d+) max=(\d+) latched=(\d)/.exec(await this._cmd('G ?'));
      return m ? { mode: m[1], index: +m[2], max: +m[3], latched: m[4] === '1' } : null;
    }
    /** Pause (false) or resume (true) the periodic text readings. */
    stream(on) { return this._cmd(`M ${on ? 1 : 0}`); }
    /** One raw capture (needs the firmware D command) -> { mhz, rate, n, gain, i: Float32Array, q: Float32Array } */
    iq(n) {
      n = int(n, 1, 16380, 'samples');
      return this._enqueue(() => new Promise((res, rej) => {
        const ms = 2000 + (n * 4 + 12) * 15000 / this.baud;
        const to = setTimeout(() => { this._iqWait = null; this._frame = null; rej(new Error('iq timeout')); }, ms);
        this._iqWait = { res: r => { clearTimeout(to); res(r); }, rej: e => { clearTimeout(to); rej(e); } };
        this.t.write(`D ${n}\n`);
      }));
    }
    /** On-board FFT (needs the firmware T command) -> { center, rate, N, gain, avg, binMHz, freqs: Float64Array (MHz), db: Float32Array (dBFS) } */
    spectrum(size, avg = 4, dc = true) {
      size = int(size, 64, 4096, 'FFT size'); avg = int(avg, 1, 32, 'averages');
      return this._enqueue(() => new Promise((res, rej) => {
        const ms = 2000 + (size + 12) * 15000 / this.baud + avg * size * 0.05;
        const to = setTimeout(() => { this._iqWait = null; this._frame = null; rej(new Error('spectrum timeout')); }, ms);
        this._iqWait = { res: r => { clearTimeout(to); res(r); }, rej: e => { clearTimeout(to); rej(e); } };
        this.t.write(`T ${size} ${avg} ${dc ? 1 : 0}\n`);
      }));
    }
    /** Power sweep (1024 samples per step, firmware-side). Resolves with one reading per step. */
    scan(from, to, step) {
      from = int(from, 100, 6000, 'from'); to = int(to, from, 6000, 'to'); step = int(step, 1, 1000, 'step');
      const fr = []; for (let f = from; f <= to; f += step) fr.push(f);
      return this._enqueue(() => new Promise((res, rej) => {
        const fin = fn => v => { clearTimeout(to_); this._scan = null; fn(v); };
        const to_ = setTimeout(() => { this._scan = null; rej(new Error('scan timeout')); }, 4000 + fr.length * 80);
        const pts = [];
        this._scan = { fr, pts, done: fin(() => res(pts)), fail: fin(rej) };
        this.t.write(`S ${from} ${to} ${step}\n`);
      }));
    }
    async close() { this.connected = false; await this.t.close(); }
  }

  // ---- transports -------------------------------------------------------------
  async function serialTransport(baud = 115200) {
    if (!navigator.serial) throw new Error('Web Serial not available: use Chrome or Edge');
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: baud });
    const enc = new TextEncoder(), w = port.writable.getWriter(), rd = port.readable.getReader();
    let cb = () => {};
    (async () => {
      try { for (;;) { const { value, done } = await rd.read(); if (done) break; cb(value); } }
      catch (e) { /* unplugged */ }
      cb(null);
    })();
    return {
      write: s => w.write(enc.encode(s)),
      onData: f => { cb = f; },
      async close() { try { await rd.cancel(); } catch (e) {} try { w.releaseLock(); } catch (e) {} try { await port.close(); } catch (e) {} },
    };
  }

  // Fake firmware for trying the UI without hardware: three Wi-Fi-like bumps over a noise floor.
  function demoTransport() {
    let cb = () => {}, f = 2437, rate = 80, on = true, timer;
    const bumps = [[2412, -28], [2437, -18], [2462, -34], [2484, -55]];
    const db = mhz => {
      let p = Math.pow(10, -68 / 10);
      for (const [c, a] of bumps) p += Math.pow(10, (a - Math.pow((mhz - c) / 9, 2) * 3) / 10);
      return 10 * Math.log10(p) + (Math.random() - 0.5) * 3;
    };
    const line = mhz => { const d = db(mhz), r = 512 * Math.pow(10, d / 20);
      return `${mhz} MHz: rms=${r.toFixed(1)} (${d.toFixed(1)} dBFS) dcI=${(-14 + Math.random() * 3).toFixed(1)} dcQ=${(2 + Math.random()).toFixed(1)} peak=${Math.round(r * 2.6)} gain=agc ${70 + Math.round(Math.random() * 9)}\n`; };
    const out = (s, d = 4) => setTimeout(() => cb(s), d);
    out('ready: F/B/R/N/S/G commands\n', 300);
    const iqFrame = n => {
      const b = new Uint8Array(12 + n * 4), dv = new DataView(b.buffer), cl = v => Math.max(-511, Math.min(511, Math.round(v)));
      b[1] = 0xA5; b[2] = 73; b[3] = 81; dv.setUint32(4, n, true); dv.setUint16(8, rate, true); dv.setUint16(10, f, true);
      const tones = [[4, 150], [-9, 70], [rate * 0.31, 25]];
      for (let k = 0; k < n; k++) {
        let I = (Math.random() - 0.5) * 8, Q = (Math.random() - 0.5) * 8;
        for (const [o, a] of tones) { const ph = 2 * Math.PI * o * k / rate; I += a * Math.cos(ph); Q += a * Math.sin(ph); }
        dv.setUint32(12 + 4 * k, ((75 << 20) | (((-cl(Q)) & 0x3ff) << 10) | (cl(I) & 0x3ff)) >>> 0, true);
      }
      return b;
    };
    const spFrame = (N, avg) => {
      const b = new Uint8Array(12 + N), dv = new DataView(b.buffer), bin = rate / N, tones = [[4, -10.7], [-9, -17], [rate * 0.31, -30]];
      b[1] = 0xA6; b[2] = 83; b[3] = 80; dv.setUint16(4, N, true); dv.setUint16(6, rate, true); dv.setUint16(8, f, true); b[10] = 75; b[11] = avg;
      for (let k = 0; k < N; k++) {
        let p = Math.pow(10, -78 / 10);
        for (const [o, a] of tones) { const d = ((k - N / 2) * bin - o) / bin; p += Math.pow(10, (a - 6 * d * d) / 10); }
        b[12 + k] = Math.max(0, Math.min(255, Math.round(-2 * (10 * Math.log10(p) + (Math.random() - 0.5) * 3 / Math.sqrt(avg)))));
      }
      return b;
    };
    timer = setInterval(() => on && cb(line(f)), 200);
    return {
      onData: x => { cb = x; },
      close() { clearInterval(timer); },
      write(s) {
        let m;
        if ((m = /^F (\d+)/.exec(s))) { f = +m[1]; out('OK\n'); }
        else if (/^G \?/.test(s)) out('GAIN HARDWARE 0 max=79 latched=1\n');
        else if ((m = /^S (\d+) (\d+) (\d+)/.exec(s))) {
          let t = 0; for (let x = +m[1]; x <= +m[2]; x += +m[3]) out(line(x), (t += 6));
        } else if ((m = /^T (\d+) (\d+)/.exec(s))) out(spFrame(+m[1], +m[2]), 10 + m[1] / 50);
        else if ((m = /^D (\d+)/.exec(s))) out(iqFrame(+m[1]), 8 + m[1] / 100);
        else if ((m = /^R (\d+)/.exec(s))) { rate = +m[1]; out('OK\n'); }
        else if ((m = /^M (\d)/.exec(s))) { on = m[1] === '1'; out('OK\n'); }
        else if (/^[BNG] /.test(s)) out('OK\n');
        else out('ERR command\n');
      },
    };
  }

  // Windowed (Hann) FFT of one capture. Returns { mhz: absolute bin frequencies (fft-shifted), db: dBFS per bin, N, binMHz }.
  // 0 dBFS = a complex tone at ADC full scale (512 counts). Uses the largest power of two <= min(size, iq.n).
  C3IQ.fft = function (iq, opts) {
    const o = opts || {}, lim = Math.min(o.size || iq.n, iq.n);
    let N = 1; while (N * 2 <= lim) N *= 2;
    const re = new Float64Array(N), im = new Float64Array(N);
    let mi = 0, mq = 0, ws = 0;
    if (o.dc !== false) { for (let k = 0; k < N; k++) { mi += iq.i[k]; mq += iq.q[k]; } mi /= N; mq /= N; }
    for (let k = 0; k < N; k++) { const w = 0.5 - 0.5 * Math.cos(2 * Math.PI * k / N); ws += w; re[k] = (iq.i[k] - mi) * w; im[k] = (iq.q[k] - mq) * w; }
    for (let i = 1, j = 0; i < N; i++) {
      let b = N >> 1; for (; j & b; b >>= 1) j ^= b; j ^= b;
      if (i < j) { let t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (let len = 2; len <= N; len <<= 1) {
      const a = -2 * Math.PI / len, wr = Math.cos(a), wi = Math.sin(a), h = len >> 1;
      for (let i = 0; i < N; i += len) {
        let cr = 1, ci = 0;
        for (let j = 0; j < h; j++) {
          const u = i + j, v = u + h, xr = re[v] * cr - im[v] * ci, xi = re[v] * ci + im[v] * cr;
          re[v] = re[u] - xr; im[v] = im[u] - xi; re[u] += xr; im[u] += xi;
          const t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
        }
      }
    }
    const db = new Float64Array(N), mhz = new Float64Array(N), ref = ws * 512;
    for (let k = 0; k < N; k++) {
      const s = (k + N / 2) % N;
      db[k] = 10 * Math.log10((re[s] * re[s] + im[s] * im[s]) / (ref * ref) + 1e-12);
      mhz[k] = iq.mhz + (k - N / 2) * iq.rate / N;
    }
    return { mhz, db, N, binMHz: iq.rate / N };
  };

  C3IQ.parseReading = parseReading;
  C3IQ.connectSerial = async (baud) => { const d = new C3IQ(await serialTransport(baud)); d.baud = baud || 115200; await d.waitReady(); return d; };
  C3IQ.connectDemo = async () => { const d = new C3IQ(demoTransport()); await d.waitReady(); return d; };

  if (typeof module !== 'undefined') module.exports = C3IQ; else root.C3IQ = C3IQ;
})(typeof window !== 'undefined' ? window : globalThis);
