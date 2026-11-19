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
      transport.onData(chunk => chunk === null ? this._closed() : this._feed(chunk));
    }
    on(name, fn) { this.addEventListener(name, fn); return this; }
    _emit(name, detail) { this.dispatchEvent(new CustomEvent(name, { detail })); }
    _closed() { this.connected = false; this._emit('close'); }

    _feed(chunk) {
      this._buf += chunk;
      const lines = this._buf.split(/\r?\n/);
      this._buf = lines.pop();
      for (const l of lines) if (l.trim()) this._line(l.trim());
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
    const enc = new TextEncoder(), dec = new TextDecoder(), w = port.writable.getWriter(), rd = port.readable.getReader();
    let cb = () => {};
    (async () => {
      try { for (;;) { const { value, done } = await rd.read(); if (done) break; cb(dec.decode(value, { stream: true })); } }
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
    let cb = () => {}, f = 2437, n = 4096, timer;
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
    timer = setInterval(() => cb(line(f)), 200);
    return {
      onData: x => { cb = x; },
      close() { clearInterval(timer); },
      write(s) {
        let m;
        if ((m = /^F (\d+)/.exec(s))) { f = +m[1]; out('OK\n'); }
        else if (/^G \?/.test(s)) out('GAIN HARDWARE 0 max=79 latched=1\n');
        else if ((m = /^S (\d+) (\d+) (\d+)/.exec(s))) {
          let t = 0; for (let x = +m[1]; x <= +m[2]; x += +m[3]) out(line(x), (t += 6));
        } else if (/^[BRNG] /.test(s)) out('OK\n');
        else out('ERR command\n');
      },
    };
  }

  C3IQ.parseReading = parseReading;
  C3IQ.connectSerial = async (baud) => { const d = new C3IQ(await serialTransport(baud)); await d.waitReady(); return d; };
  C3IQ.connectDemo = async () => { const d = new C3IQ(demoTransport()); await d.waitReady(); return d; };

  if (typeof module !== 'undefined') module.exports = C3IQ; else root.C3IQ = C3IQ;
})(typeof window !== 'undefined' ? window : globalThis);
