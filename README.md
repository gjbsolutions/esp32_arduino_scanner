# ESP32-C3 raw I/Q capture for the Arduino IDE

Raw I/Q snapshots from the Wi-Fi modem of a plain **ESP32-C3**, with a serial command interface for **frequency**, **bandwidth**, **sample rate** and **manual or automatic gain**. It runs as an ordinary Arduino sketch, with no custom firmware and no ESP-IDF project.

It is a port of the C3 capture path of [ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr) ([project page](https://espargos.net/espsdr/)) to the Arduino-ESP32 core. All credit for finding and documenting the capture mechanism goes to that project. See [Credits and license](#credits-and-license).

> **Status: experimental.** It uses undocumented modem hardware, is receive-only, and is a snapshot capture, not a continuous stream. See [Status](#status) and [Limitations](#limitations) for what has and has not been verified.

## What you get

- Up to **16,380 raw I/Q samples** per capture at **80, 40 or 16 MS/s**.
- **Frequency** control (100–6000 MHz accepted; see [Limitations](#limitations) for where the PLL actually locks).
- **Bandwidth** control from 14 to 62 MHz (analog filter), or automatic.
- **Hardware AGC** or a **manual gain index**.
- A power **scan** across a frequency range.
- Boot-time safety checks that refuse to capture if the sketch cannot do so safely.
- A **browser scanner** (Web Serial) with a live spectrum, waterfall and settings panel, plus a small JavaScript driver you can reuse in your own web app. See [Web scanner](#web-scanner).

## How it works

The ESP32-C3 modem has a debug "dump" engine that can write raw ADC-side I/Q samples into internal SRAM. The sketch:

1. Reserves a 128 KiB SRAM block (`0x3fca0000`–`0x3fcc0000`) so the heap, task stacks and Wi-Fi buffers can never live there. While a capture runs, the CPU loses access to the whole block; the engine writes the upper 64 KiB at `0x3fcb0000`.
2. Brings the Wi-Fi radio up in station mode (not connected, power save off, promiscuous on) so the receiver stays on. It never transmits.
3. Tunes the receiver through the PHY library, and optionally forces the receive gain and sets the analog filter bandwidth.
4. Hands the SRAM block to the engine for a short interval (about 205 µs at 80 MS/s for a full 16,380-sample capture), then gives it back and reads the samples.

Each 32-bit sample word is laid out as:

| Bits | Content |
|---|---|
| 31:28 | unused / reserved |
| 27:20 | gain index in effect |
| 19:10 | **−Q**, signed 10-bit (the dump stores Q negated; the sketch negates it back) |
| 9:0 | **I**, signed 10-bit |

## Requirements

- A plain **ESP32-C3** board (no PSRAM needed).
- **Arduino IDE** with the **Arduino-ESP32 core**. Tested with core **3.3.10**. The sketch calls private PHY functions, so other core versions may need changes.
- A serial connection at 115200 baud.

## Files

Keep all three files together in a folder named `c3_iq`.

| File | Purpose |
|---|---|
| `c3_iq.ino` | Thin wrapper containing `setup()` and `loop()`. |
| `c3_iq_core.cpp` | All the real code: tuning, gain, bandwidth, capture, command interface. |
| `rf_guard.c` | Reserves the SRAM block from the heap (see [How it works](#how-it-works)). |

The web scanner files are separate and do not go in the sketch folder. See [Web scanner](#web-scanner).

The code lives in a `.cpp` file on purpose. When everything was in the `.ino`, the Arduino IDE's automatic prototype generation corrupted the linkage of `setup()` and `loop()` and the link failed with `undefined reference to setup()`. A `.cpp` file is not touched by that step.

## Build and run

1. Copy the three files into a folder named `c3_iq` and open `c3_iq.ino` in the Arduino IDE.
2. Select your ESP32-C3 board and port.
3. Compile and upload.
4. Open the Serial Monitor at **115200 baud** with line ending set to **Newline**.

Expected boot output:

```
iram_end=0x... data_end=0x... bss_end=0x...
gain: PHY force_rx_gain          (or: gain: register fallback (unverified))
ready: F/B/R/N/S/G commands
```

It then prints one reading roughly every 200 ms, for example:

```
2437 MHz: rms=102.8 (-13.9 dBFS) dcI=-14.7 dcQ=1.9 peak=265 gain=agc 76
2437 MHz: rms=72.9 (-16.9 dBFS) dcI=-7.0 dcQ=4.3 peak=196 gain=agc 79
```

`rms` and `peak` are in raw ADC counts (full scale is about 512). `dcI`/`dcQ` are the mean of each channel. `gain=agc 76` is the gain index the hardware AGC chose. With `gain=manual N` it is the forced index.

## Commands

Type a command and press Enter. Capture settings apply to every reading until changed.

| Command | Effect |
|---|---|
| `F <MHz>` | Tune to a frequency, 100–6000. Replies `OK` or `ERR range`. |
| `B <MHz>` | Analog filter bandwidth, 14–62, or `0` for automatic. |
| `R <80\|40\|16>` | Sample rate in MS/s. |
| `N <samples>` | Samples per capture, 1–16380 (default 4096). |
| `G H` | Hardware AGC (default). |
| `G <index>` | Manual gain index, from 0 up to the maximum shown by `G ?`. Higher means more gain. |
| `G ?` | Show gain mode, current index, highest valid index, and whether the force flag is set. |
| `S <from> <to> <step>` | Power scan. Tunes each frequency, takes 1024 samples, prints one line per step, then returns to the current frequency. |

Examples:

```
F 2437          # Wi-Fi channel 6
R 16            # 16 MS/s
N 16380         # longest capture (about 1 ms at 16 MS/s)
S 2400 2480 5   # scan the 2.4 GHz band in 5 MHz steps
G ?             # how high can the gain index go?
G 40            # force gain index 40
G H             # back to automatic gain
```

The gain index range is only known after the receiver has been tuned, so run `F` first, then `G ?`.

## Using the samples in your own code

Edit `blockStats()` or add your own function in `c3_iq_core.cpp`. After a successful `sdrCapture(n, rate)`, the samples are in the reserved block, read through `IQ_BUF[k]`. Decode them with `unpack(word, i, q, gain)`, which returns standard-sign I and Q. Do any heavy processing outside the capture call: while the engine is running the CPU cannot touch the block, and interrupts are masked for the duration of the capture.

## Web scanner

`web/` contains a browser front end for the sketch. It needs **no firmware changes**: it opens the serial port with the Web Serial API, sends the commands from [Commands](#commands) and parses the text readings the sketch already prints.

| File | Purpose |
|---|---|
| `web/c3iq.js` | JavaScript driver: connection, command queue, parsing, sweeps, and a built-in demo board. A classic script (no modules, no build step). |
| `web/scanner.html` | The scanner interface, built on `c3iq.js`. |

Keep both files in the same folder, for example `web/`, separate from the `c3_iq` sketch folder.

### Run it

1. Flash the sketch as described in [Build and run](#build-and-run).
2. Close the Arduino Serial Monitor (only one program can hold the port).
3. Open `web/scanner.html` in **Chrome or Edge**. Web Serial is not available in Firefox or Safari. Opening the file directly works; if your browser refuses, serve the folder with `python3 -m http.server` and open `http://localhost:8000/scanner.html`.
4. Click **Connect (USB)** and pick the board's port. Opening the port can reset the board, so the page waits up to 3 s for the boot banner.

Click **Demo** instead to try the interface with a simulated board and no hardware.

### Interface

- **Receiver:** frequency, bandwidth (0 = automatic), sample rate (80, 40 or 16 MS/s), samples per capture, and hardware AGC or a manual gain index. The manual gain limit is read from the board with `G ?` after tuning. **Apply settings** sends them in order: `F`, `B`, `R`, `N`, `G`.
- **Sweep tab:** runs the `S` command repeatedly and plots power in dBFS against frequency, with optional peak hold, Wi-Fi channel markers, a waterfall (newest sweep on top) and a hover readout. Presets cover the 2.4 GHz band, 2.2-2.7 GHz and 1.85-2.2 GHz. **Sweep once** runs a single pass.
- **Monitor tab:** plots the level at the tuned frequency over time, with cards for RMS, peak, DC I/Q and the gain index.
- **Device log:** every raw line from the board.

The sweep panel warns when the range goes outside roughly 1.84-2.7 GHz, where the PLL may not lock (see [Limitations](#limitations)).

### Using the driver in your own page

```html
<script src="c3iq.js"></script>
<script>
  async function go() {
    const dev = await C3IQ.connectSerial();   // must be called from a click handler
    dev.on('reading', e => console.log(e.detail));
    await dev.tune(2437);
    await dev.bandwidth(20);
    await dev.sampleRate(16);
    await dev.samples(4096);
    await dev.gain('auto');                   // or a manual index, e.g. dev.gain(40)
    const pts = await dev.scan(2400, 2500, 1);
    console.log(pts);
  }
</script>
```

| Call | Sends | Notes |
|---|---|---|
| `C3IQ.connectSerial(baud = 115200)` | | Opens the port picker. Resolves after the boot banner (or 3 s). |
| `C3IQ.connectDemo()` | | Simulated board, same API. |
| `dev.tune(mhz)` | `F` | 100-6000. |
| `dev.bandwidth(mhz)` | `B` | 14-62, or 0 for automatic. |
| `dev.sampleRate(msps)` | `R` | 80, 40 or 16. |
| `dev.samples(n)` | `N` | 1-16380. |
| `dev.gain('auto' \| index)` | `G H` / `G <index>` | |
| `dev.gainInfo()` | `G ?` | `{mode, index, max, latched}`. Tune first; `max` is only known then. |
| `dev.scan(from, to, step)` | `S` | Resolves with one reading per step (1024 samples each). |
| `dev.close()` | | Closes the port. |

Arguments are range-checked before anything is sent, and a firmware `ERR ...` reply rejects the promise. Commands are queued, so they can be issued back to back.

Events (`dev.on(name, fn)`, payload in `e.detail`): `reading` (streaming reading at the tuned frequency), `scanpoint` (each point during a sweep), `line` (raw text), `ready`, and `close`.

A reading looks like `{ mhz, rms, dbfs, dcI, dcQ, peak, gainMode, gain, failed, t }`. `failed: true` (with only `mhz` and `t`) marks a `capture failed` line. `C3IQ.parseReading(line)` is exposed for your own use.

### Web scanner notes

- **Text protocol.** The driver parses the sketch's `report()` output. If you change that print format, update the regular expression at the top of `c3iq.js`.
- **Power only.** You get RMS, peak, DC and gain per capture, not raw samples. Sending raw I/Q to the browser would need a firmware change (a binary dump command and a faster baud rate): a full 16,380-word capture is about 65 KB, which takes several seconds at 115200 baud.
- **Streaming and sweeps share the port.** While a sweep runs, the driver matches incoming lines to the expected frequencies in order. A streaming line from just before the sweep could, rarely, be mistaken for the first point if it is at the same frequency.
- **dBFS is not calibrated.** Levels are relative to ADC full scale at whatever gain the AGC (or your manual index) chose. Gain indexes are not dB, so levels are not comparable across gain settings.
- **Untested.** The web scanner has not yet been run against real hardware by the author. Please report results.

## Boot-time safety checks and troubleshooting

The capture hands a block of SRAM to the modem. If anything the CPU needs lives there, the board crashes. So the sketch checks before it ever captures:

| Message | Meaning and what to do |
|---|---|
| `REFUSING TO CAPTURE: firmware overlaps the C3 dump block` | The compiled sketch's code or data reaches into the block (limits: IRAM end `<= 0x403a0000`, data/bss end `<= 0x3fca0000`). Reduce static memory or IRAM use, or build with a custom sdkconfig. Open an issue with the three printed addresses. |
| `REFUSING TO CAPTURE: the heap can still hand out the dump block` | The reservation in `rf_guard.c` did not take effect. Open an issue with your core version. |
| `capture failed` (repeatedly) | The engine did not finish or the buffer was not written. Check that tuning succeeded and try a shorter `N`. |
| Board keeps resetting | Hold **BOOT** while replugging USB to enter download mode, then flash a known-good sketch. |
| `undefined reference to ...` at link time | A PHY symbol that this core version does not export. Open an issue with the symbol name. |

Nothing in this project writes flash contents, the bootloader or eFuses. Capture settings live in RAM and are cleared by a reset.

## Status

Verified by the author on a plain ESP32-C3 with Arduino-ESP32 3.3.10: build, upload, boot checks pass, capture works, and **reception is confirmed** (live readings at 2437 MHz with hardware AGC).

Not yet independently verified: manual gain latching when the register fallback is used, tuning to frequencies that are not Wi-Fi channels, the bandwidth setting, and the `S` scan across a busy channel. Please report results.

The [web scanner](#web-scanner) has not yet been verified on hardware.

## Limitations

- **Snapshots, not a stream.** Each capture is a short burst (at most 16,380 samples). Between captures you see nothing, so Wi-Fi traffic shows up as bursts. Use longer captures (`R 16`, `N 16380`) to catch whole packets.
- **No per-frequency DC recalibration.** The full firmware recalibrates the receiver at each frequency using private PHY internals and linker `--wrap` flags, which the Arduino IDE cannot easily add. Expect a larger DC offset away from 2412 MHz. Subtract the mean for spectral work.
- **Frequency range.** Tuning goes through the PHY. Direct locking is realistic roughly between 2.2 and 2.7 GHz, plus 1842–2209 MHz through the alternate oscillator path. Outside that, `F` is accepted but the PLL may not lock.
- **Manual gain.** The Arduino core does not link the RF-test library that provides `force_rx_gain`, so the sketch references it weakly. If the real function is not available it writes the forced-gain register field directly, and prints `gain: register fallback (unverified)` at boot. The firmware this is based on also verifies the analog gain words by read-back; this sketch only checks the forced-index field. Always confirm that stepping the index changes the signal level.
- **Web Serial browsers only.** The web scanner needs Chrome or Edge, and the port must not be open in another program.
- **Receive only.** The radio never transmits, but regulations on reception and use of captured data vary by country. You are responsible for complying with local law.
- **Undocumented hardware.** The mechanism comes from reverse engineering. Behavior may differ between chip revisions and between Arduino-ESP32 versions.

## Credits and license

This project is derived from [ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr) (register sequences, tuning order, local-oscillator plan, bandwidth curve and memory layout follow its ESP32-C3 target). ESP-SDR is licensed under **GPL-3.0**, so this derivative is released under **GPL-3.0** too. See `LICENSE`.

Parts of this code, including the web scanner, were written with AI assistance (Claude) and validated on hardware only as described under [Status](#status).

This is an independent project and is not affiliated with ESPARGOS.

## Reporting issues

Please include: your board, the Arduino-ESP32 core version, the full boot output, the commands you ran, and the output you got. For link errors, include the complete message.
