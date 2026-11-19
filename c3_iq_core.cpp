/*
 * ESP32-C3 raw IQ snapshots with frequency / bandwidth / rate control, Arduino IDE.
 * Derived from ESPARGOS/esp-sdr main/targets/esp32c3/receiver.c. UNTESTED. Receive only.
 *
 * Files (all in a folder named "c3_iq"): c3_iq.ino (thin wrapper), c3_iq_core.cpp (this file),
 * rf_guard.c (SDR_EXTERNAL_GUARD 1, default). The code lives in a .cpp because the Arduino
 * IDE's automatic prototype generator broke the link when it was all in the .ino.
 * To drop rf_guard.c, set SDR_EXTERNAL_GUARD to 0 below and delete that file.
 *
 * Simplified vs the real firmware: NO per-frequency DC recalibration (rx_recalibrate
 * needs --wrap linker flags and private PHY offsets). Expect a larger DC offset away
 * from 2412 MHz. Manual gain is applied the same way (force_rx_gain) but is only
 * checked through the forced-index register; the firmware also verifies the analog
 * gain words, which needs --wrap on set_rx_gain_table. Tuning works where the PLL can lock (about
 * 2.2-2.7 GHz directly, 1842-2209 MHz via the alternate LO); elsewhere it is an
 * attempt that may not lock.
 *
 * Serial commands (115200, newline-terminated):
 *   F <MHz>              tune (100..6000)
 *   B <MHz>              bandwidth 14..62, 0 = automatic
 *   R <80|40|16>         sample rate in MS/s
 *   G H                  hardware AGC (default)
 *   G <index>            manual gain index, 0..max (codes, not dB; higher = more gain)
 *   G ?                  show gain mode and the highest valid index
 *   N <samples>          samples per capture (1..16380)
 *   S <from> <to> <step> power scan across frequencies
 */
#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "soc/soc.h"
#include "esp_heap_caps.h"

// ---- Reserve the dump block (the CPU loses all 128 KiB while a capture runs) -----
// Keeps the heap, task stacks and Wi-Fi buffers out of 0x3fca0000..0x3fcc0000.
// SDR_EXTERNAL_GUARD 1: the reservation is in rf_guard.c (never enable both).
// SDR_EXTERNAL_GUARD 0: built in here. If the build fails on the SOC_RESERVE_MEMORY_REGION
// line, set SDR_MANUAL_RESERVE to 1.
#define SDR_EXTERNAL_GUARD 1
#define SDR_MANUAL_RESERVE 0
#if !SDR_EXTERNAL_GUARD
#if !SDR_MANUAL_RESERVE && __has_include("heap_memory_layout.h")
#include "heap_memory_layout.h"
SOC_RESERVE_MEMORY_REGION(0x3fca0000, 0x3fcc0000, c3_rf_dump);
#else
struct SdrReservedRegion { intptr_t start; intptr_t end; };   // layout of soc_reserved_region_t
__attribute__((section(".reserved_memory_address"), used))
static SdrReservedRegion sdrReserveC3 = { 0x3fca0000, 0x3fcc0000 };
#endif
#endif

// ---- PHY / ROM entry points (resolved at link time from the Arduino core) ----
// One declaration per line (no extern "C" { } block): the Arduino IDE's prototype
// generator can mis-handle a block that precedes the sketch's own functions.
extern "C" void stop_tx_tone(unsigned);
extern "C" void rom_pbus_workmode(void);
extern "C" void rom_pbus_xpd_rx_on(unsigned);
extern "C" void rom_pbus_xpd_tx_off(void);
extern "C" void rom_set_rxclk_en(unsigned);
// Weak: the Arduino core does not link Espressif's RF-test library that provides it.
extern "C" void force_rx_gain(unsigned, unsigned, unsigned) __attribute__((weak));
extern "C" void set_chanfreq(unsigned, unsigned);
extern "C" void phy_set_freq(unsigned, int);
extern "C" unsigned rom1_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern "C" void rom1_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
extern "C" int _iram_end;
extern "C" int _data_end;
extern "C" int _bss_end;

// Types used in function signatures must be defined before the first function:
// the Arduino IDE inserts auto-generated prototypes above everything else.
struct LoPlan { unsigned mhz; int offsetKhz; bool alt; };
struct Stats { double dcI, dcQ, rms; int peak, gain; };

// Proof that the reservation took effect: hand out every free chunk of internal and
// executable heap and make sure none overlaps the dump block (DRAM view or IRAM alias).
// Runs at boot with the scheduler suspended; takes well under a millisecond.
static bool heapAvoidsDumpBlock() {
  struct Node { Node *next; };
  const uint32_t caps[2] = { MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT, MALLOC_CAP_EXEC };
  bool clean = true;
  vTaskSuspendAll();
  for (int c = 0; c < 2; c++) {
    Node *head = nullptr;
    for (;;) {
      Node *p = (Node *)heap_caps_malloc(1024, caps[c]);
      if (!p) break;
      uintptr_t a = (uintptr_t)p;
      if ((a < 0x3fcc0000u && a + 1024 > 0x3fca0000u) ||
          (a < 0x403c0000u && a + 1024 > 0x403a0000u)) clean = false;
      p->next = head; head = p;
    }
    while (head) { Node *n = head->next; heap_caps_free(head); head = n; }
  }
  xTaskResumeAll();
  return clean;
}

// ---- Capture hardware (ESP32-C3) --------------------------------------------
#define IQ_BUF       ((volatile uint32_t *)0x3fcb0000u)
#define SRAM_OWNER   0x600c1020u
#define DUMP_CTRL    0x60033d5cu
#define DUMP_LANE    0x60033d90u
#define MARKER       0xa5a0055au
#define MAX_WORDS    16380u

static portMUX_TYPE capMux = portMUX_INITIALIZER_UNLOCKED;
static bool sdrSafe = false;
static unsigned curMHz = 2437;
static int rxFilter = -1;                 // -1 = leave the analog filter alone
static unsigned rxFilterSaved[2];

// ---- Gain (esp-sdr burst_gain.h / rx_gain_verify.c, C3) ----------------------
#define GAIN_REG     0x6001c02cu          // bits 8..14: highest calibrated index; bit 23+: forced index
#define WIFI_GAINS   80u
static bool gainManual = false;
static unsigned gainCode = 40;

// Force (manual=1) or release (manual=0) the receive gain index. Uses the PHY's
// force_rx_gain() when it is linked in; otherwise writes the same forced-gain field
// the firmware reads back (bit 23 = forced, bits 30:24 = index). The fallback is
// UNVERIFIED: confirm that stepping the index really changes the signal level.
static void gainForce(unsigned manual, unsigned index) {
  if (force_rx_gain) { force_rx_gain(manual, index, 0); return; }
  uint32_t r = REG_READ(GAIN_REG);
  r = (r & ~(0xffu << 23)) | ((manual ? 1u : 0u) << 23) | ((index & 0x7fu) << 24);
  REG_WRITE(GAIN_REG, r);
}

static unsigned gainMax() {               // filled in by the PHY when it tunes
  unsigned m = (REG_READ(GAIN_REG) >> 8) & 127u;
  return m < WIFI_GAINS ? m : 0u;
}
static bool gainLatched() {
  return !gainManual || (REG_READ(GAIN_REG) >> 23) == (1u | (gainCode << 1));
}
// Force or release the receive gain. The forced-index field is also copied into the
// dump samples, so it is only a weak check that the gain really latched.
static bool gainApply() {
  if (!gainManual) { gainForce(0, gainCode); return true; }
  unsigned mx = gainMax();
  if (gainCode > mx) gainCode = mx;
  for (int attempt = 0; attempt < 4; attempt++) {
    gainForce(0, gainCode);        // low, settle, then high: a clean pulse
    esp_rom_delay_us(2);
    gainForce(1, gainCode);
    esp_rom_delay_us(2);
    if (gainLatched()) return true;
  }
  return false;
}

// ---- Local-oscillator plan (esp-sdr rx_lo.h) --------------------------------
static LoPlan loPlan(unsigned mhz) {
  bool alt = mhz >= 1842 && mhz < 2210;   // 1.2x alternate LO
  unsigned khz = mhz * (alt ? 1200u : 1000u);
  return { khz / 1000u, (int)(khz % 1000u), alt };
}
static void loSelect(bool alt) {
  unsigned old = rom1_chip_i2c_readReg(0x65, 1, 0);
  unsigned v = (old & ~0x10u) | (alt ? 0x10u : 0u);
  if (v != old) rom1_chip_i2c_writeReg(0x65, 1, 0, v);
}

// ---- Tuning (esp-sdr tune_rx + prepare_rx, minus recalibration/gain forcing) --
bool sdrTune(unsigned mhz) {
  if (mhz < 100 || mhz > 6000) return false;
  LoPlan plan = loPlan(mhz);
  bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
  loSelect(false);
  set_chanfreq(channel ? mhz : 2412, 0);
  if (!channel) phy_set_freq(plan.mhz, plan.offsetKhz);
  stop_tx_tone(1);
  rom_pbus_workmode();
  rom_pbus_xpd_tx_off();
  rom_pbus_xpd_rx_on(1);
  rom_set_rxclk_en(1);
  loSelect(plan.alt);
  esp_rom_delay_us(3000);                 // let the PLL / AGC settle
  curMHz = mhz;
  gainApply();                            // retuning resets forced gain, like prepare_rx()
  return true;
}

// ---- Bandwidth: C3 curve {capacitor code, full width MHz} from esp-sdr --------
static int dcapForBandwidth(unsigned mhz) {
  static const int cal[][2] = {{0,62},{4,50},{8,45},{12,39},{16,34},{24,27},
                               {32,23},{40,20},{48,18},{56,16},{60,15},{63,14}};
  const int n = sizeof(cal) / sizeof(cal[0]);
  if (!mhz) return -1;                    // automatic
  if ((int)mhz >= cal[0][1]) return cal[0][0];
  if ((int)mhz <= cal[n - 1][1]) return cal[n - 1][0];
  for (int i = 1; i < n; i++)
    if ((int)mhz >= cal[i][1]) {
      int span = cal[i - 1][1] - cal[i][1];
      return cal[i - 1][0] + ((cal[i][0] - cal[i - 1][0]) * (cal[i - 1][1] - (int)mhz) + span / 2) / span;
    }
  return cal[n - 1][0];
}
static void filterApply() {
  for (unsigned j = 0; j < 2; j++) {
    rxFilterSaved[j] = rom1_chip_i2c_readReg(0x67, 1, 4 + j);
    if (rxFilter >= 0)
      rom1_chip_i2c_writeReg(0x67, 1, 4 + j, (rxFilterSaved[j] & ~63u) | (unsigned)rxFilter);
  }
}
static void filterRestore() {
  if (rxFilter >= 0)
    for (unsigned j = 0; j < 2; j++) rom1_chip_i2c_writeReg(0x67, 1, 4 + j, rxFilterSaved[j]);
}

// ---- One snapshot (esp-sdr acquire_iq) ----------------------------------------
// rate: 80, 40 or 16 MS/s. Returns true if all n words were written.
bool sdrCapture(uint32_t n, int rate) {
  if (!sdrSafe || n == 0 || n > MAX_WORDS) return false;
  uint32_t rb = rate == 40 ? (1u << 15) : rate == 16 ? (1u << 16) : 0u;
  if (gainManual && !gainLatched() && !gainApply()) return false;   // manual gain lost
  for (uint32_t j = 0; j < n; j++) IQ_BUF[j] = MARKER;
  filterApply();

  uint32_t ctrl = 0x80000000u | n | rb;
  portENTER_CRITICAL(&capMux);
  uint32_t owner = REG_READ(SRAM_OWNER);
  int64_t t0 = esp_timer_get_time();
  REG_WRITE(DUMP_CTRL, 0);
  REG_WRITE(DUMP_LANE, (1u << 6) | (2u << 12) | (3u << 18));
  REG_WRITE(SRAM_OWNER, (owner & ~7u) | 2u | 8u);   // hand the block to the engine
  __asm__ volatile("fence rw,rw" ::: "memory");
  REG_WRITE(DUMP_CTRL, ctrl);
  REG_WRITE(DUMP_CTRL, ctrl | (1u << 19));          // trigger pulse
  REG_WRITE(DUMP_CTRL, ctrl);
  while (!(REG_READ(DUMP_CTRL) & (1u << 18)) && esp_timer_get_time() - t0 < 5000) {}
  bool done = (REG_READ(DUMP_CTRL) & (1u << 18)) != 0;
  REG_WRITE(DUMP_CTRL, 0);
  REG_WRITE(SRAM_OWNER, owner);                     // CPU may use the block again
  __asm__ volatile("fence rw,rw" ::: "memory");
  portEXIT_CRITICAL(&capMux);
  filterRestore();

  if (!done) return false;
  for (uint32_t j = 0; j < n; j++) if (IQ_BUF[j] == MARKER) return false;
  return true;
}

// Word: gain [27:20], -Q [19:10], I [9:0], signed 10-bit. Q is negated to standard sign.
static inline void unpack(uint32_t w, int &i, int &q, int &gain) {
  i = w & 0x3ff;               if (i & 0x200) i -= 1024;
  int nq = (w >> 10) & 0x3ff;  if (nq & 0x200) nq -= 1024;
  q = -nq;
  gain = (w >> 20) & 0xff;
}

static Stats blockStats(uint32_t n) {            // your processing can replace this
  double sI = 0, sQ = 0, p = 0; int peak = 0, gain = 0;
  for (uint32_t k = 0; k < n; k++) {
    int i, q; unpack(IQ_BUF[k], i, q, gain);
    sI += i; sQ += q; p += (double)i * i + (double)q * q;
    if (abs(i) > peak) peak = abs(i);
    if (abs(q) > peak) peak = abs(q);
  }
  return { sI / n, sQ / n, sqrt(p / n), peak, gain };
}

// ---- Command interface ---------------------------------------------------------
static uint32_t nSamples = 4096;
static int rateMsps = 80;

static void report(unsigned mhz, uint32_t n) {
  if (!sdrCapture(n, rateMsps)) { Serial.printf("%u MHz: capture failed\n", mhz); return; }
  Stats s = blockStats(n);
  Serial.printf("%u MHz: rms=%.1f (%.1f dBFS) dcI=%.1f dcQ=%.1f peak=%d gain=%s%d\n",
                mhz, s.rms, 20.0 * log10(s.rms / 512.0 + 1e-9), s.dcI, s.dcQ, s.peak,
                gainManual ? "manual " : "agc ", s.gain);
}

static void handleLine(char *line) {
  unsigned a, b, c; char x;
  if (sscanf(line, "F %u %c", &a, &x) == 1) {
    Serial.println(sdrTune(a) ? "OK" : "ERR range");
  } else if (sscanf(line, "B %u %c", &a, &x) == 1) {
    if (a && (a < 14 || a > 62)) { Serial.println("ERR bandwidth 14..62 or 0"); return; }
    rxFilter = dcapForBandwidth(a);
    Serial.println("OK");
  } else if (sscanf(line, "R %u %c", &a, &x) == 1 && (a == 80 || a == 40 || a == 16)) {
    rateMsps = a; Serial.println("OK");
  } else if (!strcmp(line, "G H")) {
    gainManual = false;
    gainApply();
    Serial.println("OK");
  } else if (!strcmp(line, "G ?")) {
    Serial.printf("GAIN %s %u max=%u latched=%d\n", gainManual ? "MANUAL" : "HARDWARE",
                  gainManual ? gainCode : 0u, gainMax(), (int)gainLatched());
  } else if (sscanf(line, "G %u %c", &a, &x) == 1) {
    if (a > gainMax()) { Serial.printf("ERR gain 0..%u (tune first)\n", gainMax()); return; }
    gainManual = true;
    gainCode = a;
    Serial.println(gainApply() ? "OK" : "ERR gain did not latch");
  } else if (sscanf(line, "N %u %c", &a, &x) == 1 && a >= 1 && a <= MAX_WORDS) {
    nSamples = a; Serial.println("OK");
  } else if (sscanf(line, "S %u %u %u %c", &a, &b, &c, &x) == 3 && c > 0 && b >= a) {
    for (unsigned f = a; f <= b; f += c) {
      if (!sdrTune(f)) { Serial.println("ERR range"); break; }
      report(f, 1024);
    }
    sdrTune(curMHz);
  } else {
    Serial.println("ERR command");
  }
}

void c3iqSetup() {
  Serial.begin(115200);
  delay(500);
  // Replaces esp-sdr's sram_guard.ld: code, data and bss must stay clear of the dump block.
  sdrSafe = (uintptr_t)&_iram_end <= 0x403a0000u && (uintptr_t)&_data_end <= 0x3fca0000u &&
            (uintptr_t)&_bss_end <= 0x3fca0000u;
  Serial.printf("iram_end=%#x data_end=%#x bss_end=%#x\n",
                (unsigned)(uintptr_t)&_iram_end, (unsigned)(uintptr_t)&_data_end, (unsigned)(uintptr_t)&_bss_end);
  if (!sdrSafe) {
    Serial.println("REFUSING TO CAPTURE: firmware overlaps the C3 dump block "
                   "(limits: iram<=0x403a0000, data/bss<=0x3fca0000).");
    return;
  }
  if (!heapAvoidsDumpBlock()) {
    sdrSafe = false;
    Serial.println("REFUSING TO CAPTURE: the heap can still hand out the dump block "
                   "(reservation not honored; try SDR_MANUAL_RESERVE 1).");
    return;
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);               // keep RX running, no TX
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  delay(20);
  sdrTune(curMHz);
  Serial.println(force_rx_gain ? "gain: PHY force_rx_gain" : "gain: register fallback (unverified)");
  Serial.println("ready: F/B/R/N/S/G commands");
}

void c3iqLoop() {
  static char line[48]; static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (len) { line[len] = 0; handleLine(line); len = 0; } }
    else if (len < sizeof(line) - 1) line[len++] = ch;
  }
  if (!sdrSafe) { delay(500); return; }
  report(curMHz, nSamples);
  delay(200);
}
