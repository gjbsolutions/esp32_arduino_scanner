// Reserve the ESP32-C3 modem dump block from the heap, stacks and Wi-Fi buffers.
// The CPU loses this whole 128 KiB block while a capture runs; the engine writes the
// upper 64 KiB (0x3fcb0000). Same reservation as esp-sdr main/targets/esp32c3/receiver.c.
// Used when SDR_EXTERNAL_GUARD is 1 in c3_iq.ino (the default).
#include "heap_memory_layout.h"

SOC_RESERVE_MEMORY_REGION(0x3fca0000, 0x3fcc0000, c3_rf_dump);
