// c3_iq.ino: thin wrapper. All code is in c3_iq_core.cpp, which the Arduino IDE's
// automatic prototype generator does not touch. Keep this file, c3_iq_core.cpp and
// rf_guard.c together in the folder "c3_iq". See c3_iq_core.cpp for the commands.
void c3iqSetup();
void c3iqLoop();

void setup() { c3iqSetup(); }
void loop()  { c3iqLoop(); }