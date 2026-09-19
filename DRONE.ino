// =============================================================================
// ESP32-S3 HANDHELD DRONE SNIFFER - ARDUINO IDE COMPATIBILITY WRAPPER
// =============================================================================
// If opening this project in the Arduino IDE:
// 1. Board: "ESP32S3 Dev Module"
// 2. USB CDC On Boot: "Enabled"
// 3. Upload Mode: "UART0 / Hardware CDC"
// 4. Flash Size: "8MB (64Mb)" or "16MB" depending on your module
// 5. Partition Scheme: "Default 4MB with spiffs" or "8MB with spiffs"
// =============================================================================

#if !defined(PLATFORMIO)
#include "include/config.h"
#include "src/rf_detector.h"
#include "src/acoustic_detector.h"
#include "src/wifi_detector.h"
#include "src/audio_cue.h"
#include "src/telemetry.h"

// Directly include implementations when compiling as a single Arduino sketch
#include "src/rf_detector.cpp"
#include "src/acoustic_detector.cpp"
#include "src/wifi_detector.cpp"
#include "src/audio_cue.cpp"
#include "src/telemetry.cpp"
#include "src/main.cpp"
#endif
