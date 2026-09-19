#pragma once

#include <Arduino.h>
#include "config.h"
#include "rf_detector.h"
#include "acoustic_detector.h"
#include "audio_cue.h"

// =============================================================================
// Telemetry Module
// Streams live telemetry to Serial at 115200 baud.
// Supports Dashboard format, Serial Plotter CSV format, and interactive commands.
// =============================================================================

enum TelemetryMode {
    TELEMETRY_MODE_DASHBOARD = 0,
    TELEMETRY_MODE_PLOTTER   = 1
};

class Telemetry {
public:
    Telemetry(RFDetector& rf, AcousticDetector& acoustic, AudioCue& audio);

    // Initialize Serial interface
    void begin();

    // Process serial commands and emit timed telemetry output
    void update();

    // Switch between Dashboard text and Plotter CSV
    void setMode(TelemetryMode mode) { _mode = mode; }
    TelemetryMode getMode() const { return _mode; }

private:
    void printDashboard();
    void printPlotter();
    void handleSerialCommands();
    void printHelp();

    RFDetector&       _rf;
    AcousticDetector& _acoustic;
    AudioCue&         _audio;

    TelemetryMode     _mode;
    uint32_t          _lastTelemetryMs;
};
