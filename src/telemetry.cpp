#include "telemetry.h"

Telemetry::Telemetry(RFDetector& rf, AcousticDetector& acoustic, AudioCue& audio)
    : _rf(rf),
      _acoustic(acoustic),
      _audio(audio),
      _mode(TELEMETRY_MODE_DASHBOARD),
      _lastTelemetryMs(0) {
}

void Telemetry::begin() {
#if SERIAL_LOGGING_ENABLED
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Optional brief delay for USB CDC enumeration on ESP32-S3
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 1500)) {
        delay(10);
    }

    Serial.println();
    Serial.println(F("=================================================="));
    Serial.println(F("   ESP32-S3 HANDHELD DRONE SNIFFER v1.0          "));
    Serial.printf( "   RF Target: %s on GPIO %d\n", RF_TARGET_BAND_LABEL, PIN_AD8318_VOUT);
    Serial.println(F("   Acoustic: INMP441 (I2S) | Audio Cue: PJ-392   "));
    Serial.println(F("=================================================="));
    Serial.printf(" [AD8318] Pin: GPIO %d | Atten: 12dB | Baseline: %.0f mV\n", 
                  PIN_AD8318_VOUT, _rf.getQuiescentMv());
    Serial.printf(" [INMP441] SD: GPIO %d | SCK: GPIO %d | WS: GPIO %d\n", 
                  PIN_I2S_SD, PIN_I2S_SCK, PIN_I2S_WS);
    Serial.printf(" [HEADPHONE] Pin: GPIO %d (PJ-392 via 220R + 10uF)\n", 
                  PIN_AUDIO_JACK);
    Serial.println(F(" Commands: 'c' = Calibrate Clean Air | 'm' = Mute/Unmute | 'p' = Toggle Plotter | 'h' = Help"));
    Serial.println(F("=================================================="));
    Serial.println();
#endif
}

void Telemetry::printHelp() {
#if SERIAL_LOGGING_ENABLED
    Serial.println(F("\n--- DRONE SNIFFER COMMANDS ---"));
    Serial.println(F(" [c] Calibrate Clean Air Floor: Sets current RF reading as baseline 0% threat"));
    Serial.println(F(" [a] Calibrate Acoustic Floor: Samples quiet ambient sound (3s) to set ENERGY_THRESHOLD"));
    Serial.println(F(" [m] Toggle Mute: Enable/Disable headphone audio click output"));
    Serial.println(F(" [p] Toggle Plotter Mode: Switch between Dashboard text and CSV Serial Plotter"));
    Serial.println(F(" [h] Help: Display this menu"));
    Serial.println(F("------------------------------\n"));
#endif
}

void Telemetry::handleSerialCommands() {
#if SERIAL_LOGGING_ENABLED
    while (Serial.available() > 0) {
        char cmd = static_cast<char>(Serial.read());
        switch (cmd) {
            case 'c':
            case 'C':
                _rf.calibrateCleanAirFloor();
                Serial.printf("\n>>> [CALIBRATION] Baseline Clean-Air set to: %.1f mV <<<\n\n", 
                              _rf.getQuiescentMv());
                break;
            case 'a':
            case 'A':
                _acoustic.calibrateAmbientFloor(ACOUSTIC_CALIBRATION_DURATION_MS);
                break;
            case 'm':
            case 'M':
                _audio.setMuted(!_audio.isMuted());
                Serial.printf("\n>>> [AUDIO] Headphone Output: %s <<<\n\n", 
                              _audio.isMuted() ? "MUTED" : "UNMUTED");
                break;
            case 'p':
            case 'P':
                _mode = (_mode == TELEMETRY_MODE_DASHBOARD) ? TELEMETRY_MODE_PLOTTER : TELEMETRY_MODE_DASHBOARD;
                if (_mode == TELEMETRY_MODE_PLOTTER) {
                    Serial.println(F("RF_mV,RF_Threat_Pct,Mic_RMS,Whine_Ratio_Pct,Mic_Energy,Mic_Thresh,Audio_CPS"));
                } else {
                    Serial.println(F("\n>>> [TELEMETRY] Switched to Dashboard mode <<<\n"));
                }
                break;
            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;
            default:
                break;
        }
    }
#endif
}

void Telemetry::printDashboard() {
#if SERIAL_LOGGING_ENABLED
    char bar[11] = "          ";
    int filled = static_cast<int>(_rf.getThreatPercent() / 10.0f);
    if (filled > 10) filled = 10;
    for (int i = 0; i < filled; ++i) {
        bar[i] = '#';
    }

    const char* statusTag = "         ";
    if (g_dualThreatConfirmed) {
        statusTag = "[!DUAL!!]";
    } else if (g_droneIncoming) {
        statusTag = "[!ALERT!]";
    } else if (_rf.isBurstDetected()) {
        statusTag = "[BURST]  ";
    }

    Serial.printf("[RF 5.8G] %4.0fmV (%5.1fdBm) [%s] %3.0f%% %s || [MIC] Energy:%6.0f (Thresh:%6.0f, Sentry:%s, Drone:%s %.0f%%) || [AUDIO] %2d cps%s\n",
                  _rf.getFilteredMilliVolts(),
                  _rf.getEstimatedDbm(),
                  bar,
                  _rf.getThreatPercent(),
                  statusTag,
                  _acoustic.getLatestFrameEnergy(),
                  _acoustic.getEnergyThreshold(),
                  _acoustic.isStage1Triggered() ? "TRIG" : "IDLE",
                  _acoustic.isDroneConfirmed() ? "YES" : " NO",
                  _acoustic.getDroneConfidence() * 100.0f,
                  _audio.getClicksPerSecond(),
                  _audio.isMuted() ? " (MUTED)" : "");
#endif
}

void Telemetry::printPlotter() {
#if SERIAL_LOGGING_ENABLED
    // CSV format: RF_mV, RF_Threat_Pct, Mic_RMS, Whine_Ratio_Pct, Mic_Energy, Mic_Thresh, Audio_CPS
    Serial.printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u\n",
                  _rf.getFilteredMilliVolts(),
                  _rf.getThreatPercent(),
                  _acoustic.getRmsAmplitude(),
                  _acoustic.getWhineRatio() * 100.0f,
                  _acoustic.getLatestFrameEnergy(),
                  _acoustic.getEnergyThreshold(),
                  _audio.getClicksPerSecond());
#endif
}

void Telemetry::update() {
#if SERIAL_LOGGING_ENABLED
    handleSerialCommands();

    uint32_t nowMs = millis();
    if (nowMs - _lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        _lastTelemetryMs = nowMs;

        if (_mode == TELEMETRY_MODE_DASHBOARD) {
            printDashboard();
        } else {
            printPlotter();
        }
    }
#endif
}
