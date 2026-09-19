#include <Arduino.h>
#include "config.h"
#include "rf_detector.h"
#include "acoustic_detector.h"
#include "audio_cue.h"
#include "telemetry.h"
#include "wifi_detector.h"

// =============================================================================
// ESP32-S3 HANDHELD DRONE SNIFFER
// Main Firmware Entry Point
// =============================================================================

// Global State Variables (accessible across all modules)
volatile bool  g_droneIncoming          = false;
volatile float g_rfThreatPercent        = 0.0f;
volatile float g_rfDbm                  = -65.0f;
volatile float g_rfMilliVolts           = 2100.0f;
volatile bool  g_acousticDroneConfirmed = false;
volatile float g_acousticConfidence     = 0.0f;
volatile bool  g_dualThreatConfirmed    = false;

// Subsystem singletons
RFDetector       rfDetector;
#if ACOUSTIC_DETECTOR_ENABLED
AcousticDetector acousticDetector;
#endif
WiFiDetector     wifiDetector;
AudioCue         audioCue;
#if ACOUSTIC_DETECTOR_ENABLED
Telemetry        telemetry(rfDetector, acousticDetector, audioCue);
#endif

// Previous state for transition edge detection
static bool     s_lastDroneIncomingState  = false;
static bool     s_lastDualConfirmedState  = false;
static uint32_t s_lastAlertAudioPlayMs    = 0;

// -----------------------------------------------------------------------------
// DRONE INCOMING TRIGGER HOOKS
// These functions are invoked automatically upon transition edges of g_droneIncoming
// -----------------------------------------------------------------------------
void onDroneIncomingTriggered(float dbm, float threatPct) {
#if SERIAL_LOGGING_ENABLED
    Serial.println();
    Serial.println(F("**************************************************"));
    if (g_dualThreatConfirmed) {
        Serial.println(F(" [ALERT] >>> DUAL-MODALITY THREAT CONFIRMED! <<<"));
        Serial.println(F(" [ALERT] Parallel 5.8 GHz RF + Acoustic Blade-Pass Verified!"));
    } else {
        Serial.println(F(" [ALERT] >>> INCOMING DRONE DETECTED! <<<"));
    }
    Serial.printf( " [ALERT] Target 5.8G Band: %s (GPIO 4)\n", RF_TARGET_BAND_LABEL);
    Serial.printf( " [ALERT] RF Power: %.1f dBm | Threat: %.0f%%\n", dbm, threatPct);
#if ACOUSTIC_DETECTOR_ENABLED
    if (acousticDetector.isDroneConfirmed() || acousticDetector.isWhineDetected()) {
        Serial.printf(" [ALERT] Acoustic Sentry: Confirmed (Confidence: %.1f%% | Energy: %.0f)\n",
                      acousticDetector.getDroneConfidence() * 100.0f,
                      acousticDetector.getLatestFrameEnergy());
    }
#endif
    if (g_drone24Detected) {
        Serial.printf(" [ALERT] 2.4 GHz Drone: Channel %d | RSSI: %d dBm | Vendor: %d\n",
                      g_drone24Channel, g_drone24Rssi, g_drone24Vendor);
        if (g_droneBehaviorFlags) {
            Serial.printf(" [ALERT] Behavioral Signatures: 0x%02X | Clock/TDD Rate: %d Hz | Channels: %d\n",
                          g_droneBehaviorFlags, g_droneDetectedRateHz, g_droneHopCount);
        }
    }
    Serial.println(F("**************************************************"));
    Serial.println();
#endif

    // If dual-modality confirmed, emit a priority warning chirp tone before speech
    if (g_dualThreatConfirmed) {
#if ACOUSTIC_DETECTOR_ENABLED
        acousticDetector.triggerHeadphoneBeep(AUDIO_BEEP_FREQ_HZ, AUDIO_BEEP_DURATION_MS);
#endif
    }

    // Trigger recorded voice alert ("Дрон летит" / "Dron letit") via PWM DAC on GPIO 15
    audioCue.playAlertAudio();
    s_lastAlertAudioPlayMs = millis();
}

void onDroneIncomingCleared() {
#if SERIAL_LOGGING_ENABLED
    Serial.println();
    Serial.println(F("[INFO] Drone signal dropped below threshold. Alert cleared. Resuming scan."));
    Serial.println();
#endif
}

// FreeRTOS task handle for background sensing on Core 0 (I2S DMA audio & 2.4 GHz radio)
TaskHandle_t backgroundTaskHandle = NULL;

// Background task pinned to Core 0
void backgroundTask(void* parameter) {
    for (;;) {
#if ACOUSTIC_DETECTOR_ENABLED
        // Continuously buffer DMA audio and run Stage 1 sentry filter on Core 0
        acousticDetector.update();
#endif
        wifiDetector.update();
        // Yield briefly to service Core 0 watchdog and IDLE task
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void setup() {
    // 1. Initialize Telemetry & USB Serial interface
#if SERIAL_LOGGING_ENABLED && ACOUSTIC_DETECTOR_ENABLED
    telemetry.begin();
#else
    Serial.begin(115200);
#endif

    // 2. Initialize AD8318 RF Power Detector (GPIO 4, ADC1)
    rfDetector.begin();

    // 3. Initialize Headphone Audio Cue Engine (GPIO 15)
    audioCue.begin();

    // 4. Initialize Visual Alert LED Indicator (GPIO 16)
    pinMode(PIN_ALERT_LED, OUTPUT);
    digitalWrite(PIN_ALERT_LED, LOW);

    // 5. Initialize INMP441 I2S Microphone and perform 3-second ambient calibration
#if ACOUSTIC_DETECTOR_ENABLED
    if (!acousticDetector.begin()) {
        Serial.println(F("[ERROR] Failed to initialize INMP441 I2S microphone driver!"));
    } else {
        Serial.println(F("[OK] INMP441 I2S microphone initialized (16 kHz, DMA 4x512)."));
        // Dynamic ambient calibration: sample quiet ambient noise for 3 seconds on startup
        acousticDetector.calibrateAmbientFloor(ACOUSTIC_CALIBRATION_DURATION_MS);
    }
#endif

    // 6. Initialize ESP32-S3 Internal 2.4 GHz Wi-Fi Promiscuous Drone Sniffer
    if (!wifiDetector.begin()) {
        Serial.println(F("[ERROR] Failed to initialize 2.4 GHz Wi-Fi promiscuous sniffer!"));
    } else {
        Serial.println(F("[OK] 2.4 GHz internal radio sniffer active (Channels 1-13)."));
    }

    // 7. Launch Background Sensing on Core 0 (I2S DMA Audio & 2.4 GHz Channel Hopper)
    xTaskCreatePinnedToCore(
        backgroundTask,        // Task function
        "BackgroundTask",      // Task name
        8192,                  // Stack size (bytes, expanded for TinyML / DMA)
        NULL,                  // Parameter
        1,                     // Priority
        &backgroundTaskHandle, // Task handle
        0                      // Core ID (Core 0)
    );

    Serial.println(F("[SYSTEM READY] ESP32-S3 Dual-Modality Drone Sniffer Active."));
    Serial.println(F("               Listening for blade-pass acoustic signatures & RF carriers...\n"));

#if BATTERY_OPTIMIZATION_ENABLED
    setCpuFrequencyMhz(80); // Start in low-power 80 MHz mode to conserve battery
#endif
}

void loop() {
    // 0. Dynamic CPU Frequency Scaling: 80 MHz in quiet air, 240 MHz during threat
#if BATTERY_OPTIMIZATION_ENABLED
    static uint32_t s_lastThreatActiveMs = 0;
    bool systemActive = g_droneIncoming || rfDetector.isBurstDetected() || 
                        (g_rfThreatPercent >= RF_THREAT_MIN_PERCENT);
#if ACOUSTIC_DETECTOR_ENABLED
    systemActive |= acousticDetector.isStage1Triggered();
#endif
    if (systemActive) {
        s_lastThreatActiveMs = millis();
        if (getCpuFrequencyMhz() < 240) {
            setCpuFrequencyMhz(240); // Max clock for instantaneous RF response & TinyML
        }
    } else if (millis() - s_lastThreatActiveMs >= 3000) {
        // 3 seconds of calm air -> scale down to 80 MHz to save >60% CPU power
        if (getCpuFrequencyMhz() > 80) {
            setCpuFrequencyMhz(80);
        }
    }
#endif

    // 1. Fast parallel RF power sampling and EMA filtering (Core 1, GPIO 4)
    rfDetector.update();

    // 2. Dual-Modality Threat Fusion:
    // Poll parallel analog RF (AD8318) and acoustic classifier (INMP441)
    bool rfElevated       = (rfDetector.getThreatPercent() >= DUAL_THREAT_RF_MIN_PCT) || rfDetector.isBurstDetected();
    bool rfThreatActive   = rfDetector.isDroneIncoming();
    bool wifiThreatActive = wifiDetector.isDroneDetected();

#if ACOUSTIC_DETECTOR_ENABLED
    bool acousticConfirmed = acousticDetector.isDroneConfirmed();
    bool acousticElevated  = acousticConfirmed || (acousticDetector.isStage1Triggered() && acousticDetector.isWhineDetected());
    bool acousticThreatActive = acousticElevated;
#else
    bool acousticConfirmed    = false;
    bool acousticElevated     = false;
    bool acousticThreatActive = false;
#endif

    // Dual-Modality Confirmation: Simultaneous 5.8 GHz RF carrier AND acoustic blade-pass
    g_dualThreatConfirmed = (rfElevated && acousticElevated);

    // Global threat declaration
    g_droneIncoming = rfThreatActive || wifiThreatActive || acousticThreatActive || g_dualThreatConfirmed;

    // 3. Evaluate Drone Incoming Alert trigger transition edges
    if (g_droneIncoming && !s_lastDroneIncomingState) {
        // Rising edge: Incoming drone confirmed!
        onDroneIncomingTriggered(g_rfDbm, g_rfThreatPercent);
    } else if (g_dualThreatConfirmed && !s_lastDualConfirmedState) {
        // Rising edge of dual-modality confirmation
        onDroneIncomingTriggered(g_rfDbm, g_rfThreatPercent);
    } else if (!g_droneIncoming && s_lastDroneIncomingState) {
        // Falling edge: Signal cleared
        onDroneIncomingCleared();
    } else if (g_droneIncoming) {
        // Sustained drone threat: periodic voice reminder if configured
#if AUDIO_ALERT_REPEAT_INTERVAL_MS > 0
        if (!audioCue.isAudioPlaying() && (millis() - s_lastAlertAudioPlayMs >= AUDIO_ALERT_REPEAT_INTERVAL_MS)) {
            audioCue.playAlertAudio();
            s_lastAlertAudioPlayMs = millis();
        }
#endif
    }
    s_lastDroneIncomingState = g_droneIncoming;
    s_lastDualConfirmedState = g_dualThreatConfirmed;

    // 4. Update Visual Alert LED (Illuminates immediately when drone is incoming)
    digitalWrite(PIN_ALERT_LED, g_droneIncoming ? HIGH : LOW);

    // 5. Audio Feedback Cue:
    // Elevate effective threat rate based on multi-sensor confirmation
    float effectiveThreat = rfDetector.getThreatPercent();
    if (g_dualThreatConfirmed) {
        effectiveThreat = 95.0f; // Maximum Geiger click rate for dual-modality verified threat
    } else if (acousticConfirmed && effectiveThreat < 50.0f) {
        effectiveThreat = 75.0f; // High click rate for confirmed acoustic drone target
    } else if (wifiThreatActive && effectiveThreat < 50.0f) {
        effectiveThreat = 60.0f; // Rapid click rate for confirmed 2.4 GHz drone target
    }
    audioCue.update(effectiveThreat, acousticThreatActive);

    // 6. Live serial telemetry and command parsing (if logging enabled)
#if SERIAL_LOGGING_ENABLED && ACOUSTIC_DETECTOR_ENABLED
    telemetry.update();
#endif

    // Minor yield to prevent Core 1 loop starving system tasks
    yield();
}
