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
volatile bool  g_droneIncoming   = false;
volatile float g_rfThreatPercent = 0.0f;
volatile float g_rfDbm           = -65.0f;
volatile float g_rfMilliVolts    = 2100.0f;

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
static bool     s_lastDroneIncomingState = false;
static uint32_t s_lastAlertAudioPlayMs   = 0;

// -----------------------------------------------------------------------------
// DRONE INCOMING TRIGGER HOOKS
// These functions are invoked automatically upon transition edges of g_droneIncoming
// -----------------------------------------------------------------------------
void onDroneIncomingTriggered(float dbm, float threatPct) {
#if SERIAL_LOGGING_ENABLED
    Serial.println();
    Serial.println(F("**************************************************"));
    Serial.println(F(" [ALERT] >>> INCOMING DRONE DETECTED! <<<"));
    Serial.printf( " [ALERT] Target 5.8G Band: %s (GPIO 4)\n", RF_TARGET_BAND_LABEL);
    Serial.printf( " [ALERT] RF Power: %.1f dBm | Threat: %.0f%%\n", dbm, threatPct);
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

// FreeRTOS task handle for background 2.4 GHz radio (and optional audio) processing on Core 0
TaskHandle_t backgroundTaskHandle = NULL;

// Background task pinned to Core 0 (2.4 GHz channel hopper & optional I2S audio)
void backgroundTask(void* parameter) {
    for (;;) {
#if ACOUSTIC_DETECTOR_ENABLED
        acousticDetector.update();
#endif
        wifiDetector.update();
        // Yield briefly to prevent watchdog trigger on Core 0
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    // 1. Initialize Telemetry & USB Serial interface (if logging enabled)
#if SERIAL_LOGGING_ENABLED && ACOUSTIC_DETECTOR_ENABLED
    telemetry.begin();
#endif

    // 2. Initialize AD8318 RF Power Detector (GPIO 4, ADC1)
    rfDetector.begin();

    // 3. Initialize Headphone Audio Cue Engine (GPIO 15)
    audioCue.begin();

    // 4. Initialize Visual Alert LED Indicator (GPIO 16)
    pinMode(PIN_ALERT_LED, OUTPUT);
    digitalWrite(PIN_ALERT_LED, LOW);

    // 5. Initialize INMP441 I2S Microphone (if enabled)
#if ACOUSTIC_DETECTOR_ENABLED
    if (!acousticDetector.begin()) {
#if SERIAL_LOGGING_ENABLED
        Serial.println(F("[ERROR] Failed to initialize INMP441 I2S microphone driver!"));
#endif
    } else {
#if SERIAL_LOGGING_ENABLED
        Serial.println(F("[OK] INMP441 I2S microphone initialized successfully."));
#endif
    }
#endif

    // 6. Initialize ESP32-S3 Internal 2.4 GHz Wi-Fi Promiscuous Drone Sniffer
    if (!wifiDetector.begin()) {
#if SERIAL_LOGGING_ENABLED
        Serial.println(F("[ERROR] Failed to initialize 2.4 GHz Wi-Fi promiscuous sniffer!"));
#endif
    } else {
#if SERIAL_LOGGING_ENABLED
        Serial.println(F("[OK] 2.4 GHz internal radio sniffer active (Channels 1-13)."));
#endif
    }

    // 7. Launch Background Sensing on Core 0 (2.4 GHz Channel Hopper)
    xTaskCreatePinnedToCore(
        backgroundTask,        // Task function
        "BackgroundTask",      // Task name
        4096,                  // Stack size (bytes)
        NULL,                  // Parameter
        1,                     // Priority
        &backgroundTaskHandle, // Task handle
        0                      // Core ID (Core 0)
    );

#if SERIAL_LOGGING_ENABLED
    Serial.println(F("[SYSTEM] Drone Sniffer detection pipeline active.\n"));
#endif
}

void loop() {
    // 1. Fast RF power sampling and EMA filtering (Core 1, GPIO 4)
    rfDetector.update();

    // 2. Multi-Sensor Threat Fusion:
    // Combine 5.8 GHz analog RF and 2.4 GHz Wi-Fi drone packets
    bool rfThreatActive       = rfDetector.isDroneIncoming();
    bool wifiThreatActive     = wifiDetector.isDroneDetected();
#if ACOUSTIC_DETECTOR_ENABLED
    bool acousticThreatActive = acousticDetector.isWhineDetected();
#else
    bool acousticThreatActive = false;
#endif

    g_droneIncoming = rfThreatActive || wifiThreatActive || acousticThreatActive;

    // 3. Evaluate Drone Incoming Alert trigger transition edges
    if (g_droneIncoming && !s_lastDroneIncomingState) {
        // Rising edge: Incoming drone confirmed!
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

    // 4. Update Visual Alert LED (Illuminates immediately when drone is incoming)
    digitalWrite(PIN_ALERT_LED, g_droneIncoming ? HIGH : LOW);

    // 5. Audio Feedback Cue:
    // Elevate effective threat if a confirmed 2.4 GHz digital drone link is detected
    float effectiveThreat = rfDetector.getThreatPercent();
    if (wifiThreatActive && effectiveThreat < 50.0f) {
        effectiveThreat = 60.0f; // Rapid Geiger click rate for confirmed 2.4 GHz drone target
    }
    audioCue.update(effectiveThreat, acousticThreatActive);

    // 6. Live serial telemetry and command parsing (if logging enabled)
#if SERIAL_LOGGING_ENABLED && ACOUSTIC_DETECTOR_ENABLED
    telemetry.update();
#endif

    // Minor yield to prevent Core 1 loop starving system tasks
    yield();
}
