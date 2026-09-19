#pragma once

#include <Arduino.h>
#include <esp_timer.h>
#include "config.h"
#include "audio_samples.h"

// =============================================================================
// Audio Cue Engine (PJ-392 3.5mm Headphone Jack on GPIO 15)
// Dual-Mode Audio Generator:
// 1. High-Speed PWM DAC (LEDC 78.125 kHz carrier) for playing recorded voice alerts
//    ("Дрон летит" / "Drone Incoming") stored in Flash (PROGMEM)
// 2. Geiger-counter style clicks with rate inversely proportional to RF voltage
// =============================================================================

class AudioCue {
public:
    AudioCue();
    ~AudioCue();

    // Initialize LEDC PWM channel, audio pin, and sample playback timer
    void begin();

    // Non-blocking state machine tick (call every loop iteration)
    // - threatPercent: 0.0 to 100.0 (from RFDetector)
    // - whineDetected: true if acoustic motor whine is flagged
    void update(float threatPercent, bool whineDetected);

    // Play recorded voice alert from flash memory
    // forceRestart: if true, restarts playback even if already playing
    void playAlertAudio(bool forceRestart = false);

    // Stop recorded audio alert immediately
    void stopAlertAudio();

    // Query if recorded voice alert is currently playing
    bool isAudioPlaying() const { return _audioPlaying; }

    // Mute or unmute headphone audio
    void setMuted(bool mute);
    bool isMuted() const { return _muted; }

    // Number of clicks produced in the last 1 second window (for telemetry)
    uint16_t getClicksPerSecond() const { return _clicksPerSec; }

    // Internal timer callback handler (called from esp_timer)
    void IRAM_ATTR handleAudioTick();

private:
    void startClick(uint32_t freqHz);
    void stopAudio();

    static void IRAM_ATTR audioTimerCallback(void* arg);

    bool     _muted;
    bool     _pulseActive;
    uint32_t _pulseStartTimeUs;
    uint32_t _lastClickTimeMs;
    uint32_t _currentPeriodMs;

    // Sample playback state
    volatile bool      _audioPlaying;
    volatile uint32_t  _audioSampleIndex;
    esp_timer_handle_t _audioTimer;

    // Click rate counter for telemetry
    uint16_t _clickCounter;
    uint16_t _clicksPerSec;
    uint32_t _lastRateWindowMs;
};
