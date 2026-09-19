#include "audio_cue.h"
#include <esp_arduino_version.h>
#include <math.h>

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    #define USE_LEDC_V3 1
#else
    #define USE_LEDC_V3 0
#endif

AudioCue::AudioCue()
    : _muted(false),
      _pulseActive(false),
      _pulseStartTimeUs(0),
      _lastClickTimeMs(0),
      _currentPeriodMs(AUDIO_QUIESCENT_PERIOD_MS),
      _audioPlaying(false),
      _audioSampleIndex(0),
      _audioTimer(NULL),
      _clickCounter(0),
      _clicksPerSec(0),
      _lastRateWindowMs(0) {
}

AudioCue::~AudioCue() {
    stopAlertAudio();
    if (_audioTimer != NULL) {
        esp_timer_stop(_audioTimer);
        esp_timer_delete(_audioTimer);
        _audioTimer = NULL;
    }
}

void AudioCue::begin() {
#if AUDIO_JACK_ENABLED
    pinMode(PIN_AUDIO_JACK, OUTPUT);
    digitalWrite(PIN_AUDIO_JACK, LOW);

#if USE_LEDC_V3
    ledcAttach(PIN_AUDIO_JACK, AUDIO_CLICK_FREQ_HZ, AUDIO_LEDC_TIMER_RES);
    ledcWrite(PIN_AUDIO_JACK, 0);
#else
    ledcSetup(AUDIO_LEDC_CHANNEL, AUDIO_CLICK_FREQ_HZ, AUDIO_LEDC_TIMER_RES);
    ledcAttachPin(PIN_AUDIO_JACK, AUDIO_LEDC_CHANNEL);
    ledcWrite(AUDIO_LEDC_CHANNEL, 0);
#endif

    _lastClickTimeMs = millis();
    _lastRateWindowMs = millis();

    // Create high-resolution periodic timer for 8-bit PWM DAC sample playback
    esp_timer_create_args_t timerArgs = {};
    timerArgs.callback = &AudioCue::audioTimerCallback;
    timerArgs.arg = this;
    timerArgs.dispatch_method = ESP_TIMER_TASK;
    timerArgs.name = "audio_dac_timer";
    timerArgs.skip_unhandled_events = true;

    esp_err_t err = esp_timer_create(&timerArgs, &_audioTimer);
    if (err != ESP_OK) {
        _audioTimer = NULL;
    }
#else
    _lastClickTimeMs = millis();
    _lastRateWindowMs = millis();
#endif
}

void AudioCue::playAlertAudio(bool forceRestart) {
#if !AUDIO_JACK_ENABLED
    return; // Headphone jack disabled in config to conserve battery
#endif
    if (_muted) return;
    if (_audioPlaying && !forceRestart) return;

    // Terminate any active Geiger click pulse
    stopAudio();

    if (_audioPlaying && _audioTimer != NULL) {
        esp_timer_stop(_audioTimer);
    }

    _audioPlaying = false;
    _audioSampleIndex = 0;

    // Switch LEDC to high-frequency ultrasonic PWM carrier (e.g. 78.125 kHz)
    // for seamless 8-bit DAC emulation
#if USE_LEDC_V3
    ledcAttach(PIN_AUDIO_JACK, AUDIO_PWM_DAC_FREQ_HZ, AUDIO_LEDC_TIMER_RES);
    ledcWrite(PIN_AUDIO_JACK, 128);
#else
    ledcSetup(AUDIO_LEDC_CHANNEL, AUDIO_PWM_DAC_FREQ_HZ, AUDIO_LEDC_TIMER_RES);
    ledcAttachPin(PIN_AUDIO_JACK, AUDIO_LEDC_CHANNEL);
    ledcWrite(AUDIO_LEDC_CHANNEL, 128);
#endif

    _audioPlaying = true;

    if (_audioTimer != NULL) {
        uint64_t periodUs = 1000000ULL / AUDIO_ALERT_SAMPLE_RATE;
        esp_timer_start_periodic(_audioTimer, periodUs);
    }
}

void AudioCue::stopAlertAudio() {
    if (_audioPlaying) {
        _audioPlaying = false;
#if AUDIO_JACK_ENABLED
        if (_audioTimer != NULL) {
            esp_timer_stop(_audioTimer);
        }
#if USE_LEDC_V3
        ledcWrite(PIN_AUDIO_JACK, 0);
#else
        ledcWrite(AUDIO_LEDC_CHANNEL, 0);
#endif
#endif
    }
}

void AudioCue::setMuted(bool mute) {
    _muted = mute;
    if (_muted) {
        stopAlertAudio();
        stopAudio();
    }
}

void IRAM_ATTR AudioCue::audioTimerCallback(void* arg) {
    AudioCue* self = static_cast<AudioCue*>(arg);
    if (self != NULL) {
        self->handleAudioTick();
    }
}

void IRAM_ATTR AudioCue::handleAudioTick() {
    if (!_audioPlaying) return;

    if (_audioSampleIndex < AUDIO_ALERT_SAMPLE_COUNT) {
        uint8_t sample = DRONE_ALERT_AUDIO_DATA[_audioSampleIndex++];
#if USE_LEDC_V3
        ledcWrite(PIN_AUDIO_JACK, sample);
#else
        ledcWrite(AUDIO_LEDC_CHANNEL, sample);
#endif
    } else {
        // Sample playback complete
        _audioPlaying = false;
        if (_audioTimer != NULL) {
            esp_timer_stop(_audioTimer);
        }
#if USE_LEDC_V3
        ledcWrite(PIN_AUDIO_JACK, 0);
#else
        ledcWrite(AUDIO_LEDC_CHANNEL, 0);
#endif
    }
}

void AudioCue::startClick(uint32_t freqHz) {
#if !AUDIO_JACK_ENABLED
    return; // Headphone jack disabled in config to conserve battery
#endif
    if (_muted || _audioPlaying) return;

#if USE_LEDC_V3
    ledcWriteTone(PIN_AUDIO_JACK, freqHz);
    // 50% duty cycle for clean square wave
    ledcWrite(PIN_AUDIO_JACK, 128);
#else
    ledcWriteTone(AUDIO_LEDC_CHANNEL, freqHz);
    ledcWrite(AUDIO_LEDC_CHANNEL, 128);
#endif

    _pulseActive = true;
    _pulseStartTimeUs = micros();
    _clickCounter++;
}

void AudioCue::stopAudio() {
#if AUDIO_JACK_ENABLED
#if USE_LEDC_V3
    ledcWrite(PIN_AUDIO_JACK, 0);
#else
    ledcWrite(AUDIO_LEDC_CHANNEL, 0);
#endif
#endif
    _pulseActive = false;
}

void AudioCue::update(float threatPercent, bool whineDetected) {
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();

    // 1. Calculate clicks per second for diagnostics
    if (nowMs - _lastRateWindowMs >= 1000) {
        _clicksPerSec = _clickCounter;
        _clickCounter = 0;
        _lastRateWindowMs = nowMs;
    }

    // 2. If voice alert audio is currently playing, suspend clicks
    if (_audioPlaying) {
        _lastClickTimeMs = nowMs;
        return;
    }

    // 3. Terminate active click pulse after AUDIO_CLICK_PULSE_US
    if (_pulseActive) {
        if ((nowUs - _pulseStartTimeUs) >= AUDIO_CLICK_PULSE_US) {
            stopAudio();
        }
    }

    // 4. Map RF threat percentage to Geiger click period using power curve
    if (threatPercent <= 0.0f) {
        // Calm clean air: sporadic heartbeat tick
        _currentPeriodMs = AUDIO_QUIESCENT_PERIOD_MS;
    } else {
        float t = threatPercent / 100.0f;
        if (t > 1.0f) t = 1.0f;

        // Exponential / power curve mapping for natural Geiger counter audio feel
        float factor = powf(1.0f - t, 2.5f);
        _currentPeriodMs = AUDIO_MIN_PERIOD_MS + static_cast<uint32_t>((AUDIO_QUIESCENT_PERIOD_MS - AUDIO_MIN_PERIOD_MS) * factor);
    }

    // 5. Trigger next click when interval expires
    if (nowMs - _lastClickTimeMs >= _currentPeriodMs) {
        _lastClickTimeMs = nowMs;

        // Choose tone frequency based on whether acoustic whine is also flagged
        uint32_t clickFreq = AUDIO_CLICK_FREQ_HZ;
        if (whineDetected) {
            // Dual-tone modulation when acoustic motor whine is confirmed
            clickFreq = (_clickCounter % 2 == 0) ? AUDIO_WHINE_FREQ_HZ : (AUDIO_CLICK_FREQ_HZ + 600);
        }

        startClick(clickFreq);
    }
}
