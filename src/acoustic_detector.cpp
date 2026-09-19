#include "acoustic_detector.h"
#include <driver/i2s.h>
#include <esp_arduino_version.h>
#include <math.h>

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    #define USE_LEDC_V3 1
#else
    #define USE_LEDC_V3 0
#endif

// Static ring buffer storage
int16_t          AcousticDetector::s_audioRingBuffer[INFERENCE_SAMPLES] = { 0 };
volatile size_t  AcousticDetector::s_bufferHead = 0;
volatile size_t  AcousticDetector::s_totalSamplesRecorded = 0;
volatile uint32_t AcousticDetector::s_dmaDropCount = 0;

AcousticDetector::AcousticDetector()
    : _rmsWideband(0.0f),
      _rmsWhineBand(0.0f),
      _peakAmplitude(0),
      _whineRatio(0.0f),
      _whineDetected(false),
      _latestFrameEnergy(0.0f),
      _ambientBaselineEnergy(0.0f),
      _energyThreshold(ACOUSTIC_DEFAULT_ENERGY_THRESHOLD),
      _stage1Triggered(false),
      _droneConfirmed(false),
      _droneConfidence(0.0f),
      _isCalibrated(false),
      _hp_b0(0), _hp_b1(0), _hp_b2(0), _hp_a1(0), _hp_a2(0),
      _hp_z1(0), _hp_z2(0),
      _lp_b0(0), _lp_b1(0), _lp_b2(0), _lp_a1(0), _lp_a2(0),
      _lp_z1(0), _lp_z2(0) {
    memset(_sampleBuffer, 0, sizeof(_sampleBuffer));
}

bool AcousticDetector::begin() {
    computeBiquadCoefficients();

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_DMA_BUFFER_COUNT,
        .dma_buf_len = I2S_DMA_BUFFER_SAMPLES,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = PIN_I2S_SCK,
        .ws_io_num = PIN_I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = PIN_I2S_SD
    };

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        return false;
    }

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        return false;
    }

    // Flush initial dummy data
    size_t bytesRead = 0;
    i2s_read(I2S_NUM_0, _sampleBuffer, sizeof(_sampleBuffer), &bytesRead, pdMS_TO_TICKS(50));

    return true;
}

void AcousticDetector::computeBiquadCoefficients() {
    const float Fs = static_cast<float>(I2S_SAMPLE_RATE);
    const float Q  = 0.7071f; // Butterworth Q factor

    // 1. High-Pass Filter @ ACOUSTIC_BAND_LOW_HZ (200 Hz)
    float omega_hp = 2.0f * static_cast<float>(M_PI) * (ACOUSTIC_BAND_LOW_HZ / Fs);
    float cos_hp   = cosf(omega_hp);
    float sin_hp   = sinf(omega_hp);
    float alpha_hp = sin_hp / (2.0f * Q);

    float hp_a0 = 1.0f + alpha_hp;
    _hp_b0 = ((1.0f + cos_hp) / 2.0f) / hp_a0;
    _hp_b1 = (-(1.0f + cos_hp))       / hp_a0;
    _hp_b2 = ((1.0f + cos_hp) / 2.0f) / hp_a0;
    _hp_a1 = (-2.0f * cos_hp)         / hp_a0;
    _hp_a2 = (1.0f - alpha_hp)        / hp_a0;

    // 2. Low-Pass Filter @ ACOUSTIC_BAND_HIGH_HZ (2500 Hz)
    float omega_lp = 2.0f * static_cast<float>(M_PI) * (ACOUSTIC_BAND_HIGH_HZ / Fs);
    float cos_lp   = cosf(omega_lp);
    float sin_lp   = sinf(omega_lp);
    float alpha_lp = sin_lp / (2.0f * Q);

    float lp_a0 = 1.0f + alpha_lp;
    _lp_b0 = ((1.0f - cos_lp) / 2.0f) / lp_a0;
    _lp_b1 = (1.0f - cos_lp)          / lp_a0;
    _lp_b2 = ((1.0f - cos_lp) / 2.0f) / lp_a0;
    _lp_a1 = (-2.0f * cos_lp)         / lp_a0;
    _lp_a2 = (1.0f - alpha_lp)        / lp_a0;

    // Reset filter states
    _hp_z1 = _hp_z2 = 0.0f;
    _lp_z1 = _lp_z2 = 0.0f;
}

float AcousticDetector::processBandpassFilter(float input) {
    // Stage 1: High-Pass Filter (Direct Form II Transposed)
    float hp_out = (_hp_b0 * input) + _hp_z1;
    _hp_z1 = (_hp_b1 * input) - (_hp_a1 * hp_out) + _hp_z2;
    _hp_z2 = (_hp_b2 * input) - (_hp_a2 * hp_out);

    // Stage 2: Low-Pass Filter (Direct Form II Transposed)
    float lp_out = (_lp_b0 * hp_out) + _lp_z1;
    _lp_z1 = (_lp_b1 * hp_out) - (_lp_a1 * lp_out) + _lp_z2;
    _lp_z2 = (_lp_b2 * hp_out) - (_lp_a2 * lp_out);

    return lp_out;
}

void AcousticDetector::calibrateAmbientFloor(uint32_t durationMs) {
#if SERIAL_LOGGING_ENABLED
    Serial.println();
    Serial.printf("[CALIBRATION] Sampling quiet ambient noise for %u ms...\n", durationMs);
#endif

    double totalEnergy = 0.0;
    uint32_t frameCount = 0;
    uint32_t startMs = millis();
    int32_t rawDma[I2S_DMA_BUFFER_SAMPLES];

    while (millis() - startMs < durationMs) {
        size_t bytesRead = 0;
        esp_err_t err = i2s_read(I2S_NUM_0, rawDma, sizeof(rawDma), &bytesRead, pdMS_TO_TICKS(50));
        if (err == ESP_OK && bytesRead > 0) {
            size_t count = bytesRead / sizeof(int32_t);
            double frameEnergy = 0.0;
            for (size_t i = 0; i < count; ++i) {
                int16_t sample16 = static_cast<int16_t>(rawDma[i] >> 14);
                // Buffer into circular ring
                s_audioRingBuffer[s_bufferHead] = sample16;
                s_bufferHead = (s_bufferHead + 1) % INFERENCE_SAMPLES;
                s_totalSamplesRecorded++;

                frameEnergy += static_cast<double>(sample16) * static_cast<double>(sample16);
            }
            frameEnergy /= count;
            totalEnergy += frameEnergy;
            frameCount++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (frameCount > 0) {
        _ambientBaselineEnergy = static_cast<float>(totalEnergy / frameCount);
        _energyThreshold = _ambientBaselineEnergy * ACOUSTIC_ENERGY_THRESHOLD_MULTIPLIER;
        if (_energyThreshold < 20000.0f) {
            _energyThreshold = 20000.0f; // Sane floor
        }
    } else {
        _ambientBaselineEnergy = 400000.0f;
        _energyThreshold = ACOUSTIC_DEFAULT_ENERGY_THRESHOLD;
    }
    _isCalibrated = true;

#if SERIAL_LOGGING_ENABLED
    Serial.printf("[CALIBRATION] Complete. Baseline Energy: %.1f | Threshold set to: %.1f (%.1fx margin)\n\n",
                  _ambientBaselineEnergy, _energyThreshold, ACOUSTIC_ENERGY_THRESHOLD_MULTIPLIER);
#endif
}

int AcousticDetector::getSignalDataCallback(size_t offset, size_t length, float *out_ptr) {
    if (out_ptr == nullptr || offset + length > INFERENCE_SAMPLES) {
        return -1;
    }
    // In a circular buffer of INFERENCE_SAMPLES, s_bufferHead points to the next write slot,
    // which is simultaneously the oldest sample in the filled ring.
    size_t head = s_bufferHead;
    for (size_t i = 0; i < length; ++i) {
        size_t ringIdx = head + offset + i;
        if (ringIdx >= INFERENCE_SAMPLES) {
            ringIdx %= INFERENCE_SAMPLES;
        }
        out_ptr[i] = static_cast<float>(s_audioRingBuffer[ringIdx]);
    }
    return 0; // EI_IMPULSE_OK
}

bool AcousticDetector::runInference(float* outConfidence) {
#if HAVE_EDGE_IMPULSE
    signal_t signal;
    signal.total_length = INFERENCE_SAMPLES;
    signal.get_data = &AcousticDetector::getSignalDataCallback;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

    if (r == EI_IMPULSE_OK) {
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "drone") == 0) {
                float conf = result.classification[ix].value;
                if (outConfidence) *outConfidence = conf;
                if (conf >= ACOUSTIC_CONFIDENCE_THRESHOLD) {
                    return true;
                }
            }
        }
    }
    return false;
#elif HAVE_PURE_C_MODEL
    // Crazy-Lightweight Pure-C Model: < 1 ms inference, < 4 KB RAM
    // Linearize ring buffer into temporary buffer for feature extraction
    static int16_t s_linearRingCopy[INFERENCE_SAMPLES];
    size_t head = s_bufferHead;
    for (size_t i = 0; i < INFERENCE_SAMPLES; i++) {
        size_t ringIdx = (head + i);
        if (ringIdx >= INFERENCE_SAMPLES) ringIdx -= INFERENCE_SAMPLES;
        s_linearRingCopy[i] = s_audioRingBuffer[ringIdx];
    }

    float features[DRONE_MODEL_NUM_FEATURES];
    extract_audio_features(s_linearRingCopy, INFERENCE_SAMPLES, features);

    float prob = drone_model_predict_prob(features);
    if (outConfidence) {
        *outConfidence = prob;
    }
    return (prob >= ACOUSTIC_CONFIDENCE_THRESHOLD);
#else
    // Standalone fallback: exercises the getSignalDataCallback and applies Stage 2 DSP confirmation
    float testChunk[320];
    getSignalDataCallback(0, 320, testChunk); // Validates callback execution

    // When whine ratio >= 35% and frame energy > threshold, flag drone
    float estimatedConfidence = (_whineRatio >= ACOUSTIC_WHINE_RATIO_THRESH) ? 
                                fminf(0.95f, 0.65f + (_whineRatio * 0.5f)) : 0.20f;
    if (outConfidence) {
        *outConfidence = estimatedConfidence;
    }
    return (estimatedConfidence >= ACOUSTIC_CONFIDENCE_THRESHOLD);
#endif
}

void AcousticDetector::triggerHeadphoneBeep(int freqHz, int durationMs) {
#if USE_LEDC_V3
    ledcAttach(PIN_AUDIO_JACK, freqHz, AUDIO_LEDC_TIMER_RES);
    ledcWrite(PIN_AUDIO_JACK, 128); // 50% duty cycle tone
    delay(durationMs);
    ledcWrite(PIN_AUDIO_JACK, 0);
#else
    ledcSetup(AUDIO_LEDC_CHANNEL, freqHz, AUDIO_LEDC_TIMER_RES);
    ledcAttachPin(PIN_AUDIO_JACK, AUDIO_LEDC_CHANNEL);
    ledcWrite(AUDIO_LEDC_CHANNEL, 128);
    delay(durationMs);
    ledcWrite(AUDIO_LEDC_CHANNEL, 0);
#endif
}

bool AcousticDetector::update() {
    size_t bytesRead = 0;
    // Bounded DMA read: 512 samples @ 16 kHz is 32 ms. Timeout 40 ms prevents stalling Core 0.
    esp_err_t res = i2s_read(I2S_NUM_0, _sampleBuffer, sizeof(_sampleBuffer), &bytesRead, pdMS_TO_TICKS(40));
    if (res != ESP_OK || bytesRead == 0) {
        s_dmaDropCount++;
        return false;
    }

    const size_t numSamples = bytesRead / sizeof(int32_t);
    if (numSamples < I2S_DMA_BUFFER_SAMPLES) {
        s_dmaDropCount++; // Partial read indicates DMA frame underrun
    }

    double sumSqRaw = 0.0;
    float sumSqWhineBand = 0.0f;
    int32_t maxAmp = 0;

    for (size_t i = 0; i < numSamples; ++i) {
        // INMP441 outputs 24-bit MSB-aligned in 32-bit frame. Shift by 14 to obtain 16-bit signed PCM.
        int32_t raw = _sampleBuffer[i];
        int16_t sample16 = static_cast<int16_t>(raw >> 14);

        // Circular ring buffer update
        s_audioRingBuffer[s_bufferHead] = sample16;
        s_bufferHead = (s_bufferHead + 1) % INFERENCE_SAMPLES;
        s_totalSamplesRecorded++;

        int32_t absSample = abs(sample16);
        if (absSample > maxAmp) {
            maxAmp = absSample;
        }

        float sF = static_cast<float>(sample16);
        sumSqRaw += static_cast<double>(sample16) * static_cast<double>(sample16);

        // Stage 1 Biquad Bandpass Filter (200 Hz - 2.5 kHz)
        float filteredSample = processBandpassFilter(sF);
        sumSqWhineBand += (filteredSample * filteredSample);
    }

    _peakAmplitude = maxAmp;

    // Instantaneous metrics for this DMA frame
    double frameEnergy = sumSqRaw / static_cast<double>(numSamples);
    _latestFrameEnergy = static_cast<float>(frameEnergy);

    float frameRmsWide = sqrtf(static_cast<float>(frameEnergy));
    float frameRmsWhine = sqrtf(sumSqWhineBand / static_cast<float>(numSamples));

    // Rolling exponential smoothing
    const float alpha = 0.25f;
    _rmsWideband  = (_rmsWideband  * (1.0f - alpha)) + (frameRmsWide  * alpha);
    _rmsWhineBand = (_rmsWhineBand * (1.0f - alpha)) + (frameRmsWhine * alpha);

    // Whine energy ratio
    if (_rmsWideband > 10.0f) {
        _whineRatio = _rmsWhineBand / _rmsWideband;
    } else {
        _whineRatio = 0.0f;
    }

    _whineDetected = (_rmsWideband >= ACOUSTIC_MIN_RMS_FLOOR && _whineRatio >= ACOUSTIC_WHINE_RATIO_THRESH);

    // =========================================================================
    // Stage 1 Sentry Evaluation: Check if energy exceeds calibrated threshold
    // =========================================================================
    if (_latestFrameEnergy > _energyThreshold) {
        _stage1Triggered = true;

#if SERIAL_LOGGING_ENABLED
        Serial.printf("[STAGE 1 TRIGGER] High Acoustic Energy (%.1f > %.1f) -> Running Stage 2...\n",
                      _latestFrameEnergy, _energyThreshold);
#endif

        // =====================================================================
        // Stage 2: Active TinyML Verification (1-second frame classification)
        // =====================================================================
        float confidence = 0.0f;
        if (runInference(&confidence)) {
            _droneConfirmed = true;
            _droneConfidence = confidence;
            g_acousticDroneConfirmed = true;
            g_acousticConfidence = confidence;

#if SERIAL_LOGGING_ENABLED
            Serial.printf("[STAGE 2 CONFIRMED] Drone Acoustic Signature Detected! Confidence: %.1f%%\n",
                          confidence * 100.0f);
#endif
        } else {
            _droneConfirmed = false;
            _droneConfidence = confidence;
            g_acousticDroneConfirmed = false;
            g_acousticConfidence = confidence;
        }
    } else {
        _stage1Triggered = false;
        _droneConfirmed = false;
        _droneConfidence = 0.0f;
        g_acousticDroneConfirmed = false;
        g_acousticConfidence = 0.0f;
    }

    return true;
}
