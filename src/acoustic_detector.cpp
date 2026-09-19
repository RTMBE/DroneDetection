#include "acoustic_detector.h"
#include <driver/i2s.h>
#include <math.h>

AcousticDetector::AcousticDetector()
    : _rmsWideband(0.0f),
      _rmsWhineBand(0.0f),
      _peakAmplitude(0),
      _whineRatio(0.0f),
      _whineDetected(false),
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

    // 1. High-Pass Filter @ ACOUSTIC_BAND_LOW_HZ (250 Hz)
    float omega_hp = 2.0f * M_PI * (ACOUSTIC_BAND_LOW_HZ / Fs);
    float cos_hp   = cosf(omega_hp);
    float sin_hp   = sinf(omega_hp);
    float alpha_hp = sin_hp / (2.0f * Q);

    float hp_a0 = 1.0f + alpha_hp;
    _hp_b0 = ((1.0f + cos_hp) / 2.0f) / hp_a0;
    _hp_b1 = (-(1.0f + cos_hp))       / hp_a0;
    _hp_b2 = ((1.0f + cos_hp) / 2.0f) / hp_a0;
    _hp_a1 = (-2.0f * cos_hp)         / hp_a0;
    _hp_a2 = (1.0f - alpha_hp)        / hp_a0;

    // 2. Low-Pass Filter @ ACOUSTIC_BAND_HIGH_HZ (1200 Hz)
    float omega_lp = 2.0f * M_PI * (ACOUSTIC_BAND_HIGH_HZ / Fs);
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

bool AcousticDetector::update() {
    size_t bytesRead = 0;
    esp_err_t res = i2s_read(I2S_NUM_0, _sampleBuffer, sizeof(_sampleBuffer), &bytesRead, pdMS_TO_TICKS(15));
    if (res != ESP_OK || bytesRead == 0) {
        return false;
    }

    const size_t numSamples = bytesRead / sizeof(int32_t);
    if (numSamples == 0) return false;

    float sumSqWideband = 0.0f;
    float sumSqWhineBand = 0.0f;
    int32_t maxAmp = 0;

    for (size_t i = 0; i < numSamples; ++i) {
        // INMP441 outputs 24-bit MSB-aligned in 32-bit slot.
        // Shift by 14 bits to scale cleanly to 16-bit range [-32768, 32767]
        int32_t raw = _sampleBuffer[i];
        int32_t sample = raw >> 14;

        int32_t absSample = abs(sample);
        if (absSample > maxAmp) {
            maxAmp = absSample;
        }

        float sF = static_cast<float>(sample);
        sumSqWideband += (sF * sF);

        // Apply digital bandpass filter for propeller motor whine
        float filteredSample = processBandpassFilter(sF);
        sumSqWhineBand += (filteredSample * filteredSample);
    }

    _peakAmplitude = maxAmp;

    // Instantaneous RMS calculation for this DMA frame
    float frameRmsWide = sqrtf(sumSqWideband / static_cast<float>(numSamples));
    float frameRmsWhine = sqrtf(sumSqWhineBand / static_cast<float>(numSamples));

    // Rolling exponential smoothing
    const float alpha = 0.25f;
    _rmsWideband  = (_rmsWideband  * (1.0f - alpha)) + (frameRmsWide  * alpha);
    _rmsWhineBand = (_rmsWhineBand * (1.0f - alpha)) + (frameRmsWhine * alpha);

    // Calculate whine energy ratio
    if (_rmsWideband > 10.0f) {
        _whineRatio = _rmsWhineBand / _rmsWideband;
    } else {
        _whineRatio = 0.0f;
    }

    // Whine classification: Must exceed absolute RMS noise floor AND energy ratio
    if (_rmsWideband >= ACOUSTIC_MIN_RMS_FLOOR && _whineRatio >= ACOUSTIC_WHINE_RATIO_THRESH) {
        _whineDetected = true;
    } else {
        _whineDetected = false;
    }

    return true;
}
