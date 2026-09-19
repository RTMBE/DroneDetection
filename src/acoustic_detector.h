#pragma once

#include <Arduino.h>
#include "config.h"

// =============================================================================
// Acoustic Detector Module (INMP441 I2S Digital Microphone)
// Captures continuous I2S audio, calculates rolling RMS sound pressure,
// and applies digital bandpass filtering to detect propeller motor whine.
// =============================================================================

class AcousticDetector {
public:
    AcousticDetector();

    // Initialize I2S peripheral and DMA buffers
    bool begin();

    // Read DMA buffer, update RMS, and execute whine detection filter
    // Returns true if a new frame of audio was processed
    bool update();

    // Rolling RMS amplitude of raw audio (wideband sound level)
    float getRmsAmplitude() const { return _rmsWideband; }

    // Rolling RMS amplitude within the 250 Hz - 1200 Hz propeller whine band
    float getWhineBandRms() const { return _rmsWhineBand; }

    // Peak amplitude observed in latest window
    int32_t getPeakAmplitude() const { return _peakAmplitude; }

    // Ratio of whine band energy to total energy (0.0 to 1.0)
    float getWhineRatio() const { return _whineRatio; }

    // True if acoustic signature matches drone motor/blade whine
    bool isWhineDetected() const { return _whineDetected; }

private:
    void computeBiquadCoefficients();
    float processBandpassFilter(float input);

    // Audio metrics
    float   _rmsWideband;
    float   _rmsWhineBand;
    int32_t _peakAmplitude;
    float   _whineRatio;
    bool    _whineDetected;

    // Direct Form II Transposed Biquad filter state (Cascaded HPF + LPF)
    // High-pass filter (cutoff ~250 Hz @ 16 kHz)
    float _hp_b0, _hp_b1, _hp_b2;
    float _hp_a1, _hp_a2;
    float _hp_z1, _hp_z2;

    // Low-pass filter (cutoff ~1200 Hz @ 16 kHz)
    float _lp_b0, _lp_b1, _lp_b2;
    float _lp_a1, _lp_a2;
    float _lp_z1, _lp_z2;

    // Raw DMA sample buffer
    int32_t _sampleBuffer[I2S_DMA_BUFFER_SAMPLES];
};
