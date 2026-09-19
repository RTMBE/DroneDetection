#pragma once

#include <Arduino.h>
#include "config.h"

// =============================================================================
// Edge Impulse / TinyML Signal Interface
// =============================================================================
// When an exported Edge Impulse library (e.g. <drone_detection_inferencing.h>)
// is present in the build, the actual neural net inferencing engine runs.
// Otherwise, clean fallback definitions allow standalone compilation, testing,
// =============================================================================
// Lightweight Pure-C Model Support
// =============================================================================
#if __has_include("drone_model.h")
    #include "drone_model.h"
    #define HAVE_PURE_C_MODEL 1
#else
    #define HAVE_PURE_C_MODEL 0
#endif

// =============================================================================
// Edge Impulse / TinyML Signal Interface
// =============================================================================
#if __has_include("edge-impulse-sdk/classifier/ei_run_classifier.h")
    #include "edge-impulse-sdk/classifier/ei_run_classifier.h"
    #define HAVE_EDGE_IMPULSE 1
#elif defined(USE_EDGE_IMPULSE)
    #define HAVE_EDGE_IMPULSE 1
#else
    #define HAVE_EDGE_IMPULSE 0

    // Lightweight mock types matching Edge Impulse API
    typedef struct {
        size_t total_length;
        int (*get_data)(size_t offset, size_t length, float *out_ptr);
    } signal_t;

    typedef struct {
        const char* label;
        float value;
    } ei_impulse_result_classification_t;

    typedef struct {
        ei_impulse_result_classification_t classification[2];
        struct {
            uint32_t dsp;
            uint32_t classification;
            uint32_t anomaly;
        } timing;
    } ei_impulse_result_t;

    enum EI_IMPULSE_ERROR {
        EI_IMPULSE_OK = 0,
        EI_IMPULSE_ERROR_SHAPE = -1,
        EI_IMPULSE_OUT_OF_MEMORY = -2
    };

    #define EI_CLASSIFIER_LABEL_COUNT 2
#endif

// =============================================================================
// Acoustic Detector Module (INMP441 I2S Digital Microphone & TinyML Sentry)
//
// Cascaded Two-Stage Sentry System:
// Stage 1 (Passive / Low-Power DSP):
//   Microcontroller continuously buffers audio via DMA and calculates rolling
//   bandpass RMS energy across the 200 Hz – 2.5 kHz propeller whine band.
// Stage 2 (Active Verification):
//   Neural network inference triggers only when acoustic energy exceeds an
//   ambient calibrated threshold, classifying 1-second frames in ~25 ms.
// =============================================================================

class AcousticDetector {
public:
    AcousticDetector();

    // Initialize I2S peripheral, DMA buffers, and biquad filters
    bool begin();

    // 3-second quiet ambient calibration on startup to determine ENERGY_THRESHOLD
    void calibrateAmbientFloor(uint32_t durationMs = ACOUSTIC_CALIBRATION_DURATION_MS);

    // Read DMA buffer, update circular ring, compute frame energy, and check sentry triggers
    // Returns true if a new frame of audio was processed
    bool update();

    // Trigger Stage 2 TinyML classifier over the 16,000-sample (1.0s) ring buffer
    // Returns true if drone signature is confirmed (confidence >= ACOUSTIC_CONFIDENCE_THRESHOLD)
    bool runInference(float* outConfidence = nullptr);

    // Edge Impulse signal callback: extracts normalized audio from the circular ring buffer
    static int getSignalDataCallback(size_t offset, size_t length, float *out_ptr);

    // Direct headphone warning chirp on GPIO 15
    void triggerHeadphoneBeep(int freqHz = AUDIO_BEEP_FREQ_HZ, int durationMs = AUDIO_BEEP_DURATION_MS);

    // Metrics and status accessors
    float   getRmsAmplitude() const          { return _rmsWideband; }
    float   getRmsLeft() const               { return _rmsLeft; }
    float   getRmsRight() const              { return _rmsRight; }
    float   getWhineBandRms() const          { return _rmsWhineBand; }
    float   getWhineBandRmsLeft() const      { return _rmsWhineLeft; }
    float   getWhineBandRmsRight() const     { return _rmsWhineRight; }
    int32_t getPeakAmplitude() const         { return _peakAmplitude; }
    float   getWhineRatio() const            { return _whineRatio; }
    float   getWhineRatioLeft() const        { return _whineRatioLeft; }
    float   getWhineRatioRight() const       { return _whineRatioRight; }
    bool    isWhineDetected() const          { return _whineDetected; }
    bool    isWhineLeft() const              { return _whineDetectedLeft; }
    bool    isWhineRight() const             { return _whineDetectedRight; }
    float   getChannelBalance() const        { return _channelBalance; }

    float   getEnergyThreshold() const       { return _energyThreshold; }
    void    setEnergyThreshold(float th)     { _energyThreshold = th; }
    float   getAmbientBaselineEnergy() const { return _ambientBaselineEnergy; }
    float   getLatestFrameEnergy() const     { return _latestFrameEnergy; }

    bool    isStage1Triggered() const        { return _stage1Triggered; }
    bool    isDroneConfirmed() const         { return _droneConfirmed; }
    float   getDroneConfidence() const       { return _droneConfidence; }
    bool    isCalibrated() const             { return _isCalibrated; }
    uint32_t getDmaDropCount() const         { return s_dmaDropCount; }

    // Direct access to ring buffer for inspection / external wrappers
    static const int16_t* getRingBuffer()    { return s_audioRingBuffer; }
    static size_t getRingHead()              { return s_bufferHead; }

private:
    void computeBiquadCoefficients();
    float processBandpassFilterLeft(float input);
    float processBandpassFilterRight(float input);

    // Audio & Sentry Metrics (Wideband & Dual Channels)
    float   _rmsWideband;
    float   _rmsWhineBand;
    float   _rmsLeft;
    float   _rmsRight;
    float   _rmsWhineLeft;
    float   _rmsWhineRight;
    int32_t _peakAmplitude;
    float   _whineRatio;
    float   _whineRatioLeft;
    float   _whineRatioRight;
    bool    _whineDetected;
    bool    _whineDetectedLeft;
    bool    _whineDetectedRight;
    float   _channelBalance;        // -1.0 (Right) to +1.0 (Left)

    float   _latestFrameEnergy;
    float   _ambientBaselineEnergy;
    float   _energyThreshold;
    bool    _stage1Triggered;
    bool    _droneConfirmed;
    float   _droneConfidence;
    bool    _isCalibrated;

    // Direct Form II Transposed Biquad Filter Coefficients (HPF 200 Hz + LPF 2500 Hz @ 16 kHz)
    float _hp_b0, _hp_b1, _hp_b2;
    float _hp_a1, _hp_a2;

    float _lp_b0, _lp_b1, _lp_b2;
    float _lp_a1, _lp_a2;

    // Separate filter states for Left and Right channels
    float _hp_z1_L, _hp_z2_L;
    float _lp_z1_L, _lp_z2_L;

    float _hp_z1_R, _hp_z2_R;
    float _lp_z1_R, _lp_z2_R;

    // Raw DMA buffer (Single mic: 512 samples; Dual mic: 1024 samples)
#if DUAL_MIC_ENABLED
    int32_t _sampleBuffer[I2S_DMA_BUFFER_SAMPLES * 2];
#else
    int32_t _sampleBuffer[I2S_DMA_BUFFER_SAMPLES];
#endif

    // Static 1-second circular ring buffer (16,000 samples @ 16 kHz)
    static int16_t          s_audioRingBuffer[INFERENCE_SAMPLES];
    static volatile size_t  s_bufferHead;
    static volatile size_t  s_totalSamplesRecorded;
    static volatile uint32_t s_dmaDropCount;
};

// Global C-linkage callback conforming to Edge Impulse signal_t.get_data
inline int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    return AcousticDetector::getSignalDataCallback(offset, length, out_ptr);
}
