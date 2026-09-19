#pragma once

#include <Arduino.h>
#include "config.h"

// =============================================================================
// RF Detector Module (AD8318 5.8 GHz Analog Power Detector)
// Handles multi-sampling, digital EMA filtering, dBm calculation, and threat level
// =============================================================================

class RFDetector {
public:
    RFDetector();

    // Initialize ADC settings and seed filter
    void begin();

    // Sample ADC, apply filter, and compute metrics (call regularly in main loop)
    void update();

    // Current filtered voltage in millivolts
    float getFilteredMilliVolts() const { return _filteredMv; }

    // Instantaneous multi-sampled voltage before EMA filter
    float getRawMilliVolts() const { return _rawMv; }

    // Instantaneous raw 12-bit ADC value
    uint32_t getRawADC() const { return _rawAdc; }

    // Estimated RF power level in dBm
    float getEstimatedDbm() const { return _estimatedDbm; }

    // Threat level normalized to 0.0% - 100.0%
    float getThreatPercent() const { return _threatPercent; }

    // True if a rapid voltage drop was detected (analog/digital FPV video burst)
    bool isBurstDetected() const { return _burstDetected; }

    // True if RF signal is noticeably above quiescent clean-air floor
    bool isSignalDetected() const { return _threatPercent >= RF_THREAT_MIN_PERCENT; }

    // True if RF signal meets or exceeds the incoming drone threat threshold (debounced)
    bool isDroneIncoming() const { return _droneIncoming; }

    // Dynamic field calibration: set current ambient RF as baseline clean-air floor
    void calibrateCleanAirFloor();

    // Set/override quiescent clean-air voltage
    void setQuiescentMv(float vMv) { _quiescentMv = vMv; }
    float getQuiescentMv() const { return _quiescentMv; }

private:
    uint32_t _rawAdc;
    float _rawMv;
    float _filteredMv;
    float _previousMv;
    float _estimatedDbm;
    float _threatPercent;
    float _quiescentMv;
    bool  _burstDetected;
    bool  _droneIncoming;
    uint32_t _triggerCandidateStartTimeMs;
    uint32_t _lastTriggerActiveTimeMs;
};
