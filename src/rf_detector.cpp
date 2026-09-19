#include "rf_detector.h"

RFDetector::RFDetector()
    : _rawAdc(0),
      _rawMv(AD8318_V_QUIESCENT_MV),
      _filteredMv(AD8318_V_QUIESCENT_MV),
      _previousMv(AD8318_V_QUIESCENT_MV),
      _estimatedDbm(-65.0f),
      _threatPercent(0.0f),
      _quiescentMv(AD8318_V_QUIESCENT_MV),
      _burstDetected(false),
      _droneIncoming(false),
      _triggerCandidateStartTimeMs(0),
      _lastTriggerActiveTimeMs(0) {
}

void RFDetector::begin() {
    // Configure ADC pin resolution and attenuation
    analogReadResolution(RF_ADC_RESOLUTION_BITS);
    analogSetPinAttenuation(PIN_AD8318_VOUT, RF_ADC_ATTENUATION);

    // Warm-up and seed filter with initial samples
    uint32_t adcSum = 0;
    uint32_t mvSum = 0;
    for (int i = 0; i < 64; ++i) {
        adcSum += analogRead(PIN_AD8318_VOUT);
        mvSum  += analogReadMilliVolts(PIN_AD8318_VOUT);
        delayMicroseconds(200);
    }
    _rawAdc = adcSum / 64;
    _rawMv = static_cast<float>(mvSum) / 64.0f;
    _filteredMv = _rawMv;
    _previousMv = _rawMv;
    _quiescentMv = _rawMv; // Initial clean-air reference
}

void RFDetector::update() {
    // 1. Multi-sample ADC to suppress high-frequency analog noise
    uint32_t adcAccum = 0;
    uint32_t mvAccum = 0;

    for (int i = 0; i < RF_ADC_OVERSAMPLES; ++i) {
        adcAccum += analogRead(PIN_AD8318_VOUT);
        mvAccum  += analogReadMilliVolts(PIN_AD8318_VOUT);
    }

    _rawAdc = adcAccum / RF_ADC_OVERSAMPLES;
    _rawMv  = static_cast<float>(mvAccum) / static_cast<float>(RF_ADC_OVERSAMPLES);

    // 2. Burst carrier detection: rapid drop in voltage
    if ((_previousMv - _rawMv) >= RF_BURST_DELTA_MV) {
        _burstDetected = true;
    } else {
        _burstDetected = false;
    }
    _previousMv = _rawMv;

    // 3. Exponential Moving Average (EMA) filter
    _filteredMv = (_filteredMv * (1.0f - RF_EMA_ALPHA)) + (_rawMv * RF_EMA_ALPHA);

    // 4. Calculate estimated RF power in dBm
    // Transfer function: Vout = Vsat + Slope * (dBm - Ref_dBm)
    // dBm = Ref_dBm + (Vout - Vsat) / Slope
    // Note: Slope is negative (approx -24 mV/dB)
    float deltaV = _filteredMv - AD8318_V_SATURATED_MV;
    _estimatedDbm = AD8318_INTERCEPT_DBM + (deltaV / AD8318_SLOPE_MV_PER_DB);

    // Clamp dBm to realistic operating range of AD8318 (-65 dBm to +5 dBm)
    if (_estimatedDbm < -65.0f) _estimatedDbm = -65.0f;
    if (_estimatedDbm > 5.0f)   _estimatedDbm = 5.0f;

    // 5. Threat Percentage (0% at clean air / quiescent, 100% at saturated 500 mV)
    // Quiescent voltage is higher than saturated voltage (inverted response)
    float span = _quiescentMv - AD8318_V_SATURATED_MV;
    if (span <= 10.0f) span = 1600.0f; // Prevent div-by-zero

    if (_filteredMv >= _quiescentMv) {
        _threatPercent = 0.0f;
    } else if (_filteredMv <= AD8318_V_SATURATED_MV) {
        _threatPercent = 100.0f;
    } else {
        _threatPercent = ((_quiescentMv - _filteredMv) / span) * 100.0f;
    }

    // Deadband filter for clean air drift
    if (_threatPercent < RF_THREAT_MIN_PERCENT) {
        _threatPercent = 0.0f;
    }

    // 6. Drone Incoming Trigger Evaluation with persistence and hysteresis
    uint32_t nowMs = millis();

    if (_threatPercent >= DRONE_INCOMING_TRIGGER_PCT) {
        if (_triggerCandidateStartTimeMs == 0) {
            _triggerCandidateStartTimeMs = nowMs;
        }

        // Must persist for >= DRONE_INCOMING_PERSIST_MS to confirm incoming drone
        if ((nowMs - _triggerCandidateStartTimeMs) >= DRONE_INCOMING_PERSIST_MS) {
            _droneIncoming = true;
            _lastTriggerActiveTimeMs = nowMs;
        }
    } else {
        // Reset candidate timer if signal dropped below trigger
        _triggerCandidateStartTimeMs = 0;

        // Apply hysteresis: only clear if signal dropped below reset threshold AND hold time expired
        if (_threatPercent <= DRONE_INCOMING_RESET_PCT) {
            if ((nowMs - _lastTriggerActiveTimeMs) >= DRONE_INCOMING_HOLD_MS) {
                _droneIncoming = false;
            }
        }
    }

    // 7. Update globally visible telemetry variables
    g_droneIncoming   = _droneIncoming;
    g_rfThreatPercent = _threatPercent;
    g_rfDbm           = _estimatedDbm;
    g_rfMilliVolts    = _filteredMv;
}

void RFDetector::calibrateCleanAirFloor() {
    // Set the current filtered voltage as the new clean-air reference
    _quiescentMv = _filteredMv;
}
