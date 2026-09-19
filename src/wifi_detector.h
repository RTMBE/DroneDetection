#pragma once

#include <Arduino.h>
#include "config.h"
#include "drone_signatures.h"
#include <esp_wifi.h>

// =============================================================================
// 2.4 GHz Drone Detector Module (ESP32-S3 Internal Radio)
// Dual-Layer Detection Engine:
// 1. Protocol Layer (Cooperative):
//    - Known drone manufacturer OUIs (DJI, Autel, Parrot, Skydio, Yuneec)
//    - Open Drone ID (ODID / ASTM F3411) & DJI Drone ID broadcast frames
//    - Telemetry packet sync bytes (MAVLink, MSP, CRSF/ELRS, SBUS)
// 2. Behavioral & Physical Layer (Non-Cooperative / Zero Remote ID):
//    - Microsecond Inter-Arrival Time (IAT) periodicity classifier (TDD 10ms/20ms & 500/250/150/50Hz RC)
//    - Frequency Hopping Spread Spectrum (FHSS >= 3 channels in 600ms)
//    - Non-standard 10 MHz flat-topped OFDM PHY burst anomalies
// =============================================================================

#define MAX_TRANSMITTER_TRACKS 24

struct TransmitterTrack {
    uint8_t  mac[6];
    uint32_t lastArrivalUs;
    uint32_t lastDeltaUs;
    uint16_t channelMask;       // Bitmask of Wi-Fi channels seen on (bit 1..13)
    uint8_t  channelCount;      // Number of distinct channels in window
    uint32_t firstSeenMs;
    uint32_t lastSeenMs;
    int8_t   lastRssi;
    uint8_t  periodicHits;      // Count of consecutive periodic frames
    uint16_t detectedRateHz;
    uint8_t  behaviorFlags;
    bool     active;
};

class WiFiDetector {
public:
    WiFiDetector();

    // Initialize ESP32-S3 Wi-Fi peripheral in promiscuous sniffer mode
    bool begin();

    // Step channel hopper, evaluate hold timers & prune tracks (Core 0 or loop)
    void update();

    // True if a 2.4 GHz drone signal was detected within hold window
    bool isDroneDetected() const { return _droneDetected; }

    // Metrics for the most recently detected drone
    int8_t   getLastRssi() const { return _lastRssi; }
    uint8_t  getLastChannel() const { return _lastChannel; }
    uint8_t  getLastVendor() const { return _lastVendor; }
    uint8_t  getCurrentChannel() const { return _currentChannel; }
    const uint8_t* getLastMac() const { return _lastMac; }

    // Behavioral detection metrics
    uint8_t  getBehaviorFlags() const { return _behaviorFlags; }
    uint16_t getDetectedRateHz() const { return _detectedRateHz; }
    uint8_t  getHopCount() const { return _hopCount; }

    // Internal promiscuous frame processor
    void processPacket(const wifi_promiscuous_pkt_t* pkt);

    // Static callback required by ESP-IDF Wi-Fi driver
    static void promiscuousRxCallback(void* buf, wifi_promiscuous_pkt_type_t type);

private:
    uint8_t matchDroneVendor(const uint8_t* mac, const uint8_t* payload, uint16_t len);

    void analyzeTimingAndHopping(const uint8_t* mac, uint8_t channel, int8_t rssi, uint32_t timestampUs, uint16_t len);
    TransmitterTrack* findOrAllocTrack(const uint8_t* mac, uint32_t nowMs);
    void pruneOldTracks(uint32_t nowMs);

    bool     _droneDetected;
    int8_t   _lastRssi;
    uint8_t  _lastChannel;
    uint8_t  _lastVendor;
    uint8_t  _lastMac[6];
    uint32_t _lastDetectionTimeMs;

    // Behavioral state
    uint8_t  _behaviorFlags;
    uint16_t _detectedRateHz;
    uint8_t  _hopCount;

    uint8_t  _currentChannel;
    uint32_t _lastHopTimeMs;

    TransmitterTrack _tracks[MAX_TRANSMITTER_TRACKS];
};
