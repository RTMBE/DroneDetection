#pragma once

#include <Arduino.h>
#include "config.h"
#include "drone_signatures.h"
#include <esp_wifi.h>

// =============================================================================
// 2.4 GHz Drone Detector Module (ESP32-S3 Internal Radio)
// Runs 802.11 promiscuous sniffer with rapid channel hopping across Channels 1-13
// Detects:
// - Known drone manufacturer OUIs (DJI, Autel, Parrot, Skydio, Yuneec)
// - Open Drone ID (ODID / ASTM F3411) & DJI Drone ID broadcast frames
// =============================================================================

class WiFiDetector {
public:
    WiFiDetector();

    // Initialize ESP32-S3 Wi-Fi peripheral in promiscuous sniffer mode
    bool begin();

    // Step channel hopper and evaluate hold timers (call in Core 0 or loop)
    void update();

    // True if a 2.4 GHz drone signal was detected within hold window
    bool isDroneDetected() const { return _droneDetected; }

    // Metrics for the most recently detected drone
    int8_t   getLastRssi() const { return _lastRssi; }
    uint8_t  getLastChannel() const { return _lastChannel; }
    uint8_t  getLastVendor() const { return _lastVendor; }
    uint8_t  getCurrentChannel() const { return _currentChannel; }
    const uint8_t* getLastMac() const { return _lastMac; }

    // Internal promiscuous frame processor
    void processPacket(const wifi_promiscuous_pkt_t* pkt);

    // Static callback required by ESP-IDF Wi-Fi driver
    static void promiscuousRxCallback(void* buf, wifi_promiscuous_pkt_type_t type);

private:
    uint8_t matchDroneVendor(const uint8_t* mac, const uint8_t* payload, uint16_t len);

    bool     _droneDetected;
    int8_t   _lastRssi;
    uint8_t  _lastChannel;
    uint8_t  _lastVendor;
    uint8_t  _lastMac[6];
    uint32_t _lastDetectionTimeMs;

    uint8_t  _currentChannel;
    uint32_t _lastHopTimeMs;
};
