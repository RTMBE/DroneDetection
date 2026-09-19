#include "wifi_detector.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

// Global 2.4 GHz state variables definition
volatile bool    g_drone24Detected = false;
volatile int8_t  g_drone24Rssi     = -100;
volatile uint8_t g_drone24Channel  = 1;
volatile uint8_t g_drone24Vendor   = DRONE_VENDOR_NONE;

static WiFiDetector* s_instance = nullptr;

WiFiDetector::WiFiDetector()
    : _droneDetected(false),
      _lastRssi(-100),
      _lastChannel(WIFI_24_CHANNEL_MIN),
      _lastVendor(DRONE_VENDOR_NONE),
      _lastDetectionTimeMs(0),
      _currentChannel(WIFI_24_CHANNEL_MIN),
      _lastHopTimeMs(0) {
    memset(_lastMac, 0, sizeof(_lastMac));
}

bool WiFiDetector::begin() {
    s_instance = this;

    // Initialize NVS (required by Wi-Fi subsystem)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) return false;

    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_start();

    // Enable Promiscuous packet sniffer mode
    esp_wifi_set_promiscuous(true);

    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
    };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(&WiFiDetector::promiscuousRxCallback);

    // Set initial Wi-Fi channel
    _currentChannel = WIFI_24_CHANNEL_MIN;
    esp_wifi_set_channel(_currentChannel, WIFI_SECOND_CHAN_NONE);
    _lastHopTimeMs = millis();

    return true;
}

void WiFiDetector::promiscuousRxCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (s_instance != nullptr) {
        wifi_promiscuous_pkt_t* pkt = reinterpret_cast<wifi_promiscuous_pkt_t*>(buf);
        s_instance->processPacket(pkt);
    }
}

uint8_t WiFiDetector::matchDroneVendor(const uint8_t* mac, const uint8_t* payload, uint16_t len) {
    // 1. Check Source MAC against database of 35+ known drone manufacturer OUIs
    for (size_t i = 0; i < NUM_KNOWN_OUIS; ++i) {
        if (mac[0] == pgm_read_byte(&KNOWN_DRONE_OUIS[i].bytes[0]) &&
            mac[1] == pgm_read_byte(&KNOWN_DRONE_OUIS[i].bytes[1]) &&
            mac[2] == pgm_read_byte(&KNOWN_DRONE_OUIS[i].bytes[2])) {
            return pgm_read_byte(&KNOWN_DRONE_OUIS[i].vendor);
        }
    }

    // 2. Scan Information Elements (IEs) in Management frames (Beacons, Probe Resp, Action frames)
    if (len > 36) {
        // Minimum 802.11 management header is 24 bytes.
        // For Beacon (subtype 8) and Probe Response (subtype 5), 12 fixed parameters follow header.
        uint8_t frameType = (payload[0] & 0x0C) >> 2;
        uint8_t frameSubtype = (payload[0] & 0xF0) >> 4;
        size_t offset = (frameType == 0 && (frameSubtype == 8 || frameSubtype == 5)) ? 36 : 24;

        while (offset + 2 < len) {
            uint8_t elementId  = payload[offset];
            uint8_t elementLen = payload[offset + 1];

            if (offset + 2 + elementLen > len) break;

            const uint8_t* ieData = &payload[offset + 2];

            // Element 221 (0xDD) is Vendor Specific IE
            if (elementId == 0xDD && elementLen >= 3) {
                // A. Open Drone ID (ODID / ASTM F3411): OUI 0xFA, 0x0B, 0xBC
                if (ieData[0] == 0xFA && ieData[1] == 0x0B && ieData[2] == 0xBC) {
                    // Check ODID message payload if present
                    if (elementLen >= 5) {
                        uint8_t msgType = ieData[4] & 0xF0;
                        // Basic ID (0x00): Inspect CTA-2063-A 4-digit serial manufacturer prefix
                        if (msgType == 0x00 && elementLen >= 10) {
                            char uasPrefix[5] = {0};
                            for (int k = 0; k < 4 && (offset + 2 + 6 + k) < len; ++k) {
                                uasPrefix[k] = (char)ieData[6 + k];
                            }
                            for (size_t c = 0; c < NUM_CTA_MANUFACTURERS; ++c) {
                                if (strncmp(uasPrefix, KNOWN_CTA_MANUFACTURERS[c].code, 4) == 0) {
                                    return pgm_read_byte(&KNOWN_CTA_MANUFACTURERS[c].vendor);
                                }
                            }
                        }
                    }
                    return DRONE_VENDOR_OPEN_DRONE_ID;
                }

                // B. DJI / French Drone ID: OUI 0x26, 0x37, 0x12 or 0x00, 0x04, 0xF4
                if ((ieData[0] == 0x26 && ieData[1] == 0x37 && ieData[2] == 0x12) ||
                    (ieData[0] == 0x00 && ieData[1] == 0x04 && ieData[2] == 0xF4)) {
                    return DRONE_VENDOR_DJI;
                }

                // C. NAN / ASD-STAN Remote ID: Service Protocol ID 0xFF, 0xFA
                if (ieData[0] == 0xFF && ieData[1] == 0xFA) {
                    return DRONE_VENDOR_OPEN_DRONE_ID;
                }
            }

            offset += (2 + elementLen);
        }
    }

    // 3. Check for Drone ID EtherType (0x8835) in LLC / SNAP header
    // LLC/SNAP header: AA AA 03 00 00 00 [EtherType: 2 bytes]
    if (len >= 32) {
        for (size_t i = 24; i + 8 <= len && i < 40; ++i) {
            if (payload[i] == 0xAA && payload[i + 1] == 0xAA && payload[i + 2] == 0x03) {
                uint16_t etherType = (payload[i + 6] << 8) | payload[i + 7];
                if (etherType == 0x8835) {
                    return DRONE_VENDOR_OPEN_DRONE_ID;
                }
                break;
            }
        }
    }

    // 4. Packet Sync & Telemetry Header Inspection (MAVLink, MSP, CRSF/ELRS, SBUS)
    if (len >= 28) {
        size_t scanStart = 24;
        if (scanStart + 8 < len && payload[scanStart] == 0xAA && payload[scanStart + 1] == 0xAA) {
            scanStart += 8;
        }

        size_t scanEnd = (scanStart + 16 < len) ? (scanStart + 16) : len;
        for (size_t i = scanStart; i < scanEnd; ++i) {
            uint8_t b = payload[i];

            // A. MAVLink 1.0 (0xFE) and MAVLink 2.0 (0xFD)
            if (b == 0xFE && (i + 5 < len)) {
                uint8_t mlen = payload[i + 1];
                if (mlen <= 255 && (i + 6 + mlen <= len)) {
                    return DRONE_VENDOR_MAVLINK;
                }
            } else if (b == 0xFD && (i + 9 < len)) {
                uint8_t mlen = payload[i + 1];
                if (mlen <= 255 && (i + 10 + mlen <= len)) {
                    return DRONE_VENDOR_MAVLINK;
                }
            }

            // B. MSP (MultiWii Serial Protocol): "$M<" or "$MX"
            if (b == 0x24 && (i + 2 < len) && payload[i + 1] == 0x4D) {
                if (payload[i + 2] == 0x3C || payload[i + 2] == 0x58) {
                    return DRONE_VENDOR_MSP;
                }
            }

            // C. CRSF / ExpressLRS: Sync bytes 0xC8, 0xEE, 0xEC
            if ((b == 0xC8 || b == 0xEE || b == 0xEC) && (i + 2 < len)) {
                uint8_t crsfLen = payload[i + 1];
                if (crsfLen >= 2 && crsfLen <= 64 && (i + 2 + crsfLen <= len)) {
                    return DRONE_VENDOR_CRSF_ELRS;
                }
            }

            // D. SBUS: Frame start sync byte 0x0F
            if (b == 0x0F && (i + 24 < len) && payload[i + 24] == 0x00) {
                return DRONE_VENDOR_GENERIC_FPV;
            }
        }
    }

    return DRONE_VENDOR_NONE;
}

void WiFiDetector::processPacket(const wifi_promiscuous_pkt_t* pkt) {
    if (pkt == nullptr) return;

    int8_t rssi = pkt->rx_ctrl.rssi;
    if (rssi < WIFI_24_DRONE_RSSI_THRESH) return;

    uint16_t len = pkt->rx_ctrl.sig_len;
    if (len < 24) return; // Must have at least standard 802.11 header

    const uint8_t* payload = pkt->payload;

    // In 802.11 frames, Source MAC (Transmitter Address TA) is at bytes 10..15
    const uint8_t* srcMac = &payload[10];

    uint8_t vendor = matchDroneVendor(srcMac, payload, len);
    if (vendor != DRONE_VENDOR_NONE) {
        _droneDetected = true;
        _lastRssi = rssi;
        _lastChannel = pkt->rx_ctrl.channel;
        _lastVendor = vendor;
        memcpy(_lastMac, srcMac, 6);
        _lastDetectionTimeMs = millis();

        // Sync global 2.4 GHz state variables
        g_drone24Detected = true;
        g_drone24Rssi     = _lastRssi;
        g_drone24Channel  = _lastChannel;
        g_drone24Vendor   = _lastVendor;
    }
}

void WiFiDetector::update() {
    uint32_t nowMs = millis();

    // 1. Rapid Channel Hopping across 2.4 GHz ISM Band (Channels 1 to 13)
    if (nowMs - _lastHopTimeMs >= WIFI_24_HOP_INTERVAL_MS) {
        _lastHopTimeMs = nowMs;

        _currentChannel++;
        if (_currentChannel > WIFI_24_CHANNEL_MAX) {
            _currentChannel = WIFI_24_CHANNEL_MIN;
        }

        esp_wifi_set_channel(_currentChannel, WIFI_SECOND_CHAN_NONE);
    }

    // 2. Hold time decay: reset detection flag after hold window expires
    if (_droneDetected && (nowMs - _lastDetectionTimeMs >= WIFI_24_HOLD_TIME_MS)) {
        _droneDetected = false;
        g_drone24Detected = false;
    }
}
