#pragma once

#include <Arduino.h>

// =============================================================================
// ESP32-S3 HANDHELD DRONE SNIFFER CONFIGURATION
// Hardware Pinout & Signal Parameters
// =============================================================================

// -----------------------------------------------------------------------------
// 1. PIN DEFINITIONS
// -----------------------------------------------------------------------------
// AD8318 5.8 GHz Analog RF Power Detector
#define PIN_AD8318_VOUT         4       // ESP32 ADC1 Channel 3

// INMP441 Digital I2S Microphone
#define PIN_I2S_SD              5       // I2S Serial Data (DIN)
#define PIN_I2S_SCK             6       // I2S Bit Clock (BCLK)
#define PIN_I2S_WS              7       // I2S Word Select / LRCLK (WS)

// PJ-392 3.5mm Headphone Jack (Audio Cue Tone / Clicks)
// Driven via 220 Ohm resistor + 10 uF DC blocking capacitor
#define PIN_AUDIO_JACK          15      // LEDC PWM audio tone generator

// Visual Alert LED (Drone Incoming Indicator)
// Driven via 220Ω - 330Ω resistor to LED anode; cathode to GND
#define PIN_ALERT_LED           16      // GPIO 16 (adjacent to GPIO 15 audio pin)

// -----------------------------------------------------------------------------
// 2. AD8318 RF DETECTOR PARAMETERS
// -----------------------------------------------------------------------------
// ESP32-S3 ADC Attenuation:
// Capture full 0.0V - 3.1V without clipping (AD8318 ranges from ~0.5V to ~2.1V).
#if defined(SOC_ADC_ATTEN_DB_12_SUPPORTED) || defined(ADC_ATTEN_DB_12)
    #define RF_ADC_ATTENUATION  ADC_ATTEN_DB_12
#else
    #define RF_ADC_ATTENUATION  ADC_11db
#endif

#define RF_ADC_RESOLUTION_BITS  12      // 12-bit ADC (0 - 4095)
#define RF_ADC_OVERSAMPLES      32      // Multisample count for noise suppression
#define RF_EMA_ALPHA            0.18f   // Exponential moving average filter factor (0.0 to 1.0)

// AD8318 Transfer Function Characteristics (Inverted Log Response):
// - High RF power (close drone) -> Low output voltage (~0.5V)
// - Low RF power (clean air)    -> High output voltage (~2.1V)
#define AD8318_V_QUIESCENT_MV   2100.0f // Millivolts at clean air / quiescent floor (~ -60 dBm)
#define AD8318_V_SATURATED_MV   500.0f  // Millivolts at maximum input power (~ 0 dBm)
#define AD8318_SLOPE_MV_PER_DB  -24.0f  // Nominally -24 mV/dB slope
#define AD8318_INTERCEPT_DBM    0.0f    // Calibrated 0 dBm reference level at saturation

// Threat detection triggers:
#define RF_THREAT_MIN_PERCENT   5.0f    // Ignore minor baseline drift below this %
#define RF_BURST_DELTA_MV       80.0f   // Instantaneous voltage drop indicative of digital carrier burst

// -----------------------------------------------------------------------------
// 2.1 FOXEER 5.8 GHz ANTENNA FULL BAND (5.5 - 6.0 GHz)
// -----------------------------------------------------------------------------
// The AD8318 is an ultra-wideband logarithmic power detector (1 MHz - 8 GHz).
// Full-range reception across 5.5 - 6.0 GHz is provided physically by the tuned
// resonance of the Foxeer 5.8 GHz antenna, capturing all 5.8 GHz FPV video and
// drone transmission channels (Raceband, Bands A, B, E, F, FatShark).
#define RF_TARGET_BAND_MIN_GHZ      5.5f
#define RF_TARGET_BAND_MAX_GHZ      6.0f
#define RF_TARGET_BAND_LABEL        "5.5 - 6.0 GHz (Foxeer 5.8GHz Full Band)"

// Incoming Drone Trigger Thresholds:
#define DRONE_INCOMING_TRIGGER_PCT  25.0f   // Threat % threshold to declare an incoming drone
#define DRONE_INCOMING_RESET_PCT    12.0f   // Threat % hysteresis reset level
#define DRONE_INCOMING_PERSIST_MS   50      // Signal must sustain >= 50ms to prevent false spikes
#define DRONE_INCOMING_HOLD_MS      400     // Hold incoming trigger active for at least 400ms

// Global Drone Incoming Alert State (accessible system-wide)
extern volatile bool  g_droneIncoming;
extern volatile float g_rfThreatPercent;
extern volatile float g_rfDbm;
extern volatile float g_rfMilliVolts;

// -----------------------------------------------------------------------------
// 2.2 ESP32-S3 INTERNAL 2.4 GHz WI-FI DRONE DETECTOR
// -----------------------------------------------------------------------------
// Uses internal radio in 802.11 promiscuous mode with continuous channel hopping
// to detect 2.4 GHz drone control links, Wi-Fi video streams, known drone vendor
// OUIs (DJI, Autel, Parrot, Skydio), and FAA/ASTM Open Drone ID (ODID) frames.
#define WIFI_24_CHANNEL_MIN         1       // 2412 MHz
#define WIFI_24_CHANNEL_MAX         13      // 2472 MHz
#define WIFI_24_HOP_INTERVAL_MS     30      // Milliseconds per channel (fast sweep)
#define WIFI_24_DRONE_RSSI_THRESH   -85     // Minimum RSSI (dBm) to consider a threat
#define WIFI_24_HOLD_TIME_MS        1500    // Hold 2.4 GHz detection state for 1.5s after last packet

// Drone Manufacturer & Protocol Identification
enum DroneVendor {
    DRONE_VENDOR_NONE         = 0,
    DRONE_VENDOR_DJI          = 1,
    DRONE_VENDOR_AUTEL        = 2,
    DRONE_VENDOR_PARROT       = 3,
    DRONE_VENDOR_SKYDIO       = 4,
    DRONE_VENDOR_OPEN_DRONE_ID= 5,
    DRONE_VENDOR_GENERIC_FPV  = 6,
    DRONE_VENDOR_YUNEEC       = 7,
    DRONE_VENDOR_MAVLINK      = 8,
    DRONE_VENDOR_CRSF_ELRS    = 9,
    DRONE_VENDOR_MSP          = 10
};

// Global 2.4 GHz Drone Detection State
extern volatile bool    g_drone24Detected;
extern volatile int8_t  g_drone24Rssi;
extern volatile uint8_t g_drone24Channel;
extern volatile uint8_t g_drone24Vendor;

// -----------------------------------------------------------------------------
// 2.3 BEHAVIORAL & PHYSICAL LAYER RF SIGNATURES (NON-COOPERATIVE DRONE DETECTION)
// -----------------------------------------------------------------------------
// Detects drones with ZERO Remote ID, disabled beacons, or spoofed MACs using:
// 1. Rigid TDD Video Interval (10ms / 20ms) vs stochastic CSMA/CA Wi-Fi
// 2. Clocked RC Control Pulses (500Hz, 250Hz, 150Hz, 50Hz)
// 3. Frequency Hopping Spread Spectrum (FHSS across >= 3 channels in 600ms)
// 4. Non-Standard 10 MHz Flat-Topped OFDM PHY Bursts
#define BEHAVIOR_DETECTION_ENABLED  1

// Timing Bins in microseconds (nominal +/- tolerance):
#define IAT_BIN_500HZ_US            2000    // 500 Hz RC link (ExpressLRS, Tracer)
#define IAT_TOL_500HZ_US            250
#define IAT_BIN_250HZ_US            4000    // 250 Hz RC link (ExpressLRS, Crossfire)
#define IAT_TOL_250HZ_US            350
#define IAT_BIN_150HZ_US            6667    // 150 Hz RC link (Crossfire)
#define IAT_TOL_150HZ_US            450
#define IAT_BIN_100HZ_TDD_US        10000   // 10 ms TDD Video / Telemetry (OcuSync, SkyLink)
#define IAT_TOL_100HZ_TDD_US        750
#define IAT_BIN_50HZ_US             20000   // 20 ms TDD Video / 50 Hz RC link
#define IAT_TOL_50HZ_US             1000

#define IAT_MIN_PERIODIC_HITS       4       // Minimum consecutive periodic frames to classify
#define FHSS_MIN_CHANNELS           3       // Minimum distinct channels seen in window to flag FHSS
#define FHSS_WINDOW_MS              600     // Sliding window for channel-hop tracking
#define BEHAVIOR_HOLD_TIME_MS       2000    // Hold behavioral threat active for 2s

// Behavioral Classification Bitmask Flags:
enum DroneBehaviorFlags {
    DRONE_BEHAVIOR_NONE         = 0,
    DRONE_BEHAVIOR_PERIODIC_TDD = (1 << 0), // 10ms or 20ms rigid TDD video frames
    DRONE_BEHAVIOR_RC_LINK      = (1 << 1), // 50/150/250/500 Hz clocked RC control pulses
    DRONE_BEHAVIOR_FHSS_HOPPING = (1 << 2), // Rapid channel hopping across 2.4 GHz band
    DRONE_BEHAVIOR_PHY_ANOMALY  = (1 << 3)  // Non-standard 10MHz OFDM / raw physical bursts
};

// Global Behavioral Detection State
extern volatile uint8_t  g_droneBehaviorFlags;
extern volatile uint16_t g_droneDetectedRateHz;
extern volatile uint8_t  g_droneHopCount;


// -----------------------------------------------------------------------------
// 3. INMP441 I2S ACOUSTIC SENSING PARAMETERS (DISABLED FOR CURRENT BUILD)
// -----------------------------------------------------------------------------
#define ACOUSTIC_DETECTOR_ENABLED   0       // 0 = Disabled (RF-only focus), 1 = Enabled

#define I2S_SAMPLE_RATE         16000   // 16 kHz sample rate (Nyquist = 8 kHz, covers 250Hz - 4kHz drone whine)
#define I2S_DMA_BUFFER_COUNT    4       // Number of DMA ring buffers
#define I2S_DMA_BUFFER_SAMPLES  256     // Samples per DMA buffer
#define ACOUSTIC_WINDOW_SAMPLES 512     // Analysis window size for RMS & filter (32 ms @ 16 kHz)

// Drone Acoustic Signature Band (Propeller blade-pass fundamental & harmonics)
#define ACOUSTIC_BAND_LOW_HZ    250.0f  // Lower bound of motor whine band
#define ACOUSTIC_BAND_HIGH_HZ   1200.0f // Upper bound of motor whine band
#define ACOUSTIC_MIN_RMS_FLOOR  500.0f  // Minimum amplitude to prevent false alarms in quiet rooms
#define ACOUSTIC_WHINE_RATIO_THRESH 0.35f // Ratio of band energy to total energy to classify as drone whine

// -----------------------------------------------------------------------------
// 4. AUDIO CUE & VOICE ALERT PARAMETERS (GPIO 15)
// -----------------------------------------------------------------------------
#define AUDIO_LEDC_CHANNEL          0       // LEDC PWM Channel
#define AUDIO_LEDC_TIMER_RES        8       // 8-bit timer resolution (0 - 255)
#define AUDIO_CLICK_FREQ_HZ         1600    // Crisp click tone frequency (Hz)
#define AUDIO_CLICK_PULSE_US        3000    // Click pulse duration in microseconds (3 ms)
#define AUDIO_WHINE_FREQ_HZ         880     // Acoustic alert tone (A5 note)
#define AUDIO_PWM_DAC_FREQ_HZ       78125   // Ultrasonic carrier for 8-bit PWM DAC audio playback (Hz)

// Geiger Counter Click Rates:
#define AUDIO_QUIESCENT_PERIOD_MS   2500 // Period between clicks in clean air (heartbeat tick)
#define AUDIO_MIN_PERIOD_MS         15   // Minimum period between clicks at 100% RF threat (rapid buzz)

// Voice Alert Repeat:
// If drone remains incoming, repeat voice alert every N ms (0 = play once per rising edge)
#define AUDIO_ALERT_REPEAT_INTERVAL_MS 10000

// -----------------------------------------------------------------------------
// 5. SERIAL LOGGING & SILENT OPERATION
// -----------------------------------------------------------------------------
// 0 = Silent mode: completely disable all serial logging and print output
// 1 = Active serial logging: dashboard, plotter, and debug lines
#define SERIAL_LOGGING_ENABLED      0
#define SERIAL_BAUD_RATE            115200
#define TELEMETRY_INTERVAL_MS       100     // 10 Hz telemetry update rate
