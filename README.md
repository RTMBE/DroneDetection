# ESP32-S3 Handheld Drone Sniffer & Detection Unit

A portable, handheld dual-mode drone detection system engineered around an **ESP32-S3**. The device combines high-frequency **5.8 GHz analog RF RSSI measurement** (analog FPV video / control link detection), **acoustic motor whine detection** via an I2S MEMS digital microphone, and a tactile **Geiger-counter style headphone audio cue** for silent, eyes-up field operation.

---

## 1. Hardware Architecture & Complete Pinout

| Module / Component | Function | Module Pin | ESP32-S3 Pin | Voltage Rail / Wiring Notes |
| :--- | :--- | :--- | :--- | :--- |
| **AD8318 Module** | 5.8 GHz Analog RF Power Detector | `Vout` (Center) | **GPIO 4** | ADC1 Channel 3 (`ADC_ATTEN_DB_12`) |
| | | `VCC` (Red) | **5V / VIN** | Powered by 5.0V regulated boost rail |
| | | `GND` (Black) | **GND** | Common system ground |
| **INMP441 Mic** | Acoustic Propeller Motor Whine | `SD` (Serial Data) | **GPIO 5** | I2S0 Data Input |
| | | `SCK` (Bit Clock) | **GPIO 6** | I2S0 BCLK |
| | | `WS` (Word Select) | **GPIO 7** | I2S0 LRCLK / Word Select |
| | | `VDD` | **3V3** | Clean 3.3V supply from ESP32 regulator |
| | | `GND` & `L/R` | **GND** | Both tied to Ground (Left channel mode) |
| **PJ-392 Audio Jack**| Headphone Audio Feedback | `Tip` (Pin 2) & `Ring` (Pin 3) | **GPIO 15** | Driven via 220Ω resistor + 10µF DC blocking cap |
| | | `Sleeve` (Pin 1) | **GND** | Audio return ground |
| **Visual Alert LED** | Drone Incoming Indicator | Anode (`+`, long leg) | **GPIO 16** | Driven via **220Ω – 330Ω** resistor |
| | | Cathode (`-`, short leg)| **GND** | Direct ground return |
| **Power Management** | Battery Boost & Regulation | AOICRIE Boost 5V | **5V / VIN** | Boosts 3.7V LiPo to regulated 5.0V |
| | | AOICRIE GND | **GND** | Common system ground |

---

## 2. Wiring Schematic & Interface Circuit

```text
                           +-------------------------------------+
                           |            ESP32-S3 DevKit          |
                           |                                     |
    +-----------------+    |                                     |
    |  AD8318 Module  |    |                                     |
    |                 |    |                                     |
    |   Vout (Center) +--->+ GPIO 4 (ADC1_CH3)                   |
    |             VCC +<---+ 5V / VIN (Regulated 5.0V)           |
    |             GND +--->+ GND                                 |
    +-----------------+    |                                     |
                           |                                     |
    +-----------------+    |                                     |
    | Visual Alert    |    |                                     |
    |   Red LED       |    |                                     |
    |     Anode (+)   +<---+ [220Ω - 330Ω] <--- GPIO 16          |
    |   Cathode (-)   +--->+ GND                                 |
    +-----------------+    |                                     |
                           |                                     |
                           | GPIO 15 (LEDC PWM)                  |
                           |    |                                |
                           +----+--------------------------------+
                                |
                              [220Ω]
                                |
                             +--+--||--+ (10µF Capacitor)
                             |
                   +---------+---------+
                   |     PJ-392 Jack   |
                   | Tip  (Pin 2) ----+ (Audio Left)
                   | Ring (Pin 3) ----+ (Audio Right)
                   | Sleeve (Pin 1) -> GND
                   +-------------------+
```

---

## 3. Signal Characteristics & Theory of Operation

### AD8318 + Foxeer 5.8 GHz Antenna (5.5 – 6.0 GHz Band on GPIO 4)
* **Frequency Range (5.5 – 6.0 GHz)**: Captures the full 5.8 GHz drone video and telemetry spectrum (Raceband 1–8: 5658–5917 MHz, Bands A, B, E, F, and FatShark).
* **Inverted Logarithmic Response**: The AD8318 outputs an analog voltage that decreases linearly with RF power expressed in dBm (~2.1V clean air down to ~0.5V at 0 dBm).
* **Multi-Sample & EMA Filtering**: 32 consecutive ADC samples are averaged and passed through an Exponential Moving Average (EMA) filter to suppress RF ripple and ADC quantization jitter.
* **Global Drone Incoming Trigger (`g_droneIncoming`)**:
  * A system-wide global flag `extern volatile bool g_droneIncoming;` triggers when RF threat exceeds `DRONE_INCOMING_TRIGGER_PCT` (default: 25%) and persists for $\ge 50\text{ ms}$.
  * Automatic event hooks `onDroneIncomingTriggered(dbm, threatPct)` and `onDroneIncomingCleared()` allow immediate integration with external sirens, LEDs, or radio alarms.

### ESP32-S3 Internal 2.4 GHz Wi-Fi Radio Sniffer (Channels 1–13)
* **802.11 Promiscuous Sniffer**: The internal 2.4 GHz radio runs in promiscuous monitor mode without associating to any access point, capturing all raw management and data packets.
* **Rapid Channel Hopper**: Sweeps through Channels 1 to 13 every 30 ms (complete 2.4 GHz spectrum sweep every ~390 ms) on Core 0.
* **Master Drone Signature Database (`drone_signatures.json`)**:
  * You can easily add, edit, or customize any signature in **`drone_signatures.json`** directly. PlatformIO automatically compiles the changes before each build:
    * **35 Hardware MAC OUIs**: DJI (13 OUIs), Autel Robotics (6 OUIs), Parrot (5 OUIs), Skydio (3 OUIs), Yuneec, and generic FPV bridges.
    * **Remote ID & Protocol IDs**: ASTM F3411 Open Drone ID (`0xFA0BBC`), DJI/French Drone ID (`0x263712`, `0x0004F4`), Wi-Fi Alliance NAN (`0xFFFA`), and Drone ID EtherType (`0x8835`).
    * **OpenDroneID Message Types**: Basic ID (`0x00`), Location (`0x10`), Auth (`0x20`), Self-ID (`0x30`), System (`0x40`), Operator ID (`0x50`), and Message Packs (`0xF0`).
    * **CTA-2063-A Manufacturer Codes**: ANSI/CTA serial prefixes for DJI (`1581`), Autel (`1588`), Skydio (`1622`), Parrot (`1587`), Yuneec (`1596`), Teal (`1688`), senseFly (`1798`), and Wingtra (`1843`).
    * **Flight Telemetry Sync Bytes**: MAVLink 1.0 (`0xFE`), MAVLink 2.0 (`0xFD`), Crossfire / ExpressLRS (`0xC8`, `0xEE`, `0xEC`), MSP flight controller (`$M<`, `$MX`), and SBUS (`0x0F`).
* **Multi-Sensor Threat Fusion**: When a 2.4 GHz drone frame is intercepted, it sets `g_drone24Detected = true`, updates `g_droneIncoming = true`, and accelerates the headphone Geiger feedback to warn the operator.

### INMP441 Acoustic Sensing (GPIO 5, 6, 7)
* **I2S Protocol**: 16 kHz sample rate, 32-bit slot, Left-channel mono.
* **Propeller Blade-Pass Frequency Filter**: Multi-rotor drone propellers spin at 8,000–25,000 RPM, producing blade-pass fundamental frequencies and harmonics concentrated in the **250 Hz – 1,200 Hz** band.
* **Cascaded Biquad Bandpass Filter**: A transposed Direct Form II 4th-order filter isolates this acoustic band from ambient wind rumbling (<150 Hz) and high-frequency noise. When both the absolute sound level and the whine energy ratio exceed preset thresholds, the acoustic alert triggers.

### Headphone Audio Cue Engine & Recorded Voice Alert (GPIO 15)
* **High-Speed PWM DAC Emulation (78.125 kHz Carrier)**:
  * Plays recorded 8-bit unsigned PCM voice alerts ("*Дрон летит*" / "*Dron letit*") stored directly in Flash memory (`PROGMEM`).
  * The 78.125 kHz ultrasonic carrier is smoothly filtered by the 220Ω series resistor and 10µF DC-blocking capacitor on the PJ-392 jack, delivering clear, loud speech directly to headphones.
  * An automated volume normalizer boosts peak signal amplitude to 95% with smooth anti-pop edge fades.
* **Non-Blocking Geiger Counter Clicks**:
  * Between voice alerts, the unit emits tactile Geiger clicks indicating drone proximity:
    * **Clean Air (0% Threat)**: 1 click every 2.5 seconds (heartbeat ticking to confirm system health).
    * **Rising Threat (10% – 80%)**: Exponentially accelerating click rate providing intuitive distance/bearing awareness.
    * **Critical Threat (90% – 100%)**: Dense audio buzz (up to 66 clicks/sec).
* **Audio Converter Utility (`tools/wav2header.py`)**:
  * Easily replace the default alert audio with any custom voice recording, MP3, or WAV sound effect:
    ```bash
    python tools/wav2header.py my_voice_alert.mp3 include/audio_samples.h --rate 11025
    ```


---

## 4. Serial Telemetry & Interactive Console (115200 Baud)

Open the Serial Monitor or Serial Plotter at **115200 baud** to view real-time diagnostics:

### Live Dashboard Output
```text
[RF] 2085mV (-66.0dBm) | Threat: [          ]   0%         || [MIC] RMS:  312 (Whine: NO, 12%) || [AUDIO]  0 cps
[RF] 1620mV (-46.7dBm) | Threat: [###       ]  30%         || [MIC] RMS:  480 (Whine: NO, 18%) || [AUDIO]  3 cps
[RF]  850mV (-14.6dBm) | Threat: [########  ]  78% [BURST] || [MIC] RMS: 1250 (Whine:YES, 49%) || [AUDIO] 22 cps
```

### Interactive Serial Commands
Send single-character commands over Serial to control the device at runtime:
* **`c`**: **Calibrate Clean Air Floor** – Captures the current ambient RF level and sets it as baseline 0% threat.
* **`m`**: **Toggle Mute** – Mutes or unmutes headphone audio output.
* **`p`**: **Toggle Plotter Mode** – Switches telemetry between dashboard text and CSV format for the Arduino Serial Plotter.
* **`h`**: **Help** – Displays the command menu.

---

## 5. Build & Flash Instructions

### Option A: VS Code with PlatformIO (Recommended)
1. Open this repository folder in VS Code.
2. Ensure the **PlatformIO IDE** extension is installed.
3. Click the PlatformIO **Build** button (`✓`) or run in terminal:
   ```bash
   pio run
   ```
4. Connect the ESP32-S3 via USB and click **Upload** (`→`) or run:
   ```bash
   pio run -t upload
   ```
5. Open the Serial Monitor at 115200 baud (`pio device monitor`).

### Option B: Arduino IDE (2.x)
1. In the Arduino IDE, open `DRONE.ino`.
2. Under **Tools**, select:
   * **Board**: `ESP32S3 Dev Module`
   * **USB CDC On Boot**: `Enabled`
   * **Upload Mode**: `UART0 / Hardware CDC`
   * **Flash Size**: `8MB` or `16MB` (matching your dev board)
3. Click **Upload** and open **Serial Monitor** at **115200 baud**.
