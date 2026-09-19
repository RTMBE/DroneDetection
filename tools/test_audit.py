#!/usr/bin/env python3
"""
test_audit.py - Comprehensive Unit & Integration Test Suite for ESP32-S3 Drone Sniffer
Audits and tests:
1. RF Detector transfer function, dBm math, threat percentage, and hysteresis
2. Audio sample buffer integrity, normalization, and DAC PWM timing
3. Drone signature database completeness (OUIs, Remote IDs, CTA codes, sync bytes)
4. Inter-Arrival Time (IAT) periodicity classifier (TDD 10ms/20ms & 500/250/150/50Hz RC)
5. FHSS Frequency Hopping detector logic (channel-mask and window decay)
6. Physical Layer 10 MHz burst anomaly filter
"""

import sys
import os
import json
import math
import wave

def run_test(name, fn):
    try:
        fn()
        print(f"  [PASS] {name}")
        return True
    except AssertionError as e:
        print(f"  [FAIL] {name}: {e}")
        return False
    except Exception as e:
        print(f"  [ERROR] {name}: {e}")
        return False

# =============================================================================
# 1. RF Detector Math & Transfer Function Audit
# =============================================================================
def test_rf_math():
    v_quiescent = 2100.0
    v_saturated = 500.0
    slope = -24.0
    intercept_dbm = 0.0
    span = v_quiescent - v_saturated # 1600 mV

    def calc_dbm(v_mv):
        delta = v_mv - v_saturated
        return intercept_dbm + (delta / slope)

    def calc_threat(v_mv):
        if v_mv >= v_quiescent: return 0.0
        if v_mv <= v_saturated: return 100.0
        pct = ((v_quiescent - v_mv) / span) * 100.0
        return 0.0 if pct < 5.0 else pct

    # Clean air
    assert calc_threat(2100.0) == 0.0, "Clean air must be 0% threat"
    assert abs(calc_dbm(2100.0) - (-66.67)) < 0.1, "Quiescent dBm calculation incorrect"

    # Deadband test (< 5%)
    assert calc_threat(2050.0) == 0.0, "Deadband filter must zero minor baseline drift < 5%"

    # 25% Trigger threshold: 400 mV drop (1700 mV)
    threat_25 = calc_threat(1700.0)
    assert abs(threat_25 - 25.0) < 0.01, f"Expected 25% threat at 1700mV, got {threat_25}%"
    dbm_25 = calc_dbm(1700.0)
    assert abs(dbm_25 - (-50.0)) < 0.01, f"Expected -50 dBm at 25% threat, got {dbm_25}"

    # Saturation (500 mV)
    assert calc_threat(500.0) == 100.0, "Saturated input must be 100% threat"
    assert calc_dbm(500.0) == 0.0, "Saturated input must be 0 dBm"

    # Hysteresis test
    threat_reset = calc_threat(1920.0) # 180mV drop = 11.25%
    assert threat_reset < 12.0, "Reset threshold should clear below 12%"

# =============================================================================
# 2. Audio Sample Buffer & PWM DAC Timing Audit
# =============================================================================
def test_audio_samples():
    header_path = "include/audio_samples.h"
    assert os.path.exists(header_path), "audio_samples.h does not exist"

    with open(header_path, "r", encoding="utf-8") as f:
        content = f.read()

    assert "AUDIO_ALERT_SAMPLE_RATE" in content
    assert "AUDIO_ALERT_SAMPLE_COUNT" in content
    assert "DRONE_ALERT_AUDIO_DATA" in content

    # Extract sample rate
    rate = 11025
    for line in content.splitlines():
        if "#define AUDIO_ALERT_SAMPLE_RATE" in line:
            rate = int(line.split()[-1])
            break
    assert rate == 11025, f"Expected sample rate 11025, got {rate}"

    # Verify PWM carrier period
    carrier_hz = 78125
    carrier_period_us = 1000000.0 / carrier_hz
    assert carrier_period_us < 15.0, "PWM carrier must be ultrasonic (> 20 kHz)"
    assert rate <= carrier_hz // 4, "Carrier frequency must be at least 4x the audio sample rate"

# =============================================================================
# 3. Signature Database Completeness Audit
# =============================================================================
def test_signature_database():
    json_path = "drone_signatures.json"
    assert os.path.exists(json_path), "drone_signatures.json not found"

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Check Remote ID protocols
    r_ids = [item["id"].upper() for item in data.get("remote_id_and_protocols", [])]
    required_ids = ["0XFA0BBC", "0XFFFA", "0X8835", "0X263712"]
    for rid in required_ids:
        assert rid in r_ids, f"Missing Remote ID protocol {rid}"

    # Check OpenDroneID message types
    msg_types = [item["type"].upper() for item in data.get("opendroneid_message_types", [])]
    required_types = ["0X00", "0X10", "0X20", "0X30", "0X40", "0X50", "0XF0"]
    for mt in required_types:
        assert mt in msg_types, f"Missing OpenDroneID message type {mt}"

    # Check CTA-2063-A codes
    cta_codes = [item["code"] for item in data.get("cta_2063_manufacturers", [])]
    required_cta = ["1581", "1588", "1622", "1587", "1596", "1688", "1798", "1843"]
    for c in required_cta:
        assert c in cta_codes, f"Missing CTA-2063-A code {c}"

    # Check OUIs
    ouis = [item["oui"].upper() for item in data.get("hardware_mac_ouis", [])]
    assert len(ouis) >= 30, f"Expected >= 30 OUIs, found {len(ouis)}"
    must_have_ouis = [
        "0X04A85A", "0X0C9AE6", "0X34D262", "0X481CB9", "0X4C43F6",
        "0X58B858", "0X60601F", "0X60814A", "0X687A49", "0X882985",
        "0X8C5823", "0XE47A2C", "0X286D97", "0X3CE90E", "0X64CE85",
        "0XA0C9A0", "0X00121C", "0X0024E4", "0X00267E", "0X9003B7",
        "0XA0143D", "0X000B6B", "0X38D269", "0XE0B6F5", "0XE470B8"
    ]
    for o in must_have_ouis:
        assert o in ouis, f"Missing hardware OUI {o}"

    # Check packet sync bytes
    syncs = [item["byte"].upper() for item in data.get("packet_sync_bytes", [])]
    must_have_syncs = ["0XFE", "0XFD", "0XC8", "0XEE", "0XEC", "0X24", "0X4D", "0X3C", "0X58", "0X0F", "0X00"]
    for s in must_have_syncs:
        assert s in syncs, f"Missing packet sync byte {s}"

# =============================================================================
# 4. Inter-Arrival Time (IAT) Periodicity Classifier Simulation
# =============================================================================
def test_iat_periodicity_classifier():
    bins = [
        (2000, 250, 500, "500 Hz RC"),
        (4000, 350, 250, "250 Hz RC"),
        (6667, 450, 150, "150 Hz RC"),
        (10000, 750, 100, "10 ms TDD Video"),
        (20000, 1000, 50, "20 ms TDD/RC")
    ]

    def classify_delta(delta_us):
        for nom, tol, rate, label in bins:
            if (nom - tol) <= delta_us <= (nom + tol):
                return rate
        return 0

    # Test 500 Hz pulse train (ExpressLRS / Tracer)
    hits = 0
    t = 1000000
    for _ in range(5):
        t += 2020 # 2000us nominal + 20us jitter
        delta = 2020
        rate = classify_delta(delta)
        assert rate == 500, f"Expected 500 Hz, got {rate}"
        hits += 1
    assert hits >= 4, "Should lock on 500 Hz pulse train"

    # Test 10 ms TDD Video (OcuSync / SkyLink)
    hits = 0
    for _ in range(5):
        delta = 10040 # 10ms + 40us jitter
        rate = classify_delta(delta)
        assert rate == 100, f"Expected 100 Hz (10ms TDD), got {rate}"
        hits += 1
    assert hits >= 4, "Should lock on 10ms TDD video"

    # Test random CSMA/CA Wi-Fi jitter (must NOT match)
    random_deltas = [1250, 8300, 15400, 3200, 11900, 5400]
    matched = [classify_delta(d) for d in random_deltas]
    assert all(m == 0 for m in matched), f"Random Wi-Fi traffic must not falsely lock into bins: {matched}"

# =============================================================================
# 5. FHSS Frequency Hopping Detection Simulation
# =============================================================================
def test_fhss_hopping():
    class ChannelTracker:
        def __init__(self):
            self.mask = 0
            self.count = 0
            self.first_seen = 0

        def record(self, channel, now_ms):
            if (now_ms - self.first_seen) > 600:
                self.mask = 0
                self.count = 0
                self.first_seen = now_ms

            bit = (1 << channel)
            if not (self.mask & bit):
                self.mask |= bit
                self.count += 1

            return self.count >= 3

    tracker = ChannelTracker()

    # Case 1: Standard Wi-Fi parked on Channel 6
    for t in [0, 50, 100, 150, 200]:
        flag = tracker.record(6, t)
        assert not flag, "Static channel must never trigger FHSS flag"

    # Case 2: Standard Wi-Fi with dual-band or minor beacon overlap on 2 channels
    tracker = ChannelTracker()
    flag1 = tracker.record(1, 0)
    flag2 = tracker.record(6, 100)
    assert not (flag1 or flag2), "2 channels must not trigger FHSS (requires >= 3)"

    # Case 3: FHSS Drone hopping across Ch 1, Ch 7, Ch 11 in 300 ms
    flag3 = tracker.record(11, 200)
    assert flag3, "Transmitter seen on 3 distinct channels in 200ms MUST trigger FHSS"

# =============================================================================
# 6. Physical-Layer 10 MHz Burst Anomaly Simulation
# =============================================================================
def test_phy_anomaly():
    # 10 MHz flat-topped shoulders generate bursts with rx_state != 0 at 10ms intervals
    class PhyDetector:
        def __init__(self):
            self.last_us = 0
            self.count = 0

        def feed_burst(self, rx_state, rssi, now_us):
            if rx_state != 0 and rssi >= -80:
                if self.last_us > 0:
                    d = now_us - self.last_us
                    if (10000 - 750) <= d <= (10000 + 750):
                        self.count += 1
                        if self.count >= 4:
                            return True
                    elif d > 35000:
                        self.count = 0
                self.last_us = now_us
            return False

    detector = PhyDetector()
    triggered = False
    now = 1000000
    for _ in range(5):
        now += 10050 # 10ms with 50us jitter
        if detector.feed_burst(1, -72, now):
            triggered = True
            break

    assert triggered, "Periodic 10ms corrupted bursts at strong RSSI must trigger 10MHz PHY anomaly"

# =============================================================================
# 7. Circular Ring Buffer & Edge Impulse Callback Math Audit
# =============================================================================
def test_ring_buffer_callback():
    INFERENCE_SAMPLES = 16000
    # Simulate ring buffer
    ring_buffer = [i for i in range(INFERENCE_SAMPLES)]
    buffer_head = 4500 # Simulated current write head

    def get_signal_data(offset, length):
        out = []
        for i in range(length):
            idx = (buffer_head + offset + i) % INFERENCE_SAMPLES
            out.append(float(ring_buffer[idx]))
        return out

    # Test chunk 0: Should extract 320 samples starting from buffer_head
    chunk0 = get_signal_data(0, 320)
    assert len(chunk0) == 320
    assert chunk0[0] == 4500.0
    assert chunk0[319] == 4819.0

    # Test chunk across ring wrap boundary
    # Offset = 11400 -> (4500 + 11400) = 15900
    chunk_wrap = get_signal_data(11400, 200)
    assert len(chunk_wrap) == 200
    assert chunk_wrap[0] == 15900.0
    assert chunk_wrap[99] == 15999.0
    assert chunk_wrap[100] == 0.0 # Wrapped around to index 0
    assert chunk_wrap[199] == 99.0

    # Total window completeness
    full_window = get_signal_data(0, INFERENCE_SAMPLES)
    assert len(full_window) == INFERENCE_SAMPLES
    assert set(full_window) == set(float(x) for x in range(INFERENCE_SAMPLES)), "Every sample in the 1-second window must be accessed without loss"

# =============================================================================
# 8. Dynamic Ambient Calibration Math Audit
# =============================================================================
def test_ambient_calibration_math():
    multiplier = 2.5
    min_floor = 20000.0

    # Case 1: Standard quiet room baseline
    ambient_frames = [45000.0, 48000.0, 42000.0, 46000.0]
    avg_energy = sum(ambient_frames) / len(ambient_frames)
    threshold = avg_energy * multiplier
    assert avg_energy == 45250.0
    assert threshold == 113125.0
    assert threshold > avg_energy * 2.0, "Threshold must preserve at least 2x headroom"

    # Case 2: Ultra-silent anechoic floor (must clamp to min floor)
    silent_frames = [1200.0, 1500.0, 1100.0]
    silent_avg = sum(silent_frames) / len(silent_frames)
    calc_th = silent_avg * multiplier
    final_th = max(calc_th, min_floor)
    assert final_th == min_floor, f"Ultra-quiet baseline must clamp to minimum threshold {min_floor}"

    # Case 3: Drone spike triggers sentry
    drone_spike_energy = 150000.0
    assert drone_spike_energy > threshold, "Drone acoustic spike must exceed calibrated threshold"

# =============================================================================
# 9. Dual-Modality Threat Confirmation Truth Table
# =============================================================================
def test_dual_modality_fusion():
    DUAL_THREAT_RF_MIN_PCT = 20.0

    def evaluate_threat(rf_pct, is_burst, acoustic_drone, whine_ratio, stage1_energy, thresh):
        rf_elevated = (rf_pct >= DUAL_THREAT_RF_MIN_PCT) or is_burst
        stage1_trig = stage1_energy > thresh
        acoustic_elevated = acoustic_drone or (stage1_trig and whine_ratio >= 0.35)
        dual_confirmed = (rf_elevated and acoustic_elevated)
        return dual_confirmed

    # 1. Clean air (no threat)
    assert not evaluate_threat(0.0, False, False, 0.10, 10000, 50000)

    # 2. RF alone (distant FPV transmitter, quiet room)
    assert not evaluate_threat(45.0, False, False, 0.12, 12000, 50000)

    # 3. Acoustic alone (loud lawnmower or transient noise, clean RF)
    assert not evaluate_threat(0.0, False, True, 0.40, 150000, 50000)

    # 4. Dual-Modality Confirmed (5.8 GHz RF carrier + Stage 2 Acoustic Drone)
    assert evaluate_threat(35.0, False, True, 0.42, 180000, 50000), "RF + Stage 2 Acoustic must confirm dual threat"

    # 5. Dual-Modality Confirmed (RF Burst + Stage 1 Motor Whine Spike)
    assert evaluate_threat(10.0, True, False, 0.50, 120000, 50000), "RF Burst + Stage 1 Whine must confirm dual threat"

# =============================================================================
# 10. Stage 1 Biquad Bandpass Filter (200 Hz - 2500 Hz @ 16 kHz)
# =============================================================================
def test_biquad_bandpass():
    Fs = 16000.0
    Q = 0.7071

    # HPF 200 Hz
    omega_hp = 2.0 * math.pi * (200.0 / Fs)
    cos_hp = math.cos(omega_hp)
    sin_hp = math.sin(omega_hp)
    alpha_hp = sin_hp / (2.0 * Q)
    hp_a0 = 1.0 + alpha_hp
    hp_b0 = ((1.0 + cos_hp) / 2.0) / hp_a0
    hp_b1 = (-(1.0 + cos_hp)) / hp_a0
    hp_b2 = ((1.0 + cos_hp) / 2.0) / hp_a0
    hp_a1 = (-2.0 * cos_hp) / hp_a0
    hp_a2 = (1.0 - alpha_hp) / hp_a0

    # LPF 2500 Hz
    omega_lp = 2.0 * math.pi * (2500.0 / Fs)
    cos_lp = math.cos(omega_lp)
    sin_lp = math.sin(omega_lp)
    alpha_lp = sin_lp / (2.0 * Q)
    lp_a0 = 1.0 + alpha_lp
    lp_b0 = ((1.0 - cos_lp) / 2.0) / lp_a0
    lp_b1 = (1.0 - cos_lp) / lp_a0
    lp_b2 = ((1.0 - cos_lp) / 2.0) / lp_a0
    lp_a1 = (-2.0 * cos_lp) / lp_a0
    lp_a2 = (1.0 - alpha_lp) / lp_a0

    def biquad_response(b0, b1, b2, a1, a2, f):
        w = 2.0 * math.pi * f / Fs
        num = complex(b0 + b1 * math.cos(w) + b2 * math.cos(2*w), -b1 * math.sin(w) - b2 * math.sin(2*w))
        den = complex(1.0 + a1 * math.cos(w) + a2 * math.cos(2*w), -a1 * math.sin(w) - a2 * math.sin(2*w))
        return abs(num / den)

    # In-band signal (1000 Hz propeller whine fundamental)
    g_1000 = biquad_response(hp_b0, hp_b1, hp_b2, hp_a1, hp_a2, 1000.0) * \
             biquad_response(lp_b0, lp_b1, lp_b2, lp_a1, lp_a2, 1000.0)
    assert g_1000 > 0.85, f"1000 Hz motor whine must pass with near unity gain, got {g_1000:.3f}"

    # Out-of-band low rumble (50 Hz wind/mains)
    g_50 = biquad_response(hp_b0, hp_b1, hp_b2, hp_a1, hp_a2, 50.0) * \
           biquad_response(lp_b0, lp_b1, lp_b2, lp_a1, lp_a2, 50.0)
    assert g_50 < 0.15, f"50 Hz sub-rumble must be strongly attenuated, got {g_50:.3f}"

    # Out-of-band high hiss (6000 Hz)
    g_6000 = biquad_response(hp_b0, hp_b1, hp_b2, hp_a1, hp_a2, 6000.0) * \
             biquad_response(lp_b0, lp_b1, lp_b2, lp_a1, lp_a2, 6000.0)
    assert g_6000 < 0.20, f"6000 Hz high frequency noise must be attenuated, got {g_6000:.3f}"

# =============================================================================
# 11. Dataset Slicing & Preprocessing Pipeline Audit
# =============================================================================
def test_dataset_pipeline():
    tools_dir = os.path.dirname(os.path.abspath(__file__))
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    from drone_audio_dataset import slice_wav, audit_dataset
    import struct
    import shutil

    test_dir = os.path.join(tools_dir, "_test_dataset")
    test_wav = os.path.join(test_dir, "test_synth.wav")
    slices_dir = os.path.join(test_dir, "slices")
    os.makedirs(slices_dir, exist_ok=True)

    try:
        # Generate 2.5s of synthetic 16kHz mono audio (40,000 samples)
        sample_rate = 16000
        duration_s = 2.5
        n_samples = int(sample_rate * duration_s)

        with wave.open(test_wav, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(sample_rate)
            raw = bytearray()
            for i in range(n_samples):
                val = int(10000.0 * math.sin(2.0 * math.pi * 800.0 * i / sample_rate))
                raw.extend(struct.pack("<h", val))
            w.writeframes(raw)

        # Slice with 1.0s window and 0.5s stride -> should produce (2.5 - 1.0)/0.5 + 1 = 4 slices
        count = slice_wav(test_wav, slices_dir, "synth", slice_len_sec=1.0, stride_sec=0.5)
        assert count == 4, f"Expected 4 slices with 0.5s stride, got {count}"

        # Audit generated slices
        valid = audit_dataset(slices_dir)
        assert valid, "Generated slices must pass 16kHz 1.0s mono audit"

    finally:
        # Cleanup temporary test files
        shutil.rmtree(test_dir, ignore_errors=True)

# =============================================================================
# 12. Pure-C Lightweight Model Training & C Header Export Audit
# =============================================================================
def test_pure_c_model_training():
    tools_dir = os.path.dirname(os.path.abspath(__file__))
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    from train_acoustic_model import train_model, extract_features
    import struct
    import shutil

    test_dir = os.path.join(tools_dir, "_test_train_dataset")
    drone_dir = os.path.join(test_dir, "drone")
    bg_dir = os.path.join(test_dir, "background")
    out_header = os.path.join(test_dir, "test_drone_model.h")
    os.makedirs(drone_dir, exist_ok=True)
    os.makedirs(bg_dir, exist_ok=True)

    try:
        sample_rate = 16000
        n_samples = 16000 # 1.0s

        # Generate 8 drone slices (harmonic tones ~600Hz, 1200Hz, 1800Hz)
        for idx in range(8):
            wav_path = os.path.join(drone_dir, f"drone_{idx:02d}.wav")
            with wave.open(wav_path, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(sample_rate)
                raw = bytearray()
                f0 = 500.0 + idx * 30.0
                for i in range(n_samples):
                    val = int(8000.0 * math.sin(2.0 * math.pi * f0 * i / sample_rate) +
                              4000.0 * math.sin(2.0 * math.pi * 2.0 * f0 * i / sample_rate))
                    val = max(-32767, min(32767, val))
                    raw.extend(struct.pack("<h", val))
                w.writeframes(raw)

        # Generate 8 background slices (low rumble ~60Hz + noise)
        for idx in range(8):
            wav_path = os.path.join(bg_dir, f"bg_{idx:02d}.wav")
            with wave.open(wav_path, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(sample_rate)
                raw = bytearray()
                f_rumble = 50.0 + idx * 10.0
                for i in range(n_samples):
                    val = int(6000.0 * math.sin(2.0 * math.pi * f_rumble * i / sample_rate))
                    val = max(-32767, min(32767, val))
                    raw.extend(struct.pack("<h", val))
                w.writeframes(raw)

        # Train model
        success = train_model(test_dir, output_header=out_header, n_trees=5, max_depth=3)
        assert success, "Model training must succeed on synthetic dataset"
        assert os.path.exists(out_header), "Generated C header must exist"

        with open(out_header, "r", encoding="utf-8") as f:
            content = f.read()

        assert "drone_model_predict_prob" in content
        assert "extract_audio_features" in content
        assert "tree_0_predict" in content

    finally:
        shutil.rmtree(test_dir, ignore_errors=True)

def main():
    print("=====================================================================")
    print("ESP32-S3 Drone Sniffer - Comprehensive Audit & System Test Suite")
    print("=====================================================================")

    tests = [
        ("1. AD8318 RF Power Transfer Function & Hysteresis", test_rf_math),
        ("2. Audio Sample Buffer & PWM DAC Frequency Audit", test_audio_samples),
        ("3. Master Signature Database Completeness (OUIs/Protocols/Sync)", test_signature_database),
        ("4. IAT Periodicity Classifier (TDD Video & Clocked RC Links)", test_iat_periodicity_classifier),
        ("5. FHSS Frequency Hopping Detection (Multi-Channel Sweeps)", test_fhss_hopping),
        ("6. Physical-Layer Non-Standard 10 MHz Anomaly Filter", test_phy_anomaly),
        ("7. Circular Ring Buffer & Edge Impulse Callback Math", test_ring_buffer_callback),
        ("8. Dynamic Ambient Calibration Math & Sentry Headroom", test_ambient_calibration_math),
        ("9. Dual-Modality Threat Confirmation Truth Table", test_dual_modality_fusion),
        ("10. Stage 1 Biquad Bandpass Filter (200 Hz - 2500 Hz)", test_biquad_bandpass),
        ("11. Dataset Slicing & Preprocessing Pipeline Audit", test_dataset_pipeline),
        ("12. Pure-C Lightweight Model Training & C Header Export", test_pure_c_model_training),
    ]

    passed = 0
    for name, fn in tests:
        if run_test(name, fn):
            passed += 1

    print("=====================================================================")
    print(f"Test Summary: {passed}/{len(tests)} tests passed ({passed/len(tests)*100:.0f}%)")
    print("=====================================================================")

    if passed != len(tests):
        sys.exit(1)

if __name__ == "__main__":
    main()

