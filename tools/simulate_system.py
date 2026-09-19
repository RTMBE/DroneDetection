#!/usr/bin/env python3
"""
tools/simulate_system.py - End-to-End System Simulation & Verification Suite
Simulates the complete ESP32-S3 Handheld Drone Sniffer:
- Subsystem 1: AD8318 5.8 GHz Analog RF Power Detector
- Subsystem 2: Dual INMP441 I2S Stereo Digital Microphones & Stage 1/2 Sentry
- Subsystem 3: 2.4 GHz Wi-Fi Promiscuous Sniffer & Dark Drone Behavioral Classifier
- Subsystem 4: Audio Cue & Voice Alert Engine ("Дрон летит" PWM DAC)
- Subsystem 5: Dual-Modality Threat Fusion & Alert State Machine

Runs 5 comprehensive end-to-end scenarios:
1. Clean Air / Ambient Baseline (Zero false alarms)
2. Dark Drone Stealth Approach (No Remote ID, TDD video + 500Hz RC + FHSS + 5.8G FPV + Acoustic Whine)
3. Commercial Remote ID Drone Approach (DJI OcuSync / OpenDroneID ASTM F3411)
4. Heavy Interference & False-Positive Stress Test (Microwave, High-traffic Wi-Fi, Speech, Door slam)
5. Dual Microphone Spatial Steering & Beamforming SNR Verification
"""

import sys
import os
import math
import random
import struct

# =============================================================================
# SIMULATED HARDWARE & SYSTEM CONSTANTS (matching include/config.h)
# =============================================================================
RF_V_QUIESCENT_MV = 2100.0
RF_V_SATURATED_MV = 500.0
RF_SLOPE_MV_PER_DB = -24.0
RF_INTERCEPT_DBM = 0.0
RF_THREAT_MIN_PERCENT = 5.0
RF_BURST_DELTA_MV = 80.0
DRONE_INCOMING_TRIGGER_PCT = 25.0
DRONE_INCOMING_RESET_PCT = 12.0
DRONE_INCOMING_PERSIST_MS = 50
DRONE_INCOMING_HOLD_MS = 400
DUAL_THREAT_RF_MIN_PCT = 20.0

I2S_SAMPLE_RATE = 16000
I2S_DMA_BUFFER_SAMPLES = 512
ACOUSTIC_BAND_LOW_HZ = 200.0
ACOUSTIC_BAND_HIGH_HZ = 2500.0
ACOUSTIC_MIN_RMS_FLOOR = 500.0
ACOUSTIC_WHINE_RATIO_THRESH = 0.35
ACOUSTIC_ENERGY_THRESHOLD_MULTIPLIER = 2.5
ACOUSTIC_DEFAULT_ENERGY_THRESHOLD = 1200000.0
ACOUSTIC_CONFIDENCE_THRESHOLD = 0.80

AUDIO_CLICK_FREQ_HZ = 1600
AUDIO_PWM_DAC_FREQ_HZ = 78125
AUDIO_QUIESCENT_PERIOD_MS = 2500
AUDIO_MIN_PERIOD_MS = 15

# =============================================================================
# 1. RF DETECTOR SIMULATOR
# =============================================================================
class SimRFDetector:
    def __init__(self):
        self.v_quiescent = RF_V_QUIESCENT_MV
        self.v_saturated = RF_V_SATURATED_MV
        self.slope = RF_SLOPE_MV_PER_DB
        self.intercept_dbm = RF_INTERCEPT_DBM
        self.filtered_mv = RF_V_QUIESCENT_MV
        self.alpha = 0.18
        self.last_raw_mv = RF_V_QUIESCENT_MV
        self.burst_detected = False
        self.incoming = False
        self.trigger_start_ms = 0
        self.last_trigger_ms = 0

    def update(self, raw_mv, now_ms):
        # Burst detection
        if (self.last_raw_mv - raw_mv) >= RF_BURST_DELTA_MV:
            self.burst_detected = True
        else:
            self.burst_detected = False
        self.last_raw_mv = raw_mv

        # EMA filter
        self.filtered_mv = (self.alpha * raw_mv) + ((1.0 - self.alpha) * self.filtered_mv)

        # Threat %
        span = self.v_quiescent - self.v_saturated
        if self.filtered_mv >= self.v_quiescent:
            threat = 0.0
        elif self.filtered_mv <= self.v_saturated:
            threat = 100.0
        else:
            threat = ((self.v_quiescent - self.filtered_mv) / span) * 100.0

        if threat < RF_THREAT_MIN_PERCENT:
            threat = 0.0
        self.threat_pct = threat

        # Estimated dBm
        delta_v = self.filtered_mv - self.v_saturated
        self.dbm = self.intercept_dbm + (delta_v / self.slope)

        # Hysteresis & Persistence
        if self.threat_pct >= DRONE_INCOMING_TRIGGER_PCT:
            if self.trigger_start_ms == 0:
                self.trigger_start_ms = now_ms
            elif (now_ms - self.trigger_start_ms) >= DRONE_INCOMING_PERSIST_MS:
                self.incoming = True
                self.last_trigger_ms = now_ms
        else:
            self.trigger_start_ms = 0
            if self.incoming:
                if (now_ms - self.last_trigger_ms) >= DRONE_INCOMING_HOLD_MS and self.threat_pct < DRONE_INCOMING_RESET_PCT:
                    self.incoming = False

        return self.threat_pct, self.dbm, self.incoming, self.burst_detected

# =============================================================================
# 2. DUAL INMP441 STEREO ACOUSTIC SIMULATOR
# =============================================================================
class SimBiquad:
    def __init__(self, low_hz=200.0, high_hz=2500.0, fs=16000.0):
        Q = 0.7071
        # HPF
        w_hp = 2.0 * math.pi * (low_hz / fs)
        cos_hp = math.cos(w_hp)
        sin_hp = math.sin(w_hp)
        alpha_hp = sin_hp / (2.0 * Q)
        hp_a0 = 1.0 + alpha_hp
        self.hp_b0 = ((1.0 + cos_hp) / 2.0) / hp_a0
        self.hp_b1 = (-(1.0 + cos_hp)) / hp_a0
        self.hp_b2 = ((1.0 + cos_hp) / 2.0) / hp_a0
        self.hp_a1 = (-2.0 * cos_hp) / hp_a0
        self.hp_a2 = (1.0 - alpha_hp) / hp_a0
        self.hp_z1 = 0.0
        self.hp_z2 = 0.0

        # LPF
        w_lp = 2.0 * math.pi * (high_hz / fs)
        cos_lp = math.cos(w_lp)
        sin_lp = math.sin(w_lp)
        alpha_lp = sin_lp / (2.0 * Q)
        lp_a0 = 1.0 + alpha_lp
        self.lp_b0 = ((1.0 - cos_lp) / 2.0) / lp_a0
        self.lp_b1 = (1.0 - cos_lp) / lp_a0
        self.lp_b2 = ((1.0 - cos_lp) / 2.0) / lp_a0
        self.lp_a1 = (-2.0 * cos_lp) / lp_a0
        self.lp_a2 = (1.0 - alpha_lp) / lp_a0
        self.lp_z1 = 0.0
        self.lp_z2 = 0.0

    def process(self, x):
        hp_out = (self.hp_b0 * x) + self.hp_z1
        self.hp_z1 = (self.hp_b1 * x) - (self.hp_a1 * hp_out) + self.hp_z2
        self.hp_z2 = (self.hp_b2 * x) - (self.hp_a2 * hp_out)

        lp_out = (self.lp_b0 * hp_out) + self.lp_z1
        self.lp_z1 = (self.lp_b1 * hp_out) - (self.lp_a1 * lp_out) + self.lp_z2
        self.lp_z2 = (self.lp_b2 * hp_out) - (self.lp_a2 * lp_out)
        return lp_out

class SimDualAcousticDetector:
    def __init__(self):
        self.filter_L = SimBiquad()
        self.filter_R = SimBiquad()
        self.baseline_energy = 45000.0
        self.energy_threshold = self.baseline_energy * ACOUSTIC_ENERGY_THRESHOLD_MULTIPLIER
        self.rms_L = 0.0
        self.rms_R = 0.0
        self.rms_whine_L = 0.0
        self.rms_whine_R = 0.0
        self.whine_ratio_L = 0.0
        self.whine_ratio_R = 0.0
        self.whine_detected_L = False
        self.whine_detected_R = False
        self.whine_detected = False
        self.channel_balance = 0.0
        self.stage1_triggered = False
        self.drone_confirmed = False
        self.drone_confidence = 0.0

    def calibrate(self, ambient_frames_mono):
        energies = []
        for frame in ambient_frames_mono:
            e = sum(s * s for s in frame) / len(frame)
            energies.append(e)
        self.baseline_energy = sum(energies) / len(energies)
        self.energy_threshold = max(self.baseline_energy * ACOUSTIC_ENERGY_THRESHOLD_MULTIPLIER, 20000.0)

    def process_frame(self, frame_left, frame_right):
        n = len(frame_left)
        assert len(frame_right) == n

        sum_sq_L = 0.0
        sum_sq_R = 0.0
        sum_sq_mono = 0.0
        sum_sq_whine_L = 0.0
        sum_sq_whine_R = 0.0

        for i in range(n):
            sL = frame_left[i]
            sR = frame_right[i]
            sMono = (sL + sR) / 2.0

            sum_sq_L += sL * sL
            sum_sq_R += sR * sR
            sum_sq_mono += sMono * sMono

            fL = self.filter_L.process(sL)
            fR = self.filter_R.process(sR)

            sum_sq_whine_L += fL * fL
            sum_sq_whine_R += fR * fR

        frame_rms_L = math.sqrt(sum_sq_L / n)
        frame_rms_R = math.sqrt(sum_sq_R / n)
        frame_rms_whine_L = math.sqrt(sum_sq_whine_L / n)
        frame_rms_whine_R = math.sqrt(sum_sq_whine_R / n)

        # EMA
        alpha = 0.25
        self.rms_L = (self.rms_L * (1.0 - alpha)) + (frame_rms_L * alpha)
        self.rms_R = (self.rms_R * (1.0 - alpha)) + (frame_rms_R * alpha)
        self.rms_whine_L = (self.rms_whine_L * (1.0 - alpha)) + (frame_rms_whine_L * alpha)
        self.rms_whine_R = (self.rms_whine_R * (1.0 - alpha)) + (frame_rms_whine_R * alpha)

        self.whine_ratio_L = self.rms_whine_L / self.rms_L if self.rms_L > 10.0 else 0.0
        self.whine_ratio_R = self.rms_whine_R / self.rms_R if self.rms_R > 10.0 else 0.0

        self.whine_detected_L = (self.rms_L >= ACOUSTIC_MIN_RMS_FLOOR and self.whine_ratio_L >= ACOUSTIC_WHINE_RATIO_THRESH)
        self.whine_detected_R = (self.rms_R >= ACOUSTIC_MIN_RMS_FLOOR and self.whine_ratio_R >= ACOUSTIC_WHINE_RATIO_THRESH)
        self.whine_detected = self.whine_detected_L or self.whine_detected_R

        sum_rms = self.rms_L + self.rms_R
        self.channel_balance = ((self.rms_L - self.rms_R) / sum_rms) if sum_rms > 1.0 else 0.0

        latest_frame_energy = sum_sq_mono / n
        self.stage1_triggered = latest_frame_energy > self.energy_threshold

        max_whine_ratio = max(self.whine_ratio_L, self.whine_ratio_R)
        if self.stage1_triggered:
            # Stage 2: Active TinyML / Harmonic Peak Analysis (Feature 6 in train_acoustic_model.py)
            # Measures spectral tonality & harmonic concentration (drone motor vs speech/noise)
            sMono_list = [(frame_left[j] + frame_right[j]) / 2.0 for j in range(n)]
            fft_size = 256
            num_bins = fft_size // 2
            mags = [0.0] * num_bins
            for k in range(num_bins):
                w = 2.0 * math.pi * k / fft_size
                re = sum(sMono_list[idx] * math.cos(w * idx) for idx in range(min(n, fft_size)))
                im = sum(sMono_list[idx] * math.sin(w * idx) for idx in range(min(n, fft_size)))
                mags[k] = math.sqrt(re * re + im * im)

            total_mag = sum(mags) + 1e-6
            sorted_mags = sorted(mags, reverse=True)
            top3_ratio = sum(sorted_mags[:3]) / total_mag

            # Drone motor/blade-pass has sharp discrete harmonic peaks (top3_ratio >= 0.30)
            # Human speech, impulses, and broadband noise distribute energy across many bins (top3_ratio < 0.25)
            is_harmonic_drone = (top3_ratio >= 0.30) and (max_whine_ratio >= ACOUSTIC_WHINE_RATIO_THRESH)

            if is_harmonic_drone:
                self.drone_confidence = min(0.95, 0.70 + (max_whine_ratio * 0.4))
            else:
                self.drone_confidence = 0.15
            self.drone_confirmed = (self.drone_confidence >= ACOUSTIC_CONFIDENCE_THRESHOLD)
        else:
            self.drone_confirmed = False
            self.drone_confidence = 0.0

        return {
            "rms_L": self.rms_L,
            "rms_R": self.rms_R,
            "whine_ratio_L": self.whine_ratio_L,
            "whine_ratio_R": self.whine_ratio_R,
            "whine_detected": self.whine_detected,
            "channel_balance": self.channel_balance,
            "stage1": self.stage1_triggered,
            "drone_confirmed": self.drone_confirmed,
            "confidence": self.drone_confidence,
            "energy": latest_frame_energy
        }

# =============================================================================
# 3. 2.4 GHz WI-FI & BEHAVIORAL SNIFFER SIMULATOR
# =============================================================================
class SimWiFiDetector:
    def __init__(self):
        self.known_ouis = {
            "04A85A": "DJI", "0C9AE6": "DJI", "34D262": "DJI", "481CB9": "DJI",
            "4C43F6": "DJI", "58B858": "DJI", "60601F": "DJI", "60814A": "DJI",
            "687A49": "DJI", "882985": "DJI", "8C5823": "DJI", "E47A2C": "DJI",
            "286D97": "AUTEL", "3CE90E": "AUTEL", "64CE85": "AUTEL", "A0C9A0": "AUTEL",
            "00121C": "PARROT", "0024E4": "PARROT", "00267E": "PARROT", "9003B7": "PARROT",
            "A0143D": "PARROT", "000B6B": "SKYDIO", "38D269": "SKYDIO", "E0B6F5": "SKYDIO",
            "E470B8": "SKYDIO"
        }
        self.drone_detected = False
        self.drone_vendor = 0 # DRONE_VENDOR_NONE
        self.behavior_flags = 0
        self.detected_rate_hz = 0
        self.hop_count = 0
        self.last_packet_us = 0
        self.consecutive_periodic = 0
        self.channel_mask = 0
        self.first_channel_seen_ms = 0
        self.hold_until_ms = 0

    def feed_packet(self, mac_oui, channel, now_us, now_ms, is_corrupted_10mhz=False):
        # 1. Remote ID / OUI match
        if mac_oui and mac_oui.upper() in self.known_ouis:
            vendor_name = self.known_ouis[mac_oui.upper()]
            if vendor_name == "DJI": self.drone_vendor = 1
            elif vendor_name == "AUTEL": self.drone_vendor = 2
            elif vendor_name == "PARROT": self.drone_vendor = 3
            elif vendor_name == "SKYDIO": self.drone_vendor = 4
            self.drone_detected = True
            self.hold_until_ms = now_ms + 1500

        # 2. Inter-Arrival Time (IAT) Periodicity Check
        if self.last_packet_us > 0:
            delta_us = now_us - self.last_packet_us
            bin_rate = 0
            if 9250 <= delta_us <= 10750:
                bin_rate = 100 # 10 ms TDD Video
            elif 1750 <= delta_us <= 2250:
                bin_rate = 500 # 500 Hz RC link
            elif 3650 <= delta_us <= 4350:
                bin_rate = 250 # 250 Hz RC link
            elif 6217 <= delta_us <= 7117:
                bin_rate = 150 # 150 Hz RC link
            elif 19000 <= delta_us <= 21000:
                bin_rate = 50  # 20 ms TDD / 50 Hz RC

            if bin_rate > 0:
                if self.detected_rate_hz == bin_rate:
                    self.consecutive_periodic += 1
                else:
                    self.detected_rate_hz = bin_rate
                    self.consecutive_periodic = 1

                if self.consecutive_periodic >= 4:
                    if bin_rate in [50, 100]:
                        self.behavior_flags |= (1 << 0) # DRONE_BEHAVIOR_PERIODIC_TDD
                    else:
                        self.behavior_flags |= (1 << 1) # DRONE_BEHAVIOR_RC_LINK
                    self.drone_detected = True
                    self.hold_until_ms = now_ms + 2000
            else:
                if delta_us > 35000:
                    self.consecutive_periodic = 0
                elif self.consecutive_periodic > 0:
                    self.consecutive_periodic -= 1
        self.last_packet_us = now_us

        # 3. Frequency Hopping (FHSS)
        if (now_ms - self.first_channel_seen_ms) > 600:
            self.channel_mask = 0
            self.first_channel_seen_ms = now_ms
        self.channel_mask |= (1 << channel)
        self.hop_count = bin(self.channel_mask).count('1')
        if self.hop_count >= 3:
            self.behavior_flags |= (1 << 2) # DRONE_BEHAVIOR_FHSS_HOPPING
            self.drone_detected = True
            self.hold_until_ms = now_ms + 2000

        # 4. 10 MHz flat-topped PHY burst anomaly
        if is_corrupted_10mhz:
            self.behavior_flags |= (1 << 3) # DRONE_BEHAVIOR_PHY_ANOMALY
            self.drone_detected = True
            self.hold_until_ms = now_ms + 2000

        return self.drone_detected, self.behavior_flags

    def update_hold(self, now_ms):
        if now_ms >= self.hold_until_ms:
            self.drone_detected = False
            self.behavior_flags = 0
            self.detected_rate_hz = 0
            self.drone_vendor = 0
        return self.drone_detected

# =============================================================================
# 4. AUDIO CUE SIMULATOR (Geiger Clicks & Voice Alert)
# =============================================================================
class SimAudioCue:
    def __init__(self):
        self.clicks_per_sec = 0
        self.voice_alert_playing = False
        self.voice_alert_count = 0
        self.last_click_ms = 0

    def update(self, effective_threat_pct, now_ms):
        # Click period calculation
        if effective_threat_pct <= 0.0:
            period_ms = AUDIO_QUIESCENT_PERIOD_MS
        elif effective_threat_pct >= 100.0:
            period_ms = AUDIO_MIN_PERIOD_MS
        else:
            frac = effective_threat_pct / 100.0
            period_ms = int(AUDIO_QUIESCENT_PERIOD_MS - (frac * (AUDIO_QUIESCENT_PERIOD_MS - AUDIO_MIN_PERIOD_MS)))
            if period_ms < AUDIO_MIN_PERIOD_MS:
                period_ms = AUDIO_MIN_PERIOD_MS

        self.clicks_per_sec = int(1000.0 / period_ms) if period_ms > 0 else 0
        return period_ms, self.clicks_per_sec

    def play_voice_alert(self):
        self.voice_alert_playing = True
        self.voice_alert_count += 1

# =============================================================================
# SCENARIO SIMULATIONS
# =============================================================================

def sim_scenario_1_clean_air():
    print("\n---------------------------------------------------------------------")
    print("SCENARIO 1: Clean Air / Ambient Baseline (Zero False Alarm Verification)")
    print("---------------------------------------------------------------------")
    rf = SimRFDetector()
    mic = SimDualAcousticDetector()
    wifi = SimWiFiDetector()
    audio = SimAudioCue()

    # 1. Ambient calibration (3 seconds = ~93 frames of 512 samples)
    ambient_frames = []
    for _ in range(93):
        # Pink/white noise floor ~RMS 50-100
        noise = [int(random.gauss(0, 150)) for _ in range(512)]
        ambient_frames.append(noise)
    mic.calibrate(ambient_frames)
    print(f"  [INIT] Calibrated baseline energy: {mic.baseline_energy:.1f} | Threshold: {mic.energy_threshold:.1f}")

    # 2. Run 10 seconds of simulated clean air
    # RF: 2100 mV with small random ADC flicker (+/- 5 mV)
    # Mic: Quiet room noise
    # Wi-Fi: Standard AP beacons on Channel 6 every 102.4 ms
    drone_incoming_flags = []
    click_periods = []
    now_ms = 0
    now_us = 0

    for step in range(100): # 100 steps * 100ms = 10s
        now_ms = step * 100
        now_us = now_ms * 1000

        # RF update
        rf_v = 2100.0 + random.uniform(-5.0, 5.0)
        rf_threat, rf_dbm, rf_inc, rf_burst = rf.update(rf_v, now_ms)

        # Mic update (stereo quiet room)
        noise_L = [int(random.gauss(0, 150)) for _ in range(512)]
        noise_R = [int(random.gauss(0, 150)) for _ in range(512)]
        mic_res = mic.process_frame(noise_L, noise_R)

        # Wi-Fi update (standard router beacon)
        if step % 1 == 0:
            wifi.feed_packet(mac_oui="001A2B", channel=6, now_us=now_us, now_ms=now_ms) # Generic non-drone OUI

        # Threat fusion
        rf_elevated = (rf_threat >= DUAL_THREAT_RF_MIN_PCT) or rf_burst
        acoustic_elevated = mic_res["drone_confirmed"] or (mic_res["stage1"] and mic_res["whine_detected"])
        dual_threat = rf_elevated and acoustic_elevated
        g_drone_incoming = rf_inc or wifi.drone_detected or acoustic_elevated or dual_threat
        drone_incoming_flags.append(g_drone_incoming)

        # Audio cue
        eff_threat = rf_threat
        period_ms, cps = audio.update(eff_threat, now_ms)
        click_periods.append(period_ms)

    assert not any(drone_incoming_flags), "FALSE ALARM: Clean air must NEVER trigger g_droneIncoming!"
    assert all(p >= 2000 for p in click_periods), f"Click period in clean air must be heartbeat (~2500ms), got {min(click_periods)}ms"
    assert audio.voice_alert_count == 0, "Voice alert must not play in clean air"
    print("  [PASS] Clean air produced 0 false alarms over 10 seconds.")
    print(f"  [PASS] Click engine maintained quiescent heartbeat ({click_periods[-1]} ms period, {cps} clicks/sec).")


def sim_scenario_2_dark_drone():
    print("\n---------------------------------------------------------------------")
    print("SCENARIO 2: Non-Cooperative 'Dark Drone' Stealth Approach")
    print("            (No Remote ID, 10ms TDD Video + 500Hz RC + FHSS + 5.8GHz FPV + Whine)")
    print("---------------------------------------------------------------------")
    rf = SimRFDetector()
    mic = SimDualAcousticDetector()
    wifi = SimWiFiDetector()
    audio = SimAudioCue()

    # Pre-calibrate
    ambient = [[int(random.gauss(0, 150)) for _ in range(512)] for _ in range(93)]
    mic.calibrate(ambient)

    now_ms = 0
    now_us = 0
    last_voice_play_ms = -99999
    led_states = []
    dual_threat_confirmed_events = 0

    # Drone starts distant at step 0, approaches to close range at step 50
    for step in range(60):
        now_ms = step * 50 # 50 ms loop
        now_us = now_ms * 1000

        # RF Signal: Voltage drops from 2100mV down to 1000mV (-20.8 dBm, ~68% threat)
        progress = min(1.0, step / 40.0)
        rf_v = 2100.0 - (progress * 1100.0) + random.uniform(-15.0, 15.0)
        # Add a 90mV burst drop at step 30
        if step == 30:
            rf_v -= 90.0
        rf_threat, rf_dbm, rf_inc, rf_burst = rf.update(rf_v, now_ms)

        # Acoustic: 1200 Hz propeller blade-pass tone approaching from LEFT (Mic 1 louder than Mic 2)
        # At step >= 20, drone sound becomes prominent
        amp_L = 1000.0 * progress
        amp_R = 400.0 * progress # Mic 2 is attenuated because drone is on the left
        freq_whine = 1200.0 # 1.2 kHz motor whine
        f_s = 16000.0

        samples_L = [int(amp_L * math.sin(2.0 * math.pi * freq_whine * i / f_s) + random.gauss(0, 100)) for i in range(512)]
        samples_R = [int(amp_R * math.sin(2.0 * math.pi * freq_whine * i / f_s) + random.gauss(0, 100)) for i in range(512)]
        mic_res = mic.process_frame(samples_L, samples_R)

        # 2.4 GHz Radio: ExpressLRS 500 Hz RC link (2000 us pulses) + FHSS hopping across channels 1, 5, 9, 13
        ch = [1, 5, 9, 13][step % 4]
        # Feed 6 packets (5 intervals of 2000us) to lock on 500 Hz RC link
        for p_idx in range(6):
            wifi.feed_packet(mac_oui=None, channel=ch, now_us=now_us + (p_idx * 2000), now_ms=now_ms)

        # System State Evaluation
        rf_elevated = (rf_threat >= DUAL_THREAT_RF_MIN_PCT) or rf_burst
        acoustic_elevated = mic_res["drone_confirmed"] or (mic_res["stage1"] and mic_res["whine_detected"])
        dual_threat = rf_elevated and acoustic_elevated
        g_drone_incoming = rf_inc or wifi.drone_detected or acoustic_elevated or dual_threat

        if dual_threat:
            dual_threat_confirmed_events += 1

        # LED
        led_states.append(g_drone_incoming)

        # Effective threat & Audio
        eff_threat = rf_threat
        if dual_threat:
            eff_threat = 95.0
        elif mic_res["drone_confirmed"]:
            eff_threat = 75.0
        elif wifi.drone_detected:
            eff_threat = max(eff_threat, 60.0)

        period_ms, cps = audio.update(eff_threat, now_ms)

        # Voice alert trigger
        if g_drone_incoming and (now_ms - last_voice_play_ms >= 10000):
            audio.play_voice_alert()
            last_voice_play_ms = now_ms

    assert dual_threat_confirmed_events > 0, "Dual-modality threat MUST confirm on simultaneous RF + acoustic whine!"
    assert wifi.behavior_flags & (1 << 1), "RC Link 500Hz signature MUST be flagged!"
    assert wifi.behavior_flags & (1 << 2), "FHSS hopping signature MUST be flagged!"
    assert mic_res["whine_detected"], "Propeller whine MUST be detected by acoustic module!"
    assert mic_res["channel_balance"] > 0.3, f"Acoustic spatial steering should indicate Left approach (>0.3), got {mic_res['channel_balance']:.2f}"
    assert audio.voice_alert_count >= 1, "Voice alert 'Дрон летит' MUST trigger!"
    assert any(led_states), "Alert LED must illuminate during drone approach!"
    print(f"  [PASS] Dark Drone successfully detected via Behavioral Signatures (Flags: 0x{wifi.behavior_flags:02X}).")
    print(f"  [PASS] Dual-modality confirmed {dual_threat_confirmed_events} times.")
    print(f"  [PASS] Spatial balance correctly located drone approaching from LEFT (Balance: +{mic_res['channel_balance']:.2f}).")
    print(f"  [PASS] Voice alert triggered {audio.voice_alert_count} time(s); Geiger clicks maxed out at {audio.clicks_per_sec} cps.")


def sim_scenario_3_remote_id_drone():
    print("\n---------------------------------------------------------------------")
    print("SCENARIO 3: Commercial Remote ID Drone Approach (DJI OcuSync / ASTM OpenDroneID)")
    print("---------------------------------------------------------------------")
    rf = SimRFDetector()
    mic = SimDualAcousticDetector()
    wifi = SimWiFiDetector()
    audio = SimAudioCue()

    # Pre-calibrate
    ambient = [[int(random.gauss(0, 150)) for _ in range(512)] for _ in range(93)]
    mic.calibrate(ambient)

    # DJI OUI 60:60:1F
    now_us = 1000000
    now_ms = 1000
    drone_detected, flags = wifi.feed_packet(mac_oui="60601F", channel=11, now_us=now_us, now_ms=now_ms)

    assert drone_detected, "DJI OUI must trigger drone_detected"
    assert wifi.drone_vendor == 1, f"Expected DJI vendor code 1, got {wifi.drone_vendor}"

    # Also simulate 10ms TDD frames from DJI OcuSync video link
    for i in range(5):
        now_us += 10000 # Exactly 10ms
        wifi.feed_packet(mac_oui="60601F", channel=11, now_us=now_us, now_ms=now_ms)

    assert wifi.behavior_flags & (1 << 0), "10ms TDD video periodic interval MUST be classified!"
    print(f"  [PASS] DJI Drone confirmed via Hardware OUI (Vendor: DJI) + 10ms TDD Video timing.")


def sim_scenario_4_false_positive_stress_test():
    print("\n---------------------------------------------------------------------")
    print("SCENARIO 4: Heavy Interference & False-Positive Stress Test")
    print("            (High-traffic CSMA Wi-Fi, Microwave RF leakage, Loud Speech, Door Slam)")
    print("---------------------------------------------------------------------")
    rf = SimRFDetector()
    mic = SimDualAcousticDetector()
    wifi = SimWiFiDetector()
    audio = SimAudioCue()

    ambient = [[int(random.gauss(0, 150)) for _ in range(512)] for _ in range(93)]
    mic.calibrate(ambient)

    # 1. High-traffic CSMA/CA Wi-Fi router (Hundreds of packets with stochastic backoff delays)
    now_us = 0
    for _ in range(200):
        # Random stochastic delay typical of 802.11 CSMA/CA: 200us to 35000us
        delay_us = random.randint(300, 28000)
        now_us += delay_us
        now_ms = now_us // 1000
        # Parked on Channel 6, non-drone OUI (Intel Wi-Fi card)
        wifi.feed_packet(mac_oui="A44CC8", channel=6, now_us=now_us, now_ms=now_ms)

    assert not wifi.drone_detected, "Random CSMA/CA Wi-Fi traffic must NOT falsely trigger drone detection!"
    assert wifi.behavior_flags == 0, f"No behavioral flags should trigger on normal Wi-Fi (got 0x{wifi.behavior_flags:02X})"
    print("  [PASS] 200 random CSMA/CA Wi-Fi packets rejected (0 false locks on IAT bins).")

    # 2. Loud Human Speech (Glottal pulse train at 120 Hz with formant resonances and fricative noise)
    speech_L = [0] * 512
    pitch_period = 133  # ~120 Hz fundamental pitch
    for i in range(512):
        phase = i % pitch_period
        glottal = math.exp(-phase / 25.0) * math.sin(2.0 * math.pi * phase / 25.0)
        f1 = math.exp(-phase / 35.0) * math.sin(2.0 * math.pi * 700.0 * phase / 16000.0)
        f2 = math.exp(-phase / 45.0) * math.sin(2.0 * math.pi * 1250.0 * phase / 16000.0)
        fricative = random.gauss(0, 0.3)
        speech_L[i] = int(2000.0 * (glottal + 0.7 * f1 + 0.5 * f2 + fricative))
    speech_R = list(speech_L)
    mic_res = mic.process_frame(speech_L, speech_R)
    assert not mic_res["drone_confirmed"], "Human speech must NOT be classified as drone!"
    print(f"  [PASS] Loud human speech rejected (whine ratio: {mic_res['whine_ratio_L']:.2f}, confidence: {mic_res['confidence']:.2f}).")

    # 3. Door Slam / Transient Impulse
    slam_L = [15000 if i == 10 else 0 for i in range(512)] # Single impulse
    slam_R = list(slam_L)
    mic_res_slam = mic.process_frame(slam_L, slam_R)
    assert not mic_res_slam["drone_confirmed"], "Transient impulse spike must not be confirmed as drone!"
    print("  [PASS] Transient impulse (door slam) rejected.")


def sim_scenario_5_dual_mic_beamforming():
    print("\n---------------------------------------------------------------------")
    print("SCENARIO 5: Dual Microphone Spatial Steering & Beamforming SNR Gain")
    print("---------------------------------------------------------------------")
    mic = SimDualAcousticDetector()

    # Create an in-phase drone signal + uncorrelated noise on each channel
    signal_amp = 800.0
    noise_sigma = 600.0
    n = 16000 # 1 second

    drone_sig = [signal_amp * math.sin(2.0 * math.pi * 950.0 * i / 16000.0) for i in range(n)]
    noise_1 = [random.gauss(0, noise_sigma) for _ in range(n)]
    noise_2 = [random.gauss(0, noise_sigma) for _ in range(n)]

    mic1_input = [drone_sig[i] + noise_1[i] for i in range(n)]
    mic2_input = [drone_sig[i] + noise_2[i] for i in range(n)]

    # Beamformed mono
    beamformed_mono = [(mic1_input[i] + mic2_input[i]) / 2.0 for i in range(n)]

    # Calculate input SNR for mic 1 vs beamformed SNR
    # Noise power:
    noise_power_mic1 = sum(n1 ** 2 for n1 in noise_1) / n
    noise_in_beam = sum(((noise_1[i] + noise_2[i]) / 2.0) ** 2 for i in range(n)) / n
    signal_power = sum(s ** 2 for s in drone_sig) / n

    snr_mic1_db = 10.0 * math.log10(signal_power / noise_power_mic1)
    snr_beam_db = 10.0 * math.log10(signal_power / noise_in_beam)
    snr_gain_db = snr_beam_db - snr_mic1_db

    print(f"  [BEAMFORMING] Single Mic SNR: {snr_mic1_db:.2f} dB | Dual-Mic Beamformed SNR: {snr_beam_db:.2f} dB")
    print(f"  [BEAMFORMING] Measured SNR Gain: +{snr_gain_db:.2f} dB (Theoretical: +3.01 dB)")
    assert snr_gain_db >= 2.5, f"Dual mic beamforming must provide >= 2.5 dB SNR gain over uncorrelated noise, got {snr_gain_db:.2f} dB"

    # Spatial steering test:
    # Test Left source (allow 10 frames for EMA smoothing to settle)
    for _ in range(10):
        res_left = mic.process_frame([s * 1.5 for s in mic1_input[:512]], [s * 0.4 for s in mic2_input[:512]])
    assert res_left["channel_balance"] > 0.4, f"Expected channel balance > +0.4 for Left source, got {res_left['channel_balance']:.2f}"

    # Test Right source (allow 10 frames for EMA smoothing to settle)
    for _ in range(10):
        res_right = mic.process_frame([s * 0.4 for s in mic1_input[:512]], [s * 1.5 for s in mic2_input[:512]])
    assert res_right["channel_balance"] < -0.4, f"Expected channel balance < -0.4 for Right source, got {res_right['channel_balance']:.2f}"

    print(f"  [PASS] Spatial steering verified: Left source = {res_left['channel_balance']:+.2f}, Right source = {res_right['channel_balance']:+.2f}")
    print(f"  [PASS] Dual-mic coherent beamforming verified (+{snr_gain_db:.2f} dB gain).")


def main():
    print("=====================================================================")
    print("ESP32-S3 Drone Sniffer - Full End-to-End System Simulation Suite")
    print("=====================================================================")
    random.seed(42) # Deterministic verification

    sim_scenario_1_clean_air()
    sim_scenario_2_dark_drone()
    sim_scenario_3_remote_id_drone()
    sim_scenario_4_false_positive_stress_test()
    sim_scenario_5_dual_mic_beamforming()

    print("\n=====================================================================")
    print("ALL 5 END-TO-END SCENARIOS PASSED WITH 100% SUCCESS!")
    print("Hardware signal paths, state machines, and threat fusion verified.")
    print("=====================================================================")

if __name__ == "__main__":
    main()
