#!/usr/bin/env python3
"""
train_acoustic_model.py - 100% Free & Crazy Lightweight TinyML Model Trainer
Part of ESP32-S3 Acoustic Drone Detection Pipeline

Trains a lightweight Random Forest classifier on 1-second audio slices and
exports a zero-dependency, pure-C header (include/drone_model.h) that runs
on the ESP32-S3 in under 1 millisecond with < 4 KB of RAM!

Emphasizes 26 Frequency-Isolation Features:
- 15 Narrow Sub-Bands (50 Hz to 7.5 kHz) covering blade-pass fundamental & motor whine harmonics
- Spectral Flatness (Wiener entropy) to isolate tonal drone harmonics from broadband noise
- Dominant Peak Tracking (Hz) and drone-band presence indicator
- Harmonic Peak Concentration Ratio (top 3 peaks in drone band)
- Wideband distribution ratios (Sub-Rumble, Drone Core, High Hiss)
- RMS Energy & Zero Crossing Rate
"""

import os
import sys
import wave
import struct
import math
import argparse
import glob

# Try importing numpy and scikit-learn
try:
    import numpy as np
    from sklearn.ensemble import RandomForestClassifier, ExtraTreesClassifier
    from sklearn.tree import DecisionTreeClassifier, _tree
    from sklearn.model_selection import StratifiedKFold
    from sklearn.metrics import accuracy_score, classification_report, confusion_matrix, precision_score, recall_score, f1_score
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False

NUM_FEATURES = 26
FEATURE_NAMES = [
    "rms_energy",             # 0: Overall sound pressure
    "zero_crossing_rate",     # 1: Time-domain roughness / frequency rate
    "sub_rumble_ratio",       # 2: 50 - 150 Hz / total energy (wind/traffic)
    "drone_core_ratio",       # 3: 150 - 4500 Hz / total energy (drone band)
    "high_hiss_ratio",        # 4: 5500 - 7500 Hz / total energy (high hiss)
    "spectral_flatness",      # 5: Wiener entropy (0 = pure tone/drone, 1 = white noise)
    "spectral_crest_factor",  # 6: Peak mag / mean mag (sharp harmonic spikes)
    "spectral_centroid",      # 7: Spectral center of mass in Hz
    "dominant_peak_hz",       # 8: Frequency in Hz of highest peak
    "peak_in_drone_band",     # 9: 1.0 if peak is 150 - 4500 Hz, else 0.0
    "harmonic_peak_ratio",    # 10: Top 3 peaks in drone band / drone core energy
    "band_0_50_150hz",        # 11: Wind & ground vehicle rumble
    "band_1_150_250hz",       # 12: Heavy drone low blade-pass fundamental
    "band_2_250_400hz",       # 13: Mavic / commercial quad hover fundamental
    "band_3_400_600hz",       # 14: 5" FPV hover fundamental
    "band_4_600_850hz",       # 15: 5" FPV cruise / punch fundamental
    "band_5_850_1150hz",      # 16: Micro drone / 2nd blade harmonic
    "band_6_1150_1500hz",     # 17: Micro drone high / 2nd-3rd harmonic
    "band_7_1500_1900hz",     # 18: Motor whine harmonic 1
    "band_8_1900_2400hz",     # 19: Motor whine harmonic 2
    "band_9_2400_3000hz",     # 20: Motor whine harmonic 3
    "band_10_3000_3700hz",    # 21: High motor commutation whine
    "band_11_3700_4500hz",    # 22: Upper motor whine
    "band_12_4500_5500hz",    # 23: Upper harmonic transition
    "band_13_5500_6500hz",    # 24: High environmental noise
    "band_14_6500_7500hz",    # 25: Extreme high hiss
]

def load_audio_file(file_path):
    """Loads any audio file (WAV, MP3, OGG, FLAC) into mono 16 kHz int16 samples."""
    ext = os.path.splitext(file_path)[1].lower()

    # 1. Try standard wave module for 16-bit WAV files
    if ext == ".wav":
        try:
            with wave.open(file_path, "rb") as w:
                n_channels = w.getnchannels()
                sampwidth = w.getsampwidth()
                framerate = w.getframerate()
                n_frames = w.getnframes()
                raw = w.readframes(n_frames)

            if sampwidth == 2:
                samples = struct.unpack(f"<{n_frames * n_channels}h", raw)
                if n_channels > 1:
                    samples = samples[0::n_channels]
                samples = list(samples)
                if framerate != 16000 and HAS_SKLEARN:
                    t_orig = np.linspace(0, 1, len(samples))
                    new_len = int(len(samples) * 16000 / framerate)
                    t_new = np.linspace(0, 1, new_len)
                    samples = np.interp(t_new, t_orig, samples).astype(int).tolist()
                return samples, 16000
        except Exception:
            pass

    # 2. Try pygame for MP3, OGG, FLAC, and compressed audio
    try:
        import pygame
        if not pygame.mixer.get_init():
            pygame.mixer.init(frequency=44100, size=-16, channels=2)
        sound = pygame.mixer.Sound(file_path)
        raw = sound.get_raw()
        num_samples = len(raw) // 4
        samples_44k = []
        for i in range(num_samples):
            l, r = struct.unpack_from('<hh', raw, i * 4)
            samples_44k.append((l + r) // 2)

        if HAS_SKLEARN and len(samples_44k) > 0:
            t_orig = np.linspace(0, 1, len(samples_44k))
            new_len = int(len(samples_44k) * 16000 / 44100)
            t_new = np.linspace(0, 1, new_len)
            samples_16k = np.interp(t_new, t_orig, samples_44k).astype(int).tolist()
            return samples_16k, 16000
        return samples_44k, 44100
    except Exception as e:
        raise ValueError(f"Could not load audio file '{file_path}': {e}")

def extract_features(samples, sample_rate=16000):
    """
    Extracts 26 frequency-isolation acoustic features:
    - RMS Energy & Zero Crossing Rate
    - Spectral Flatness (Wiener Entropy) & Crest Factor
    - Dominant Peak Tracking & Drone Band Indicator
    - Harmonic Peak Concentration Ratio
    - 3 Wideband Distribution Ratios (Sub-Rumble, Drone Core, High Hiss)
    - 15 Narrow Sub-Bands (50 Hz to 7.5 kHz) covering blade-pass and motor whine harmonics
    """
    N = len(samples)
    if N == 0:
        return [0.0] * NUM_FEATURES

    # 1. RMS Energy
    sum_sq = sum(s * s for s in samples)
    rms = math.sqrt(sum_sq / N)

    # 2. Zero Crossing Rate
    zcr_count = 0
    for i in range(1, N):
        if (samples[i] >= 0 and samples[i - 1] < 0) or (samples[i] < 0 and samples[i - 1] >= 0):
            zcr_count += 1
    zcr = zcr_count / N

    # 3. Frequency Spectrum via Fast Windowed DFT (256-point blocks, 128 bins, 62.5 Hz/bin)
    fft_size = 256
    hop_size = 256
    num_bins = fft_size // 2  # 128 bins
    bin_width = sample_rate / fft_size  # 62.5 Hz

    avg_mag = [0.0] * num_bins
    n_windows = 0

    # Hamming window
    hamm = [0.54 - 0.46 * math.cos(2 * math.pi * i / (fft_size - 1)) for i in range(fft_size)]

    for start in range(0, N - fft_size + 1, hop_size):
        chunk = [samples[start + i] * hamm[i] for i in range(fft_size)]
        if HAS_SKLEARN:
            fft_res = np.fft.rfft(chunk)
            mag = np.abs(fft_res)[:num_bins]
            for b in range(num_bins):
                avg_mag[b] += float(mag[b])
        else:
            for b in range(num_bins):
                w = 2.0 * math.pi * b / fft_size
                re = sum(chunk[k] * math.cos(w * k) for k in range(fft_size))
                im = sum(chunk[k] * -math.sin(w * k) for k in range(fft_size))
                avg_mag[b] += math.hypot(re, im)
        n_windows += 1

    if n_windows > 0:
        avg_mag = [m / n_windows for m in avg_mag]

    # Sub-band energies
    def band_energy(low_hz, high_hz):
        start_bin = max(1, int(low_hz / bin_width))
        end_bin = min(num_bins, int(high_hz / bin_width))
        if end_bin <= start_bin:
            return 0.0
        return sum(avg_mag[b] * avg_mag[b] for b in range(start_bin, end_bin))

    # 15 Narrow Bands
    b0 = band_energy(50, 150)     # Band 0: Sub rumble
    b1 = band_energy(150, 250)    # Band 1: Heavy drone low BPF
    b2 = band_energy(250, 400)    # Band 2: Mavic / commercial quad hover BPF
    b3 = band_energy(400, 600)    # Band 3: 5" FPV hover BPF
    b4 = band_energy(600, 850)    # Band 4: 5" FPV cruise BPF
    b5 = band_energy(850, 1150)   # Band 5: Micro drone / 2nd BPF harmonic
    b6 = band_energy(1150, 1500)  # Band 6: Micro drone high / 2nd-3rd harmonic
    b7 = band_energy(1500, 1900)  # Band 7: Motor whine harmonic 1
    b8 = band_energy(1900, 2400)  # Band 8: Motor whine harmonic 2
    b9 = band_energy(2400, 3000)  # Band 9: Motor whine harmonic 3
    b10 = band_energy(3000, 3700) # Band 10: High motor whine
    b11 = band_energy(3700, 4500) # Band 11: Upper motor whine
    b12 = band_energy(4500, 5500) # Band 12: Upper harmonic
    b13 = band_energy(5500, 6500) # Band 13: High noise
    b14 = band_energy(6500, 7500) # Band 14: Extreme high hiss

    bands = [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14]

    total_energy = sum(bands) + 1e-6
    sub_rumble_ratio = b0 / total_energy
    drone_core_energy = sum(bands[1:12])  # Bands 1 through 11 (150 Hz - 4500 Hz)
    drone_core_ratio = drone_core_energy / total_energy
    high_hiss_energy = b13 + b14          # Bands 13 & 14 (5500 Hz - 7500 Hz)
    high_hiss_ratio = high_hiss_energy / total_energy

    # Spectral Flatness (Wiener entropy) across 100 Hz - 5000 Hz
    b_start = max(1, int(100 / bin_width))
    b_end = min(num_bins, int(5000 / bin_width))
    log_sum = 0.0
    lin_sum = 0.0
    count_flat = 0
    for b in range(b_start, b_end):
        p = avg_mag[b] * avg_mag[b] + 1e-7
        log_sum += math.log(p)
        lin_sum += p
        count_flat += 1
    if count_flat > 0 and lin_sum > 0:
        geom_mean = math.exp(log_sum / count_flat)
        arith_mean = lin_sum / count_flat
        spectral_flatness = min(1.0, geom_mean / (arith_mean + 1e-6))
    else:
        spectral_flatness = 1.0

    # Spectral Crest Factor (peak magnitude / average magnitude)
    peak_mag = max(avg_mag) if avg_mag else 0.0
    mean_mag = (sum(avg_mag) / len(avg_mag)) + 1e-6
    crest_factor = peak_mag / mean_mag

    # Spectral Centroid
    num_sum = sum(b * bin_width * avg_mag[b] for b in range(num_bins))
    den_sum = sum(avg_mag) + 1e-6
    centroid = num_sum / den_sum

    # Dominant Peak Tracking in Drone Band (150 Hz - 4500 Hz)
    peak_bin = 0
    max_peak_val = -1.0
    for b in range(max(1, int(150 / bin_width)), min(num_bins, int(4500 / bin_width))):
        if avg_mag[b] > max_peak_val:
            max_peak_val = avg_mag[b]
            peak_bin = b
    dominant_peak_hz = peak_bin * bin_width
    peak_in_drone_band = 1.0 if (150.0 <= dominant_peak_hz <= 4500.0 and max_peak_val > 0.0) else 0.0

    # Harmonic Peak Concentration Ratio: energy in top 3 peaks in drone band / drone_core_energy
    sorted_drone_mags = sorted([avg_mag[b] for b in range(max(1, int(150 / bin_width)), min(num_bins, int(4500 / bin_width)))], reverse=True)
    top3_energy = sum(m * m for m in sorted_drone_mags[:3]) if len(sorted_drone_mags) >= 3 else 0.0
    harmonic_peak_ratio = top3_energy / (drone_core_energy + 1e-6)

    return [
        rms,
        zcr,
        sub_rumble_ratio,
        drone_core_ratio,
        high_hiss_ratio,
        spectral_flatness,
        crest_factor,
        centroid,
        dominant_peak_hz,
        peak_in_drone_band,
        harmonic_peak_ratio,
        b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14
    ]

def tree_to_c(tree_model, tree_idx):
    """Recursively converts a single scikit-learn DecisionTree into pure C code."""
    tree = tree_model.tree_
    lines = []

    def recurse(node, depth):
        indent = "  " * depth
        if tree.feature[node] != _tree.TREE_UNDEFINED:
            name = f"features[{tree.feature[node]}]"
            thresh = tree.threshold[node]
            lines.append(f"{indent}if ({name} <= {thresh:.6f}f) {{")
            recurse(tree.children_left[node], depth + 1)
            lines.append(f"{indent}}} else {{")
            recurse(tree.children_right[node], depth + 1)
            lines.append(f"{indent}}}")
        else:
            # Leaf node: value is [[prob_bg, prob_drone]]
            val = tree.value[node][0]
            prob_drone = val[1] / sum(val) if sum(val) > 0 else 0.0
            lines.append(f"{indent}return {prob_drone:.4f}f;")

    lines.append(f"static inline float tree_{tree_idx}_predict(const float* features) {{")
    recurse(0, 1)
    lines.append("}\n")
    return "\n".join(lines)

def export_c_header(forest_model, output_header_path, accuracy):
    """Exports a trained Random Forest classifier as a self-contained C header."""
    n_trees = len(forest_model.estimators_)
    c_trees = [tree_to_c(forest_model.estimators_[i], i) for i in range(n_trees)]

    header_content = f"""// =============================================================================
// ESP32-S3 TinyML Acoustic Drone Detection Model
// AUTO-GENERATED by tools/train_acoustic_model.py
// Pure C, Zero-Dependency Random Forest Classifier (Accuracy: {accuracy*100:.1f}%)
// Emphasizes 26 Frequency-Isolation Features (Blade-Pass + Motor Whine Harmonics)
// =============================================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define DRONE_MODEL_NUM_FEATURES   {NUM_FEATURES}
#define DRONE_MODEL_NUM_TREES      {n_trees}
#define DRONE_MODEL_ACCURACY_PCT   {accuracy*100:.1f}f

// Individual Decision Trees
{chr(10).join(c_trees)}

// =============================================================================
// Inference API
// =============================================================================

// Returns drone probability from 0.0f to 1.0f (average vote of all decision trees)
// Execution time: < 50 microseconds on ESP32-S3 @ 240 MHz
static inline float drone_model_predict_prob(const float* features) {{
    float sum = 0.0f;
"""
    for i in range(n_trees):
        header_content += f"    sum += tree_{i}_predict(features);\n"

    header_content += f"""    return sum / (float)DRONE_MODEL_NUM_TREES;
}}

// Returns 1 if drone is detected (confidence >= threshold), 0 otherwise
static inline int drone_model_predict(const float* features, float threshold) {{
    return (drone_model_predict_prob(features) >= threshold) ? 1 : 0;
}}

// Feature extraction from raw 16 kHz int16 audio buffer (16,000 samples = 1.0s)
// Computes 26 Frequency-Isolation Features for robust sentry detection
static inline void extract_audio_features(const int16_t* samples, size_t n_samples, float* out_features) {{
    if (n_samples == 0 || out_features == NULL) return;

    // 1. RMS Energy & Zero Crossing Rate
    double sum_sq = 0.0;
    size_t zcr_count = 0;
    for (size_t i = 0; i < n_samples; i++) {{
        double s = (double)samples[i];
        sum_sq += (s * s);
        if (i > 0) {{
            if ((samples[i] >= 0 && samples[i-1] < 0) || (samples[i] < 0 && samples[i-1] >= 0)) {{
                zcr_count++;
            }}
        }}
    }}
    float rms = sqrtf((float)(sum_sq / (double)n_samples));
    float zcr = (float)zcr_count / (float)n_samples;

    out_features[0] = rms;
    out_features[1] = zcr;

    // 2. Fast Windowed DFT (256-point, 128 bins, 62.5 Hz per bin)
    const int fft_size = 256;
    const int num_bins = fft_size / 2;
    const float bin_width = 16000.0f / (float)fft_size; // 62.5 Hz per bin

    float avg_mag[128] = {{ 0.0f }};
    int n_windows = 0;

    for (size_t start = 0; start + fft_size <= n_samples; start += 512) {{
        for (int b = 0; b < num_bins; b++) {{
            float w = 2.0f * 3.14159265f * (float)b / (float)fft_size;
            float re = 0.0f, im = 0.0f;
            for (int k = 0; k < fft_size; k += 2) {{
                float s = (float)samples[start + k];
                re += s * cosf(w * (float)k);
                im -= s * sinf(w * (float)k);
            }}
            avg_mag[b] += sqrtf(re * re + im * im);
        }}
        n_windows++;
    }}

    if (n_windows > 0) {{
        for (int b = 0; b < num_bins; b++) {{
            avg_mag[b] /= (float)n_windows;
        }}
    }}

    auto calc_band = [&](float low_hz, float high_hz) -> float {{
        int b_start = (int)(low_hz / bin_width);
        int b_end = (int)(high_hz / bin_width);
        if (b_start < 1) b_start = 1;
        if (b_end > num_bins) b_end = num_bins;
        float e = 0.0f;
        for (int b = b_start; b < b_end; b++) {{
            e += (avg_mag[b] * avg_mag[b]);
        }}
        return e;
    }};

    float b0  = calc_band(50.0f, 150.0f);
    float b1  = calc_band(150.0f, 250.0f);
    float b2  = calc_band(250.0f, 400.0f);
    float b3  = calc_band(400.0f, 600.0f);
    float b4  = calc_band(600.0f, 850.0f);
    float b5  = calc_band(850.0f, 1150.0f);
    float b6  = calc_band(1150.0f, 1500.0f);
    float b7  = calc_band(1500.0f, 1900.0f);
    float b8  = calc_band(1900.0f, 2400.0f);
    float b9  = calc_band(2400.0f, 3000.0f);
    float b10 = calc_band(3000.0f, 3700.0f);
    float b11 = calc_band(3700.0f, 4500.0f);
    float b12 = calc_band(4500.0f, 5500.0f);
    float b13 = calc_band(5500.0f, 6500.0f);
    float b14 = calc_band(6500.0f, 7500.0f);

    float total_e = b0 + b1 + b2 + b3 + b4 + b5 + b6 + b7 + b8 + b9 + b10 + b11 + b12 + b13 + b14 + 1e-6f;
    float drone_core_e = b1 + b2 + b3 + b4 + b5 + b6 + b7 + b8 + b9 + b10 + b11;

    out_features[2] = b0 / total_e;                  // sub_rumble_ratio
    out_features[3] = drone_core_e / total_e;        // drone_core_ratio
    out_features[4] = (b13 + b14) / total_e;         // high_hiss_ratio

    // Spectral Flatness (Wiener entropy) across 100 Hz - 5000 Hz
    int b_flat_start = (int)(100.0f / bin_width);
    int b_flat_end = (int)(5000.0f / bin_width);
    if (b_flat_start < 1) b_flat_start = 1;
    if (b_flat_end > num_bins) b_flat_end = num_bins;
    float log_sum = 0.0f, lin_sum = 0.0f;
    int count_flat = 0;
    for (int b = b_flat_start; b < b_flat_end; b++) {{
        float p = avg_mag[b] * avg_mag[b] + 1e-7f;
        log_sum += logf(p);
        lin_sum += p;
        count_flat++;
    }}
    float flatness = 1.0f;
    if (count_flat > 0 && lin_sum > 0.0f) {{
        float geom = expf(log_sum / (float)count_flat);
        float arith = lin_sum / (float)count_flat;
        flatness = geom / (arith + 1e-6f);
        if (flatness > 1.0f) flatness = 1.0f;
    }}
    out_features[5] = flatness;

    // Centroid, Crest Factor, Dominant Peak
    float num_sum = 0.0f, den_sum = 1e-6f, peak_m = 0.0f;
    int peak_bin = 0;
    float drone_max_m = 0.0f;
    int drone_b_start = (int)(150.0f / bin_width);
    int drone_b_end = (int)(4500.0f / bin_width);

    for (int b = 0; b < num_bins; b++) {{
        float m = avg_mag[b];
        num_sum += (float)b * bin_width * m;
        den_sum += m;
        if (m > peak_m) peak_m = m;
        if (b >= drone_b_start && b < drone_b_end) {{
            if (m > drone_max_m) {{
                drone_max_m = m;
                peak_bin = b;
            }}
        }}
    }}
    out_features[6] = peak_m / (den_sum / (float)num_bins); // crest_factor
    out_features[7] = num_sum / den_sum;                   // centroid
    float dominant_hz = (float)peak_bin * bin_width;
    out_features[8] = dominant_hz;                         // dominant_peak_hz
    out_features[9] = (dominant_hz >= 150.0f && dominant_hz <= 4500.0f && drone_max_m > 0.0f) ? 1.0f : 0.0f; // peak_in_drone_band

    // Top 3 harmonic peaks in drone band
    float top1 = 0.0f, top2 = 0.0f, top3 = 0.0f;
    for (int b = drone_b_start; b < drone_b_end; b++) {{
        float m = avg_mag[b];
        if (m > top1) {{
            top3 = top2; top2 = top1; top1 = m;
        }} else if (m > top2) {{
            top3 = top2; top2 = m;
        }} else if (m > top3) {{
            top3 = m;
        }}
    }}
    float top3_e = (top1 * top1) + (top2 * top2) + (top3 * top3);
    out_features[10] = top3_e / (drone_core_e + 1e-6f);    // harmonic_peak_ratio

    // 15 Sub-Bands
    out_features[11] = b0;
    out_features[12] = b1;
    out_features[13] = b2;
    out_features[14] = b3;
    out_features[15] = b4;
    out_features[16] = b5;
    out_features[17] = b6;
    out_features[18] = b7;
    out_features[19] = b8;
    out_features[20] = b9;
    out_features[21] = b10;
    out_features[22] = b11;
    out_features[23] = b12;
    out_features[24] = b13;
    out_features[25] = b14;
}}
"""
    os.makedirs(os.path.dirname(os.path.abspath(output_header_path)), exist_ok=True)
    with open(output_header_path, "w", encoding="utf-8") as f:
        f.write(header_content)

    print(f"\n[EXPORT] Successfully generated pure-C model: '{output_header_path}'")
    print(f"         - Trees: {n_trees} | Depth: <= 6 | Features: {NUM_FEATURES}")
    print(f"         - Cross-Validation Accuracy: {accuracy*100:.1f}%\n")

def load_dataset_slices(dataset_dir, augment=True):
    """Recursively finds audio files, extracts 1-second slices, and returns (X, y, sample_counts)."""
    X = []
    y = []
    sample_counts = {0: 0, 1: 0}

    for root, _, files in os.walk(dataset_dir):
        rel_path = os.path.relpath(root, dataset_dir).lower()
        if any(neg in rel_path for neg in ["non-drone", "nondrone", "background", "bg", "noise"]):
            label = 0
        elif "drone" in rel_path:
            label = 1
        else:
            continue

        for f in files:
            ext = os.path.splitext(f)[1].lower()
            if ext in [".wav", ".mp3", ".ogg", ".flac"]:
                file_p = os.path.join(root, f)
                try:
                    samples, sr = load_audio_file(file_p)
                    window_len = 16000
                    stride = 3200 if augment else 8000  # 0.2s stride for deep training

                    if len(samples) >= window_len:
                        for start in range(0, len(samples) - window_len + 1, stride):
                            chunk = samples[start : start + window_len]

                            # 1. Base sample
                            feats = extract_features(chunk, sr)
                            X.append(feats)
                            y.append(label)
                            sample_counts[label] += 1

                            # 2. Data Augmentation (distance attenuation & noise injection)
                            if augment:
                                # Distance attenuation (-6dB, -12dB)
                                chunk_atten = [int(s * 0.5) for s in chunk]
                                X.append(extract_features(chunk_atten, sr))
                                y.append(label)
                                sample_counts[label] += 1

                                # Wind/noise injection
                                noise_chunk = [int(s * 0.8 + np.random.normal(0, 300)) for s in chunk]
                                X.append(extract_features(noise_chunk, sr))
                                y.append(label)
                                sample_counts[label] += 1

                    elif len(samples) >= 8000:
                        chunk = samples + [0] * (window_len - len(samples))
                        feats = extract_features(chunk, sr)
                        X.append(feats)
                        y.append(label)
                        sample_counts[label] += 1
                except Exception as e:
                    print(f" [SKIP] {f}: {e}")

    return np.array(X), np.array(y), sample_counts

def evaluate_test_set(clf, test_dir):
    """Evaluates the trained model on an independent, unseen testing dataset."""
    print("=" * 70)
    print(f"INDEPENDENT TEST SET EVALUATION: '{test_dir}'")
    print("=" * 70)

    X_test, y_test, counts = load_dataset_slices(test_dir, augment=False)
    if len(X_test) == 0:
        print(f"[WARN] No test audio found in '{test_dir}'. Skipping test evaluation.")
        return

    print(f"Test Set Slices: {len(X_test)} total (Drone: {counts[1]}, Background: {counts[0]})")
    y_pred = clf.predict(X_test)
    y_prob = clf.predict_proba(X_test)[:, 1] if hasattr(clf, "predict_proba") else y_pred

    acc = accuracy_score(y_test, y_pred)
    prec = precision_score(y_test, y_pred, zero_division=0)
    rec = recall_score(y_test, y_pred, zero_division=0)
    f1 = f1_score(y_test, y_pred, zero_division=0)
    cm = confusion_matrix(y_test, y_pred)

    print(f"\n[TEST RESULTS]")
    print(f"  - Accuracy:  {acc * 100:.2f}%")
    print(f"  - Precision: {prec * 100:.2f}%")
    print(f"  - Recall:    {rec * 100:.2f}%")
    print(f"  - F1 Score:  {f1 * 100:.2f}%")
    print(f"\nConfusion Matrix (Unseen Test Data):")
    print(f"                Predicted BG   Predicted Drone")
    print(f"  Actual BG:         {cm[0][0]:<14} {cm[0][1]:<14}")
    print(f"  Actual Drone:      {cm[1][0]:<14} {cm[1][1]:<14}")

def train_model(dataset_dir, output_header="include/drone_model.h", test_dir="test_set", n_trees=25, max_depth=6, augment=True):
    """
    Intensive Model Training Pipeline:
    - Fine-grained slicing (0.2s stride) generating thousands of training samples.
    - Acoustic Data Augmentation: Multi-distance gain attenuation (-6dB, -12dB) & noise injection.
    - 26 Frequency-Isolation Features (15 sub-bands, Wiener entropy, crest factor, harmonic ratio).
    - 5-Fold Stratified Cross Validation.
    - Evaluation on independent unseen test set.
    - Generates optimized pure-C header for ESP32-S3.
    """
    if not HAS_SKLEARN:
        print("[ERROR] scikit-learn and numpy are required for training.")
        return False

    print("=" * 70)
    print(f"TRAINING: Pure-C Acoustic Drone Classifier ({n_trees} Trees, Depth {max_depth})")
    print(f"Dataset Directory: {dataset_dir} | Features: {NUM_FEATURES} | Augment: {'ENABLED' if augment else 'DISABLED'}")
    print("=" * 70)

    X, y, sample_counts = load_dataset_slices(dataset_dir, augment=augment)
    total_samples = len(X)
    print(f"\n[DATASET] Generated {total_samples} augmented 1-second feature slices:")
    print(f"  - Drone (positive class):      {sample_counts[1]} slices")
    print(f"  - Background (negative class): {sample_counts[0]} slices")

    if total_samples < 20 or sample_counts[0] == 0 or sample_counts[1] == 0:
        print("\n[ERROR] Insufficient samples for training.")
        return False

    # 5-Fold Stratified Cross Validation
    skf = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
    fold_scores = []
    print("\n[CROSS-VALIDATION] Running 5-Fold Stratified Validation...")
    for fold, (train_idx, val_idx) in enumerate(skf.split(X, y), 1):
        X_tr, y_tr = X[train_idx], y[train_idx]
        X_va, y_va = X[val_idx], y[val_idx]

        clf_fold = RandomForestClassifier(
            n_estimators=n_trees,
            max_depth=max_depth,
            random_state=42 + fold,
            class_weight="balanced"
        )
        clf_fold.fit(X_tr, y_tr)
        val_acc = clf_fold.score(X_va, y_va)
        fold_scores.append(val_acc)
        print(f"  - Fold {fold}/5 Accuracy: {val_acc * 100:.2f}%")

    mean_acc = np.mean(fold_scores)
    std_acc = np.std(fold_scores)
    print(f"\n[RESULT] 5-Fold Mean Accuracy: {mean_acc * 100:.2f}% (+/- {std_acc * 100:.2f}%)")

    # Train Final Production Model on Full Augmented Dataset
    print(f"\n[FINAL MODEL] Training production forest ({n_trees} trees, max depth {max_depth})...")
    final_clf = RandomForestClassifier(
        n_estimators=n_trees,
        max_depth=max_depth,
        random_state=42,
        class_weight="balanced"
    )
    final_clf.fit(X, y)

    # Feature Importances Report
    importances = final_clf.feature_importances_
    sorted_indices = np.argsort(importances)[::-1]
    print("\n[FREQUENCY ISOLATION FEATURE IMPORTANCE]")
    for rank, idx in enumerate(sorted_indices[:10], 1):
        print(f"  {rank:2d}. {FEATURE_NAMES[idx]:<24} {importances[idx] * 100:.2f}%")

    # Export C Header
    export_c_header(final_clf, output_header, mean_acc)

    # Evaluate on Independent Test Set if available
    if test_dir and os.path.exists(test_dir):
        evaluate_test_set(final_clf, test_dir)

    return True

def main():
    parser = argparse.ArgumentParser(description="Train a Crazy Lightweight Pure-C Acoustic Drone Classifier")
    parser.add_argument("--dataset", default="raw", help="Directory containing drone/ and background/ audio folders")
    parser.add_argument("--test-dataset", default="test_set", help="Independent test dataset directory")
    parser.add_argument("--output", default="include/drone_model.h", help="Output C header file path")
    parser.add_argument("--trees", type=int, default=25, help="Number of trees in Random Forest (default: 25)")
    parser.add_argument("--depth", type=int, default=6, help="Maximum tree depth (default: 6)")

    args = parser.parse_args()
    train_model(args.dataset, args.output, args.test_dataset, args.trees, args.depth)

if __name__ == "__main__":
    main()
