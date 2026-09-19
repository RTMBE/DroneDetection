#!/usr/bin/env python3
"""
drone_audio_dataset.py - Dataset Preprocessing Pipeline for TinyML Drone Audio Classifier
Part of ESP32-S3 Acoustic Drone Detection Pipeline

Features:
1. Video to Audio Extraction: Converts MP4/MKV/AVI/MOV to mono 16 kHz 16-bit PCM WAV using FFmpeg.
2. Automated 1-Second Slicing: Slices raw recordings into 1.0s (16,000-sample) blocks for Edge Impulse.
3. Overlapping Window Augmentation: Optional stride (e.g. 0.5s for 50% overlap) to double training data.
4. Train/Test Partitioning: Automatically organizes files into Edge Impulse ready directory structure.
5. Dataset Validator/Audit: Verifies sample rate, bit depth, channel count, and duration.
"""

import os
import sys
import argparse
import subprocess
import wave
import glob
import random
import shutil

TARGET_SAMPLE_RATE = 16000
TARGET_CHANNELS = 1
TARGET_SAMPLE_WIDTH = 2 # 16-bit PCM (2 bytes)
TARGET_SLICE_DURATION_SEC = 1.0

def extract_audio(input_video, output_wav, ffmpeg_bin="ffmpeg"):
    """
    Extracts and normalizes audio from video file to mono 16 kHz 16-bit PCM WAV.
    ffmpeg -i input_video.mp4 -vn -acodec pcm_s16le -ar 16000 -ac 1 drone_sample.wav -y
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_wav)), exist_ok=True)
    cmd = [
        ffmpeg_bin,
        "-i", input_video,
        "-vn",
        "-acodec", "pcm_s16le",
        "-ar", str(TARGET_SAMPLE_RATE),
        "-ac", str(TARGET_CHANNELS),
        output_wav,
        "-y"
    ]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        print(f"[EXTRACT] Converted '{input_video}' -> '{output_wav}' (16kHz Mono 16-bit PCM)")
        return True
    except FileNotFoundError:
        print(f"[ERROR] '{ffmpeg_bin}' executable not found. Please ensure FFmpeg is installed and in PATH.")
        return False
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] FFmpeg failed with error:\n{e.stderr.decode('utf-8', errors='ignore')}")
        return False

def slice_wav(input_file, output_folder, prefix, slice_len_sec=1.0, stride_sec=None):
    """
    Slices raw .wav recording into 1-second training blocks for Edge Impulse.
    Optional stride_sec enables sliding window data augmentation.
    """
    os.makedirs(output_folder, exist_ok=True)
    with wave.open(input_file, 'rb') as w:
        n_channels = w.getnchannels()
        sampwidth = w.getsampwidth()
        framerate = w.getframerate()
        n_frames = w.getnframes()

        if framerate != TARGET_SAMPLE_RATE or n_channels != TARGET_CHANNELS or sampwidth != TARGET_SAMPLE_WIDTH:
            print(f"[WARNING] '{input_file}' format ({framerate}Hz, {n_channels}ch, {sampwidth*8}-bit) differs "
                  f"from target ({TARGET_SAMPLE_RATE}Hz, {TARGET_CHANNELS}ch, {TARGET_SAMPLE_WIDTH*8}-bit).")

        frames_per_slice = int(framerate * slice_len_sec)
        stride_frames = int(framerate * stride_sec) if stride_sec else frames_per_slice

        if n_frames < frames_per_slice:
            print(f"[SKIP] '{input_file}' duration ({n_frames/framerate:.2f}s) is shorter than slice length ({slice_len_sec}s).")
            return 0

        raw_frames = w.readframes(n_frames)
        bytes_per_frame = n_channels * sampwidth

        slice_count = 0
        start_frame = 0
        while (start_frame + frames_per_slice) <= n_frames:
            start_byte = start_frame * bytes_per_frame
            end_byte = (start_frame + frames_per_slice) * bytes_per_frame
            slice_data = raw_frames[start_byte:end_byte]

            out_name = os.path.join(output_folder, f"{prefix}_{slice_count:04d}.wav")
            with wave.open(out_name, 'wb') as out_w:
                out_w.setnchannels(n_channels)
                out_w.setsampwidth(sampwidth)
                out_w.setframerate(framerate)
                out_w.writeframes(slice_data)

            slice_count += 1
            start_frame += stride_frames

    print(f"[SLICE] Exported {slice_count} slices ({slice_len_sec}s) to '{output_folder}'")
    return slice_count

def audit_dataset(dataset_dir):
    """
    Audits an Edge Impulse dataset directory to verify all samples adhere to:
    - 16000 Hz sample rate
    - 1 channel (mono)
    - 16-bit signed PCM
    - Exactly 16000 frames (1.00s)
    """
    print("=" * 60)
    print(f"Auditing TinyML Audio Dataset in: {dataset_dir}")
    print("=" * 60)

    classes = {}
    valid_count = 0
    issue_count = 0

    for root, _, files in os.walk(dataset_dir):
        for file in files:
            if file.lower().endswith(".wav"):
                full_path = os.path.join(root, file)
                rel_class = os.path.basename(root)
                classes[rel_class] = classes.get(rel_class, 0) + 1

                try:
                    with wave.open(full_path, "rb") as w:
                        rate = w.getframerate()
                        channels = w.getnchannels()
                        width = w.getsampwidth()
                        frames = w.getnframes()
                        duration = frames / rate

                        issues = []
                        if rate != TARGET_SAMPLE_RATE:
                            issues.append(f"Rate: {rate}Hz != {TARGET_SAMPLE_RATE}Hz")
                        if channels != TARGET_CHANNELS:
                            issues.append(f"Channels: {channels} != {TARGET_CHANNELS}")
                        if width != TARGET_SAMPLE_WIDTH:
                            issues.append(f"Width: {width*8}bit != {TARGET_SAMPLE_WIDTH*8}bit")
                        if abs(duration - TARGET_SLICE_DURATION_SEC) > 0.05:
                            issues.append(f"Duration: {duration:.2f}s != {TARGET_SLICE_DURATION_SEC}s")

                        if issues:
                            print(f" [FAIL] {file}: {', '.join(issues)}")
                            issue_count += 1
                        else:
                            valid_count += 1
                except Exception as e:
                    print(f" [ERROR] {file}: {e}")
                    issue_count += 1

    print("\nDataset Class Distribution:")
    for cls_name, count in sorted(classes.items()):
        print(f"  - {cls_name}: {count} samples")

    print(f"\nAudit Result: {valid_count} valid slices, {issue_count} issues.")
    print("=" * 60)
    return issue_count == 0 and valid_count > 0

def prepare_pipeline(input_drone_dir, input_bg_dir, output_dir, train_ratio=0.8, stride_sec=0.5):
    """
    Processes drone and background directories into Edge Impulse training and testing partitions.
    """
    train_drone_dir = os.path.join(output_dir, "training", "drone")
    test_drone_dir = os.path.join(output_dir, "testing", "drone")
    train_bg_dir = os.path.join(output_dir, "training", "background")
    test_bg_dir = os.path.join(output_dir, "testing", "background")

    for d in [train_drone_dir, test_drone_dir, train_bg_dir, test_bg_dir]:
        os.makedirs(d, exist_ok=True)

    def process_class(src_dir, train_dst, test_dst, label):
        if not os.path.exists(src_dir):
            print(f"[SKIP] Source directory '{src_dir}' not found.")
            return

        temp_slice_dir = os.path.join(output_dir, f"_temp_{label}")
        os.makedirs(temp_slice_dir, exist_ok=True)

        idx = 0
        for f in os.listdir(src_dir):
            ext = os.path.splitext(f)[1].lower()
            full_p = os.path.join(src_dir, f)
            if ext in [".mp4", ".mkv", ".avi", ".mov"]:
                wav_p = os.path.join(temp_slice_dir, f"{label}_src_{idx:03d}.wav")
                if extract_audio(full_p, wav_p):
                    slice_wav(wav_p, temp_slice_dir, f"{label}_{idx:03d}", stride_sec=stride_sec)
                    os.remove(wav_p)
                idx += 1
            elif ext == ".wav":
                slice_wav(full_p, temp_slice_dir, f"{label}_{idx:03d}", stride_sec=stride_sec)
                idx += 1

        all_slices = glob.glob(os.path.join(temp_slice_dir, "*.wav"))
        random.shuffle(all_slices)
        split_idx = int(len(all_slices) * train_ratio)

        for s in all_slices[:split_idx]:
            shutil.move(s, os.path.join(train_dst, os.path.basename(s)))
        for s in all_slices[split_idx:]:
            shutil.move(s, os.path.join(test_dst, os.path.basename(s)))

        shutil.rmtree(temp_slice_dir, ignore_errors=True)
        print(f"[PARTITION] {label}: {split_idx} train samples, {len(all_slices) - split_idx} test samples.")

    print(f"\nProcessing Drone Audio Data -> {output_dir}")
    process_class(input_drone_dir, train_drone_dir, test_drone_dir, "drone")
    print(f"\nProcessing Background Audio Data -> {output_dir}")
    process_class(input_bg_dir, train_bg_dir, test_bg_dir, "bg")

    audit_dataset(output_dir)

def main():
    parser = argparse.ArgumentParser(description="Dataset Preprocessing Pipeline for TinyML Drone Acoustic Classifier")
    subparsers = parser.add_subparsers(dest="command", help="Sub-commands")

    # Command: extract
    p_extract = subparsers.add_parser("extract", help="Extract and normalize audio from video file to 16kHz mono WAV")
    p_extract.add_argument("video", help="Input video file path")
    p_extract.add_argument("output", help="Output .wav file path")
    p_extract.add_argument("--ffmpeg", default="ffmpeg", help="Path to FFmpeg binary")

    # Command: slice
    p_slice = subparsers.add_parser("slice", help="Slice WAV file into 1-second training blocks")
    p_slice.add_argument("input", help="Input .wav file path")
    p_slice.add_argument("output", help="Output directory for slices")
    p_slice.add_argument("--prefix", default="sample", help="File prefix for slices")
    p_slice.add_argument("--duration", type=float, default=1.0, help="Slice duration in seconds (default: 1.0)")
    p_slice.add_argument("--stride", type=float, default=None, help="Stride duration in seconds for overlapping slices")

    # Command: pipeline
    p_pipe = subparsers.add_parser("pipeline", help="Automated batch processing and train/test partition for Edge Impulse")
    p_pipe.add_argument("--drone-dir", required=True, help="Directory containing drone raw video/WAV files")
    p_pipe.add_argument("--bg-dir", required=True, help="Directory containing background noise video/WAV files")
    p_pipe.add_argument("--output-dir", required=True, help="Target directory for structured dataset")
    p_pipe.add_argument("--train-ratio", type=float, default=0.8, help="Train/test ratio (default: 0.8)")
    p_pipe.add_argument("--stride", type=float, default=0.5, help="Sliding window stride in seconds (default: 0.5)")

    # Command: audit
    p_audit = subparsers.add_parser("audit", help="Audit dataset directory against Edge Impulse requirements")
    p_audit.add_argument("dataset_dir", help="Path to dataset directory to audit")

    args = parser.parse_args()

    if args.command == "extract":
        extract_audio(args.video, args.output, args.ffmpeg)
    elif args.command == "slice":
        slice_wav(args.input, args.output, args.prefix, args.duration, args.stride)
    elif args.command == "pipeline":
        prepare_pipeline(args.drone_dir, args.bg_dir, args.output_dir, args.train_ratio, args.stride)
    elif args.command == "audit":
        audit_dataset(args.dataset_dir)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
