#!/usr/bin/env python3
"""
fetch_datasets.py - Downloads real drone audio and non-drone environmental noise
from open-source repositories (Sara Al-Emadi DroneAudioDataset).
Splits into new training data and an independent test set.
"""

import os
import sys
import json
import urllib.request
from concurrent.futures import ThreadPoolExecutor

BASE_GITHUB_API = "https://api.github.com/repos/saraalemadi/DroneAudioDataset/contents/Binary_Drone_Audio"

def fetch_file_list(subfolder):
    url = f"{BASE_GITHUB_API}/{subfolder}"
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req) as resp:
        data = json.loads(resp.read().decode())
    # Return list of (filename, download_url)
    return [(item['name'], item['download_url']) for item in data if item['name'].endswith('.wav')]

def download_file(item_tuple, dest_dir):
    filename, url = item_tuple
    dest_path = os.path.join(dest_dir, filename)
    if os.path.exists(dest_path) and os.path.getsize(dest_path) > 1000:
        return dest_path, True
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req, timeout=15) as resp:
            content = resp.read()
        with open(dest_path, 'wb') as f:
            f.write(content)
        return dest_path, True
    except Exception as e:
        print(f"Error downloading {filename}: {e}")
        return dest_path, False

def main():
    print("=====================================================================")
    print("Fetching Open-Source Drone & Noise Datasets for Training and Testing")
    print("=====================================================================")

    # Directories
    train_drone_dir = "raw/drone/al_emadi"
    train_bg_dir = "raw/background/al_emadi"
    test_drone_dir = "test_set/drone"
    test_bg_dir = "test_set/background"

    for d in [train_drone_dir, train_bg_dir, test_drone_dir, test_bg_dir]:
        os.makedirs(d, exist_ok=True)

    print("[1/3] Querying GitHub for dataset listings...")
    drone_files = fetch_file_list("yes_drone")
    bg_files = fetch_file_list("unknown")
    print(f"  Available Drone files:      {len(drone_files)}")
    print(f"  Available Background files: {len(bg_files)}")

    # We select:
    # 100 drone files for training, 25 for testing
    # 100 bg files for training, 25 for testing
    n_train = 100
    n_test = 25

    drone_train = drone_files[:n_train]
    drone_test = drone_files[n_train:n_train + n_test]

    bg_train = bg_files[:n_train]
    bg_test = bg_files[n_train:n_train + n_test]

    print(f"[2/3] Downloading {len(drone_train)} drone & {len(bg_train)} bg training files...")
    with ThreadPoolExecutor(max_workers=8) as executor:
        for item in drone_train:
            executor.submit(download_file, item, train_drone_dir)
        for item in bg_train:
            executor.submit(download_file, item, train_bg_dir)

    print(f"[3/3] Downloading {len(drone_test)} drone & {len(bg_test)} bg testing files...")
    with ThreadPoolExecutor(max_workers=8) as executor:
        for item in drone_test:
            executor.submit(download_file, item, test_drone_dir)
        for item in bg_test:
            executor.submit(download_file, item, test_bg_dir)

    # Verify counts
    n_dt = len([f for f in os.listdir(train_drone_dir) if f.endswith('.wav')])
    n_bt = len([f for f in os.listdir(train_bg_dir) if f.endswith('.wav')])
    n_dte = len([f for f in os.listdir(test_drone_dir) if f.endswith('.wav')])
    n_bte = len([f for f in os.listdir(test_bg_dir) if f.endswith('.wav')])

    print("\nDataset Download Complete:")
    print(f"  - New Training Drone:      {n_dt} files in '{train_drone_dir}'")
    print(f"  - New Training Background: {n_bt} files in '{train_bg_dir}'")
    print(f"  - Independent Test Drone:  {n_dte} files in '{test_drone_dir}'")
    print(f"  - Independent Test BG:     {n_bte} files in '{test_bg_dir}'")

if __name__ == "__main__":
    main()
