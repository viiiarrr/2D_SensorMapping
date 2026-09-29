import numpy as np
import csv
import sys
import os
import matplotlib.pyplot as plt

# Parameter dari visualisasi
MIN_COUNT = 4
NUM_SENSOR = 8
SENSOR_STEP = 45.0
EMA_ALPHA = 0.25
MAX_DIST = 250.0
MIN_DIST = 2.0
OUTLIER_SIGMA = 2.0
OUTLIER_WINDOW = 10

def fit_circle(pts):
    x = pts[:, 0]
    y = pts[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    b = -(x**2 + y**2)
    res, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    D, E, F = res
    cx = -D / 2
    cy = -E / 2
    r = np.sqrt(max(0, cx**2 + cy**2 - F))
    return cx, cy, r

def analyze_csv(csv_path):
    stable_map = np.zeros(360)
    count_map = np.zeros(360, dtype=int)
    hist_map = [[] for _ in range(360)]

    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 10: continue
            yaw_raw = float(row[1])
            distances = [float(x) for x in row[2:10]]

            for i in range(NUM_SENSOR):
                dist = distances[i]
                if dist < MIN_DIST or dist > MAX_DIST: continue
                physical_deg = (-yaw_raw + i * SENSOR_STEP) % 360
                idx = int(physical_deg) % 360
                
                hist = hist_map[idx]
                if len(hist) >= OUTLIER_WINDOW:
                    med = np.median(hist)
                    std = np.std(hist)
                    if std > 0 and abs(dist - med) > OUTLIER_SIGMA * std:
                        continue
                hist.append(dist)
                if len(hist) > OUTLIER_WINDOW:
                    hist.pop(0)

                if stable_map[idx] == 0:
                    stable_map[idx] = dist
                else:
                    stable_map[idx] = (1 - EMA_ALPHA) * stable_map[idx] + EMA_ALPHA * dist
                count_map[idx] += 1

    sx, sy = [], []
    for i in range(360):
        if stable_map[i] > 0 and count_map[i] >= MIN_COUNT:
            rad = np.radians(i)
            sx.append(stable_map[i] * np.cos(rad))
            sy.append(stable_map[i] * np.sin(rad))

    pts = np.column_stack([sx, sy])
    print(f"Total valid points: {len(pts)}")
    
    # Fit circle
    cx, cy, r = fit_circle(pts)
    print(f"Fitted Circle: Center=({cx:.2f}, {cy:.2f}), R={r:.2f}")

    # Calculate distance to circle
    dists = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
    
    # Sort distances
    sorted_idx = np.argsort(dists)[::-1]
    print("\nTop 10 Outliers (Distance from ideal shape):")
    for i in range(10):
        idx = sorted_idx[i]
        print(f"Point {idx}: ({pts[idx][0]:.2f}, {pts[idx][1]:.2f}) - Dist: {dists[idx]:.2f} cm")
        
    # Also find isolated points (density based)
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    lonely = []
    for i in range(len(pts)):
        neighbors = len(tree.query_ball_point(pts[i], 20.0)) - 1
        if neighbors < 3:
            lonely.append(i)
            
    print(f"\nIsolated points (Density radius=20, min_pts=3): {len(lonely)}")
    for idx in lonely:
        print(f"Point {idx}: ({pts[idx][0]:.2f}, {pts[idx][1]:.2f})")

if __name__ == "__main__":
    analyze_csv("e:\\code_skripsi\\TugasAkhir\\Data\\percobaan_63\\koordinat.csv")
