import pandas as pd
import numpy as np
import os
import argparse
import math

# ==========================================
# PARAMETER OTOMATISASI GROUND TRUTH
# ==========================================
# Mata manusia biasanya lebih jeli (menganggap meleset 3.9 cm sebagai anomali),
# sedangkan algoritma NWA lebih toleran (baru membuang di atas 5.0 cm).
GROUND_TRUTH_THR = 3.9  # cm (Batas sebuah titik dianggap "Titik Semu Asli" secara matematis)

def _fit_circle(pts):
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

def _ransac_line(pts, iter=150, thr=5.0):
    if len(pts) < 2: return None, None
    xs, ys = pts[:, 0], pts[:, 1]
    best_inliers = None
    best_count = 0
    rng = np.random.default_rng()
    
    for _ in range(iter):
        i, j = rng.choice(len(pts), 2, replace=False)
        x1, y1 = xs[i], ys[i]
        x2, y2 = xs[j], ys[j]
        dx, dy = x2 - x1, y2 - y1
        if abs(dx) < 1e-9 and abs(dy) < 1e-9: continue
        
        a, b, c = dy, -dx, dx*y1 - dy*x1
        norm = np.hypot(a, b)
        dists = np.abs(a*xs + b*ys + c) / norm
        inliers = dists < thr
        cnt = inliers.sum()
        if cnt > best_count:
            best_count = cnt
            best_inliers = inliers
            
    if best_inliers is None or best_count < 2:
        return None, None
        
    pts_in = pts[best_inliers]
    cx, cy = pts_in[:, 0].mean(), pts_in[:, 1].mean()
    M = np.column_stack([pts_in[:, 0] - cx, pts_in[:, 1] - cy])
    _, _, Vt = np.linalg.svd(M)
    dx, dy = Vt[0]
    a, b, c = dy, -dx, -(dy*cx - dx*cy)
    norm = np.hypot(a, b)
    return (a/norm, b/norm, c/norm), best_inliers

def calculate_distances_to_shape(X, Y):
    pts = np.column_stack([X, Y])
    
    # 1. Test Circle
    cx, cy, r = _fit_circle(pts)
    dists_circle = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
    circle_inliers = np.sum(dists_circle < 5.0)
    
    # 2. Test Walls (up to 4 walls)
    remaining = pts.copy()
    wall_lines = []
    wall_inliers_count = 0
    
    for _ in range(4):
        if len(remaining) < 15: break
        line, inlier_mask = _ransac_line(remaining)
        if line is None: break
        wall_lines.append(line)
        wall_inliers_count += inlier_mask.sum()
        remaining = remaining[~inlier_mask]
        
    # Pilih bentuk terbaik
    if circle_inliers > wall_inliers_count:
        shape = f"Bulat (R={r:.1f})"
        return dists_circle, shape
    else:
        shape = f"Garis/Persegi/Segitiga ({len(wall_lines)} dinding)"
        dists_walls = np.full(len(pts), np.inf)
        for a, b, c in wall_lines:
            d = np.abs(a * pts[:, 0] + b * pts[:, 1] + c)
            dists_walls = np.minimum(dists_walls, d)
        return dists_walls, shape

def auto_analyze(csv_path):
    if not os.path.exists(csv_path):
        print(f"[!] File {csv_path} tidak ditemukan.")
        return

    df = pd.read_csv(csv_path)
    X = df['X'].values
    Y = df['Y'].values
    
    # ---------------------------------------------------------
    # 1. CARI BENTUK ASLI & JARAK MATEMATIS
    # ---------------------------------------------------------
    dists, shape_name = calculate_distances_to_shape(X, Y)
    
    # Titik yang jaraknya > 3.9 cm dari bentuk asli = Anomali Asli
    anomali_asli = (dists > GROUND_TRUTH_THR).astype(int)
    df['Anomali_Asli_Auto'] = anomali_asli
    
    # ---------------------------------------------------------
    # 2. EVALUASI HASIL SKRIPSI
    # ---------------------------------------------------------
    y_true = df['Anomali_Asli_Auto'].values
    y_pred = pd.to_numeric(df['NWA_Phantom_Terdeteksi'], errors='coerce').fillna(0).astype(int).values

    # A. Total Titik Semu Terdeteksi (Ground Truth)
    A = sum(y_true)
    
    # B. Jumlah Titik Semu Terbuang (NWA berhasil membuang anomali asli)
    B = sum((y_true == 1) & (y_pred == 1))
    
    salah_buang = sum((y_true == 0) & (y_pred == 1))

    tingkat_keberhasilan = (B / A) * 100 if A > 0 else 0.0

    print("==================================================")
    print("      HASIL PERHITUNGAN SESUAI RUMUS SKRIPSI      ")
    print("           (Auto-Detector Matematika AI)          ")
    print("==================================================")
    print(f"Bentuk Ruangan Terdeteksi       : {shape_name}")
    print(f"Batas Toleransi Ground Truth    : {GROUND_TRUTH_THR} cm")
    print("--------------------------------------------------")
    print(f"Total Titik Keseluruhan         : {len(df)}")
    print(f"Total Titik Semu Terdeteksi (A) : {A}")
    print(f"Jumlah Titik Semu Terbuang  (B) : {B}")
    print("--------------------------------------------------")
    
    print(f"Tingkat Keberhasilan (%)        = (B / A) * 100%")
    if A > 0:
        print(f"                                = ({B} / {A}) * 100%")
    print(f"                                = {tingkat_keberhasilan:.2f} %\n")
    
    print("Tambahan Info Evaluasi Algoritma:")
    print(f"- Anomali yang GAGAL dibuang (False Negative)           : {A - B}")
    print(f"- Titik Valid yang SALAH dibuang NWA (False Positive)   : {salah_buang}")
    print("==================================================")
    
    out_path = csv_path.replace("evaluasi_nwa.csv", "evaluasi_nwa_ai.csv")
    df.to_csv(out_path, index=False)
    print(f"\n[+] Label ground truth otomatis telah disimpan ke: {out_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Auto Detector NWA")
    parser.add_argument("--csv", type=str, required=True, help="Path ke evaluasi_nwa.csv")
    args = parser.parse_args()
    auto_analyze(args.csv)
