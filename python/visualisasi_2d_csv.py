"""
Visualisasi 2D Real-Time — Menggunakan sudut IMU (Yaw) dari ESP32

Koreksi logika sudut:
  - CW = yaw NEGATIF  → physical_angle = (-yaw) % 360
  - CCW = yaw POSITIF → physical_angle = (-yaw) % 360  (SAMA!)
  - Formula ini berlaku untuk kedua arah tanpa perlu reset yaw

Stabilitas peta:
  - Peta disimpan per derajat (0-359) dengan EMA
  - Scatter & boundary digambar DARI stable_map (bukan raw points)
  - Obstacle tidak bergerak karena tiap derajat dirata-rata

Wall Fitting & Phantom Point Detection (kontribusi skripsi):
  - Split-and-Merge membagi point cloud menjadi segmen-segmen dinding
  - RANSAC fitting per segmen → garis nominal wall (merah)
  - Titik yang jauh dari garis = phantom point (oranye)
"""

import socket, threading, numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mc
from matplotlib.animation import FuncAnimation
import os, csv, datetime

# =====================
# KONFIGURASI UMUM
# =====================
UDP_IP        = "0.0.0.0"
UDP_PORT      = 5005
NUM_SENSOR    = 8
SENSOR_STEP   = 45.0    # Jarak antar sensor (derajat)
EMA_ALPHA     = 0.25    # Bobot data baru (lebih kecil = lebih stabil)
MAX_DIST      = 250.0   # Filter jarak maksimum (cm)
MIN_DIST      = 2.0     # Filter jarak minimum (cm)

# Filter outlier per sudut
OUTLIER_SIGMA  = 2.0    # Toleransi deviasi standar
OUTLIER_WINDOW = 10     # Jumlah sampel histori per sudut

MIN_COUNT = 4           # Sudut harus diukur minimal N kali agar ditampilkan

# =====================
# KONFIGURASI WALL & CIRCLE FITTING & NWA PHANTOM DETECTION
# =====================
N_WALLS           = 4      # Jumlah dinding maksimum (4 = persegi/kotak)

# RANSAC
RANSAC_ITER       = 150   # Jumlah iterasi RANSAC
RANSAC_INLIER_THR = 8.0   # cm — jarak titik ke garis/lingkaran agar dianggap inlier
MIN_SEGMENT_PTS   = 30    # Jumlah titik minimum per dinding (dinaikkan agar noise tidak jadi dinding)

# NWA threshold
PHANTOM_DIST_THR  = 10.0  # cm — jika jarak ke model terdekat > ini → phantom

# Filter densitas — buang titik terpencil sebelum RANSAC
DENSITY_RADIUS    = 20.0  # cm — radius pencarian tetangga
MIN_NEIGHBORS     = 5     # minimal tetangga dalam radius → kurang dari ini = lonely = phantom

# =====================
# KONFIGURASI CSV
# =====================
CSV_FILE_PATH = r"e:\code_skripsi\TugasAkhir\Data\percobaan_67\koordinat.csv"

# 47 46 persegi
# 63 64 bulat
# 65 66 persegi panjang 
# 67 57 segitiga

# ──────────────────────────────────────────────────────────────
# FUNGSI BANTU GEOMETRI
# ──────────────────────────────────────────────────────────────

def _point_to_line_dist(px, py, x1, y1, x2, y2):
    """Jarak tegak lurus titik (px,py) ke garis melalui (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    len2   = dx*dx + dy*dy
    if len2 == 0:
        return np.hypot(px - x1, py - y1)
    t = ((px - x1)*dx + (py - y1)*dy) / len2
    return np.hypot(px - (x1 + t*dx), py - (y1 + t*dy))


def _ransac_line(pts):
    """
    Fit garis terbaik dari sekumpulan titik menggunakan RANSAC.
    Kembalikan (a, b, c) koefisien garis  ax + by + c = 0  (ternormalisasi, c > 0)
    dan mask inlier (boolean array).
    KONVENSI: c > 0 → normal (a,b) mengarah ke origin (ke dalam ruangan).
    Jika gagal, kembalikan None, None.
    """
    if len(pts) < 2:
        return None, None

    xs, ys = pts[:, 0], pts[:, 1]
    best_inliers = None
    best_count   = 0
    rng = np.random.default_rng()

    for _ in range(RANSAC_ITER):
        i, j = rng.choice(len(pts), 2, replace=False)
        x1, y1 = xs[i], ys[i]
        x2, y2 = xs[j], ys[j]
        dx, dy = x2 - x1, y2 - y1
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            continue
        a, b, c = dy, -dx, dx*y1 - dy*x1
        norm = np.hypot(a, b)
        dists = np.abs(a*xs + b*ys + c) / norm
        inliers = dists < RANSAC_INLIER_THR
        cnt = inliers.sum()
        if cnt > best_count:
            best_count   = cnt
            best_inliers = inliers

    if best_inliers is None or best_count < 2:
        return None, None

    # Re-fit dengan semua inlier via SVD (lebih akurat)
    pts_in = pts[best_inliers]
    cx, cy = pts_in[:, 0].mean(), pts_in[:, 1].mean()
    M = np.column_stack([pts_in[:, 0] - cx, pts_in[:, 1] - cy])
    _, _, Vt = np.linalg.svd(M)
    dx, dy = Vt[0]
    a, b, c = dy, -dx, -(dy*cx - dx*cy)
    norm_ab = np.hypot(a, b)
    if norm_ab == 0:
        return None, None
    a, b, c = a/norm_ab, b/norm_ab, c/norm_ab

    # NORMALISASI PENTING: pastikan c > 0
    # c > 0 berarti origin berada di sisi positif garis → normal (a,b) mengarah KE DALAM (ke origin)
    if c < 0:
        a, b, c = -a, -b, -c

    return (a, b, c), best_inliers


def _split_and_merge(pts_idx, points, depth=0):
    """
    Rekursif Split-and-Merge.
    pts_idx : list/array indeks ke array `points`.
    Kembalikan list segmen (tiap segmen = array indeks).
    """
    if len(pts_idx) < 2:
        return [np.array(pts_idx)] if len(pts_idx) >= MIN_SEGMENT_PTS else []

    idx = np.array(pts_idx)
    seg = points[idx]
    x1, y1 = seg[0]
    x2, y2 = seg[-1]

    dists = np.array([_point_to_line_dist(p[0], p[1], x1, y1, x2, y2)
                      for p in seg])
    max_d = dists.max()
    max_i = dists.argmax()

    if max_d > SPLIT_THRESHOLD and depth < 12:
        left  = _split_and_merge(idx[:max_i+1].tolist(), points, depth+1)
        right = _split_and_merge(idx[max_i:].tolist(),   points, depth+1)
        return left + right
    else:
        return [idx] if len(idx) >= MIN_SEGMENT_PTS else []


def _fit_circle_lstsq(xs, ys):
    x = np.array(xs)
    y = np.array(ys)
    M = np.column_stack([x, y, np.ones(len(x))])
    rhs = -(x**2 + y**2)
    res, _, _, _ = np.linalg.lstsq(M, rhs, rcond=None)
    A, B, C = res
    xc = -A / 2
    yc = -B / 2
    r = np.sqrt(xc**2 + yc**2 - C)
    return xc, yc, r

def _ransac_circle(pts):
    if len(pts) < 3:
        return None, None
    xs = pts[:, 0]
    ys = pts[:, 1]
    
    best_inliers = None
    best_count = 0
    rng = np.random.default_rng()
    
    for _ in range(RANSAC_ITER):
        idx = rng.choice(len(pts), 3, replace=False)
        p1, p2, p3 = pts[idx]
        
        temp1 = p2 - p1
        temp2 = p3 - p1
        
        det = temp1[0]*temp2[1] - temp1[1]*temp2[0]
        if abs(det) < 1e-6:
            continue
            
        c1 = (temp1[0]**2 + temp1[1]**2) / 2
        c2 = (temp2[0]**2 + temp2[1]**2) / 2
        
        xc_rel = (c1*temp2[1] - c2*temp1[1]) / det
        yc_rel = (temp1[0]*c2 - temp2[0]*c1) / det
        
        xc = xc_rel + p1[0]
        yc = yc_rel + p1[1]
        r = np.hypot(xc_rel, yc_rel)
        
        dists = np.abs(np.hypot(xs - xc, ys - yc) - r)
        inliers = dists < RANSAC_INLIER_THR
        cnt = inliers.sum()
        
        if cnt > best_count:
            best_count = cnt
            best_inliers = inliers

    if best_inliers is None or best_count < 3:
        return None, None

    pts_in = pts[best_inliers]
    if len(pts_in) < 3:
        return None, None
        
    xc, yc, r = _fit_circle_lstsq(pts_in[:, 0], pts_in[:, 1])
    dists = np.abs(np.hypot(xs - xc, ys - yc) - r)
    best_inliers = dists < RANSAC_INLIER_THR
    
    return (xc, yc, r), best_inliers

def _detect_best_shape_and_phantoms(sx, sy, counts):
    """
    1. Filter titik terpencil & tidak stabil (count < MIN_COUNT).
    2. Coba RANSAC Circle.
    3. Coba Sequential RANSAC Lines (N_WALLS).
    4. Bandingkan inliers. Pilih model terbaik.
    """
    n = len(sx)
    if n < MIN_SEGMENT_PTS * 2:
        return 'none', None, np.ones(n, bool), np.zeros(n, bool)

    sx_np = np.array(sx, dtype=float)
    sy_np = np.array(sy, dtype=float)
    pts   = np.column_stack([sx_np, sy_np])
    counts_np = np.array(counts, dtype=int)

    is_unstable = counts_np < MIN_COUNT

    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    neighbor_counts = np.array([
        len(tree.query_ball_point(pts[i], DENSITY_RADIUS)) - 1
        for i in range(n)
    ])
    is_lonely = neighbor_counts < MIN_NEIGHBORS
    
    is_phantom_base = is_unstable | is_lonely
    dense_pts = pts[~is_phantom_base]

    if len(dense_pts) < MIN_SEGMENT_PTS:
        return 'none', None, ~is_phantom_base, is_phantom_base

    # --- Test Lingkaran ---
    circle_model, circle_inlier_mask = _ransac_circle(dense_pts)
    circle_inlier_count = circle_inlier_mask.sum() if circle_inlier_mask is not None else 0

    # --- Test Dinding (Garis) ---
    remaining = dense_pts.copy()
    candidates = []
    wall_inlier_count = 0

    for _ in range(N_WALLS + 3):
        if len(remaining) < MIN_SEGMENT_PTS:
            break
        line, inlier_mask = _ransac_line(remaining)
        if line is None:
            break
        inlier_pts  = remaining[inlier_mask]
        remaining = remaining[~inlier_mask]
        if len(inlier_pts) >= MIN_SEGMENT_PTS:
            candidates.append((len(inlier_pts), line))
            wall_inlier_count += len(inlier_pts)

    candidates.sort(key=lambda x: x[0], reverse=True)
    valid_candidates = []
    if candidates:
        valid_candidates.append(candidates[0])
        for i in range(1, len(candidates)):
            if candidates[i][0] < candidates[i-1][0] * 0.5:
                break
            valid_candidates.append(candidates[i])
            
    wall_lines = [c[1] for c in valid_candidates[:N_WALLS]]

    # 1. Hitung Segment Dinding (Polygon) DULU
    wall_seg_data = []
    polygon_formed = False
    
    if len(wall_lines) >= 3:
        std_lines = []
        for a, b, c in wall_lines:
            if c < 0:
                std_lines.append((-a, -b, -c))
            else:
                std_lines.append((a, b, c))
        import math
        std_lines.sort(key=lambda l: math.atan2(l[1], l[0]))
        
        corners = []
        for i in range(len(std_lines)):
            a1, b1, c1 = std_lines[i]
            a2, b2, c2 = std_lines[(i+1)%len(std_lines)]
            det = a1*b2 - a2*b1
            if abs(det) > 1e-6:
                x = (b1*c2 - b2*c1)/det
                y = (a2*c1 - a1*c2)/det
                corners.append((x, y))
            else:
                corners.append(None)
        
        if all(c is not None for c in corners):
            for i in range(len(corners)):
                x1, y1 = corners[i]
                x2, y2 = corners[(i+1)%len(corners)]
                wall_seg_data.append((x1, y1, x2, y2))
            polygon_formed = True

    if not polygon_formed:
        for count, line in valid_candidates[:N_WALLS]:
            a, b, c = line
            t_min = float('inf')
            t_max = float('-inf')
            for i in range(len(dense_pts)):
                px, py = dense_pts[i]
                if abs(a * px + b * py + c) <= RANSAC_INLIER_THR:
                    t = -b * px + a * py
                    if t < t_min: t_min = t
                    if t > t_max: t_max = t
            if t_min != float('inf') and t_max != float('-inf'):
                x1 = -b * t_min - a * c
                y1 =  a * t_min - b * c
                x2 = -b * t_max - a * c
                y2 =  a * t_max - b * c
                wall_seg_data.append((x1, y1, x2, y2))

    # --- Pilih Model Terbaik ---
    # Hitung Phantom Points untuk Lingkaran
    circle_phantom_count = n
    if circle_model is not None:
        xc, yc, r = circle_model
        dists_c = np.abs(np.hypot(sx_np - xc, sy_np - yc) - r)
        circle_phantom_mask = (dists_c > PHANTOM_DIST_THR) | is_phantom_base
        circle_phantom_count = circle_phantom_mask.sum()
        
    # Hitung Phantom Points untuk Dinding (dengan batas segmen polygon)
    wall_phantom_mask = is_phantom_base.copy()
    if wall_lines:
        for gi in range(n):
            if wall_phantom_mask[gi]:
                continue
            px, py = sx_np[gi], sy_np[gi]
            
            valid_in_segment = False
            for w in range(len(wall_lines)):
                a, b, c = wall_lines[w]
                if abs(a * px + b * py + c) <= PHANTOM_DIST_THR:
                    if polygon_formed and w < len(wall_seg_data):
                        x1, y1, x2, y2 = wall_seg_data[w]
                        dx, dy = x2 - x1, y2 - y1
                        L2 = dx*dx + dy*dy
                        if L2 > 1e-6:
                            t = ((px - x1)*dx + (py - y1)*dy) / L2
                            if -0.1 <= t <= 1.1:
                                valid_in_segment = True
                    else:
                        valid_in_segment = True
            
            if not valid_in_segment:
                wall_phantom_mask[gi] = True
                
    wall_phantom_count = wall_phantom_mask.sum()

    if SHAPE_MODE == 'circle' and circle_model is not None:
        best_shape = 'circle'
    elif SHAPE_MODE == 'walls':
        best_shape = 'walls'
    else:
        # AUTO MODE
        if circle_model is not None and circle_phantom_count <= wall_phantom_count:
            best_shape = 'circle'
        else:
            best_shape = 'walls'
            
    if best_shape == 'circle' and circle_model is not None:
        return 'circle', circle_model, ~circle_phantom_mask, circle_phantom_mask
    else:
        return 'walls', wall_seg_data, ~wall_phantom_mask, wall_phantom_mask
