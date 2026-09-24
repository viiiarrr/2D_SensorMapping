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
CSV_FILE_PATH = r"e:\code_skripsi\TugasAkhir\Data\percobaan_66\koordinat.csv"

# 47 46 persegi
# 63 64 bulat
# 65 66 persegi panjang
# 67 57 segitiga

# ──────────────────────────────────────────────────────────────
# FUNGSI BANTU GEOMETRI
# ──────────────────────────────────────────────────────────────

def _point_to_line_dist(px, py, x1, y1, x2, y2):
    """Jarak titik (px,py) ke segmen garis (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    len2   = dx*dx + dy*dy
    if len2 == 0:
        return np.hypot(px - x1, py - y1)
    t = ((px - x1)*dx + (py - y1)*dy) / len2
    t = max(0.0, min(1.0, t))
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

def _detect_best_shape_and_phantoms(sx, sy):
    """
    1. Filter titik terpencil.
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

    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    neighbor_counts = np.array([
        len(tree.query_ball_point(pts[i], DENSITY_RADIUS)) - 1
        for i in range(n)
    ])
    is_lonely = neighbor_counts < MIN_NEIGHBORS
    dense_pts = pts[~is_lonely]

    if len(dense_pts) < MIN_SEGMENT_PTS:
        return 'none', None, ~is_lonely, is_lonely

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
    wall_lines = [c[1] for c in candidates[:N_WALLS]]

    # --- Pilih Model Terbaik ---
    # Hitung Phantom Points untuk Lingkaran
    circle_phantom_count = n
    if circle_model is not None:
        xc, yc, r = circle_model
        dists_c = np.abs(np.hypot(sx_np - xc, sy_np - yc) - r)
        circle_phantom_mask = (dists_c > PHANTOM_DIST_THR) | is_lonely
        circle_phantom_count = circle_phantom_mask.sum()
        
    wall_seg_data = []
    import math
    if len(wall_lines) >= 3:
        sorted_lines = []
        for a, b, c in wall_lines:
            angle = math.atan2(b, a)
            sorted_lines.append((angle, a, b, c))
        sorted_lines.sort(key=lambda item: item[0])
        
        corners = []
        valid_polygon = True
        for i in range(len(sorted_lines)):
            _, a1, b1, c1 = sorted_lines[i]
            _, a2, b2, c2 = sorted_lines[(i+1) % len(sorted_lines)]
            det = a1 * b2 - a2 * b1
            if abs(det) > 1e-3:
                x = (b1 * c2 - b2 * c1) / det
                y = (a2 * c1 - a1 * c2) / det
                corners.append((x, y))
            else:
                valid_polygon = False
                break
                
        if valid_polygon:
            for i in range(len(corners)):
                x1, y1 = corners[i]
                x2, y2 = corners[(i+1) % len(corners)]
                wall_seg_data.append((x1, y1, x2, y2))
                
    # Fallback jika polygon gagal terbentuk
    if not wall_seg_data:
        for count, line in candidates[:N_WALLS]:
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

    # Hitung Phantom Points untuk Dinding (menggunakan segmen kotak jika ada)
    wall_phantom_mask = is_lonely.copy()
    if wall_seg_data:
        for gi in range(n):
            if wall_phantom_mask[gi]:
                continue
            px, py = sx_np[gi], sy_np[gi]
            
            is_outside = False
            for a, b, c in wall_lines:
                if (a * px + b * py + c) < -RANSAC_INLIER_THR: # toleransi setebal inlier RANSAC agar dinding tidak terpotong
                    is_outside = True
                    break
            
            if is_outside:
                wall_phantom_mask[gi] = True
                continue
                
            min_dist = min(_point_to_line_dist(px, py, x1, y1, x2, y2) for x1, y1, x2, y2 in wall_seg_data)
            if min_dist > PHANTOM_DIST_THR:
                wall_phantom_mask[gi] = True
    elif wall_lines:
        for gi in range(n):
            if wall_phantom_mask[gi]:
                continue
            px, py = sx_np[gi], sy_np[gi]
            
            is_outside = False
            for a, b, c in wall_lines:
                if (a * px + b * py + c) < -RANSAC_INLIER_THR: # toleransi setebal inlier RANSAC
                    is_outside = True
                    break
                    
            if is_outside:
                wall_phantom_mask[gi] = True
                continue
                
            min_dist = min(abs(a * px + b * py + c) for a, b, c in wall_lines)
            if min_dist > PHANTOM_DIST_THR:
                wall_phantom_mask[gi] = True
    wall_phantom_count = wall_phantom_mask.sum()

    if SHAPE_MODE == 'circle' and circle_model is not None:
        best_shape = 'circle'
    elif SHAPE_MODE == 'walls':
        best_shape = 'walls'
    else:
        # AUTO MODE
        if circle_model is not None and circle_phantom_count < wall_phantom_count:
            best_shape = 'circle'
        else:
            best_shape = 'walls'
            
    if best_shape == 'circle' and circle_model is not None:
        return 'circle', circle_model, ~circle_phantom_mask, circle_phantom_mask
    else:
        return 'walls', wall_seg_data, ~wall_phantom_mask, wall_phantom_mask



class Visualisasi2D:
    def __init__(self):
        global SHAPE_MODE
        SHAPE_MODE = 'auto' # 'auto', 'walls', 'circle'
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))

        # Peta stabil: indeks 0-359° → jarak EMA
        # 0 berarti belum ada data
        self.stable_map = np.zeros(360)
        self.count_map  = np.zeros(360, dtype=int)  # Berapa kali sudut ini diisi

        # Histori per sudut untuk filter outlier (ring buffer)
        self.hist_map = [[] for _ in range(360)]

        self.current_yaw = 0.0
        self.record_log  = []
        self.running     = True
        self.paket_count = 0

        # =====================
        # Setup Figure
        # =====================
        self.fig, self.ax = plt.subplots(figsize=(9, 9), facecolor='white')
        self.ax.set_facecolor('white')

        # Scatter titik inlier (biru) — dekat dinding
        self.scatter_inlier, = self.ax.plot([], [], 'o',
            color='#1a6fb5', markersize=5, alpha=0.85,
            label='Titik Dinding (Inlier)', zorder=4)

        # Scatter titik phantom (oranye) — jauh dari garis dinding
        self.scatter_phantom, = self.ax.plot([], [], 'o',
            color='#f57c00', markersize=5, alpha=0.9,
            label='Phantom Point', zorder=5)

        # Titik mentah (kecil, transparan) – referensi sensor
        self.boundary, = self.ax.plot([], [], 'o',
            color='#aaaaaa', markersize=2, alpha=0.4,
            label='Titik Sensor (Raw)', zorder=2)

        # Garis nominal wall (merah, putus-putus)
        self.wall_collection = mc.LineCollection(
            [], colors='#cc2222', linewidths=2.5, linestyles='dashed', alpha=0.9,
            label='Nominal Wall (RANSAC)', zorder=6)
        self.ax.add_collection(self.wall_collection)
        
        # Lingkaran nominal (merah, putus-putus)
        self.circle_patch = plt.Circle((0,0), 10, color='#cc2222', fill=False, linewidth=2.5, ls='dashed', alpha=0.9,
                                      label='Nominal Circle (RANSAC)', zorder=6)
        self.circle_patch.set_visible(False)
        self.ax.add_patch(self.circle_patch)
        
        self.ax.plot(0, 0, 'k+', ms=12, mew=2.5, zorder=7)

        # Batas tampilan & dekorasi
        R = 100
        self.ax.set_xlim(-R, R)
        self.ax.set_ylim(-R, R)
        self.ax.set_aspect('equal')
        self.ax.grid(True, ls='--', alpha=0.3, color='#aaaaaa')
        self.ax.axhline(0, color='#888888', lw=0.7, alpha=0.5)
        self.ax.axvline(0, color='#888888', lw=0.7, alpha=0.5)
        self.ax.set_xlabel('X (cm)', color='#333333')
        self.ax.set_ylabel('Y (cm)', color='#333333')
        self.ax.tick_params(colors='#333333')
        for s in self.ax.spines.values():
            s.set_edgecolor('#cccccc')

        self.title_obj = self.ax.set_title(
            'Pemetaan 2D — Menunggu data ESP32...',
            color='#111111', fontsize=13, fontweight='bold', pad=12)

        # Info phantom di pojok kiri bawah
        self.phantom_text = self.ax.text(
            -98, -96, '', fontsize=8, color='#f57c00',
            ha='left', va='bottom', zorder=8)

        # Mode hint di pojok kanan bawah
        self.mode_text = self.ax.text(
            98, -96, "Tekan 'M' untuk ubah mode (Auto/Garis/Lingkaran)", 
            fontsize=8, color='#555555',
            ha='right', va='bottom', zorder=8)

        self.ax.legend(loc='upper right', facecolor='white',
                       labelcolor='#111111', fontsize=8, framealpha=0.9,
                       edgecolor='#cccccc')

        # Lingkaran referensi jarak
        for r in [25, 50, 75, 100]:
            c = plt.Circle((0,0), r, fill=False, color='#aaaaaa',
                           lw=0.6, alpha=0.5)
            self.ax.add_patch(c)
            self.ax.text(r+1, 2, f'{r}cm', color='#888888',
                        fontsize=7, alpha=0.8)

        # Garis arah mata angin (tipis)
        for ang in [0, 45, 90, 135, 180, 225, 270, 315]:
            r = np.radians(ang)
            self.ax.plot([0, R*np.cos(r)], [0, R*np.sin(r)],
                        color='#cccccc', lw=0.4, alpha=0.6)

        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        # Mulai thread penerima
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _on_key(self, event):
        global SHAPE_MODE
        if event.key == 'm' or event.key == 'M':
            if SHAPE_MODE == 'auto':
                SHAPE_MODE = 'walls'
            elif SHAPE_MODE == 'walls':
                SHAPE_MODE = 'circle'
            else:
                SHAPE_MODE = 'auto'
            print(f"[*] Mode diubah ke: {SHAPE_MODE.upper()}")
            # Force update plot to apply mode immediately if data is stopped
            if hasattr(self, 'last_paket_count'):
                self.last_paket_count = -1 
                
    # ==========================================
    def _receive(self):
        print(f"[*] Membaca data CSV: {CSV_FILE_PATH}")
        import time, csv
        try:
            with open(CSV_FILE_PATH, 'r') as f:
                reader = csv.reader(f)
                next(reader, None) # skip header
                for row in reader:
                    if not self.running:
                        break
                    if len(row) < 10:
                        continue
                    
                    yaw_raw = row[1]
                    distances = ",".join(row[2:10])
                    msg = f"{yaw_raw},{distances}"
                    self._parse(msg)
                    time.sleep(0.05) # simulate 20Hz update
        except Exception as e:
            if self.running:
                print(f"[!] {e}")

    def _parse(self, message):
        try:
            parts = message.split(',')
            if len(parts) != 9:
                return

            yaw_raw   = float(parts[0])
            distances = [float(x) for x in parts[1:]]
            self.current_yaw = yaw_raw
            self.paket_count += 1

            # Log
            now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.record_log.append(
                [now, round(yaw_raw, 2)] + [round(d, 1) for d in distances])

            for i in range(NUM_SENSOR):
                dist = distances[i]
                if dist < MIN_DIST or dist > MAX_DIST:
                    continue

                # ─────────────────────────────────────────
                # RUMUS KUNCI:
                # CW  = yaw negatif → physical = -yaw → naik positif
                # CCW = yaw positif → physical = -yaw → turun ke 0
                # Keduanya pakai formula SAMA: (-yaw + offset_sensor) % 360
                # ─────────────────────────────────────────
                physical_deg = (-yaw_raw + i * SENSOR_STEP) % 360
                idx = int(physical_deg) % 360

                # ─────────────────────────────────────────
                # FILTER OUTLIER per sudut:
                # Jika histori sudah cukup, tolak data yang menyimpang
                # lebih dari OUTLIER_SIGMA * std dari median histori
                # ─────────────────────────────────────────
                hist = self.hist_map[idx]
                if len(hist) >= OUTLIER_WINDOW:
                    med = np.median(hist)
                    std = np.std(hist)
                    if std > 0 and abs(dist - med) > OUTLIER_SIGMA * std:
                        continue  # Buang data noise/spike

                # Simpan ke histori (ring buffer)
                hist.append(dist)
                if len(hist) > OUTLIER_WINDOW:
                    hist.pop(0)

                # Update EMA di stable_map
                if self.stable_map[idx] == 0:
                    self.stable_map[idx] = dist
                else:
                    self.stable_map[idx] = (
                        (1 - EMA_ALPHA) * self.stable_map[idx]
                        + EMA_ALPHA * dist)
                self.count_map[idx] += 1

        except (ValueError, IndexError):
            pass

    def _fit_rectangle(self, xs, ys):
        """
        Fit persegi panjang ke titik-titik point cloud menggunakan PCA.
        PCA menemukan orientasi utama ruangan secara otomatis,
        lalu bounding rectangle digambar pada sumbu principal tersebut.
        """
        pts = np.column_stack([xs, ys])
        if len(pts) < 10:
            return [], []

        # Pusat massa titik-titik
        center = pts.mean(axis=0)
        pts_c  = pts - center

        # PCA: eigen-decomposition dari covariance matrix
        cov = np.cov(pts_c.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # Urutkan: eigenvalue terbesar = sumbu utama (panjang ruangan)
        order = eigenvalues.argsort()[::-1]
        V = eigenvectors[:, order]          # Kolom = sumbu utama PCA

        # Proyeksikan semua titik ke sumbu PCA
        proj = pts_c @ V

        # Batas rectangle dalam ruang PCA
        lo = proj.min(axis=0)
        hi = proj.max(axis=0)

        # 4 sudut rectangle dalam ruang PCA (polygon tertutup)
        corners_p = np.array([
            [lo[0], lo[1]],
            [hi[0], lo[1]],
            [hi[0], hi[1]],
            [lo[0], hi[1]],
            [lo[0], lo[1]],   # Tutup polygon
        ])

        # Transformasi balik ke koordinat dunia (cm)
        corners = corners_p @ V.T + center
        return corners[:, 0].tolist(), corners[:, 1].tolist()

    def _build_from_map(self):
        """Bangun daftar titik terfilter dari stable_map."""
        sx, sy       = [], []
        raw_x, raw_y = [], []

        for i in range(360):
            d = self.stable_map[i]
            if d > 0:
                rad = np.radians(i)
                x = d * np.cos(rad)
                y = d * np.sin(rad)
                raw_x.append(x)
                raw_y.append(y)
                if self.count_map[i] >= MIN_COUNT:
                    sx.append(x)
                    sy.append(y)

        return sx, sy, raw_x, raw_y

    def _update_plot(self, frame):
        # Hentikan update (mencegah jitter RANSAC) jika data CSV sudah selesai / tidak ada paket baru
        if hasattr(self, 'last_paket_count') and self.last_paket_count == self.paket_count:
            return
        self.last_paket_count = self.paket_count
        
        filled   = int(np.count_nonzero(self.stable_map))
        filtered = int(np.sum(self.count_map >= MIN_COUNT))
        
        mode_str = "AUTO" if SHAPE_MODE == 'auto' else ("KOTAK/GARIS" if SHAPE_MODE == 'walls' else "LINGKARAN")

        self.title_obj.set_text(
            f'Pemetaan 2D | Mode: {mode_str} | '
            f'{filtered}/{filled} stabil | '
            f'Yaw: {self.current_yaw:.1f}\u00b0 | '
            f'Paket: {self.paket_count}')

        if filled == 0:
            return

        sx, sy, raw_x, raw_y = self._build_from_map()
        self.boundary.set_data(raw_x, raw_y)

        if len(sx) < MIN_SEGMENT_PTS * 2:
            self.scatter_inlier.set_data(sx, sy)
            self.scatter_phantom.set_data([], [])
            self.wall_collection.set_segments([])
            self.circle_patch.set_visible(False)
            self.phantom_text.set_text('')
        else:
            sx_np = np.array(sx, dtype=float)
            sy_np = np.array(sy, dtype=float)

            shape_type, shape_data, inlier_mask, phantom_mask = \
                _detect_best_shape_and_phantoms(sx_np.tolist(), sy_np.tolist())

            # Titik inlier (biru)
            self.scatter_inlier.set_data(sx_np[inlier_mask],
                                         sy_np[inlier_mask])
            # Titik phantom (oranye)
            self.scatter_phantom.set_data(sx_np[phantom_mask],
                                          sy_np[phantom_mask])
                                          
            # Gambar model yang terpilih
            if shape_type == 'circle':
                self.wall_collection.set_segments([])
                if shape_data is not None:
                    xc, yc, r = shape_data
                    self.circle_patch.set_center((xc, yc))
                    self.circle_patch.set_radius(r)
                    self.circle_patch.set_visible(True)
            elif shape_type == 'walls':
                self.circle_patch.set_visible(False)
                if shape_data:
                    self.wall_collection.set_segments(
                        [[(x1, y1), (x2, y2)] for x1, y1, x2, y2 in shape_data])
                else:
                    self.wall_collection.set_segments([])
            else:
                self.wall_collection.set_segments([])
                self.circle_patch.set_visible(False)

            # Info teks
            n_ph  = int(phantom_mask.sum())
            n_tot = len(sx)
            pct   = 100 * n_ph / n_tot if n_tot > 0 else 0
            
            if shape_type == 'circle':
                r = shape_data[2] if shape_data else 0
                shape_text = f' | Shape: Lingkaran (R: {r:.1f}cm)'
            elif shape_type == 'walls':
                shape_text = f' | Shape: Garis Dinding ({len(shape_data)} segmen)'
            else:
                shape_text = ''
                
            self.phantom_text.set_text(
                f'Phantom: {n_ph}/{n_tot} titik ({pct:.1f}%){shape_text}')

    def save_data(self):
        if not self.record_log:
            print("\n[-] Tidak ada data. Simpan dilewati.")
            return
        base = "Data"
        os.makedirs(base, exist_ok=True)
        idx = 1
        while os.path.exists(os.path.join(base, f"percobaan_{idx}")):
            idx += 1
        d = os.path.join(base, f"percobaan_{idx}")
        os.makedirs(d)
        with open(os.path.join(d, "koordinat.csv"), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["Waktu", "Yaw_IMU"] +
                        [f"Sensor_{i+1}" for i in range(8)])
            w.writerows(self.record_log)
        self.fig.savefig(os.path.join(d, "peta_2d.png"),
                         dpi=300, bbox_inches='tight',
                         facecolor='white')
        print(f"\n[+] Tersimpan: {d}")
        print(f"    {len(self.record_log)} baris | "
              f"{int(np.count_nonzero(self.stable_map))}/360 sudut terpetakan")

    def start(self):
        self.anim = FuncAnimation(
            self.fig, self._update_plot,
            interval=200, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    app = Visualisasi2D()
    try:
        app.start()
    except KeyboardInterrupt:
        print("\n[!] Menutup...")
    finally:
        app.running = False
        app.save_data()
        app.sock.close()
