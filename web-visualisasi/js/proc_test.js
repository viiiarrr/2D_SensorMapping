/**
 * processor.js — Port algoritma Python ke JavaScript
 * Algoritma: EMA, Outlier Filter, Sequential RANSAC, NWA (Nominal Wall Angle), Polygon Close
 */

const DEFAULTS = {
  ema_alpha:         0.25,
  max_dist:          250.0,
  min_dist:          2.0,
  outlier_sigma:     2.0,
  outlier_window:    10,
  min_count:         4,
  min_segment_pts:   8,
  ransac_iter:       120,
  ransac_inlier_thr: 8.0,
  phantom_dist_thr:  10.0,  // NWA threshold (cm): titik di luar batas ini = phantom
  n_walls:           3,     // Jumlah dinding target (3 = segitiga)
  // Filter densitas — buang titik "terpencil" sebelum RANSAC
  density_radius:    20.0,  // cm — radius pencarian tetangga
  min_neighbors:     5,     // minimal tetangga dalam radius → kalau kurang = lonely = phantom
  num_sensor:        8,
  sensor_step:       45.0,
  snapshot_interval: 30,
};

class SensorProcessor {
  constructor(params = {}) {
    this.p = { ...DEFAULTS, ...params };
    this.rows = [];
    this.snapshots = [];
    this.reset();
  }

  reset() {
    this.stableMap = new Float64Array(360);
    this.countMap  = new Int32Array(360);
    this.histMap   = Array.from({ length: 360 }, () => []);
    this.currentYaw  = 0;
    this.packetCount = 0;
  }

  _takeSnapshot(rowIndex) {
    return {
      rowIndex,
      stableMap: new Float64Array(this.stableMap),
      countMap:  new Int32Array(this.countMap),
      histMap:   this.histMap.map(h => h.slice()),
      currentYaw:  this.currentYaw,
      packetCount: this.packetCount,
    };
  }

  _loadSnapshot(snap) {
    this.stableMap   = new Float64Array(snap.stableMap);
    this.countMap    = new Int32Array(snap.countMap);
    this.histMap     = snap.histMap.map(h => h.slice());
    this.currentYaw  = snap.currentYaw;
    this.packetCount = snap.packetCount;
  }

  parseCSV(text) {
    const lines = text.replace(/\r/g, '').trim().split('\n');
    const rows  = [];
    for (let i = 1; i < lines.length; i++) {
      const cols = lines[i].split(',');
      if (cols.length < 10) continue;
      const yaw  = parseFloat(cols[1]);
      const dists = cols.slice(2, 10).map(Number);
      if (isNaN(yaw)) continue;
      rows.push({ time: cols[0].trim(), yaw, distances: dists });
    }
    return rows;
  }

  loadCSV(text) {
    this.rows      = this.parseCSV(text);
    this.snapshots = [];
    this.reset();
    const N = this.rows.length;
    const interval = this.p.snapshot_interval;
    this.snapshots.push(this._takeSnapshot(-1));
    for (let i = 0; i < N; i++) {
      this.processRow(this.rows[i]);
      if ((i + 1) % interval === 0 || i === N - 1)
        this.snapshots.push(this._takeSnapshot(i));
    }
  }

  seekTo(rowIndex) {
    rowIndex = Math.max(0, Math.min(rowIndex, this.rows.length - 1));
    let snapIdx = 0;
    for (let i = 0; i < this.snapshots.length; i++) {
      if (this.snapshots[i].rowIndex <= rowIndex) snapIdx = i;
      else break;
    }
    this._loadSnapshot(this.snapshots[snapIdx]);
    const start = this.snapshots[snapIdx].rowIndex + 1;
    for (let i = start; i <= rowIndex; i++) this.processRow(this.rows[i]);
  }

  processRow(row) {
    this.currentYaw = row.yaw;
    this.packetCount++;
    const p = this.p;
    for (let i = 0; i < p.num_sensor; i++) {
      const dist = row.distances[i];
      if (isNaN(dist) || dist < p.min_dist || dist > p.max_dist) continue;
      const physDeg = ((-row.yaw + i * p.sensor_step) % 360 + 360) % 360;
      const idx     = Math.floor(physDeg) % 360;
      const hist = this.histMap[idx];
      if (hist.length >= p.outlier_window) {
        const sorted = hist.slice().sort((a, b) => a - b);
        const med    = sorted[Math.floor(sorted.length / 2)];
        const mean   = hist.reduce((s, x) => s + x, 0) / hist.length;
        const std    = Math.sqrt(hist.reduce((s, x) => s + (x - mean) ** 2, 0) / hist.length);
        if (std > 0 && Math.abs(dist - med) > p.outlier_sigma * std) continue;
      }
      hist.push(dist);
      if (hist.length > p.outlier_window) hist.shift();
      if (this.stableMap[idx] === 0) {
        this.stableMap[idx] = dist;
      } else {
        this.stableMap[idx] = (1 - p.ema_alpha) * this.stableMap[idx] + p.ema_alpha * dist;
      }
      this.countMap[idx]++;
    }
  }

  getMapPoints() {
    const sx = [], sy = [], rawX = [], rawY = [];
    for (let i = 0; i < 360; i++) {
      const d = this.stableMap[i];
      if (d > 0) {
        const rad = i * Math.PI / 180;
        const x   = d * Math.cos(rad);
        const y   = d * Math.sin(rad);
        rawX.push(x); rawY.push(y);
        if (this.countMap[i] >= this.p.min_count) { sx.push(x); sy.push(y); }
      }
    }
    return { sx, sy, rawX, rawY };
  }

  getStats() {
    const filled = 0 | this.stableMap.reduce((s, v) => s + (v > 0 ? 1 : 0), 0);
    const stable = 0 | [...this.countMap].filter(v => v >= this.p.min_count).length;
    return { filled, stable, total: 360, packets: this.packetCount, yaw: this.currentYaw };
  }

  // ─────────────────────────────────────────────────────
  //  RANSAC: fit garis terbaik dari sekumpulan titik
  //  Return: { line:[a,b,c], inlierMask } atau null
  //  Garis sudah ternormalisasi |(a,b)|=1
  //  KONVENSI: c > 0  →  normal (a,b) mengarah KE DALAM (ke arah origin)
  // ─────────────────────────────────────────────────────
  _ransacLine(pts) {
    const p = this.p;
    if (pts.length < 2) return null;

    let bestMask = null, bestCount = 0;

    for (let iter = 0; iter < p.ransac_iter; iter++) {
      let i = Math.floor(Math.random() * pts.length);
      let j = Math.floor(Math.random() * (pts.length - 1));
      if (j >= i) j++;

      const [x1, y1] = pts[i], [x2, y2] = pts[j];
      const dx = x2 - x1, dy = y2 - y1;
      if (Math.abs(dx) < 1e-9 && Math.abs(dy) < 1e-9) continue;

      const rawA = dy, rawB = -dx, rawC = dx * y1 - dy * x1;
      const norm = Math.hypot(rawA, rawB);
      const mask = pts.map(pp => Math.abs(rawA * pp[0] + rawB * pp[1] + rawC) / norm < p.ransac_inlier_thr);
      const cnt  = mask.filter(Boolean).length;
      if (cnt > bestCount) { bestCount = cnt; bestMask = mask; }
    }

    if (!bestMask || bestCount < 2) return null;

    // Re-fit via SVD pada inlier
    const inPts = pts.filter((_, i) => bestMask[i]);
    const cx = inPts.reduce((s, pp) => s + pp[0], 0) / inPts.length;
    const cy = inPts.reduce((s, pp) => s + pp[1], 0) / inPts.length;
    let sxx = 0, sxy = 0, syy = 0;
    for (const [x, y] of inPts) {
      sxx += (x - cx) ** 2; sxy += (x - cx) * (y - cy); syy += (y - cy) ** 2;
    }
    const tr   = sxx + syy;
    const det  = sxx * syy - sxy * sxy;
    const disc = Math.sqrt(Math.max(0, (tr / 2) ** 2 - det));
    const lam  = tr / 2 + disc;
    let dx2, dy2;
    if (Math.abs(sxy) > 1e-9) { dx2 = lam - syy; dy2 = sxy; }
    else { dx2 = sxx >= syy ? 1 : 0; dy2 = sxx >= syy ? 0 : 1; }

    const nAb = Math.hypot(dy2, -dx2);
    if (nAb === 0) return null;
    let a = dy2 / nAb, b = -dx2 / nAb;
    let c = -(a * cx + b * cy);

    // Normalisasi: pastikan c > 0 → normal (a,b) menunjuk ke origin (ke dalam ruangan)
    // Karena|(a,b)|=1, nilai c = jarak origin ke garis dengan tanda.
    // c > 0 berarti origin di sisi positif = sisi yang ditunjuk normal (a,b)
    if (c < 0) { a = -a; b = -b; c = -c; }

    return { line: [a, b, c], inlierMask: bestMask, inlierCount: bestCount };
  }

  // ─────────────────────────────────────────────────────
  //  NWA — Nominal Wall Angle + Sequential RANSAC
  //  1. Sequential RANSAC → temukan dinding-dinding utama
  //  2. Pilih N_WALLS dinding terbaik (terbanyak inlier)
  //  3. Sort berdasarkan sudut normal → urutan CCW
  //  4. Intersect pasangan bersebelahan → sudut-sudut polygon
  //  5. Phantom: jarak ke dinding terdekat > threshold
  // ─────────────────────────────────────────────────────
  detectWalls(sx, sy) {
    const n = sx.length;
    const p = this.p;
    const N_WALLS = p.n_walls;

    if (n < p.min_segment_pts * 2) {
      return { wallSegs: [], inlierMask: new Array(n).fill(true), phantomMask: new Array(n).fill(false) };
    }

    const pts = sx.map((x, i) => [x, sy[i]]);

    // ── PRE-FILTER: Buang titik terpencil (lonely point) ───
    // Titik yang tidak punya cukup tetangga dalam radius → phantom langsung
    // Ini mencegah RANSAC "ketarik" ke titik terisolasi
    const R2 = p.density_radius * p.density_radius;
    const isLonely = new Array(n).fill(false);
    for (let i = 0; i < n; i++) {
      let neighborCount = 0;
      for (let j = 0; j < n; j++) {
        if (i === j) continue;
        const dx = pts[i][0] - pts[j][0];
        const dy = pts[i][1] - pts[j][1];
        if (dx*dx + dy*dy <= R2) {
          neighborCount++;
          if (neighborCount >= p.min_neighbors) break;
        }
      }
      if (neighborCount < p.min_neighbors) isLonely[i] = true;
    }

    // Hanya gunakan titik yang tidak lonely untuk RANSAC
    const densePts = pts.filter((_, i) => !isLonely[i]);

    // ── STEP 1: Sequential RANSAC ──────────────────────────
    // Cari dinding satu per satu: fit garis terbaik → hapus inliernya → ulangi
    let remaining = densePts.slice();
    const candidates = []; // { line, inlierCount }

    for (let iter = 0; iter < N_WALLS + 3; iter++) {
      if (remaining.length < p.min_segment_pts) break;
      const res = this._ransacLine(remaining);
      if (!res) break;

      const [a, b, c] = res.line;
      const nextRemaining = [];
      let inlierCnt = 0;
      for (const pt of remaining) {
        if (Math.abs(a * pt[0] + b * pt[1] + c) <= p.ransac_inlier_thr) {
          inlierCnt++;
        } else {
          nextRemaining.push(pt);
        }
      }
      if (inlierCnt >= p.min_segment_pts) {
        candidates.push({ line: res.line, inlierCount: inlierCnt });
      }
      remaining = nextRemaining;
    }

    // ── STEP 2: Pilih N_WALLS dinding terbaik ──────────────
    candidates.sort((a, b) => b.inlierCount - a.inlierCount);
    const wallLines = candidates.slice(0, N_WALLS).map(c => c.line);
    console.log("WALL LINES:", wallLines);

    const wallSegs = [];

    const N = wallLines.length;

    // ── STEP 3: Batasi garis sesuai dengan titik Inlier ─────────
    // Potong garis merah TEPAT di ujung titik-titik inlier-nya.
    // Gunakan jarak ke garis untuk menentukan inlier (bukan inlierMask yang sudah hilang).
    for (const { line } of candidates.slice(0, N_WALLS)) {
      const [a, b, c] = line;
      // Vektor arah garis: (-b, a)  (tegak lurus terhadap normal (a,b))
      let tMin = Infinity, tMax = -Infinity;
      for (let i = 0; i < n; i++) {
        if (isLonely[i]) continue;
        // Cek apakah titik ini cukup dekat ke garis ini (= inlier)
        const dist = Math.abs(a * sx[i] + b * sy[i] + c);
        if (dist <= p.ransac_inlier_thr) {
          // Proyeksi titik ke arah garis
          const t = -b * sx[i] + a * sy[i];
          if (t < tMin) tMin = t;
          if (t > tMax) tMax = t;
        }
      }
      
      if (tMin !== Infinity && tMax !== -Infinity) {
        // Koordinat ujung garis dari parameter t
        const x1 = -b * tMin - a * c;
        const y1 =  a * tMin - b * c;
        const x2 = -b * tMax - a * c;
        const y2 =  a * tMax - b * c;
        wallSegs.push([x1, y1, x2, y2]);
      }
    }


    // ── STEP 4: NWA Phantom Detection ──────────────────────
    // Titik dinyatakan phantom jika:
    //   (a) Titik terpencil (lonely) — tidak punya tetangga cukup, ATAU
    //   (b) Jarak ke dinding terdekat > NWA threshold
    const isPhantom = isLonely.slice(); // mulai dari hasil lonely filter
    if (wallLines.length > 0) {
      for (let gi = 0; gi < n; gi++) {
        if (isPhantom[gi]) continue; // sudah phantom karena lonely
        const px = sx[gi], py = sy[gi];
        let minDist = Infinity;
        for (const [a, b, c] of wallLines) {
          const d = Math.abs(a * px + b * py + c);
          if (d < minDist) minDist = d;
        }
        if (minDist > p.phantom_dist_thr) isPhantom[gi] = true;
      }
    }

    return {
      wallSegs,
      inlierMask:  isPhantom.map(v => !v),
      phantomMask: isPhantom,
    };
  }
}

module.exports = { SensorProcessor };