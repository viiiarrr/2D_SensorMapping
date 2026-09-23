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
  n_walls:           4,     // Jumlah dinding maksimal target (4 = persegi)
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
    const p = this.p;
    
    // ── SAVITZKY-GOLAY FILTER (1D) ──
    // Window size 7, Polynomial order 2 atau 3
    // Coeffs: [-2, 3, 6, 7, 6, 3, -2] / 21
    const sg = [...this.stableMap];
    for (let i = 0; i < 360; i++) {
      if (this.stableMap[i] > 0 && this.countMap[i] >= p.min_count) {
        let valid = true;
        let window = [];
        for (let j = -3; j <= 3; j++) {
          let idx = (i + j + 360) % 360;
          if (this.stableMap[idx] === 0) { valid = false; break; }
          window.push(this.stableMap[idx]);
        }
        if (valid) {
          sg[i] = (-2*window[0] + 3*window[1] + 6*window[2] + 7*window[3] + 6*window[4] + 3*window[5] - 2*window[6]) / 21;
        }
      }
    }

    const sx = [], sy = [], rawX = [], rawY = [], counts = [];
    for (let i = 0; i < 360; i++) {
      const d = this.stableMap[i];
      if (d > 0) {
        const rad = i * Math.PI / 180;
        const x   = d * Math.cos(rad);
        const y   = d * Math.sin(rad);
        rawX.push(x); rawY.push(y);
        counts.push(this.countMap[i]);

        if (this.countMap[i] >= p.min_count && sg[i] > 0) {
          sx.push(sg[i] * Math.cos(rad));
          sy.push(sg[i] * Math.sin(rad));
        } else {
          sx.push(x);
          sy.push(y);
        }
      }
    }
    return { sx, sy, rawX, rawY, counts };
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
  //  RANSAC Circle Fitting & Least Squares Refinement
  // ─────────────────────────────────────────────────────
  _fitCircleLS(pts, mask) {
    const inPts = pts.filter((_, i) => mask[i]);
    const N = inPts.length;
    if (N < 3) return null;
    
    let sumX = 0, sumY = 0;
    for (const [x, y] of inPts) { sumX += x; sumY += y; }
    const cx0 = sumX / N, cy0 = sumY / N;
    
    let sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0;
    for (const [x, y] of inPts) {
      const u = x - cx0, v = y - cy0;
      const z = u*u + v*v;
      sxx += u*u; syy += v*v; sxy += u*v;
      sxz += u*z; syz += v*z;
    }
    const det = sxx*syy - sxy*sxy;
    if (Math.abs(det) < 1e-9) return null;
    
    const D = (syz * sxy - sxz * syy) / det;
    const E = (sxz * sxy - syz * sxx) / det;
    
    const cx = cx0 - D/2;
    const cy = cy0 - E/2;
    let rSum = 0;
    for (const [x, y] of inPts) rSum += Math.hypot(x - cx, y - cy);
    return { cx, cy, r: rSum / N };
  }

  _ransacCircle(pts) {
    const p = this.p;
    if (pts.length < 3) return null;

    let bestMask = null, bestCount = 0;

    for (let iter = 0; iter < p.ransac_iter; iter++) {
      let i = Math.floor(Math.random() * pts.length);
      let j = Math.floor(Math.random() * (pts.length - 1)); if (j >= i) j++;
      let k = Math.floor(Math.random() * (pts.length - 2)); if (k >= i) k++; if (k >= j) k++;

      const [x1, y1] = pts[i];
      const [x2, y2] = pts[j];
      const [x3, y3] = pts[k];

      const D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
      if (Math.abs(D) < 1e-9) continue; // Collinear points

      const cx = ((x1*x1 + y1*y1)*(y2 - y3) + (x2*x2 + y2*y2)*(y3 - y1) + (x3*x3 + y3*y3)*(y1 - y2)) / D;
      const cy = ((x1*x1 + y1*y1)*(x3 - x2) + (x2*x2 + y2*y2)*(x1 - x3) + (x3*x3 + y3*y3)*(x2 - x1)) / D;
      const r = Math.hypot(x1 - cx, y1 - cy);

      const mask = pts.map(pp => Math.abs(Math.hypot(pp[0] - cx, pp[1] - cy) - r) < p.ransac_inlier_thr);
      const cnt = mask.filter(Boolean).length;
      if (cnt > bestCount) { bestCount = cnt; bestMask = mask; }
    }

    if (!bestMask || bestCount < 3) return null;
    const refined = this._fitCircleLS(pts, bestMask);
    if (!refined) return null;
    
    const mask = pts.map(pp => Math.abs(Math.hypot(pp[0] - refined.cx, pp[1] - refined.cy) - refined.r) < p.ransac_inlier_thr);
    const cnt = mask.filter(Boolean).length;
    
    return { circle: refined, inlierMask: mask, inlierCount: cnt };
  }

  // ─────────────────────────────────────────────────────
  //  NWA — Nominal Wall Angle + Sequential RANSAC
  //  1. Sequential RANSAC → temukan dinding-dinding utama
  //  2. Pilih N_WALLS dinding terbaik (terbanyak inlier)
  //  3. Sort berdasarkan sudut normal → urutan CCW
  //  4. Intersect pasangan bersebelahan → sudut-sudut polygon
  //  5. Phantom: jarak ke dinding terdekat > threshold
  // ─────────────────────────────────────────────────────
  detectWalls(sx, sy, counts) {
    const n = sx.length;
    const p = this.p;
    const N_WALLS = p.n_walls;

    if (n < p.min_segment_pts * 2) {
      return { isCircle: false, wallSegs: [], inlierMask: new Array(n).fill(true), phantomMask: new Array(n).fill(false), snappedX: [...sx], snappedY: [...sy] };
    }

    const pts = sx.map((x, i) => [x, sy[i]]);
    const isUnstable = counts ? counts.map(c => c < p.min_count) : new Array(n).fill(false);

    // ── ISOLASI TITIK KESEPIAN (Lonely Points) ──
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

    const isPhantomBase = isUnstable.map((u, i) => u || isLonely[i]);
    const densePts = pts.filter((_, i) => !isPhantomBase[i]);

    // ── OPSI 1: LINGKARAN (CIRCLE RANSAC) ──
    const circleRes = this._ransacCircle(densePts);
    let circleScore = circleRes ? circleRes.inlierCount : 0;

    // ── OPSI 2: GARIS LURUS (LINE RANSAC) ──
    let remaining = densePts.slice();
    const candidates = [];
    let lineScore = 0;

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
        lineScore += inlierCnt;
      }
      remaining = nextRemaining;
    }

    candidates.sort((a, b) => b.inlierCount - a.inlierCount);
    
    // ── AUTO SHAPE DETECTION: Deteksi otomatis Segitiga vs Persegi ──
    // Hanya ambil dinding utama. Jika terjadi penurunan drastis inlier antar iterasi, anggap iterasi berikutnya hanya noise.
    const validCandidates = [];
    if (candidates.length > 0) {
      validCandidates.push(candidates[0]);
      for (let i = 1; i < candidates.length; i++) {
        if (candidates[i].inlierCount < candidates[0].inlierCount * 0.4) {
          break; // relative drop to largest wall
        }
        validCandidates.push(candidates[i]);
      }
    }
    const wallLines = validCandidates.slice(0, N_WALLS).map(c => c.line);

    // ── AUTO-DETECTION: Gunakan pendekatan Phantom Count (seperti versi Python) ──
    // Model yang benar akan menyisakan LEBIH SEDIKIT phantom point.
    let circlePhantomCount = n;
    if (circleRes) {
      const { cx, cy, r } = circleRes.circle;
      const cPhantom = isPhantomBase.slice();
      for (let i = 0; i < n; i++) {
        if (cPhantom[i]) continue;
        const dCenter = Math.hypot(sx[i] - cx, sy[i] - cy);
        if (Math.abs(dCenter - r) > p.phantom_dist_thr) cPhantom[i] = true;
      }
      circlePhantomCount = cPhantom.filter(Boolean).length;
    }

    let wallPhantomCount = n;
    const wPhantom = isPhantomBase.slice();
    if (wallLines.length > 0) {
      for (let i = 0; i < n; i++) {
        if (wPhantom[i]) continue;
        let minDist = Infinity;
        for (const [a, b, c] of wallLines) {
          const d = Math.abs(a * sx[i] + b * sy[i] + c);
          if (d < minDist) minDist = d;
        }
        if (minDist > p.phantom_dist_thr) wPhantom[i] = true;
      }
      wallPhantomCount = wPhantom.filter(Boolean).length;
    }

    const isCircle = circleRes && (circlePhantomCount <= wallPhantomCount);
    console.log(`[Auto-Detect] Circle Phantoms: ${circlePhantomCount}, Wall Phantoms: ${wallPhantomCount} -> Mode: ${isCircle ? 'CIRCLE' : 'LINES'}`);

    const snappedX = [...sx];
    const snappedY = [...sy];
    const isPhantom = isLonely.slice(); 
    
    if (isCircle) {
      // ── MODE LINGKARAN ──
      const { cx, cy, r } = circleRes.circle;
      const inlierMask = new Array(n).fill(false);
      
      for (let i = 0; i < n; i++) {
        if (isLonely[i]) continue;
        const dCenter = Math.hypot(sx[i] - cx, sy[i] - cy);
        if (Math.abs(dCenter - r) <= p.ransac_inlier_thr) {
          inlierMask[i] = true;
          // Point Snapping ke Lingkaran
          const angle = Math.atan2(sy[i] - cy, sx[i] - cx);
          snappedX[i] = cx + r * Math.cos(angle);
          snappedY[i] = cy + r * Math.sin(angle);
        }
      }
      
      // Semua titik yang bukan inlier adalah Phantom Point
      const isPhantom = inlierMask.map(is_inlier => !is_inlier);
      
      return {
        isCircle: true,
        circleData: { cx, cy, r },
        wallSegs: [],
        inlierMask,
        phantomMask: isPhantom,
        snappedX,
        snappedY,
      };
      
    } else {
      // ── MODE GARIS LURUS ──
      const finalWallSegs = [];
      const inlierOfWall = new Array(n).fill(-1);

      let polygonFormed = false;
      const sortedWallSegsMap = {}; // Map original index to its polygon segment
      
      if (wallLines.length >= 3) {
        const stdLines = wallLines.map(([a, b, c], origIdx) => 
          c < 0 ? [-a, -b, -c, origIdx] : [a, b, c, origIdx]
        );
        stdLines.sort((L1, L2) => Math.atan2(L1[1], L1[0]) - Math.atan2(L2[1], L2[0]));
        
        const corners = [];
        for (let i = 0; i < stdLines.length; i++) {
          const [a1, b1, c1] = stdLines[i];
          const [a2, b2, c2] = stdLines[(i+1)%stdLines.length];
          const det = a1*b2 - a2*b1;
          if (Math.abs(det) > 1e-6) {
            corners.push([(b1*c2 - b2*c1)/det, (a2*c1 - a1*c2)/det]);
          } else {
            corners.push(null);
          }
        }
        
        if (corners.every(c => c !== null)) {
          for (let i = 0; i < corners.length; i++) {
            const [x1, y1] = corners[i];
            const [x2, y2] = corners[(i+1)%corners.length];
            const origIdx = stdLines[i][3];
            sortedWallSegsMap[origIdx] = [x1, y1, x2, y2];
          }
          polygonFormed = true;
        }
      }

      for (let w = 0; w < wallLines.length; w++) {
        const line = wallLines[w];
        const [a, b, c] = line;
        const seg = polygonFormed ? sortedWallSegsMap[w] : null;
        let tMin = Infinity, tMax = -Infinity;
        
        for (let i = 0; i < n; i++) {
          if (isPhantomBase[i]) continue; // Langsung skip unstable/lonely
          const dist = Math.abs(a * sx[i] + b * sy[i] + c);
          if (dist <= p.ransac_inlier_thr) {
            
            // Cek apakah proyeksi jatuh di dalam segmen polygon
            let validInSegment = true;
            if (seg) {
               const [x1, y1, x2, y2] = seg;
               const dx = x2 - x1, dy = y2 - y1;
               const L2 = dx*dx + dy*dy;
               if (L2 > 1e-6) {
                 const t = ((sx[i] - x1)*dx + (sy[i] - y1)*dy) / L2;
                 if (t < -0.1 || t > 1.1) validInSegment = false;
               }
            }
            
            if (validInSegment) {
              if (inlierOfWall[i] === -1) inlierOfWall[i] = w;
              
              if (inlierOfWall[i] === w) {
                const tProj = -b * sx[i] + a * sy[i];
                if (tProj < tMin) tMin = tProj;
                if (tProj > tMax) tMax = tProj;
                
                snappedX[i] = -b * tProj - a * c;
                snappedY[i] =  a * tProj - b * c;
              }
            }
          }
        }
        
        if (polygonFormed) {
          finalWallSegs.push(seg);
        } else if (tMin !== Infinity && tMax !== -Infinity) {
          const x1 = -b * tMin - a * c;
          const y1 =  a * tMin - b * c;
          const x2 = -b * tMax - a * c;
          const y2 =  a * tMax - b * c;
          finalWallSegs.push([x1, y1, x2, y2]);
        }
      }

      // Semua titik yang bukan inlier adalah Phantom Point
      const isPhantom = inlierOfWall.map(w => w === -1);

      return {
        isCircle: false,
        wallSegs: finalWallSegs,
        inlierMask: inlierOfWall.map(w => w !== -1),
        phantomMask: isPhantom,
        snappedX,
        snappedY,
      };
    }
  }

  // ─────────────────────────────────────────────────────
  //  EVALUASI PROPOSAL
  //  Menghitung Error Jarak, NWA Success, dan Simpangan Dimensi berdasarkan Ground Truth
  // ─────────────────────────────────────────────────────
  evaluateProposalMetrics(sx, sy, wallResult, gtType) {
    const p = this.p;
    const n = sx.length;
    let evalMetrics = { errorJarak: 0, nwaSuccess: 0, errorDimensi: 0 };
    
    if (n === 0 || !wallResult || gtType === 'none') return evalMetrics;

    const { isCircle, circleData, wallSegs, inlierMask, phantomMask } = wallResult;

    // 1. Tentukan Ground Truth ideal
    let gtPerimeter = 0;
    let getIdealDist = (angleRad) => 0;

    if (gtType === 'rect') {
      gtPerimeter = 2 * (150 + 90);
      getIdealDist = (a) => {
        let aw = 150 / 2;
        let ah = 90 / 2;
        let c = Math.abs(Math.cos(a)), s = Math.abs(Math.sin(a));
        if (c === 0) return ah;
        if (s === 0) return aw;
        return Math.min(aw / c, ah / s);
      };
    } else if (gtType === 'square') {
      gtPerimeter = 4 * 120;
      getIdealDist = (a) => {
        let h = 120 / 2;
        let c = Math.abs(Math.cos(a)), s = Math.abs(Math.sin(a));
        if (c === 0) return h;
        if (s === 0) return h;
        return Math.min(h / c, h / s);
      };
    } else if (gtType === 'triangle') {
      gtPerimeter = 3 * 120;
      const r_in = 120 / (2 * Math.sqrt(3)); // ~34.641
      getIdealDist = (a) => {
        let deg = (a * 180 / Math.PI + 360) % 360;
        let t = ((deg % 120) - 60) * Math.PI / 180;
        return r_in / Math.cos(t);
      };
    } else if (gtType === 'circle') {
      gtPerimeter = 2 * Math.PI * 50;
      getIdealDist = (a) => 50;
    }

    // 2. Hitung Error Jarak (Khusus untuk inlier)
    let sumError = 0;
    let countInlier = 0;
    for (let i = 0; i < n; i++) {
      if (inlierMask[i]) {
        let ptDist = Math.hypot(sx[i], sy[i]);
        let angle = Math.atan2(sy[i], sx[i]);
        let idealDist = getIdealDist(angle);
        
        let errPct = Math.abs(ptDist - idealDist) / idealDist;
        sumError += errPct;
        countInlier++;
      }
    }
    evalMetrics.errorJarak = countInlier > 0 ? (sumError / countInlier) * 100 : 0;

    // 3. Hitung NWA Success Rate (Recall dari Phantom Point Sebenarnya)
    let truePhantomCount = 0;
    let removedTruePhantomCount = 0;
    
    for (let i = 0; i < n; i++) {
      let ptDist = Math.hypot(sx[i], sy[i]);
      let angle = Math.atan2(sy[i], sx[i]);
      let idealDist = getIdealDist(angle);
      
      if (Math.abs(ptDist - idealDist) > p.phantom_dist_thr) {
        truePhantomCount++;
        if (phantomMask[i]) {
          removedTruePhantomCount++;
        }
      }
    }
    
    if (truePhantomCount > 0) {
      evalMetrics.nwaSuccess = (removedTruePhantomCount / truePhantomCount) * 100;
    } else {
      evalMetrics.nwaSuccess = 100;
    }

    // 4. Hitung Error Dimensi (Menggunakan Keliling sebagai representasi linier)
    let detectPerim = 0;
    if (isCircle && circleData) {
      detectPerim = 2 * Math.PI * circleData.r;
    } else if (wallSegs && wallSegs.length > 0) {
      for (const [x1, y1, x2, y2] of wallSegs) {
        detectPerim += Math.hypot(x2 - x1, y2 - y1);
      }
    }
    
    if (gtPerimeter > 0 && detectPerim > 0) {
      evalMetrics.errorDimensi = Math.abs(detectPerim - gtPerimeter) / gtPerimeter * 100;
    } else {
      evalMetrics.errorDimensi = 0;
    }

    return evalMetrics;
  }

  // ─────────────────────────────────────────────────────
  //  METRIK KUANTITATIF
  //  Menghitung RMSE, MAE, Max Error, Coverage, Luas, Keliling, dll.
  // ─────────────────────────────────────────────────────
  computeQuantitativeMetrics(sx, sy, wallResult, gtType = 'none') {
    const p = this.p;
    const n = sx.length;

    // ── Default kosong ──
    const empty = {
      shapeAccuracy: { rmse: 0, mae: 0, maxError: 0, inlierRatio: 0 },
      mappingQuality: { coverage: 0, stablePercent: 0, phantomRate: 0, avgDensity: 0 },
      geometry: { type: '—', dimensions: '—', area: 0, perimeter: 0 },
      sensorStability: { avgStdDev: 0, convergenceScore: 0 },
    };

    if (n === 0 || !wallResult) return empty;

    const { isCircle, circleData, wallSegs, inlierMask, phantomMask, snappedX, snappedY } = wallResult;

    // ════════════════════════════════════════
    // 1. AKURASI BENTUK (Shape Accuracy)
    // ════════════════════════════════════════
    const errors = [];
    for (let i = 0; i < n; i++) {
      if (!inlierMask[i]) continue;
      if (isCircle && circleData) {
        // Jarak titik asli ke lingkaran fitting
        const dCenter = Math.hypot(sx[i] - circleData.cx, sy[i] - circleData.cy);
        errors.push(Math.abs(dCenter - circleData.r));
      } else if (wallSegs && wallSegs.length > 0) {
        // Jarak titik asli ke garis dinding terdekat
        let minDist = Infinity;
        for (const [x1, y1, x2, y2] of wallSegs) {
          const d = this._pointToSegmentDist(sx[i], sy[i], x1, y1, x2, y2);
          if (d < minDist) minDist = d;
        }
        errors.push(minDist);
      }
    }

    let rmse = 0, mae = 0, maxError = 0;
    if (errors.length > 0) {
      const sumSq = errors.reduce((s, e) => s + e * e, 0);
      rmse = Math.sqrt(sumSq / errors.length);
      mae = errors.reduce((s, e) => s + e, 0) / errors.length;
      maxError = Math.max(...errors);
    }

    const inlierCount = inlierMask.filter(Boolean).length;
    const inlierRatio = n > 0 ? (inlierCount / n) * 100 : 0;

    // ════════════════════════════════════════
    // 2. KUALITAS PEMETAAN (Mapping Quality)
    // ════════════════════════════════════════
    const filled = [...this.stableMap].filter(v => v > 0).length;
    const stable = [...this.countMap].filter(v => v >= p.min_count).length;
    const coverage = (filled / 360) * 100;
    const stablePercent = filled > 0 ? (stable / filled) * 100 : 0;
    const phantomCount = phantomMask.filter(Boolean).length;
    const phantomRate = n > 0 ? (phantomCount / n) * 100 : 0;

    // Densitas: rata-rata jumlah pengukuran per sudut yang terisi
    let totalCounts = 0, filledAngles = 0;
    for (let i = 0; i < 360; i++) {
      if (this.countMap[i] > 0) {
        totalCounts += this.countMap[i];
        filledAngles++;
      }
    }
    const avgDensity = filledAngles > 0 ? totalCounts / filledAngles : 0;

    // ════════════════════════════════════════
    // 3. GEOMETRI TERDETEKSI
    // ════════════════════════════════════════
    let type = '—', dimensions = '—', area = 0, perimeter = 0;

    if (isCircle && circleData) {
      type = 'Lingkaran';
      const r = circleData.r;
      dimensions = `R = ${r.toFixed(1)} cm`;
      area = Math.PI * r * r;
      perimeter = 2 * Math.PI * r;
    } else if (wallSegs && wallSegs.length > 0) {
      const nWalls = wallSegs.length;
      if (nWalls === 3) type = 'Segitiga';
      else if (nWalls === 4) type = 'Persegi/Persegi Panjang';
      else if (nWalls === 5) type = 'Pentagon';
      else if (nWalls === 6) type = 'Heksagon';
      else type = `Polygon (${nWalls} sisi)`;

      // Hitung panjang tiap sisi
      const sideLengths = [];
      for (const [x1, y1, x2, y2] of wallSegs) {
        sideLengths.push(Math.hypot(x2 - x1, y2 - y1));
      }
      perimeter = sideLengths.reduce((s, l) => s + l, 0);

      if (sideLengths.length > 0) {
        const avgSide = perimeter / sideLengths.length;
        const minSide = Math.min(...sideLengths);
        const maxSide = Math.max(...sideLengths);
        if (sideLengths.length === 1) {
          dimensions = `${sideLengths[0].toFixed(1)} cm`;
        } else {
          dimensions = `${minSide.toFixed(1)}–${maxSide.toFixed(1)} cm`;
        }
      }

      // Luas menggunakan Shoelace formula dari interseksi wall segments
      area = this._computePolygonArea(wallSegs);
    }

    // ════════════════════════════════════════
    // 4. STABILITAS SENSOR
    // ════════════════════════════════════════
    let stdDevSum = 0, stdDevCount = 0;
    for (let i = 0; i < 360; i++) {
      const hist = this.histMap[i];
      if (hist.length >= 3) {
        const mean = hist.reduce((s, x) => s + x, 0) / hist.length;
        const variance = hist.reduce((s, x) => s + (x - mean) ** 2, 0) / hist.length;
        stdDevSum += Math.sqrt(variance);
        stdDevCount++;
      }
    }
    const avgStdDev = stdDevCount > 0 ? stdDevSum / stdDevCount : 0;

    // Convergence Score: 100% = sempurna stabil (std dev = 0), menurun seiring std dev naik
    // Menggunakan formula: score = 100 * exp(-avgStdDev / 10)
    const convergenceScore = 100 * Math.exp(-avgStdDev / 10);

    // Hitung Metrik Evaluasi Proposal
    const evalProposal = this.evaluateProposalMetrics(sx, sy, wallResult, gtType);

    return {
      shapeAccuracy: {
        rmse: Math.round(rmse * 100) / 100,
        mae: Math.round(mae * 100) / 100,
        maxError: Math.round(maxError * 100) / 100,
        inlierRatio: Math.round(inlierRatio * 10) / 10,
      },
      mappingQuality: {
        coverage: Math.round(coverage * 10) / 10,
        stablePercent: Math.round(stablePercent * 10) / 10,
        phantomRate: Math.round(phantomRate * 10) / 10,
        avgDensity: Math.round(avgDensity * 10) / 10,
      },
      geometry: { type, dimensions, area: Math.round(area), perimeter: Math.round(perimeter * 10) / 10 },
      sensorStability: {
        avgStdDev: Math.round(avgStdDev * 100) / 100,
        convergenceScore: Math.round(convergenceScore * 10) / 10,
      },
      evalProposal: {
        errorJarak: Math.round(evalProposal.errorJarak * 100) / 100,
        nwaSuccess: Math.round(evalProposal.nwaSuccess * 100) / 100,
        errorDimensi: Math.round(evalProposal.errorDimensi * 100) / 100,
      },
    };
  }

  // Helper: jarak titik ke segmen garis
  _pointToSegmentDist(px, py, x1, y1, x2, y2) {
    const dx = x2 - x1, dy = y2 - y1;
    const len2 = dx * dx + dy * dy;
    if (len2 === 0) return Math.hypot(px - x1, py - y1);
    let t = ((px - x1) * dx + (py - y1) * dy) / len2;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
  }

  // Helper: luas polygon dari wall segments menggunakan Shoelace
  _computePolygonArea(wallSegs) {
    if (wallSegs.length < 3) return 0;

    // Kumpulkan semua endpoint dan hitung centroid
    const pts = [];
    for (const [x1, y1, x2, y2] of wallSegs) {
      pts.push([(x1 + x2) / 2, (y1 + y2) / 2]); // midpoint tiap segment
    }

    // Sort berdasarkan sudut dari centroid
    const cx = pts.reduce((s, p) => s + p[0], 0) / pts.length;
    const cy = pts.reduce((s, p) => s + p[1], 0) / pts.length;
    pts.sort((a, b) => Math.atan2(a[1] - cy, a[0] - cx) - Math.atan2(b[1] - cy, b[0] - cx));

    // Gunakan semua endpoint segment, sorted by angle
    const allPts = [];
    for (const [x1, y1, x2, y2] of wallSegs) {
      allPts.push([x1, y1]);
      allPts.push([x2, y2]);
    }
    allPts.sort((a, b) => Math.atan2(a[1] - cy, a[0] - cx) - Math.atan2(b[1] - cy, b[0] - cx));

    // Hapus duplikat yang berdekatan (jarak < 1cm)
    const unique = [allPts[0]];
    for (let i = 1; i < allPts.length; i++) {
      const d = Math.hypot(allPts[i][0] - unique[unique.length - 1][0], allPts[i][1] - unique[unique.length - 1][1]);
      if (d > 1) unique.push(allPts[i]);
    }

    // Shoelace formula
    let area = 0;
    const m = unique.length;
    for (let i = 0; i < m; i++) {
      const j = (i + 1) % m;
      area += unique[i][0] * unique[j][1];
      area -= unique[j][0] * unique[i][1];
    }
    return Math.abs(area) / 2;
  }
}
