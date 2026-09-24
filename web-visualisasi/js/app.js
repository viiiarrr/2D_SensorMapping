/**
 * app.js — Main application logic
 * Mengelola: UI, loading CSV, rendering Plotly, replay mode
 */

/* ============================================================
   KONSTANTA & STATE GLOBAL
   ============================================================ */
const DATA_BASE = '../Data/';   // Relatif ke web-visualisasi/

const proc = new SensorProcessor();

let layers = { inlier: true, phantom: true, raw: true, wall: true };
let plotInitialized = false;
let isPlaying = false;
let playTimer  = null;
let currentFrame = 0;
let speedMultiplier = 1;

let ws = null;
let isLiveMode = false;
let lastRenderTime = 0;
let lastMetrics = null;

/* ============================================================
   INIT
   ============================================================ */
document.addEventListener('DOMContentLoaded', () => {
  loadManifest();
  initDropZone();
  initLayerToggles();
  initReplayBar();
  initParamPanel();
  initExportBtn();
  initQuantPanel();
  initMetricsExport();
  initPlot();
  initWebSocket();
});

/* ============================================================
   MANIFEST — daftar semua percobaan
   ============================================================ */
async function loadManifest() {
  try {
    const resp = await fetch('data/manifest.json');
    if (!resp.ok) throw new Error('manifest not found');
    const { experiments } = await resp.json();
    const sel = document.getElementById('exp-select');
    experiments.forEach(exp => {
      const opt = document.createElement('option');
      opt.value = exp.file;
      opt.textContent = exp.name;
      sel.appendChild(opt);
    });
    sel.addEventListener('change', () => {
      if (sel.value) loadCSVFromURL('data/' + sel.value);
    });
  } catch (e) {
    // Tidak ada manifest (misal: buka file:// lokal) — hanya drag & drop
    const sel = document.getElementById('exp-select');
    const opt = document.createElement('option');
    opt.value = '';
    opt.textContent = '(Upload CSV manual)';
    opt.disabled = true;
    sel.appendChild(opt);
    sel.disabled = true;
    console.log('[i] Manifest tidak tersedia. Gunakan drag & drop.');
  }
}

/* ============================================================
   LOAD CSV — dari URL atau dari File object
   ============================================================ */
async function loadCSVFromURL(url) {
  setStatus('processing', `Memuat ${url}…`);
  showLoading();
  try {
    const resp = await fetch(url);
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const text = await resp.text();
    processCSVText(text, url.split('/').pop());
  } catch (e) {
    setStatus('', `Gagal memuat: ${e.message}`);
    showEmpty();
  }
}

function loadCSVFromFile(file) {
  setStatus('processing', `Memuat ${file.name}…`);
  showLoading();
  const reader = new FileReader();
  reader.onload = e => processCSVText(e.target.result, file.name);
  reader.onerror = () => { setStatus('', 'Gagal membaca file.'); showEmpty(); };
  reader.readAsText(file);
}

function processCSVText(text, filename) {
  // Proses di "background" agar UI tidak freeze (gunakan setTimeout)
  setTimeout(() => {
    try {
      proc.loadCSV(text);
      const stats = proc.getStats();

      // Info file
      const info = document.getElementById('loaded-file-info');
      info.style.display = 'block';
      info.textContent = `${filename}  |  ${proc.rows.length} paket  |  ${stats.filled}/360 sudut`;

      // Setup replay slider
      const slider = document.getElementById('replay-slider');
      slider.max   = proc.rows.length - 1;
      slider.value = proc.rows.length - 1;
      currentFrame = proc.rows.length - 1;
      document.getElementById('replay-bar').style.display = 'flex';

      // Render full state
      renderCurrentState();
      updateStats();
      updateReplayTime();
      setStatus('loaded', `${filename} — ${proc.rows.length} paket`);
      document.getElementById('btn-export').disabled = false;
      document.getElementById('btn-reprocess').disabled = false;

    } catch(e) {
      console.error(e);
      setStatus('', `Error proses CSV: ${e.message}`);
      showEmpty();
    }
  }, 20);
}

/* ============================================================
   PLOTLY — inisialisasi & update
   ============================================================ */

const PLOTLY_CONFIG = {
  displaylogo: false,
  modeBarButtonsToRemove: ['select2d','lasso2d','autoScale2d'],
  scrollZoom: true,
  responsive: true,
};

const PLOT_LAYOUT = {
  paper_bgcolor: '#eef0f4',
  plot_bgcolor:  '#ffffff',
  font: { family: 'Inter, sans-serif', color: '#374151' },
  margin: { t: 44, b: 52, l: 58, r: 20 },
  xaxis: {
    title: { text: 'X (cm)', font: { size: 12, color: '#6b7280' } },
    range: [-110, 110],
    zeroline: true, zerolinecolor: 'rgba(0,0,0,0.2)', zerolinewidth: 1,
    gridcolor: 'rgba(0,0,0,0.06)', gridwidth: 1,
    tickcolor: 'rgba(0,0,0,0.1)', tickfont: { size: 11, color: '#6b7280' },
    scaleanchor: 'y',
    linecolor: 'rgba(0,0,0,0.1)',
  },
  yaxis: {
    title: { text: 'Y (cm)', font: { size: 12, color: '#6b7280' } },
    range: [-110, 110],
    zeroline: true, zerolinecolor: 'rgba(0,0,0,0.2)', zerolinewidth: 1,
    gridcolor: 'rgba(0,0,0,0.06)', gridwidth: 1,
    tickcolor: 'rgba(0,0,0,0.1)', tickfont: { size: 11, color: '#6b7280' },
    linecolor: 'rgba(0,0,0,0.1)',
  },
  legend: {
    x: 1, xanchor: 'right', y: 1, yanchor: 'top',
    bgcolor:     'rgba(255,255,255,0.92)',
    bordercolor: 'rgba(0,0,0,0.12)',
    borderwidth: 1,
    font: { size: 11, color: '#374151' },
  },
  title: {
    text: 'Pemetaan 2D (IMU Yaw) — Menunggu data...',
    font: { size: 13, color: '#111827', weight: 600 },
    x: 0.5,
  },
  shapes: buildCircleShapes(),
  annotations: buildCircleLabels(),
  hovermode: 'closest',
  dragmode: 'pan',
};

function buildCircleShapes() {
  return [25, 50, 75, 100].map(r => ({
    type: 'circle', xref: 'x', yref: 'y',
    x0: -r, y0: -r, x1: r, y1: r,
    line: { color: 'rgba(0,0,0,0.12)', width: 1, dash: 'dot' },
    layer: 'below',
  }));
}

function buildCircleLabels() {
  return [25, 50, 75, 100].map(r => ({
    x: r + 1, y: 3, text: `${r}cm`,
    showarrow: false,
    font: { size: 9, color: 'rgba(100,116,139,0.7)' },
    xanchor: 'left',
  }));
}

function initPlot() {
  const traces = buildEmptyTraces();
  Plotly.newPlot('plot', traces, PLOT_LAYOUT, PLOTLY_CONFIG);
  plotInitialized = true;

  // Sembunyikan container plot sampai ada data
  document.getElementById('plot-container').style.display = 'none';
}

function buildEmptyTraces() {
  return [
    // 0: Raw sensor
    { name: 'Titik Sensor (Raw)', type: 'scatter', mode: 'markers', x: [], y: [],
      marker: { color: '#aaaaaa', size: 3, opacity: 0.5 },
      hovertemplate: 'Raw: (%{x:.1f}, %{y:.1f}) cm<extra></extra>' },
    // 1: Inlier (titik dinding)
    { name: 'Titik Dinding (Inlier)', type: 'scatter', mode: 'markers', x: [], y: [],
      marker: { color: '#1a6fb5', size: 6, opacity: 0.9 },
      hovertemplate: 'Inlier: (%{x:.1f}, %{y:.1f}) cm<extra></extra>' },
    // 2: Phantom
    { name: 'Phantom Point', type: 'scatter', mode: 'markers', x: [], y: [],
      marker: { color: '#1a6fb5', size: 7, opacity: 0.95 },
      hovertemplate: 'Phantom: (%{x:.1f}, %{y:.1f}) cm<extra></extra>' },
    // 3: Wall lines (null-separated)
    { name: 'Nominal Wall (RANSAC)', type: 'scatter', mode: 'lines', x: [], y: [],
      line: { color: '#cc2222', width: 2.5, dash: 'dash' },
      hoverinfo: 'skip' },
    // 4: Pusat sensor (cross)
    { name: 'Posisi Sensor', type: 'scatter', mode: 'markers', x: [0], y: [0],
      marker: { color: '#111827', size: 10, symbol: 'cross', line: { width: 2.5, color: '#111827' } },
      hoverinfo: 'skip', showlegend: false },
  ];
}

function renderCurrentState() {
  if (!plotInitialized) return;

  const { sx, sy, rawX, rawY, counts } = proc.getMapPoints();
  const stats = proc.getStats();

  let wallX = [], wallY = [];
  let inlierX = sx, inlierY = sy;
  let phantomX = [], phantomY = [];
  let wallSegCount = 0;
  let phantomCount = 0;
  let wallResult = null;

  if (sx.length >= proc.p.min_segment_pts * 2) {
    wallResult = proc.detectWalls(sx, sy, counts);
    const { isCircle, circleData, wallSegs, inlierMask, phantomMask, snappedX, snappedY } = wallResult;
    wallSegCount = isCircle ? 1 : wallSegs.length;
    phantomCount = phantomMask.filter(Boolean).length;

    inlierX  = sx.filter((_, i) => inlierMask[i]);
    inlierY  = sy.filter((_, i) => inlierMask[i]);
    phantomX = sx.filter((_, i) => phantomMask[i]);
    phantomY = sy.filter((_, i) => phantomMask[i]);

    if (isCircle) {
      const { cx, cy, r } = circleData;
      for (let i = 0; i <= 100; i++) {
        const theta = (i / 100) * 2 * Math.PI;
        wallX.push(cx + r * Math.cos(theta));
        wallY.push(cy + r * Math.sin(theta));
      }
    } else {
      // Null-separated segments untuk Plotly
      for (const [x1, y1, x2, y2] of wallSegs) {
        wallX.push(x1, x2, null);
        wallY.push(y1, y2, null);
      }
    }
  }

  const pct = sx.length > 0 ? (100 * phantomCount / sx.length).toFixed(1) : '0.0';

  // Update traces
  Plotly.update('plot',
    {
      x: [
        layers.raw     ? rawX    : [],
        layers.inlier  ? inlierX : [],
        layers.phantom ? phantomX: [],
        layers.wall    ? wallX   : [],
        [0],
      ],
      y: [
        layers.raw     ? rawY    : [],
        layers.inlier  ? inlierY : [],
        layers.phantom ? phantomY: [],
        layers.wall    ? wallY   : [],
        [0],
      ],
      visible: [
        layers.raw, layers.inlier, layers.phantom, layers.wall, true,
      ],
    },
    {
      title: {
        text: `Pemetaan 2D (IMU Yaw)  |  ${stats.stable}/${stats.filled} sudut stabil  |  ` +
              `Yaw: ${stats.yaw.toFixed(1)}\u00b0  |  Paket: ${stats.packets}`,
        font: { size: 13, color: '#111827' },
        x: 0.5,
      },
    }
  );

  // Update stats panel
  el('stat-packets').textContent = stats.packets.toLocaleString();
  el('stat-stable').textContent  = `${stats.stable}/${stats.filled}`;
  el('stat-yaw').textContent     = `${stats.yaw.toFixed(1)}°`;
  el('stat-phantom').textContent = `${phantomCount} (${pct}%)`;
  el('stat-walls').textContent   = wallSegCount;

  // Update quantitative metrics panel
  const gtType = el('gt-select') ? el('gt-select').value : 'none';
  updateQuantitativePanel(sx, sy, wallResult, gtType);

  showPlot();
}

function updateStats() { /* Stats already updated in renderCurrentState */ }

/* ============================================================
   LAYER TOGGLES
   ============================================================ */
function initLayerToggles() {
  const map = {
    'chk-inlier':  'inlier',
    'chk-phantom': 'phantom',
    'chk-raw':     'raw',
    'chk-wall':    'wall',
  };
  for (const [id, key] of Object.entries(map)) {
    document.getElementById(id).addEventListener('change', e => {
      layers[key] = e.target.checked;
      if (plotInitialized) renderCurrentState();
    });
  }
}

/* ============================================================
   REPLAY BAR
   ============================================================ */
function initReplayBar() {
  const slider   = document.getElementById('replay-slider');
  const btnPlay  = document.getElementById('btn-play');
  const btnReset = document.getElementById('btn-reset-replay');
  const speedSel = document.getElementById('speed-select');
  const playIcon = document.getElementById('play-icon');
  const pauseIcon= document.getElementById('pause-icon');

  slider.addEventListener('input', () => {
    stopPlay();
    currentFrame = parseInt(slider.value);
    proc.seekTo(currentFrame);
    renderCurrentState();
    updateReplayTime();
    updateSliderStyle();
  });

  btnPlay.addEventListener('click', () => {
    if (isPlaying) stopPlay();
    else startPlay();
  });

  btnReset.addEventListener('click', () => {
    stopPlay();
    slider.value = proc.rows.length - 1;
    currentFrame = proc.rows.length - 1;
    proc.seekTo(currentFrame);
    renderCurrentState();
    updateReplayTime();
    updateSliderStyle();
  });

  speedSel.addEventListener('change', () => {
    speedMultiplier = parseFloat(speedSel.value);
    if (isPlaying) { stopPlay(); startPlay(); }
  });

  function startPlay() {
    isPlaying = true;
    btnPlay.classList.add('active');
    playIcon.style.display = 'none';
    pauseIcon.style.display = '';

    if (currentFrame >= proc.rows.length - 1) {
      currentFrame = 0;
      slider.value = 0;
      proc.seekTo(0);
    }

    const baseInterval = 80; // ms per frame
    const ms = Math.max(10, baseInterval / speedMultiplier);

    playTimer = setInterval(() => {
      currentFrame++;
      if (currentFrame >= proc.rows.length) {
        stopPlay();
        currentFrame = proc.rows.length - 1;
      }
      slider.value = currentFrame;
      proc.seekTo(currentFrame);
      renderCurrentState();
      updateReplayTime();
      updateSliderStyle();
    }, ms);
  }

  function stopPlay() {
    isPlaying = false;
    clearInterval(playTimer);
    btnPlay.classList.remove('active');
    playIcon.style.display = '';
    pauseIcon.style.display = 'none';
  }

  window.stopPlay = stopPlay;
}

function updateReplayTime() {
  const el_time = document.getElementById('replay-time');
  if (!proc.rows.length) { el_time.textContent = '—'; return; }
  const row = proc.rows[Math.min(currentFrame, proc.rows.length - 1)];
  const last = proc.rows[proc.rows.length - 1];
  el_time.textContent = `${row.time} / ${last.time}`;
}

function updateSliderStyle() {
  const slider = document.getElementById('replay-slider');
  const pct = (slider.value / slider.max * 100).toFixed(1);
  slider.style.background =
    `linear-gradient(to right, #6366f1 ${pct}%, rgba(255,255,255,0.1) ${pct}%)`;
}

/* ============================================================
   PARAMETER PANEL
   ============================================================ */
function initParamPanel() {
  const toggle = document.getElementById('params-toggle');
  const body   = document.getElementById('params-body');
  toggle.addEventListener('click', () => {
    body.classList.toggle('collapsed');
    toggle.classList.toggle('open');
  });

  // Sliders
  const sliders = [
    { id: 'prm-ema',  valId: 'val-ema', key: 'ema_alpha',       fmt: v => v.toFixed(2) },
    { id: 'prm-rit',  valId: 'val-rit', key: 'ransac_inlier_thr', fmt: v => v.toFixed(0) },
    { id: 'prm-pht',  valId: 'val-pht', key: 'phantom_dist_thr',  fmt: v => v.toFixed(0) },
    { id: 'prm-spt',  valId: 'val-spt', key: 'split_threshold',   fmt: v => v.toFixed(0) },
    { id: 'prm-msp',  valId: 'val-msp', key: 'min_segment_pts',   fmt: v => v.toFixed(0) },
  ];

  sliders.forEach(({ id, valId, key, fmt }) => {
    const input = document.getElementById(id);
    const span  = document.getElementById(valId);
    input.addEventListener('input', () => {
      span.textContent = fmt(parseFloat(input.value));
      updateSliderTrack(input);
    });
    updateSliderTrack(input);
  });

  document.getElementById('btn-reprocess').addEventListener('click', () => {
    // Kumpulkan parameter baru dari slider
    const newParams = {
      ema_alpha:        parseFloat(document.getElementById('prm-ema').value),
      ransac_inlier_thr: parseFloat(document.getElementById('prm-rit').value),
      phantom_dist_thr:  parseFloat(document.getElementById('prm-pht').value),
      split_threshold:   parseFloat(document.getElementById('prm-spt').value),
      min_segment_pts:   parseInt(document.getElementById('prm-msp').value),
    };
    Object.assign(proc.p, newParams);

    // Re-proses dengan parameter baru
    setStatus('processing', 'Memproses ulang…');
    showLoading();
    setTimeout(() => {
      const savedRows = proc.rows.slice();
      proc.loadCSV(savedRows.map(r =>
        `${r.time},${r.yaw},${r.distances.join(',')}`
      ).join('\n'));
      // Re-assign rows (loadCSV re-parses teks, tapi kita inject langsung)
      proc.rows = savedRows;

      // Rebuild snapshots
      proc.snapshots = [];
      proc.reset();
      const N = proc.rows.length;
      proc.snapshots.push(proc._takeSnapshot(-1));
      for (let i = 0; i < N; i++) {
        proc.processRow(proc.rows[i]);
        if ((i + 1) % proc.p.snapshot_interval === 0 || i === N - 1)
          proc.snapshots.push(proc._takeSnapshot(i));
      }

      const slider = document.getElementById('replay-slider');
      currentFrame = parseInt(slider.value);
      proc.seekTo(currentFrame);
      renderCurrentState();
      setStatus('loaded', 'Proses ulang selesai.');
    }, 20);
  });
}

function updateSliderTrack(input) {
  const min = parseFloat(input.min), max = parseFloat(input.max), val = parseFloat(input.value);
  const pct = ((val - min) / (max - min) * 100).toFixed(1);
  input.style.background =
    `linear-gradient(to right, #6366f1 ${pct}%, rgba(255,255,255,0.1) ${pct}%)`;
}

/* ============================================================
   DRAG & DROP
   ============================================================ */
function initDropZone() {
  const zone   = document.getElementById('drop-zone');
  const input  = document.getElementById('file-input');

  zone.addEventListener('dragover', e => { e.preventDefault(); zone.classList.add('drag-over'); });
  zone.addEventListener('dragleave', () => zone.classList.remove('drag-over'));
  zone.addEventListener('drop', e => {
    e.preventDefault();
    zone.classList.remove('drag-over');
    const file = e.dataTransfer.files[0];
    if (file && file.name.endsWith('.csv')) loadCSVFromFile(file);
  });

  input.addEventListener('change', () => {
    if (input.files[0]) loadCSVFromFile(input.files[0]);
  });
}

/* ============================================================
   EXPORT PNG
   ============================================================ */
function initExportBtn() {
  document.getElementById('btn-export').addEventListener('click', () => {
    Plotly.downloadImage('plot', {
      format: 'png', width: 1200, height: 1000,
      filename: `peta_2d_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}`,
    });
  });
}

/* ============================================================
   QUANTITATIVE ANALYSIS PANEL
   ============================================================ */
function initQuantPanel() {
  const toggle = document.getElementById('quant-toggle');
  const body   = document.getElementById('quant-body');
  toggle.addEventListener('click', () => {
    body.classList.toggle('collapsed');
    toggle.classList.toggle('open');
  });

  const evalToggle = document.getElementById('eval-toggle');
  const evalBody   = document.getElementById('eval-body');
  if(evalToggle && evalBody) {
    evalToggle.addEventListener('click', () => {
      evalBody.classList.toggle('collapsed');
      evalToggle.classList.toggle('open');
    });
  }

  const gtSelect = document.getElementById('gt-select');
  if(gtSelect) {
    gtSelect.addEventListener('change', () => {
      if (plotInitialized) renderCurrentState();
    });
  }
}

function updateQuantitativePanel(sx, sy, wallResult, gtType = 'none') {
  const metrics = proc.computeQuantitativeMetrics(sx, sy, wallResult, gtType);
  lastMetrics = metrics;

  const { shapeAccuracy, mappingQuality, geometry, sensorStability, evalProposal } = metrics;

  // ── Akurasi Bentuk ──
  setQuantValue('q-rmse', shapeAccuracy.rmse, 'cm',
    shapeAccuracy.rmse <= 3 ? 'good' : shapeAccuracy.rmse <= 6 ? 'warn' : 'bad');
  setQuantValue('q-mae', shapeAccuracy.mae, 'cm',
    shapeAccuracy.mae <= 2.5 ? 'good' : shapeAccuracy.mae <= 5 ? 'warn' : 'bad');
  setQuantValue('q-maxerr', shapeAccuracy.maxError, 'cm',
    shapeAccuracy.maxError <= 8 ? 'good' : shapeAccuracy.maxError <= 15 ? 'warn' : 'bad');
  setQuantValue('q-inlier', shapeAccuracy.inlierRatio, '%',
    shapeAccuracy.inlierRatio >= 70 ? 'good' : shapeAccuracy.inlierRatio >= 50 ? 'warn' : 'bad');

  // ── Kualitas Pemetaan ──
  setQuantValue('q-coverage', mappingQuality.coverage, '%',
    mappingQuality.coverage >= 80 ? 'good' : mappingQuality.coverage >= 50 ? 'warn' : 'bad');
  setQuantValue('q-stable', mappingQuality.stablePercent, '%',
    mappingQuality.stablePercent >= 80 ? 'good' : mappingQuality.stablePercent >= 50 ? 'warn' : 'bad');
  setQuantValue('q-phantom', mappingQuality.phantomRate, '%',
    mappingQuality.phantomRate <= 10 ? 'good' : mappingQuality.phantomRate <= 25 ? 'warn' : 'bad');
  setQuantValue('q-density', mappingQuality.avgDensity, '');

  // ── Geometri ──
  el('q-type').textContent = geometry.type;
  el('q-dims').textContent = geometry.dimensions;
  setQuantValue('q-area', geometry.area > 0 ? geometry.area.toLocaleString() : '—', '');
  setQuantValue('q-perim', geometry.perimeter > 0 ? geometry.perimeter : '—', '');

  // ── Stabilitas Sensor ──
  setQuantValue('q-stddev', sensorStability.avgStdDev, 'cm',
    sensorStability.avgStdDev <= 3 ? 'good' : sensorStability.avgStdDev <= 8 ? 'warn' : 'bad');
  setQuantValue('q-convergence', sensorStability.convergenceScore, '%',
    sensorStability.convergenceScore >= 70 ? 'good' : sensorStability.convergenceScore >= 40 ? 'warn' : 'bad');

  // ── Evaluasi Proposal ──
  if (evalProposal) {
    setQuantValue('ev-error', evalProposal.errorJarak > 0 ? evalProposal.errorJarak.toFixed(1) : '—', '%');
    setQuantValue('ev-nwa', evalProposal.nwaSuccess > 0 ? evalProposal.nwaSuccess.toFixed(1) : (gtType !== 'none' ? '0.0' : '—'), '%');
    setQuantValue('ev-dim', evalProposal.errorDimensi > 0 ? evalProposal.errorDimensi.toFixed(1) : '—', '%');
  }

  // Enable export button
  el('btn-export-metrics').disabled = false;
}

function setQuantValue(id, value, unit, colorClass) {
  const elem = el(id);
  if (!elem) return;
  elem.textContent = typeof value === 'number' ? value : value;
  // Remove old color classes
  elem.classList.remove('good', 'warn', 'bad');
  if (colorClass) elem.classList.add(colorClass);
}

function initMetricsExport() {
  document.getElementById('btn-export-metrics').addEventListener('click', exportMetricsCSV);
}

function exportMetricsCSV() {
  if (!lastMetrics) return;
  const m = lastMetrics;
  const rows = [
    ['Kategori', 'Metrik', 'Nilai', 'Satuan'],
    ['Akurasi Bentuk', 'RMSE', m.shapeAccuracy.rmse, 'cm'],
    ['Akurasi Bentuk', 'MAE', m.shapeAccuracy.mae, 'cm'],
    ['Akurasi Bentuk', 'Max Error', m.shapeAccuracy.maxError, 'cm'],
    ['Akurasi Bentuk', 'Inlier Ratio', m.shapeAccuracy.inlierRatio, '%'],
    ['Kualitas Pemetaan', 'Coverage', m.mappingQuality.coverage, '%'],
    ['Kualitas Pemetaan', 'Stable Points', m.mappingQuality.stablePercent, '%'],
    ['Kualitas Pemetaan', 'Phantom Rate', m.mappingQuality.phantomRate, '%'],
    ['Kualitas Pemetaan', 'Avg Density', m.mappingQuality.avgDensity, 'sampel/sudut'],
    ['Geometri', 'Tipe Bentuk', m.geometry.type, ''],
    ['Geometri', 'Dimensi', m.geometry.dimensions, ''],
    ['Geometri', 'Luas', m.geometry.area, 'cm²'],
    ['Geometri', 'Keliling', m.geometry.perimeter, 'cm'],
    ['Stabilitas Sensor', 'Avg StdDev', m.sensorStability.avgStdDev, 'cm'],
    ['Stabilitas Sensor', 'Convergence Score', m.sensorStability.convergenceScore, '%'],
    ['Evaluasi Proposal', 'Error Jarak Titik', m.evalProposal ? m.evalProposal.errorJarak : 0, '%'],
    ['Evaluasi Proposal', 'NWA Success Rate', m.evalProposal ? m.evalProposal.nwaSuccess : 0, '%'],
    ['Evaluasi Proposal', 'Error Dimensi (Keliling)', m.evalProposal ? m.evalProposal.errorDimensi : 0, '%'],
  ];

  const csv = rows.map(r => r.join(',')).join('\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `metrik_kuantitatif_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.csv`;
  a.click();
  URL.revokeObjectURL(url);
}

/* ============================================================
   UI HELPERS
   ============================================================ */
function el(id) { return document.getElementById(id); }

function showEmpty() {
  el('empty-state').style.display   = '';
  el('loading-state').style.display = 'none';
  el('plot-container').style.display= 'none';
}

function showLoading() {
  el('empty-state').style.display   = 'none';
  el('loading-state').style.display = '';
  el('plot-container').style.display= 'none';
}

function showPlot() {
  el('empty-state').style.display   = 'none';
  el('loading-state').style.display = 'none';
  el('plot-container').style.display= '';
  // Trigger resize agar Plotly menyesuaikan ukuran
  window.dispatchEvent(new Event('resize'));
}

function setStatus(type, text) {
  const dot  = el('status-dot');
  const span = el('status-text');
  dot.className  = 'status-dot' + (type ? ' ' + type : '');
  span.textContent = text;
}

// Resize Plotly saat window resize
window.addEventListener('resize', () => {
  if (plotInitialized) Plotly.Plots.resize('plot');
});

/* ============================================================
   WEBSOCKET (LIVE MODE)
   ============================================================ */
function initWebSocket() {
  // Jika diakses dari GitHub Pages, paksa koneksi ke localhost
  // karena script server.py berjalan di komputer lokal pengguna.
  const isGitHub = location.hostname.includes('github.io');
  const wsHost = isGitHub ? 'localhost' : (location.hostname || 'localhost');
  ws = new WebSocket(`ws://${wsHost}:8765`);
  
  ws.onopen = () => {
    isLiveMode = true;
    console.log('[+] WebSocket terhubung: LIVE MODE');
    setStatus('processing', 'Live Mode: Menunggu data...');
    showPlot();
    
    // Nonaktifkan input manual
    el('exp-select').disabled = true;
    el('file-input').disabled = true;
    el('drop-zone').style.opacity = '0.5';
    el('drop-zone').style.pointerEvents = 'none';
    
    // Sembunyikan replay bar di awal
    el('replay-bar').style.display = 'none';
    
    // Aktifkan tombol Export
    el('btn-export').disabled = false;
    
    proc.reset();
    proc.rows = [];
    proc.snapshots = [];
    proc.snapshots.push(proc._takeSnapshot(-1));
  };
  
  ws.onmessage = (e) => {
    if (!isLiveMode) return;
    
    // Parse data dari ESP32 (yaw, d1, d2, ..., d8) -> 9 kolom
    // Jika di masa depan server menyisipkan waktu, formatnya jadi 10 kolom
    const cols = e.data.split(',');
    let timeStr, yaw, dists;
    
    if (cols.length === 9) {
      // Format asli ESP32
      timeStr = new Date().toISOString().slice(11, 23); // HH:mm:ss.SSS lokal
      yaw = parseFloat(cols[0]);
      dists = cols.slice(1, 9).map(Number);
    } else if (cols.length >= 10) {
      // Format file CSV
      timeStr = cols[0].trim();
      yaw = parseFloat(cols[1]);
      dists = cols.slice(2, 10).map(Number);
    } else {
      return;
    }
    
    if (isNaN(yaw)) return;
    
    const row = { time: timeStr, yaw, distances: dists };
    proc.rows.push(row);
    proc.processRow(row);
    currentFrame = proc.rows.length - 1;
    
    // Simpan snapshot untuk replay
    if ((currentFrame + 1) % proc.p.snapshot_interval === 0) {
      proc.snapshots.push(proc._takeSnapshot(currentFrame));
    }
    
    // Batasi update Plotly maks 15 FPS supaya browser tidak lag (±66ms)
    const now = performance.now();
    if (now - lastRenderTime > 66) {
      renderCurrentState();
      updateStats();
      lastRenderTime = now;
      setStatus('processing', `Live: ${currentFrame + 1} paket`);
    }
  };
  
  ws.onerror = () => {
    // Jika gagal, berarti mode statis (dari GitHub Pages atau tanpa server.py)
    isLiveMode = false;
  };
  
  ws.onclose = () => {
    if (isLiveMode) {
      setStatus('', 'Koneksi Live terputus. Beralih ke offline.');
      isLiveMode = false;
      
      // Aktifkan replay bar dengan data yang sudah terkumpul
      const slider = el('replay-slider');
      slider.max = proc.rows.length - 1;
      slider.value = currentFrame;
      el('replay-bar').style.display = 'flex';
      updateReplayTime();
    }
  };
}
