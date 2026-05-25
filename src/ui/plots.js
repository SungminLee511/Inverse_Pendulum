// ui/plots.js — rolling-buffer canvas plots.
//
// Four panels in #plots-pane:
//   plot-angles      θ_i(t) for each joint
//   plot-velocities  θ̇_i(t)
//   plot-phase       phase portrait θ_1 vs θ̇_1 (always link 1, fading trail)
//   plot-force       u_cmd(t) and u_applied(t) with ±F_max dashed reference
//
// Each panel keeps a rolling ring buffer of recent samples. The simulation
// pushes a sample every 1/sampleHz seconds of SIM time (not wall time) so the
// x-axis reflects sim progress. Render redraws at ~30 Hz of wall time.
//
// Sign convention: angles wrapped to (−π, π] for display.

import { state, on } from '../state.js';

const TWO_PI = 2 * Math.PI;
// Wraps an angle into the half-open interval (−π, π] — so −π maps to +π and
// +π stays at +π. Used for displaying angles symmetric around upright.
function wrap(a) {
  let x = a;
  while (x >  Math.PI) x -= TWO_PI;
  while (x <= -Math.PI) x += TWO_PI;
  return x;
}

// Persistent colour cycle for joints (matched across angle / vel / phase).
const JOINT_COLORS = ['#58a6ff', '#f0883e', '#56d364'];   // blue / orange / green

/** Ring buffer of (t, ...samples). Capacity ~ T_window × sampleHz. */
class TimeSeries {
  constructor(capacity) {
    this.cap = capacity;
    this.t  = new Float64Array(capacity);
    this.y  = [];   // array of Float64Array per channel; built on first push
    this.n  = 0;    // number stored
    this.head = 0;  // next write idx (write then advance, wrap)
  }
  _ensureChannels(k) {
    while (this.y.length < k) this.y.push(new Float64Array(this.cap));
  }
  push(t, samples) {
    this._ensureChannels(samples.length);
    this.t[this.head] = t;
    for (let i = 0; i < samples.length; i++) this.y[i][this.head] = samples[i];
    this.head = (this.head + 1) % this.cap;
    if (this.n < this.cap) this.n++;
  }
  clear() { this.n = 0; this.head = 0; }
  /** Iterate in time order, calling cb(t, [y0,y1,...]). */
  forEach(cb) {
    if (this.n === 0) return;
    const start = (this.head - this.n + this.cap) % this.cap;
    const k = this.y.length;
    const row = new Array(k);
    for (let i = 0; i < this.n; i++) {
      const idx = (start + i) % this.cap;
      for (let j = 0; j < k; j++) row[j] = this.y[j][idx];
      cb(this.t[idx], row);
    }
  }
  /** Min / max over time axis. */
  trange() {
    if (this.n === 0) return [0, 1];
    const head_idx = (this.head - 1 + this.cap) % this.cap;
    const tail_idx = (this.head - this.n + this.cap) % this.cap;
    return [this.t[tail_idx], this.t[head_idx]];
  }
}

// ---------------------- Geometry helpers ----------------------

function dpiFit(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const W = Math.max(1, Math.round(rect.width * ratio));
  const H = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== W || canvas.height !== H) {
    canvas.width = W; canvas.height = H;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  return { ctx, W, H, ratio };
}

function axesBox(W, H) {
  // pixel padding for axes
  return { x0: 32, x1: W - 6, y0: 6, y1: H - 18 };
}

function clearPlot(ctx, W, H) {
  ctx.fillStyle = '#0e1116';
  ctx.fillRect(0, 0, W, H);
}

function drawGrid(ctx, box, xMin, xMax, yMin, yMax, opts = {}) {
  const { x0, x1, y0, y1 } = box;
  ctx.strokeStyle = '#222a35';
  ctx.lineWidth = 1;
  ctx.beginPath();
  // box outline
  ctx.rect(x0, y0, x1 - x0, y1 - y0);
  ctx.stroke();
  // zero line (y=0)
  if (yMin < 0 && yMax > 0) {
    const y = y1 - (0 - yMin) / (yMax - yMin) * (y1 - y0);
    ctx.beginPath();
    ctx.moveTo(x0, y);
    ctx.lineTo(x1, y);
    ctx.strokeStyle = '#3d4854';
    ctx.stroke();
  }
  // labels
  ctx.fillStyle = '#8b949e';
  ctx.font = `${10}px ui-monospace, Menlo, monospace`;
  ctx.textBaseline = 'middle';
  ctx.textAlign = 'right';
  ctx.fillText(yMax.toFixed(opts.yDigits ?? 1), x0 - 3, y0 + 5);
  ctx.fillText(yMin.toFixed(opts.yDigits ?? 1), x0 - 3, y1 - 5);
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillText(`${xMin.toFixed(1)}`, x0,     y1 + 2);
  ctx.textAlign = 'right';
  ctx.fillText(`${xMax.toFixed(1)}`, x1,     y1 + 2);
}

function plotLine(ctx, box, xs, ys, xMin, xMax, yMin, yMax, color) {
  const { x0, x1, y0, y1 } = box;
  const xSpan = Math.max(1e-9, xMax - xMin);
  const ySpan = Math.max(1e-9, yMax - yMin);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  let started = false;
  for (let i = 0; i < xs.length; i++) {
    const x = x0 + (xs[i] - xMin) / xSpan * (x1 - x0);
    const y = y1 - (ys[i] - yMin) / ySpan * (y1 - y0);
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function plotDashedHorz(ctx, box, yMin, yMax, yVal, color) {
  const { x0, x1, y0, y1 } = box;
  if (yVal < yMin || yVal > yMax) return;
  const y = y1 - (yVal - yMin) / (yMax - yMin) * (y1 - y0);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(x0, y); ctx.lineTo(x1, y);
  ctx.stroke();
  ctx.setLineDash([]);
}

// ---------------------- Public API ----------------------

const PLOTS = {};       // canvas id → element
let _bufAngles = null, _bufVels = null, _bufForce = null, _bufPhase = null;
let _sampleHz = 100;    // [Hz] sim-time sample rate (every 10 ms sim)
let _renderHz = 30;     // [Hz] wall-time render rate
let _windowSec = 10;    // rolling window in sim seconds
let _lastSampleT = -Infinity;
let _lastRenderMs = 0;
let _phaseCap = 600;    // bigger trail for the phase portrait

export function initPlots() {
  PLOTS.angles     = document.getElementById('plot-angles');
  PLOTS.velocities = document.getElementById('plot-velocities');
  PLOTS.phase      = document.getElementById('plot-phase');
  PLOTS.force      = document.getElementById('plot-force');
  const cap = Math.ceil(_windowSec * _sampleHz) + 4;
  _bufAngles = new TimeSeries(cap);
  _bufVels   = new TimeSeries(cap);
  _bufForce  = new TimeSeries(cap);
  _bufPhase  = new TimeSeries(_phaseCap);
  on('mode-change', clearBuffers);
  on('reset', clearBuffers);
}

function clearBuffers() {
  _bufAngles && _bufAngles.clear();
  _bufVels   && _bufVels.clear();
  _bufForce  && _bufForce.clear();
  _bufPhase  && _bufPhase.clear();
  _lastSampleT = -Infinity;
}

/** Push one sample at sim time t if enough sim time has elapsed since last. */
export function plotsSampleTick() {
  if (!_bufAngles) return;
  const t = state.t;
  if (t - _lastSampleT < 1 / _sampleHz) return;
  _lastSampleT = t;
  // angles (wrapped)
  const angs = [];
  const vels = [];
  for (let i = 1; i <= state.n; i++) {
    angs.push(wrap(state.q[i]));
    vels.push(state.qdot[i]);
  }
  _bufAngles.push(t, angs);
  _bufVels.push(t, vels);
  _bufForce.push(t, [state.u_cmd, state.u_applied]);
  _bufPhase.push(t, [wrap(state.q[1]), state.qdot[1]]);
}

/** Render all plots. Throttled to _renderHz. */
export function renderPlots(nowMs) {
  if (!_bufAngles) return;
  if (nowMs - _lastRenderMs < 1000 / _renderHz) return;
  _lastRenderMs = nowMs;
  drawAngles();
  drawVelocities();
  drawForce();
  drawPhase();
}

// ---------- Per-plot draw functions ----------

function drawAngles() {
  const cv = PLOTS.angles;
  const { ctx, W, H } = dpiFit(cv);
  clearPlot(ctx, W, H);
  const box = axesBox(W, H);
  const [tMin, tMax] = _bufAngles.trange();
  const tLo = Math.max(tMin, tMax - _windowSec);
  drawGrid(ctx, box, tLo, tMax, -Math.PI, Math.PI, { yDigits: 1 });

  // collect each channel
  const k = _bufAngles.y.length;
  if (k === 0) return;
  const xs = [], yChannels = Array.from({ length: k }, () => []);
  _bufAngles.forEach((t, row) => {
    if (t < tLo) return;
    xs.push(t);
    for (let j = 0; j < k; j++) yChannels[j].push(row[j]);
  });
  for (let j = 0; j < k; j++) {
    plotLine(ctx, box, xs, yChannels[j], tLo, tMax, -Math.PI, Math.PI, JOINT_COLORS[j % 3]);
  }
}

function drawVelocities() {
  const cv = PLOTS.velocities;
  const { ctx, W, H } = dpiFit(cv);
  clearPlot(ctx, W, H);
  const box = axesBox(W, H);
  const [tMin, tMax] = _bufVels.trange();
  const tLo = Math.max(tMin, tMax - _windowSec);

  // auto-y based on observed range, padded
  let yMin = -1, yMax = 1;
  _bufVels.forEach((t, row) => {
    if (t < tLo) return;
    for (const v of row) {
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
    }
  });
  const pad = 0.1 * Math.max(Math.abs(yMin), Math.abs(yMax), 1);
  yMin -= pad; yMax += pad;
  if (yMin === yMax) { yMin -= 1; yMax += 1; }
  drawGrid(ctx, box, tLo, tMax, yMin, yMax, { yDigits: 1 });

  const k = _bufVels.y.length;
  if (k === 0) return;
  const xs = [], chans = Array.from({ length: k }, () => []);
  _bufVels.forEach((t, row) => {
    if (t < tLo) return;
    xs.push(t);
    for (let j = 0; j < k; j++) chans[j].push(row[j]);
  });
  for (let j = 0; j < k; j++) {
    plotLine(ctx, box, xs, chans[j], tLo, tMax, yMin, yMax, JOINT_COLORS[j % 3]);
  }
}

function drawForce() {
  const cv = PLOTS.force;
  const { ctx, W, H } = dpiFit(cv);
  clearPlot(ctx, W, H);
  const box = axesBox(W, H);
  const [tMin, tMax] = _bufForce.trange();
  const tLo = Math.max(tMin, tMax - _windowSec);
  const F_max = state.params.F_max || 30;
  const yMax = F_max * 1.15;
  const yMin = -yMax;
  drawGrid(ctx, box, tLo, tMax, yMin, yMax, { yDigits: 0 });
  plotDashedHorz(ctx, box, yMin, yMax,  F_max, '#d29922');
  plotDashedHorz(ctx, box, yMin, yMax, -F_max, '#d29922');

  const xs = [], cmd = [], app = [];
  _bufForce.forEach((t, row) => {
    if (t < tLo) return;
    xs.push(t); cmd.push(row[0]); app.push(row[1]);
  });
  // applied = solid bright, cmd = thinner
  plotLine(ctx, box, xs, cmd, tLo, tMax, yMin, yMax, '#8b949e');
  plotLine(ctx, box, xs, app, tLo, tMax, yMin, yMax, '#f85149');
}

function drawPhase() {
  const cv = PLOTS.phase;
  const { ctx, W, H } = dpiFit(cv);
  clearPlot(ctx, W, H);
  const box = axesBox(W, H);
  // Auto-range over phase trail
  let yMin = -1, yMax = 1;
  _bufPhase.forEach((t, row) => {
    const v = row[1];
    if (v < yMin) yMin = v;
    if (v > yMax) yMax = v;
  });
  const pad = 0.1 * Math.max(Math.abs(yMin), Math.abs(yMax), 1);
  yMin -= pad; yMax += pad;
  if (yMin === yMax) { yMin -= 1; yMax += 1; }
  drawGrid(ctx, box, -Math.PI, Math.PI, yMin, yMax, { yDigits: 1 });

  // fading trail: older = darker, newer = lighter
  const { x0, x1, y0, y1 } = box;
  const xSpan = TWO_PI, ySpan = yMax - yMin;
  const N = _bufPhase.n;
  if (N === 0) return;
  let i = 0;
  _bufPhase.forEach((t, row) => {
    const alpha = (i + 1) / N;
    i++;
    const px = x0 + (row[0] - (-Math.PI)) / xSpan * (x1 - x0);
    const py = y1 - (row[1] - yMin) / ySpan * (y1 - y0);
    ctx.fillStyle = `rgba(88, 166, 255, ${(alpha * 0.9).toFixed(3)})`;
    ctx.beginPath();
    ctx.arc(px, py, 1.5, 0, TWO_PI);
    ctx.fill();
  });
  // emphasize the latest point
  const last_t_idx = (_bufPhase.head - 1 + _bufPhase.cap) % _bufPhase.cap;
  const lx = x0 + (_bufPhase.y[0][last_t_idx] - (-Math.PI)) / xSpan * (x1 - x0);
  const ly = y1 - (_bufPhase.y[1][last_t_idx] - yMin) / ySpan * (y1 - y0);
  ctx.fillStyle = '#f0883e';
  ctx.beginPath();
  ctx.arc(lx, ly, 3, 0, TWO_PI);
  ctx.fill();
}

/** Tweak helpers (used by panel / tests). */
export function setSampleHz(hz) { _sampleHz = Math.max(10, hz); }
export function setRenderHz(hz) { _renderHz = Math.max(5, hz);  }
export function setWindowSec(s) { _windowSec = Math.max(1, s);  }

// Test handles
export const _internal = {
  getBuffers: () => ({
    angles: _bufAngles, velocities: _bufVels, force: _bufForce, phase: _bufPhase
  }),
  TimeSeries,            // exposed for direct unit tests
  wrap,
};
