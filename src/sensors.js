// sensors.js — discrete sensor pipeline.
//
// One Sensor instance per measurable channel (cart x + each joint theta).
// On every "sample tick" the controller calls sensorTick(state); the sensor:
//   1. Reads the true state value
//   2. Adds Gaussian white noise (sigma from params)
//   3. Quantizes to a configurable bit depth over a configurable full range
//   4. Pushes the noisy/quantized value into a delay ring buffer
//   5. Pops the value from the head of the buffer (effective transport delay)
//
// Velocity is NOT measured directly — the controller estimates it via a filtered
// finite difference between consecutive samples. The filter is a single-pole IIR
// low-pass; cutoff is set so that the noise σ on the velocity estimate is small
// relative to the velocity itself.
//
// All RNGs use a seeded Mulberry32 so tests are reproducible.

import { state, on } from './state.js';

// ---------- Reproducible RNG ----------
function mulberry32(seed) {
  let s = seed >>> 0;
  return function () {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Box-Muller from uniform-[0,1) RNG
function gaussianFn(rng) {
  let cached = null;
  return function () {
    if (cached != null) { const r = cached; cached = null; return r; }
    let u1, u2;
    do { u1 = rng(); } while (u1 === 0);
    u2 = rng();
    const mag = Math.sqrt(-2 * Math.log(u1));
    cached = mag * Math.sin(2 * Math.PI * u2);
    return mag * Math.cos(2 * Math.PI * u2);
  };
}

// ---------- Quantization ----------
export function quantize(val, fullRange, bits) {
  if (!bits || bits <= 0) return val;
  const counts = (1 << bits);
  const lsb = fullRange / counts;
  return Math.round(val / lsb) * lsb;
}

// ---------- Sensor channel ----------
export class Sensor {
  constructor({ name, sigma, fullRange, bits, delaySec, rng } = {}) {
    this.name = name;
    this.sigma = sigma || 0;
    this.fullRange = fullRange || 1;
    this.bits = bits || 0;
    this.delaySec = delaySec || 0;
    this.gauss = gaussianFn(rng);
    // delay ring buffer: array of {t, value}; we drain entries whose age >= delaySec.
    this.buffer = [];
    this.lastSample = NaN;
    // velocity filter state
    this.lastVel = 0;
    this.lastSampleTime = null;
  }

  // Push the true state through the noise/quant pipeline and store with timestamp.
  push(trueValue, simTime) {
    const noisy = trueValue + this.sigma * this.gauss();
    const q = quantize(noisy, this.fullRange, this.bits);
    this.buffer.push({ t: simTime, value: q });
    // Trim head: anything older than delay can stay, but we drop entries > 2*delay
    // (small bound — keeps buffer length manageable).
    const cutoff = simTime - Math.max(this.delaySec * 4, 0.05);
    while (this.buffer.length > 1 && this.buffer[0].t < cutoff) this.buffer.shift();
  }

  // Read the value that should be visible NOW (= simTime - delaySec).
  read(simTime) {
    const targetT = simTime - this.delaySec;
    // Find newest sample with t <= targetT.
    let v = this.lastSample;
    for (let i = this.buffer.length - 1; i >= 0; i--) {
      if (this.buffer[i].t <= targetT + 1e-12) { v = this.buffer[i].value; break; }
    }
    if (!Number.isFinite(v)) v = 0;
    this.lastSample = v;
    return v;
  }

  // Filtered finite-difference velocity estimator. Call AFTER read() at every sample tick.
  // dt = sensor period. cutoff = filter cutoff frequency [rad/s]. simple one-pole IIR.
  // On first valid FD sample (after bootstrap), snap lastVel to raw to avoid a
  // multi-time-constant warm-up that would leave the controller blind to velocity
  // for the first ~1/cutoff seconds.
  updateVelocity(currentSample, dt, cutoff = 200) {
    if (this.lastSampleTime == null) {
      this.lastSampleTime = currentSample; // bootstrap, no velocity yet
      this.lastVel = 0;
      this._warmStart = true;
      return 0;
    }
    const raw = (currentSample - this.lastSampleTime) / dt;
    if (this._warmStart) {
      this.lastVel = raw;
      this._warmStart = false;
    } else {
      const alpha = (dt * cutoff) / (1 + dt * cutoff);
      this.lastVel = (1 - alpha) * this.lastVel + alpha * raw;
    }
    this.lastSampleTime = currentSample;
    return this.lastVel;
  }

  reset() {
    this.buffer = [];
    this.lastSample = NaN;
    this.lastVel = 0;
    this.lastSampleTime = null;
  }
}

// ---------- Bank wired to global state ----------
let _sensors = null;        // [cart, joint_1, ..., joint_n]
let _rng = null;
let _cutoff = 200;          // velocity LPF cutoff [rad/s] — high enough to track
                            // the fastest closed-loop pole on triple pendulum

function rebuild(n, seed) {
  _rng = mulberry32(seed >>> 0);
  const p = state.params;
  _sensors = [];
  // cart x — assume track range ±2 m (full range 4)
  _sensors.push(new Sensor({
    name: 'x', sigma: p.cart_noise, fullRange: 4.0, bits: p.quant_bits,
    delaySec: p.sensor_delay, rng: _rng,
  }));
  for (let i = 1; i <= n; i++) {
    // angle full range 2*pi
    _sensors.push(new Sensor({
      name: `theta_${i}`, sigma: p.angle_noise, fullRange: 2 * Math.PI,
      bits: p.quant_bits, delaySec: p.sensor_delay, rng: _rng,
    }));
  }
}

export function initSensors(seed) {
  rebuild(state.n, seed || state.params.seed || 12345);
  on('mode-change', () => rebuild(state.n, state.params.seed || 12345));
  on('reset', () => _sensors && _sensors.forEach(s => s.reset()));
}

// Called from the loop's sensor step at the sensor sample rate.
export function sensorTick() {
  if (!_sensors) return;
  const t = state.t;
  // push true values into each sensor
  for (let i = 0; i < _sensors.length; i++) {
    const trueVal = state.q[i];
    _sensors[i].push(trueVal, t);
  }
  // read delayed samples + update velocity estimates
  const dt = state.params.sensor_period;
  const sample = new Float64Array(state.n + 1);
  const vel    = new Float64Array(state.n + 1);
  for (let i = 0; i < _sensors.length; i++) {
    const v = _sensors[i].read(t);
    sample[i] = v;
    vel[i] = _sensors[i].updateVelocity(v, dt, _cutoff);
  }
  state.sensor_last = sample;
  state.sensor_vel_est = vel;
}

export function setVelocityCutoff(rps) { _cutoff = rps; }

// Test handles
export const _internal = {
  get sensors() { return _sensors; },
  mulberry32, gaussianFn,
};
