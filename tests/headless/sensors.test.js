// tests/headless/sensors.test.js — sensor pipeline assertions.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Sensor, quantize, _internal } from '../../src/sensors.js';

const { mulberry32, gaussianFn } = _internal;

test('quantize: rounds to nearest LSB and clamps to multiples', () => {
  // 4-bit over 1.0 -> LSB = 1/16 = 0.0625
  assert.equal(quantize(0.30, 1, 4), 0.3125);
  assert.equal(quantize(0.50, 1, 4), 0.5);
  assert.equal(quantize(-0.30, 1, 4), -0.3125);
  // 12-bit over 2*pi -> LSB ~ 0.001534
  const v = quantize(1.234567, 2 * Math.PI, 12);
  const lsb = (2 * Math.PI) / 4096;
  assert.ok(Math.abs(v - Math.round(1.234567 / lsb) * lsb) < 1e-12);
});

test('Gaussian noise: sample mean ~ 0, std ~ sigma over N=20k', () => {
  const rng = mulberry32(42);
  const g = gaussianFn(rng);
  const N = 20000;
  let sum = 0, sumSq = 0;
  for (let i = 0; i < N; i++) { const x = g(); sum += x; sumSq += x*x; }
  const mean = sum / N;
  const std = Math.sqrt(sumSq / N - mean * mean);
  assert.ok(Math.abs(mean) < 0.05, `mean ~ 0 (got ${mean.toFixed(4)})`);
  assert.ok(Math.abs(std - 1) < 0.05, `std ~ 1 (got ${std.toFixed(4)})`);
});

test('Sensor: delay buffer returns value from delaySec ago', () => {
  const rng = mulberry32(1);
  const s = new Sensor({ name: 'theta', sigma: 0, fullRange: 2*Math.PI, bits: 0,
                          delaySec: 0.005, rng });
  // simulate samples every 1ms with linearly ramping value
  for (let k = 0; k < 50; k++) {
    const t = k * 0.001;
    s.push(t * 10, t);   // value = 10 * t
  }
  // After t = 0.049 the read at delay 0.005 should approximate 10 * (0.049 - 0.005) = 0.44
  const v = s.read(0.049);
  assert.ok(Math.abs(v - 0.44) < 1e-2, `delayed sample ~ 0.44 (got ${v})`);
});

test('Sensor: at delaySec = 0, read returns the latest push', () => {
  const rng = mulberry32(7);
  const s = new Sensor({ name: 'x', sigma: 0, fullRange: 4, bits: 0, delaySec: 0, rng });
  for (let k = 0; k < 10; k++) s.push(k * 0.1, k * 0.001);
  const v = s.read(0.009);
  assert.ok(Math.abs(v - 0.9) < 1e-9, `latest sample (got ${v})`);
});

test('Sensor: noise statistics — many samples have std near sigma', () => {
  const rng = mulberry32(123);
  const sigma = 0.05;
  const s = new Sensor({ name: 'x', sigma, fullRange: 4, bits: 0, delaySec: 0, rng });
  const N = 10000;
  const true_val = 1.0;
  let sum = 0, sumSq = 0;
  for (let k = 0; k < N; k++) { s.push(true_val, k * 1e-3); const r = s.read(k * 1e-3); sum += r; sumSq += r*r; }
  const mean = sum/N, std = Math.sqrt(sumSq/N - mean*mean);
  assert.ok(Math.abs(mean - true_val) < 0.005, `mean ~ ${true_val}, got ${mean.toFixed(4)}`);
  assert.ok(Math.abs(std - sigma) < 0.005, `std ~ ${sigma}, got ${std.toFixed(4)}`);
});

test('Sensor: quantization is applied (visible LSB granularity)', () => {
  const rng = mulberry32(9);
  // 4-bit over 1 -> LSB = 0.0625
  const s = new Sensor({ name: 'x', sigma: 0, fullRange: 1, bits: 4, delaySec: 0, rng });
  s.push(0.3, 0);
  assert.equal(s.read(0), 0.3125);
});

test('Filtered velocity estimator: tracks ramp velocity 5 m/s', () => {
  const rng = mulberry32(11);
  const s = new Sensor({ name: 'x', sigma: 0, fullRange: 4, bits: 0, delaySec: 0, rng });
  const dt = 0.002;
  let v_est = 0;
  for (let k = 0; k < 600; k++) {
    const t = k * dt;
    const trueVal = 5 * t;        // 5 m/s ramp
    s.push(trueVal, t);
    const r = s.read(t);
    v_est = s.updateVelocity(r, dt, 50);
  }
  assert.ok(Math.abs(v_est - 5) < 0.05, `velocity estimate ~5 (got ${v_est.toFixed(3)})`);
});

test('Filtered velocity estimator: rejects noise on stationary input', () => {
  const rng = mulberry32(13);
  const sigma = 0.005;
  const s = new Sensor({ name: 'x', sigma, fullRange: 4, bits: 0, delaySec: 0, rng });
  const dt = 0.002;
  let v_est = 0;
  // Warm-up the filter
  for (let k = 0; k < 500; k++) {
    const t = k * dt;
    s.push(0.0, t);
    const r = s.read(t);
    v_est = s.updateVelocity(r, dt, 30);
  }
  // Raw FD noise std ≈ sigma * sqrt(2)/dt = 0.005 * 1.414 / 0.002 ≈ 3.5
  // After filter at cutoff 30 rad/s with dt=2ms, attenuation factor ≈ alpha*sqrt(N)
  // We accept anything < 1.0 (well below raw FD noise).
  assert.ok(Math.abs(v_est) < 1.0, `filtered velocity noise small (got ${v_est})`);
});

test('Reproducibility: same seed → identical noisy sample stream', () => {
  const rng1 = mulberry32(99);
  const rng2 = mulberry32(99);
  const a = new Sensor({ name: 'x', sigma: 0.1, fullRange: 4, bits: 0, delaySec: 0, rng: rng1 });
  const b = new Sensor({ name: 'x', sigma: 0.1, fullRange: 4, bits: 0, delaySec: 0, rng: rng2 });
  for (let k = 0; k < 100; k++) {
    a.push(0, k * 1e-3); b.push(0, k * 1e-3);
    assert.equal(a.read(k * 1e-3), b.read(k * 1e-3));
  }
});
