// tests/headless/actuator.test.js — saturation, slew, lag, Coulomb, noise.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Actuator, _internal } from '../../src/actuator.js';

const { mulberry32 } = _internal;

test('Saturation: u_cmd above F_max clips to F_max', () => {
  const a = new Actuator({ F_max: 20, slew_max: 1e9, motor_tau: 0, force_noise: 0, cart_coulomb: 0 });
  const out = a.step(100, 0.005, 0);
  assert.equal(out.u_applied, 20, `clipped to +F_max (got ${out.u_applied})`);
  const out2 = a.step(-200, 0.005, 0);
  assert.equal(out2.u_applied, -20, `clipped to -F_max`);
});

test('Slew rate limit: 0 → F_max with slew=100 N/s reaches F_max in F_max/100 s', () => {
  const F = 50, slew = 100;
  const a = new Actuator({ F_max: F, slew_max: slew, motor_tau: 0, force_noise: 0, cart_coulomb: 0 });
  const dt = 0.005;
  let u = 0;
  let elapsed = 0;
  for (let k = 0; k < 200; k++) {
    const out = a.step(F * 2, dt, 0);
    u = out.u_applied;
    elapsed += dt;
    if (u >= F - 1e-6) break;
  }
  // expected ~ F/slew = 0.5 s
  assert.ok(elapsed >= F/slew - 0.01 && elapsed <= F/slew + 0.02,
    `reaches F_max around ${F/slew}s (got ${elapsed}s)`);
});

test('First-order lag: step input → response time constant τ', () => {
  const tau = 0.02;     // 20 ms
  const a = new Actuator({ F_max: 1e6, slew_max: 1e9, motor_tau: tau, force_noise: 0, cart_coulomb: 0 });
  const dt = 1e-3;
  const target = 10;
  const samples = [];
  for (let k = 0; k < 200; k++) {
    const out = a.step(target, dt, 0);
    samples.push(out.u_applied);
  }
  // value at t = tau should be ≈ target*(1 - 1/e) = 0.632 * target
  const idxTau = Math.round(tau / dt);
  const v = samples[idxTau];
  assert.ok(Math.abs(v - 0.632 * target) < 0.5,
    `at t=tau u≈0.632*target (target=${target}, got ${v.toFixed(3)})`);
  // converges
  const vFinal = samples[samples.length - 1];
  assert.ok(Math.abs(vFinal - target) < 0.05, `converges to target (got ${vFinal})`);
});

test('Coulomb friction opposes cart motion', () => {
  const a = new Actuator({ F_max: 1e6, slew_max: 1e9, motor_tau: 0, force_noise: 0, cart_coulomb: 5 });
  // zero command, cart moving +0.5 m/s
  const out_pos = a.step(0, 1e-3, +0.5);
  assert.ok(out_pos.u_effective < -4.5, `u_effective opposes +xdot (got ${out_pos.u_effective.toFixed(3)})`);
  const out_neg = a.step(0, 1e-3, -0.5);
  assert.ok(out_neg.u_effective > 4.5, `u_effective opposes -xdot (got ${out_neg.u_effective.toFixed(3)})`);
});

test('Coulomb friction smooth near xdot=0 (no chatter)', () => {
  const a = new Actuator({ F_max: 1e6, slew_max: 1e9, motor_tau: 0, force_noise: 0, cart_coulomb: 5 });
  const out = a.step(0, 1e-3, 0);
  assert.ok(Math.abs(out.f_coulomb) < 0.01, `f_coulomb ~0 at xdot=0 (got ${out.f_coulomb})`);
});

test('Force noise: σ_noise statistics match spec', () => {
  const sigma = 2.0;
  const rng = mulberry32(7);
  const a = new Actuator({ F_max: 1e6, slew_max: 1e9, motor_tau: 0, force_noise: sigma, cart_coulomb: 0, rng });
  const N = 5000;
  let sum = 0, sumSq = 0;
  for (let k = 0; k < N; k++) {
    const out = a.step(10, 1e-3, 0);
    const noise = out.u_with_noise - out.u_applied;
    sum += noise; sumSq += noise * noise;
  }
  const mean = sum/N, std = Math.sqrt(sumSq/N - mean*mean);
  assert.ok(Math.abs(mean) < 0.1, `noise mean ~0 (got ${mean.toFixed(3)})`);
  assert.ok(Math.abs(std - sigma) < 0.1, `noise std ~σ (got ${std.toFixed(3)})`);
});

test('Sat + slew + lag interplay: u_cmd = +inf for many steps converges to F_max', () => {
  const a = new Actuator({ F_max: 30, slew_max: 1e9, motor_tau: 0.005, force_noise: 0, cart_coulomb: 0 });
  let last = 0;
  for (let k = 0; k < 200; k++) last = a.step(1e6, 1e-3, 0).u_applied;
  assert.ok(Math.abs(last - 30) < 0.01, `converges to F_max (got ${last})`);
});

test('Reset clears internal state', () => {
  const a = new Actuator({ F_max: 30, slew_max: 1e9, motor_tau: 0.005, force_noise: 0, cart_coulomb: 0 });
  a.step(30, 1e-3, 0); a.step(30, 1e-3, 0);
  assert.ok(a.u_applied > 0);
  a.reset();
  assert.equal(a.u_applied, 0);
  assert.equal(a.u_pre_lag, 0);
});
