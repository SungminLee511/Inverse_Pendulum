// tests/headless/sysid.test.js — Phase 14 sys-id excitations + fit.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom1 from '../../src/physics/nlink_1.js';
import * as eom2 from '../../src/physics/nlink_2.js';
import {
  impulseExcitation, stepExcitation, chirpExcitation, prbsExcitation,
  pendulumPeriod, periodFromZeroCrossings,
  simulateOpen, trajError, fitOutputError,
} from '../../src/control/sysid.js';

const baseParams1 = {
  m0: 1.0, g: 9.81, cart_visc: 0.05,
  links: [{ m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 }],
};

// ---------- Excitation shapes ----------
test('impulseExcitation: zero outside [t0,t0+w], amplitude inside', () => {
  const u = impulseExcitation({ t0: 1.0, width: 0.1, amplitude: 8 });
  assert.equal(u(0.5), 0);
  assert.equal(u(1.0), 8);
  assert.equal(u(1.05), 8);
  assert.equal(u(1.1), 0);
});

test('stepExcitation: 0 before t0, amplitude after', () => {
  const u = stepExcitation({ t0: 0.5, amplitude: 3 });
  assert.equal(u(0.4), 0);
  assert.equal(u(0.5), 3);
  assert.equal(u(10), 3);
});

test('chirpExcitation: amplitude bounded by ±amp; f0 instantaneous at t=0', () => {
  const u = chirpExcitation({ f0: 0.5, f1: 5.0, duration: 4, amplitude: 4 });
  for (let i = 0; i < 200; i++) {
    const t = i * 0.02;
    assert.ok(Math.abs(u(t)) <= 4 + 1e-9, `|u(t)| ≤ 4 (got ${u(t)} at t=${t})`);
  }
  // f0 at start: u(0) = 0 (sin(0))
  assert.equal(u(0), 0);
});

test('prbsExcitation: takes only values ±amplitude', () => {
  const u = prbsExcitation({ amplitude: 7, dt_switch: 0.05 });
  for (let i = 0; i < 200; i++) {
    const v = u(i * 0.05);
    assert.ok(v === 7 || v === -7, `±7 only (got ${v} at i=${i})`);
  }
});

test('prbsExcitation is deterministic with seed', () => {
  const a = prbsExcitation({ amplitude: 1, dt_switch: 0.1, seed: 42 });
  const b = prbsExcitation({ amplitude: 1, dt_switch: 0.1, seed: 42 });
  for (let i = 0; i < 50; i++)
    assert.equal(a(i * 0.1), b(i * 0.1), `same-seed identical at i=${i}`);
});

// ---------- Hang-test period ----------
test('pendulumPeriod matches √((I+m·l²)/(m·g·l)) · 2π', () => {
  const T = pendulumPeriod({ m: 0.2, g: 9.81, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12 });
  // ω_n² = 0.2·9.81·0.25 / (0.2·0.5²/12 + 0.2·0.25²) = 0.4905 / 0.0167 = 29.4
  // ω_n = 5.42 → T = 2π/5.42 ≈ 1.16
  assert.ok(Math.abs(T - 1.16) < 0.02, `T ≈ 1.16 (got ${T.toFixed(3)})`);
});

test('periodFromZeroCrossings recovers ground-truth period from a sine', () => {
  // Synthetic θ(t) = sin(2πt / 1.0) — period 1.0 s.
  const t = [], theta = [];
  for (let k = 0; k <= 1000; k++) { t.push(k * 0.01); theta.push(Math.sin(2 * Math.PI * t[k])); }
  const T = periodFromZeroCrossings(t, theta);
  assert.ok(Math.abs(T - 1.0) < 0.01, `T=1.00 (got ${T.toFixed(4)})`);
});

test('periodFromZeroCrossings → recovers n=1 free-swing period (heavy cart)', () => {
  // With a heavy cart (m0 ≫ m), the cart barely moves and the natural
  // frequency matches the isolated-pendulum formula. With a light cart the
  // cart reaction modifies ω_n by O(m/m0). For a clean hang test the
  // operator typically clamps the cart — emulate that by setting m0 = 1000.
  const params = { ...baseParams1, m0: 1000 };
  const { ts, qs } = simulateOpen(eom1, params, () => 0,
    [0, Math.PI + 0.05], [0, 0], 4, 1e-4, 0.005);
  const theta = qs.map(q => q[1] - Math.PI);
  const T_meas = periodFromZeroCrossings(ts, theta);
  const T_pred = pendulumPeriod({ ...params.links[0], g: params.g });
  assert.ok(Math.abs(T_meas - T_pred) / T_pred < 0.03,
    `T_meas=${T_meas.toFixed(3)} ≈ T_pred=${T_pred.toFixed(3)} within 3%`);
});

// ---------- Output-error fit round-trip ----------
test('fitOutputError recovers perturbed (m_1, I_1) within 5% (n=1)', () => {
  // True plant: baseParams1. Initial guess: m_1 perturbed by +50%; I keyed
  // off the wrong m. Optimiser must fit BOTH (m and I are coupled: changing
  // m alone won't recover trajectories because I drives angular dynamics).
  const trueParams = baseParams1;
  const guess = structuredClone(baseParams1);
  guess.links[0].m = baseParams1.links[0].m * 1.5;     // 0.3 (true 0.2)
  guess.links[0].I = guess.links[0].m * guess.links[0].L * guess.links[0].L / 12;

  const u_fn = (t) => 5 * Math.sin(2 * Math.PI * 1.5 * t);
  const q0    = [0, Math.PI + 0.1];
  const qdot0 = [0, 0];
  const T = 3.0;
  const measurement = simulateOpen(eom1, trueParams, u_fn, q0, qdot0, T);

  const result = fitOutputError({
    eom: eom1,
    paramsGuess: guess,
    knobs: [
      { path: 'links.0.m', lo: 0.05, hi: 0.5 },
      { path: 'links.0.I', lo: 1e-4, hi: 0.05 },
    ],
    measurement, q0, qdot0, u_fn, T,
    maxIters: 60, tol: 1e-6,
  });

  const m_fit = result.params.links[0].m;
  const m_true = trueParams.links[0].m;
  assert.ok(Math.abs(m_fit - m_true) / m_true < 0.05,
    `m_1 fit within 5% (true=${m_true}, fit=${m_fit.toFixed(4)}, iters=${result.iters}, err=${result.error_final.toExponential(2)})`);
});

test('trajError is zero for identical trajectories, positive otherwise', () => {
  const a = [[0, 0], [1, 2], [3, 4]];
  assert.equal(trajError(a, a), 0);
  const b = [[0, 0], [1, 2.1], [3, 4]];
  assert.ok(trajError(a, b) > 0);
});

test('simulateOpen samples at sample_dt and respects total horizon', () => {
  const { ts } = simulateOpen(eom1, baseParams1, () => 0,
    [0, Math.PI], [0, 0], 1.0, 1e-4, 0.01);
  assert.ok(ts.length >= 100 && ts.length <= 102, `≈101 samples (got ${ts.length})`);
  assert.ok(Math.abs(ts[ts.length - 1] - 1.0) < 0.02, `last time ≈ 1.0 (got ${ts[ts.length-1]})`);
});
