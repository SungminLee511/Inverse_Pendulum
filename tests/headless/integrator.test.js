// tests/headless/integrator.test.js — Euler / SI Euler / RK4 correctness.
// 1. Energy conservation (no friction, no input)
// 2. Order-of-accuracy via Richardson convergence
// 3. Cross-integrator sanity: same problem, RK4 < SI Euler < Euler in energy drift

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_1.js';
import { step, totalEnergy } from '../../src/physics/integrator.js';

const noFricParams = {
  m0: 1.0, g: 9.81, cart_visc: 0,
  links: [{ m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0 }],
};

function run(method, q0, qd0, dt, T, params = noFricParams) {
  let q = q0.slice(), qd = qd0.slice();
  const Esamples = [];
  const N = Math.round(T / dt);
  for (let k = 0; k <= N; k++) {
    if (k % Math.max(1, Math.floor(N / 100)) === 0) {
      Esamples.push(totalEnergy(q, qd, params));
    }
    if (k === N) break;
    const [qn, qdn] = step(method, q, qd, 0.0, dt, params, eom);
    q = qn; qd = qdn;
  }
  return { q, qd, Esamples };
}

test('RK4 conserves energy < 0.1% over 10 s (n=1, no friction, no input)', () => {
  // Initial: pendulum hanging at ~30 deg, small velocity. Free oscillation.
  const theta0 = Math.PI - 0.5;   // 0.5 rad below upright = nearly hanging
  const r = run('rk4', [0, theta0], [0, 0], 1e-4, 10);
  const Emax = Math.max(...r.Esamples), Emin = Math.min(...r.Esamples);
  const drift = Math.abs(Emax - Emin) / Math.max(Math.abs(Emax), Math.abs(Emin), 1e-12);
  assert.ok(drift < 1e-3, `RK4 energy drift ${drift} < 1e-3`);
});

test('Semi-implicit Euler energy drift < 5% over 10 s', () => {
  const r = run('si_euler', [0, Math.PI - 0.5], [0, 0], 1e-4, 10);
  const Emax = Math.max(...r.Esamples), Emin = Math.min(...r.Esamples);
  const drift = Math.abs(Emax - Emin) / Math.max(Math.abs(Emax), Math.abs(Emin), 1e-12);
  assert.ok(drift < 0.05, `SI Euler drift ${drift} < 5%`);
});

test('Forward Euler drifts much more than RK4 (sanity check)', () => {
  const rEul = run('euler', [0, Math.PI - 0.5], [0, 0], 1e-3, 5);
  const rRk4 = run('rk4',   [0, Math.PI - 0.5], [0, 0], 1e-3, 5);

  const drift = arr => {
    const mx = Math.max(...arr), mn = Math.min(...arr);
    return Math.abs(mx - mn);
  };
  const dE = drift(rEul.Esamples);
  const dR = drift(rRk4.Esamples);
  assert.ok(dE > dR * 5, `Euler drift (${dE.toExponential(2)}) > 5x RK4 drift (${dR.toExponential(2)})`);
});

test('RK4 is O(dt^4): halving dt drops error by ~16x', () => {
  // Compute the "true" solution with a very small dt RK4, then measure error
  // at finite dts. Order-of-accuracy = log2(err_h / err_h/2).
  const T = 1.0;
  const q0 = [0, Math.PI - 0.5], qd0 = [0, 1.5];
  const refDt = 1e-5;
  const refSteps = Math.round(T / refDt);
  let qR = q0.slice(), qdR = qd0.slice();
  for (let k = 0; k < refSteps; k++) {
    [qR, qdR] = step('rk4', qR, qdR, 0, refDt, noFricParams, eom);
  }
  const refTheta = qR[1];

  function errAtDt(dt) {
    let q = q0.slice(), qd = qd0.slice();
    const N = Math.round(T / dt);
    for (let k = 0; k < N; k++) {
      [q, qd] = step('rk4', q, qd, 0, dt, noFricParams, eom);
    }
    return Math.abs(q[1] - refTheta);
  }
  const e1 = errAtDt(2e-3);
  const e2 = errAtDt(1e-3);
  const ratio = e1 / e2;
  // Theoretical ratio = 16; allow a wide tolerance because near zero error the ratio is noisy.
  assert.ok(ratio > 8, `RK4 order ratio ${ratio.toFixed(2)} > 8 (theory: 16)`);
});

test('SI Euler is O(dt): halving dt drops error by ~2x', () => {
  const T = 1.0;
  const q0 = [0, Math.PI - 0.5], qd0 = [0, 1.5];
  // reference via RK4 small dt
  const refDt = 1e-5;
  let qR = q0.slice(), qdR = qd0.slice();
  const refSteps = Math.round(T / refDt);
  for (let k = 0; k < refSteps; k++) {
    [qR, qdR] = step('rk4', qR, qdR, 0, refDt, noFricParams, eom);
  }
  const refTheta = qR[1];

  function errAtDt(dt) {
    let q = q0.slice(), qd = qd0.slice();
    const N = Math.round(T / dt);
    for (let k = 0; k < N; k++) {
      [q, qd] = step('si_euler', q, qd, 0, dt, noFricParams, eom);
    }
    return Math.abs(q[1] - refTheta);
  }
  const e1 = errAtDt(1e-3);
  const e2 = errAtDt(5e-4);
  const ratio = e1 / e2;
  assert.ok(ratio > 1.5 && ratio < 4.0,
    `SI Euler order ratio ${ratio.toFixed(2)} in (1.5, 4) — theory ~2`);
});

test('Friction dissipates energy monotonically', () => {
  const frParams = { ...noFricParams, cart_visc: 0.5,
    links: [{ ...noFricParams.links[0], joint_viscous: 0.05 }] };
  const r = run('rk4', [0, Math.PI - 1.0], [0, 0], 1e-4, 5, frParams);
  const E0 = r.Esamples[0];
  const E_end = r.Esamples[r.Esamples.length - 1];
  assert.ok(E_end < E0, `Final energy ${E_end.toFixed(4)} < initial ${E0.toFixed(4)}`);
  // monotone-ish: each successive sample should be <= previous (allow tiny RK4 jitter)
  let nonMono = 0;
  for (let i = 1; i < r.Esamples.length; i++) {
    if (r.Esamples[i] > r.Esamples[i - 1] + 1e-6) nonMono++;
  }
  assert.ok(nonMono <= 1, `energy mostly monotone with friction (jitter ${nonMono})`);
});
