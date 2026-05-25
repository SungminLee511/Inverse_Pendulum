// tests/headless/lqr_nonlinear_n3.test.js — closed-loop LQR on the full
// nonlinear n=3 EOM.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_3.js';
import { stepRK4 } from '../../src/physics/integrator.js';
import { linearize, solveCARE } from '../../src/control/lqr.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0.05,
  links: [
    { m: 0.2,  L: 0.5, l: 0.25, I: 0.2  * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.15, L: 0.4, l: 0.2,  I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
    { m: 0.1,  L: 0.3, l: 0.15, I: 0.1  * 0.3 * 0.3 / 12, joint_viscous: 0.001 },
  ],
};

function diag(arr) {
  return Array.from({ length: arr.length }, (_, i) =>
    Array.from({ length: arr.length }, (_, j) => i === j ? arr[i] : 0));
}

function wrap(a) {
  let x = a;
  while (x >  Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
}

function runClosedLoop(K, q0, qdot0, T, F_max = 60, params_run = params) {
  let q = q0.slice(), qdot = qdot0.slice();
  const dt = 1e-4;
  const N = Math.round(T / dt);
  for (let k = 0; k < N; k++) {
    const x = [q[0], wrap(q[1]), wrap(q[2]), wrap(q[3]),
               qdot[0], qdot[1], qdot[2], qdot[3]];
    let u = 0;
    for (let i = 0; i < 8; i++) u -= K[i] * x[i];
    u = Math.max(-F_max, Math.min(F_max, u));
    [q, qdot] = stepRK4(q, qdot, u, dt, params_run, eom);
    if (!Number.isFinite(q[0]) || !Number.isFinite(q[1])) {
      return { q, qdot, diverged: true };
    }
  }
  return { q, qdot, diverged: false };
}

function defaultK(p) {
  const { A, B } = linearize(3, p);
  const Q = diag([10, 600, 600, 600, 1, 30, 30, 30]);
  const R = 0.05;
  return solveCARE(A, B, Q, R).K;
}

test('n=3 LQR: linearize → CARE → K finite and length 8', () => {
  const K = defaultK(params);
  assert.equal(K.length, 8);
  for (const k of K) assert.ok(Number.isFinite(k), `K entry finite (${k})`);
});

test('n=3 LQR stabilises a small (0.03, 0.03, 0.03) perturbation in 6 s', () => {
  const K = defaultK(params);
  const r = runClosedLoop(K, [0, 0.03, 0.03, 0.03], [0, 0, 0, 0], 6, 60);
  assert.ok(!r.diverged, 'no divergence');
  for (let i = 1; i < 4; i++)
    assert.ok(Math.abs(wrap(r.q[i])) < 0.05,
      `final |θ_${i}| < 0.05 rad (got ${wrap(r.q[i]).toFixed(4)})`);
});

test('n=3 LQR: heavier R reduces ||K||', () => {
  const { A, B } = linearize(3, params);
  const Q = diag([10, 600, 600, 600, 1, 30, 30, 30]);
  const K_light = solveCARE(A, B, Q, 0.02).K;
  const K_heavy = solveCARE(A, B, Q, 0.5).K;
  const nrm = (K) => Math.sqrt(K.reduce((s, k) => s + k * k, 0));
  assert.ok(nrm(K_heavy) < nrm(K_light),
    `||K(R=0.5)|| < ||K(R=0.02)|| (${nrm(K_heavy).toFixed(2)} < ${nrm(K_light).toFixed(2)})`);
});

test('n=3 LQR: higher Q[θ_3] → larger |K[3]|', () => {
  const { A, B } = linearize(3, params);
  const K_low  = solveCARE(A, B, diag([10, 600, 600, 50,    1, 30, 30, 30]), 0.05).K;
  const K_high = solveCARE(A, B, diag([10, 600, 600, 50000, 1, 30, 30, 30]), 0.05).K;
  assert.ok(Math.abs(K_high[3]) > Math.abs(K_low[3]) * 1.5,
    `|K[3]| grows with Q[θ_3] (${K_low[3].toFixed(2)} → ${K_high[3].toFixed(2)})`);
});
