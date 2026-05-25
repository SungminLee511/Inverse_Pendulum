// tests/headless/lqr_nonlinear_n2.test.js — closed-loop LQR on the full
// nonlinear n=2 EOM. Bypasses sensors/actuator/loop — direct integrator +
// feedback.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_2.js';
import { stepRK4 } from '../../src/physics/integrator.js';
import { linearize, solveCARE } from '../../src/control/lqr.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0.05,
  links: [
    { m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.15, L: 0.4, l: 0.2, I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
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

// Run closed-loop sim from initial (q0, qdot0). Returns trajectory snapshots.
function runClosedLoop(K, q0, qdot0, T, F_max = 50) {
  let q = q0.slice(), qdot = qdot0.slice();
  const dt = 1e-4;
  const N = Math.round(T / dt);
  let max_abs_theta = 0;
  for (let k = 0; k < N; k++) {
    // wrap angles to (-π, π]
    const x = [q[0], wrap(q[1]), wrap(q[2]),
               qdot[0], qdot[1], qdot[2]];
    let u = 0;
    for (let i = 0; i < 6; i++) u -= K[i] * x[i];
    u = Math.max(-F_max, Math.min(F_max, u));
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
    max_abs_theta = Math.max(max_abs_theta,
      Math.max(Math.abs(wrap(q[1])), Math.abs(wrap(q[2]))));
  }
  return { q, qdot, max_abs_theta };
}

test('n=2 LQR stabilises θ_1=0.05, θ_2=0.05 perturbation within 6 s', () => {
  const { A, B } = linearize(2, params);
  const Q = diag([10, 200, 200, 1, 20, 20]);
  const R = 0.1;
  const { K } = solveCARE(A, B, Q, R);
  console.log('K(n=2) =', K.map(v => v.toFixed(3)));
  const r = runClosedLoop(K, [0, 0.05, 0.05], [0, 0, 0], 6);
  assert.ok(Math.abs(r.q[1]) < 0.02, `θ_1 → ~0 (got ${r.q[1].toExponential(2)})`);
  assert.ok(Math.abs(r.q[2]) < 0.02, `θ_2 → ~0 (got ${r.q[2].toExponential(2)})`);
});

test('n=2 LQR keeps peak |θ| bounded < 0.5 rad during a 0.1 rad pulse', () => {
  const { A, B } = linearize(2, params);
  const Q = diag([10, 200, 200, 1, 20, 20]);
  const R = 0.1;
  const { K } = solveCARE(A, B, Q, R);
  const r = runClosedLoop(K, [0, 0.1, -0.05], [0, 0, 0], 4);
  assert.ok(r.max_abs_theta < 0.5, `peak |θ| < 0.5 (got ${r.max_abs_theta.toFixed(3)})`);
});

test('n=2 LQR closed-loop has no NaN over 5 s under modest perturbation', () => {
  const { A, B } = linearize(2, params);
  const Q = diag([10, 200, 200, 1, 20, 20]);
  const R = 0.1;
  const { K } = solveCARE(A, B, Q, R);
  const r = runClosedLoop(K, [0.2, 0.1, -0.08], [0, 0, 0], 5);
  assert.ok(Number.isFinite(r.q[0]) && Number.isFinite(r.q[1]) && Number.isFinite(r.q[2]),
    `q finite (${r.q})`);
});

test('n=2 LQR: heavier R reduces ||K||', () => {
  const { A, B } = linearize(2, params);
  const Q = diag([10, 200, 200, 1, 20, 20]);
  const K_light = solveCARE(A, B, Q, 0.05).K;
  const K_heavy = solveCARE(A, B, Q, 0.5).K;
  const nrm = (K) => Math.sqrt(K.reduce((s, k) => s + k * k, 0));
  assert.ok(nrm(K_heavy) < nrm(K_light),
    `||K(R=0.5)|| < ||K(R=0.05)|| (${nrm(K_heavy).toFixed(2)} < ${nrm(K_light).toFixed(2)})`);
});

test('n=2 LQR: higher Q[θ_1] → larger |K[1]|', () => {
  const { A, B } = linearize(2, params);
  const K_low  = solveCARE(A, B, diag([10, 10,    200, 1, 20, 20]), 0.1).K;
  const K_high = solveCARE(A, B, diag([10, 50000, 200, 1, 20, 20]), 0.1).K;
  assert.ok(Math.abs(K_high[1]) > Math.abs(K_low[1]) * 1.5,
    `|K[1]| grows with Q[θ_1] (${K_low[1].toFixed(2)} → ${K_high[1].toFixed(2)})`);
});
