// tests/headless/swingup_n1.test.js — energy-based swing-up + switcher (n=1).
//
// Runs the closed-loop sim (no sensors, no actuator chain) directly through the
// integrator and the swingup/switcher logic. Tests:
//   1. From hanging (θ=π) with no friction, the swing-up pumps energy until
//      the ROA is reached within 30 s.
//   2. The switcher hands over to LQR and the pendulum stabilises within an
//      extra 10 s with final |θ| < 5°.
//   3. swingupDiag reports E*, E, Ẽ at the upright eq (E ≈ E*).

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_1.js';
import { stepRK4, totalEnergy } from '../../src/physics/integrator.js';
import { linearize, solveCARE } from '../../src/control/lqr.js';
import { swingupU, swingupDiag, resetSwingup } from '../../src/control/swingup.js';
import { HandoverSwitcher } from '../../src/control/switcher.js';

const baseParams = {
  m0: 1.0, g: 9.81, cart_visc: 0.05, cart_coulomb: 0,
  links: [{ m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 }],
  F_max: 12,
  // Let swingup.js use its tuned defaults (k_E=80, k_xP=0.6, k_xD=0.8).
  handover_theta: 0.35,
  handover_omega: 2.5,
  handover_blend_ms: 80,
};

function diag(arr) {
  return Array.from({ length: arr.length }, (_, i) =>
    Array.from({ length: arr.length }, (_, j) => i === j ? arr[i] : 0));
}

function wrap(a) {
  let x = a;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
}

function lqrK(params) {
  const { A, B } = linearize(1, params);
  const Q = diag([10, 100, 1, 10]);
  const R = 0.05;
  return solveCARE(A, B, Q, R).K;
}

test('swingup pumps energy from hanging into ROA within 30 s (n=1)', () => {
  resetSwingup();
  const params = { ...baseParams };
  let q = [0, Math.PI];
  let qdot = [0, 0];
  const switcher = new HandoverSwitcher();
  const dt = 1e-4;
  const T = 30;
  const N = Math.round(T / dt);
  let t_enter = -1;
  for (let k = 0; k < N; k++) {
    const t = k * dt;
    const u = swingupU(1, q, qdot, params, t);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
    if (switcher.inROA(q, qdot, params)) {
      t_enter = t;
      break;
    }
  }
  assert.ok(t_enter > 0 && t_enter < 30,
    `entered ROA at t=${t_enter.toFixed(2)} s (< 30 s)`);
});

test('swingup → handover blend → LQR drives θ to 0 within 40 s', () => {
  resetSwingup();
  const params = { ...baseParams };
  const K = lqrK(params);
  let q = [0, Math.PI];
  let qdot = [0, 0];
  const switcher = new HandoverSwitcher();
  const dt = 1e-4;
  const T = 40;
  const N = Math.round(T / dt);
  let final_theta = q[1];
  for (let k = 0; k < N; k++) {
    const t = k * dt;
    const u_swing = () => swingupU(1, q, qdot, params, t);
    const u_lqr = () => {
      const x = [q[0], wrap(q[1]), qdot[0], qdot[1]];
      let u = 0;
      for (let i = 0; i < 4; i++) u -= K[i] * x[i];
      return Math.max(-params.F_max, Math.min(params.F_max, u));
    };
    const u = switcher.mix(t, q, qdot, params, u_swing, u_lqr);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
    final_theta = wrap(q[1]);
  }
  assert.ok(Math.abs(final_theta) < 0.087, // 5 deg
    `final |θ| < 5° (got ${(final_theta * 180 / Math.PI).toFixed(2)}°)`);
});

test('swingupDiag at upright equilibrium reports E ≈ E*', () => {
  const params = { ...baseParams };
  const d = swingupDiag(1, [0, 0], [0, 0], params);
  assert.ok(Math.abs(d.E - d.E_star) < 1e-9,
    `E (${d.E}) ≈ E* (${d.E_star}) at upright`);
  // Hanging should have E = -E* (since reference is upright)
  const d2 = swingupDiag(1, [0, Math.PI], [0, 0], params);
  // E at hanging = m g (-l). E* = m g (l). So E = -E*.
  assert.ok(Math.abs(d2.E - (-d2.E_star)) < 1e-6,
    `hanging E (${d2.E}) ≈ -E* (${-d2.E_star})`);
});

test('Switcher latches and unlatches based on ROA boundary', () => {
  const params = { ...baseParams };
  const sw = new HandoverSwitcher();
  assert.equal(sw.inROA([0, 0], [0, 0], params), true, 'upright + still ⇒ in ROA');
  assert.equal(sw.inROA([0, Math.PI], [0, 0], params), false, 'hanging ⇒ not in ROA');
  assert.equal(sw.inROA([0, 0.1], [0, 1.0], params), true, 'small θ, small ω ⇒ in ROA');
  assert.equal(sw.inROA([0, 0.1], [0, 5.0], params), false, 'fast ω ⇒ outside');
});

test('Switcher mix outputs pure swingup before crossing, blends to LQR after', () => {
  const params = { ...baseParams };
  const sw = new HandoverSwitcher();
  const u_swing = () => 11;
  const u_lqr = () => 1;
  // outside ROA → swing-up only
  let u = sw.mix(0, [0, Math.PI], [0, 0], params, u_swing, u_lqr);
  assert.equal(u, 11);
  // inside ROA, t=0 (blend just started) → mostly swing-up
  u = sw.mix(0, [0, 0.1], [0, 0.1], params, u_swing, u_lqr);
  // alpha=0 at t=0 ⇒ pure swing-up
  assert.equal(u, 11);
  // after blend window, alpha=1 ⇒ pure LQR
  u = sw.mix(0.2, [0, 0.1], [0, 0.1], params, u_swing, u_lqr);
  assert.equal(u, 1);
});
