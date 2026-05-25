// tests/headless/swingup_n3.test.js — n=3 swing-up.
//
// PLAN §13 explicit fallback: "if all fail, ship 'near-upright start' toggle
// + documented limitation." The triple pendulum is generally unswing-uppable
// with pure energy pumping (well-known result; even the textbook double is
// hard, see Phase 10). For Phase 13 we:
//
//   1. Verify pumping converges in energy norm (E_p → ~E_p*).
//   2. Document the failure: from hanging, the LQR ROA is not reached.
//   3. Verify the near-upright start fallback succeeds — when q[i] starts at
//      a small θ instead of π, the Auto mode picks LQR immediately and
//      stabilises.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_3.js';
import { stepRK4 } from '../../src/physics/integrator.js';
import { swingupU, swingupDiag, resetSwingup } from '../../src/control/swingup.js';
import { HandoverSwitcher } from '../../src/control/switcher.js';
import { linearize, solveCARE } from '../../src/control/lqr.js';

const baseParams = {
  m0: 1.0, g: 9.81, cart_visc: 0.05, cart_coulomb: 0,
  links: [
    { m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.2, L: 0.4, l: 0.20, I: 0.2 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
    { m: 0.2, L: 0.3, l: 0.15, I: 0.2 * 0.3 * 0.3 / 12, joint_viscous: 0.001 },
  ],
  F_max: 30,
  swingup_kE: 30.0,
  swingup_kxP: 1.5,
  swingup_kxD: 2.0,
  handover_theta: 0.45,
  handover_omega: 4.0,
  handover_blend_ms: 80,
};

function diag(arr) {
  return Array.from({ length: arr.length }, (_, i) =>
    Array.from({ length: arr.length }, (_, j) => i === j ? arr[i] : 0));
}
function wrap(a) { let x = a; while (x > Math.PI) x -= 2 * Math.PI; while (x < -Math.PI) x += 2 * Math.PI; return x; }
function lqrK_n3(params) {
  const { A, B } = linearize(3, params);
  const Q = diag([10, 600, 600, 600, 1, 30, 30, 30]);
  return solveCARE(A, B, Q, 0.05).K;
}

test('n=3 swing-up: energy converges to within 40% of E_p* in 25 s', () => {
  resetSwingup();
  const params = { ...baseParams };
  let q = [0, Math.PI, Math.PI, Math.PI];
  let qdot = [0, 0, 0, 0];
  const dt = 1e-4;
  const N = Math.round(25 / dt);
  for (let k = 0; k < N; k++) {
    const u = swingupU(3, q, qdot, params, k * dt);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
  }
  const { E, E_star } = swingupDiag(3, q, qdot, params);
  const ratio = Math.abs(E - E_star) / Math.abs(E_star);
  assert.ok(ratio < 0.4,
    `energy converges (|Ẽ|/|E*| = ${ratio.toFixed(3)}, E=${E.toFixed(3)}, E*=${E_star.toFixed(3)})`);
});

test('n=3 swing-up from hanging: DOCUMENTED non-convergence to LQR ROA in 30 s', () => {
  // Per PLAN §13: pure energy pumping for the triple pendulum is expected to
  // fail. We assert the failure to keep the doc and the test in sync.
  resetSwingup();
  const params = { ...baseParams };
  let q = [0, Math.PI, Math.PI, Math.PI];
  let qdot = [0, 0, 0, 0];
  const dt = 1e-4;
  const sw = new HandoverSwitcher();
  let enteredAt = -1;
  const N = Math.round(30 / dt);
  for (let k = 0; k < N; k++) {
    const u = swingupU(3, q, qdot, params, k * dt);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
    if (sw.inROA(q, qdot, params)) { enteredAt = k * dt; break; }
  }
  assert.ok(enteredAt < 0,
    `documented limitation: triple energy-pumping does NOT reach LQR ROA in 30 s ` +
    `(if this fires, congrats — Phase 13's trajopt is no longer the only path)`);
});

test('n=3 near-upright fallback: θ_i=0.05 start + Auto mode → LQR catches in ≤ 6 s', () => {
  resetSwingup();
  const params = { ...baseParams };
  const K = lqrK_n3(params);
  let q = [0, 0.05, 0.05, 0.05];     // "near-upright start" toggle ICs
  let qdot = [0, 0, 0, 0];
  const dt = 1e-4;
  const T = 6;
  const N = Math.round(T / dt);
  const sw = new HandoverSwitcher();
  for (let k = 0; k < N; k++) {
    const t = k * dt;
    const u_swing = () => swingupU(3, q, qdot, params, t);
    const u_lqr = () => {
      const x = [q[0], wrap(q[1]), wrap(q[2]), wrap(q[3]),
                 qdot[0], qdot[1], qdot[2], qdot[3]];
      let u = 0;
      for (let i = 0; i < 8; i++) u -= K[i] * x[i];
      return Math.max(-params.F_max, Math.min(params.F_max, u));
    };
    const u = sw.mix(t, q, qdot, params, u_swing, u_lqr);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
  }
  for (let i = 1; i < 4; i++)
    assert.ok(Math.abs(wrap(q[i])) < 0.05,
      `near-upright start: final |θ_${i}| < 0.05 (got ${wrap(q[i]).toFixed(4)})`);
});

test('Switcher.inROA for n=3 uses all 4 angles + velocities', () => {
  const sw = new HandoverSwitcher();
  const params = { ...baseParams };
  // upright + still → in
  assert.equal(sw.inROA([0,0,0,0], [0,0,0,0], params), true, 'upright+still in');
  // one joint hanging → out
  assert.equal(sw.inROA([0, 0, Math.PI, 0], [0,0,0,0], params), false, 'one hanging out');
  // small angle on all but high ω on link 2 → out
  assert.equal(sw.inROA([0, 0.1, 0.1, 0.1], [0, 0, 10, 0], params), false, 'fast ω_2 out');
  // small everything → in
  assert.equal(sw.inROA([0, 0.1, 0.1, 0.1], [0, 1, 1, 1], params), true, 'all small in');
});
