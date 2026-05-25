// tests/headless/swingup_n2.test.js — n=2 swing-up + handover.
//
// PLAN §10 explicitly notes: "If swing-up cannot stabilize reliably, document
// specific failure regime." Energy-based pumping for a cart-driven double
// pendulum is a known-hard problem; pure Åström-Furuta brings the total
// pendulum energy near E* but the joint coordinates rarely fall inside the
// LQR's region of attraction simultaneously. Phase 13 addresses this with
// trajectory optimization. For Phase 10 we verify:
//
//   1. Energy pumping converges — within 20 s, |E_p − E_p*| / |E_p*| < 0.3.
//   2. Mass-weighted smooth sign for n>1 (so a heavy short link doesn't drown
//      out a light long link in the pumping average).
//   3. LQR catches a near-upright start (θ_1=0.05, θ_2=0.05) — confirms the
//      Auto path's handover-then-LQR branch works for n=2.
//   4. Documented failure regime: from full hanging, the (θ_1, θ̇_1, θ_2,
//      θ̇_2) state visits the LQR ROA only intermittently / not at all in
//      reasonable time. This is the test that codifies the "swing-up double
//      is iterative" assertion in PLAN §10.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_2.js';
import { stepRK4 } from '../../src/physics/integrator.js';
import { swingupU, swingupDiag, resetSwingup, pendulumEnergy } from '../../src/control/swingup.js';
import { HandoverSwitcher } from '../../src/control/switcher.js';
import { linearize, solveCARE } from '../../src/control/lqr.js';

const baseParams = {
  m0: 1.0, g: 9.81, cart_visc: 0.05, cart_coulomb: 0,
  links: [
    { m: 0.2,  L: 0.5, l: 0.25, I: 0.2  * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.15, L: 0.4, l: 0.2,  I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
  ],
  F_max: 25,
  swingup_kE: 20.0,
  swingup_kxP: 1.2,
  swingup_kxD: 1.5,
  handover_theta: 0.40,
  handover_omega: 3.5,
  handover_blend_ms: 80,
};

function diag(arr) {
  return Array.from({ length: arr.length }, (_, i) =>
    Array.from({ length: arr.length }, (_, j) => i === j ? arr[i] : 0));
}
function wrap(a) { let x = a; while (x > Math.PI) x -= 2 * Math.PI; while (x < -Math.PI) x += 2 * Math.PI; return x; }

function lqrK_n2(params) {
  const { A, B } = linearize(2, params);
  const Q = diag([10, 500, 500, 1, 20, 20]);
  const R = 0.05;
  return solveCARE(A, B, Q, R).K;
}

test('n=2 swing-up: pendulum energy converges to within 30% of E* within 20 s', () => {
  resetSwingup();
  const params = { ...baseParams };
  let q = [0, Math.PI, Math.PI];
  let qdot = [0, 0, 0];
  const dt = 1e-4;
  const N = Math.round(20 / dt);
  for (let k = 0; k < N; k++) {
    const u = swingupU(2, q, qdot, params, k * dt);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
  }
  const { E, E_star } = swingupDiag(2, q, qdot, params);
  const ratio = Math.abs(E - E_star) / Math.abs(E_star);
  assert.ok(ratio < 0.3,
    `pendulum energy converged near E* (|Ẽ|/|E*| = ${ratio.toFixed(3)})`);
});

test('n=2 swing-up mass-weighted average uses per-link m·l weights', () => {
  // Sign should flip when link-2's weight dominates. Pick a configuration
  // where the two links contribute OPPOSITE-sign tanh terms: θ̇_1·cos θ_1
  // negative, θ̇_2·cos θ_2 positive. With nearly all weight on link 1, the
  // signed average is negative → u_pump direction follows that. With nearly
  // all weight on link 2, sign flips. Use small k_E + zero cart-centering so
  // the only thing driving u is the pumping term, not saturation.
  const small = (p) => ({
    ...baseParams,
    ...p,
    swingup_kE: 0.5,
    swingup_kxP: 0,
    swingup_kxD: 0,
    swingup_omegaMin: 0.1,    // accept "swinging" immediately
    swingup_bootMin: 0,
    F_max: 100,
  });
  // q1=π/2 (cosθ=0) wouldn't work — pick angles where cos has opposite signs.
  // θ_1 = 2.5 (just past π/2, cos≈-0.80), θ_2 = 0.5 (cos≈+0.88).
  // θ̇_1=+1, θ̇_2=+1 → contributions: link1 = m1·l1·tanh(-0.80/0.4)=-w1,
  // link2 = m2·l2·tanh(0.88/0.4)= +w2.
  // Tick swingupU twice with the same swinging state so the bootstrap stage
  // latches off (first call records start time; second call has elapsed>0).
  const probeU = (params) => {
    resetSwingup();
    swingupU(2, [0, 2.5, 0.5], [0, 1, 1], params, 0);   // start
    return swingupU(2, [0, 2.5, 0.5], [0, 1, 1], params, 0.01); // post-boot
  };
  const p_heavy_link1 = small({ links: [
    { ...baseParams.links[0], m: 2.0 }, { ...baseParams.links[1], m: 0.02 }] });
  const u1 = probeU(p_heavy_link1);
  const p_heavy_link2 = small({ links: [
    { ...baseParams.links[0], m: 0.02 }, { ...baseParams.links[1], m: 2.0 }] });
  const u2 = probeU(p_heavy_link2);
  assert.ok(Math.sign(u1) !== Math.sign(u2),
    `weighting flips pump direction: u(heavy L1)=${u1.toFixed(3)} sign != u(heavy L2)=${u2.toFixed(3)}`);
});

test('n=2 Auto-mode: near-upright start (θ_1=0.05, θ_2=0.05) → LQR catches in ≤ 6 s', () => {
  resetSwingup();
  const params = { ...baseParams };
  const K = lqrK_n2(params);
  let q = [0, 0.05, 0.05];
  let qdot = [0, 0, 0];
  const dt = 1e-4;
  const T = 6;
  const N = Math.round(T / dt);
  const sw = new HandoverSwitcher();
  for (let k = 0; k < N; k++) {
    const t = k * dt;
    const u_swing = () => swingupU(2, q, qdot, params, t);
    const u_lqr = () => {
      const x = [q[0], wrap(q[1]), wrap(q[2]), qdot[0], qdot[1], qdot[2]];
      let u = 0;
      for (let i = 0; i < 6; i++) u -= K[i] * x[i];
      return Math.max(-params.F_max, Math.min(params.F_max, u));
    };
    const u = sw.mix(t, q, qdot, params, u_swing, u_lqr);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
  }
  assert.ok(Math.abs(wrap(q[1])) < 0.05, `final |θ_1| < 0.05 (got ${wrap(q[1]).toFixed(4)})`);
  assert.ok(Math.abs(wrap(q[2])) < 0.05, `final |θ_2| < 0.05 (got ${wrap(q[2]).toFixed(4)})`);
});

test('n=2 swing-up from full hanging: DOCUMENTED non-convergence to ROA in 30 s', () => {
  // This test ENCODES the failure regime called out in PLAN §10:
  //   "If swing-up cannot stabilize reliably, document specific failure regime."
  // We verify the negative result holds reproducibly so the doc/test pair
  // stays honest: pumping makes pendulum energy circulate near E* but the
  // (θ, θ̇) coordinates don't fall inside the LQR ROA in ≤ 30 s of sim.
  resetSwingup();
  const params = { ...baseParams };
  let q = [0, Math.PI, Math.PI];
  let qdot = [0, 0, 0];
  const dt = 1e-4;
  const N = Math.round(30 / dt);
  const sw = new HandoverSwitcher();
  let enteredAt = -1;
  for (let k = 0; k < N; k++) {
    const u = swingupU(2, q, qdot, params, k * dt);
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
    if (sw.inROA(q, qdot, params)) { enteredAt = k * dt; break; }
  }
  // Assert the failure: we expect enteredAt < 0 (never reached). If a future
  // controller improvement makes this succeed, the test will flag a happy
  // regression and we'll re-tier this test as a positive result.
  assert.ok(enteredAt < 0,
    `documented limitation: pure energy pumping does NOT reach LQR ROA from hanging in 30 s ` +
    `(if you see this firing, congrats — your swing-up got better; promote this test)`);
});
