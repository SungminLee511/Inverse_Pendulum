// tests/headless/lqr_nonlinear_n1.test.js — close-loop LQR on the full nonlinear EOM.
// Bypasses sensors/actuator/loop — direct integrator + feedback.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_1.js';
import { stepRK4 } from '../../src/physics/integrator.js';
import { linearize, solveCARE } from '../../src/control/lqr.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0,
  links: [{ m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0 }],
};

function diag(arr) {
  return Array.from({ length: arr.length }, (_, i) =>
    Array.from({ length: arr.length }, (_, j) => i === j ? arr[i] : 0));
}

test('LQR on full nonlinear EOM brings θ=0.15 to ~0 within 6s (R=0.05)', () => {
  const { A, B } = linearize(1, params);
  const Q = diag([10, 100, 1, 10]);
  const R = 0.05;
  const { K } = solveCARE(A, B, Q, R);
  console.log('K =', K.map(v => v.toFixed(3)));

  let q = [0, 0.15], qdot = [0, 0];
  const dt = 1e-4;
  const T = 6;
  const N = Math.round(T / dt);
  let peakAbsTheta = Math.abs(q[1]);
  const F_max = 50;
  for (let k = 0; k < N; k++) {
    // x_state = [q, qdot] with angle wrap
    let theta_wrapped = q[1];
    while (theta_wrapped >  Math.PI) theta_wrapped -= 2 * Math.PI;
    while (theta_wrapped < -Math.PI) theta_wrapped += 2 * Math.PI;
    const x = [q[0], theta_wrapped, qdot[0], qdot[1]];
    let u = 0;
    for (let i = 0; i < 4; i++) u -= K[i] * x[i];
    u = Math.max(-F_max, Math.min(F_max, u));
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
    peakAbsTheta = Math.max(peakAbsTheta, Math.abs(q[1]));
    if (k < 50 && k % 10 === 0) {
      console.log(`t=${(k*dt).toFixed(4)} θ=${q[1].toFixed(3)} θ̇=${qdot[1].toFixed(3)} x=${q[0].toFixed(3)} u=${u.toFixed(2)}`);
    }
  }
  console.log('final:', q, qdot, 'peakAbsTheta=', peakAbsTheta);
  assert.ok(Math.abs(q[1]) < 0.05, `final θ < 0.05 (got ${q[1].toExponential(2)})`);
  assert.ok(peakAbsTheta < 0.5, `peak θ < 0.5 rad (got ${peakAbsTheta.toFixed(3)})`);
});

test('LQR on full nonlinear EOM with browser-default R=0.01 also stabilises', () => {
  const { A, B } = linearize(1, params);
  const Q = diag([10, 100, 1, 10]);
  const R = 0.01;
  const { K } = solveCARE(A, B, Q, R);
  console.log('K(R=0.01) =', K.map(v => v.toFixed(3)));
  let q = [0, 0.15], qdot = [0, 0];
  const dt = 1e-4;
  const T = 6;
  const N = Math.round(T / dt);
  let peak = Math.abs(q[1]);
  const F_max = 50;
  for (let k = 0; k < N; k++) {
    let th = q[1];
    while (th >  Math.PI) th -= 2 * Math.PI;
    while (th < -Math.PI) th += 2 * Math.PI;
    const x = [q[0], th, qdot[0], qdot[1]];
    let u = 0; for (let i = 0; i < 4; i++) u -= K[i] * x[i];
    u = Math.max(-F_max, Math.min(F_max, u));
    [q, qdot] = stepRK4(q, qdot, u, dt, params, eom);
    peak = Math.max(peak, Math.abs(q[1]));
  }
  console.log('R=0.01 final:', q[1].toExponential(2), 'peak:', peak.toFixed(3));
  assert.ok(Math.abs(q[1]) < 0.05, `final θ < 0.05`);
  assert.ok(peak < 1.0, `peak θ < 1.0 rad`);
});
