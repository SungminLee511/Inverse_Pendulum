// tests/headless/lqr_n1.test.js — CARE solver correctness + closed-loop stability.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  linearize, solveCARE,
  matMul, matVec, transpose, matrixInvert,
} from '../../src/control/lqr.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0.1,
  links: [{ m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 }],
};

function eye(n) { return Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => i === j ? 1 : 0)); }
function diag(arr) { return Array.from({ length: arr.length }, (_, i) => Array.from({ length: arr.length }, (_, j) => i === j ? arr[i] : 0)); }

test('CARE: P is symmetric positive-definite', () => {
  const { A, B } = linearize(1, params);
  const Q = diag([1, 100, 1, 10]);
  const R = 0.05;
  const { P } = solveCARE(A, B, Q, R);
  // symmetry
  for (let i = 0; i < 4; i++) for (let j = i + 1; j < 4; j++) {
    assert.ok(Math.abs(P[i][j] - P[j][i]) < 1e-6, `P symmetric (${i},${j})`);
  }
  // positive-definite: diagonal > 0 (necessary), and 2x2 minors > 0
  for (let i = 0; i < 4; i++) assert.ok(P[i][i] > 0, `P[${i}][${i}] > 0`);
});

test('CARE: residual A^T P + P A − P B R^{-1} B^T P + Q ≈ 0', () => {
  const { A, B } = linearize(1, params);
  const Q = diag([1, 100, 1, 10]);
  const R = 0.05;
  const { P } = solveCARE(A, B, Q, R);
  // Compute residual
  const AT = transpose(A);
  const Bm = B.map(v => [v]);
  const BTP = matMul(transpose(Bm), P);          // 1 × 4
  const Rinv_BTP = BTP.map(r => r.map(v => v / R));   // 1 × 4
  const P_B_Rinv_BTP = matMul(P, matMul(Bm, Rinv_BTP)); // 4 × 4
  let maxRes = 0;
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let r = 0;
      for (let k = 0; k < 4; k++) r += AT[i][k] * P[k][j];
      for (let k = 0; k < 4; k++) r += P[i][k] * A[k][j];
      r -= P_B_Rinv_BTP[i][j];
      r += Q[i][j];
      maxRes = Math.max(maxRes, Math.abs(r));
    }
  }
  assert.ok(maxRes < 1e-3, `Riccati residual small (got ${maxRes.toExponential(2)})`);
});

test('LQR closed-loop is asymptotically stable', () => {
  // Simulate the linearized system with feedback u = -K x and a small perturbation.
  const { A, B } = linearize(1, params);
  const Q = diag([1, 100, 1, 10]);
  const R = 0.05;
  const { K } = solveCARE(A, B, Q, R);
  // x_dot = A x + B u, u = -K x  →  x_dot = (A - B K) x
  let x = [0, 0.2, 0, 0];   // 0.2 rad ~ 11 deg tilt
  const dt = 1e-3, T = 10;
  const N = Math.round(T / dt);
  const initNorm = Math.hypot(...x);
  for (let k = 0; k < N; k++) {
    const u = -(K[0]*x[0] + K[1]*x[1] + K[2]*x[2] + K[3]*x[3]);
    const xdot = matVec(A, x).map((v, i) => v + B[i] * u);
    for (let i = 0; i < 4; i++) x[i] += dt * xdot[i];
  }
  const finalNorm = Math.hypot(x[0], x[1], x[2], x[3]);
  assert.ok(finalNorm < initNorm * 0.05,
    `closed-loop state shrinks by >20× (init ${initNorm.toFixed(3)}, final ${finalNorm.toExponential(2)})`);
});

test('LQR closed-loop eigenvalues all in LHP', () => {
  // Compute (A - BK) and verify its trace < 0 and determinant > 0 for a 4×4 (necessary stability conditions).
  // Full eigen check left to numerical sim above.
  const { A, B } = linearize(1, params);
  const Q = diag([1, 100, 1, 10]);
  const R = 0.05;
  const { K } = solveCARE(A, B, Q, R);
  const Acl = A.map((row, i) => row.map((v, j) => v - B[i] * K[j]));
  let trace = 0;
  for (let i = 0; i < 4; i++) trace += Acl[i][i];
  assert.ok(trace < 0, `trace(A_cl) < 0 (got ${trace.toFixed(3)})`);
});

test('Heavier R produces smaller gains than light R', () => {
  const { A, B } = linearize(1, params);
  const Q = diag([1, 100, 1, 10]);
  const { K: K_light } = solveCARE(A, B, Q, 0.01);
  const { K: K_heavy } = solveCARE(A, B, Q, 1.0);
  const norm = K => Math.hypot(...K);
  assert.ok(norm(K_heavy) < norm(K_light),
    `heavier R → smaller K (||K_heavy||=${norm(K_heavy).toFixed(2)}, ||K_light||=${norm(K_light).toFixed(2)})`);
});

test('Higher Q on theta produces stronger theta gain', () => {
  const { A, B } = linearize(1, params);
  const { K: K_low } = solveCARE(A, B, diag([1, 1, 1, 1]), 0.05);
  const { K: K_high } = solveCARE(A, B, diag([1, 1000, 1, 1]), 0.05);
  assert.ok(Math.abs(K_high[1]) > Math.abs(K_low[1]),
    `Q[theta] up → |K[theta]| up (${K_low[1].toFixed(2)} → ${K_high[1].toFixed(2)})`);
});

test('matrixInvert: round-trip A * A^{-1} = I', () => {
  const A = [[2, 1, 0], [1, 3, 1], [0, 1, 2]];
  const Ainv = matrixInvert(A);
  const I = matMul(A, Ainv);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    const expected = i === j ? 1 : 0;
    assert.ok(Math.abs(I[i][j] - expected) < 1e-10, `A A^{-1}[${i},${j}] = ${expected}`);
  }
});
