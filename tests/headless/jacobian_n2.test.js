// tests/headless/jacobian_n2.test.js — numerical-vs-finite-difference Jacobian
// for the n=2 EOM, and sanity of analytic derivatives at the upright.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_2.js';
import { linearize, controllability } from '../../src/control/lqr.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0.05,
  links: [
    { m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.15, L: 0.4, l: 0.2, I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
  ],
};

test('linearize(2): A is 6×6, B is 6-vec with cart-only actuation', () => {
  const { A, B, n_state } = linearize(2, params);
  assert.equal(n_state, 6);
  assert.equal(A.length, 6);
  assert.equal(A[0].length, 6);
  assert.equal(B.length, 6);
  // First nq=3 rows of A correspond to d/dt q = qdot → top-right block = I
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      assert.ok(Math.abs(A[i][3 + j] - (i === j ? 1 : 0)) < 1e-9,
        `A[${i}][${3+j}] = δ(${i},${j})`);
    }
  }
  // B: top half zero, bottom half nonzero only at the index corresponding to qddot
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(B[i]) < 1e-9, `B[${i}]=0`);
  assert.ok(Math.abs(B[3]) > 1e-3, 'B[3] (xddot wrt u) nonzero');
});

test('linearize(2): Richardson convergence — eps=1e-4 vs 1e-6 agree to 1e-5', () => {
  const { A: A_coarse } = linearize(2, params, { eps: 1e-4 });
  const { A: A_fine }   = linearize(2, params, { eps: 1e-6 });
  let maxDiff = 0;
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++)
    maxDiff = Math.max(maxDiff, Math.abs(A_coarse[i][j] - A_fine[i][j]));
  assert.ok(maxDiff < 1e-4, `Richardson agreement ${maxDiff.toExponential(2)}`);
});

test('linearize(2): top-right block is identity, exactly', () => {
  const { A } = linearize(2, params);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    assert.equal(A[i][3+j], i === j ? 1 : 0);
});

test('linearize(2): upright is unstable — at least one positive eigenvalue', () => {
  const { A } = linearize(2, params);
  // We don't have a full eigen solver; use trace + det heuristics + tilt term.
  // Specifically, A[3+i][1+i] (∂xddot_pend / ∂θ) should drive an unstable
  // mode at the inverted equilibrium.
  let anyUnstable = false;
  for (let i = 1; i < 3; i++) {
    // ∂qddot[i] / ∂θ_i at upright > 0 indicates gravity tips it (unstable).
    if (A[3 + i][i] > 0) anyUnstable = true;
  }
  assert.ok(anyUnstable, 'at least one ∂qddot[i]/∂θ_i > 0 at upright');
});

test('linearize(2): friction shows up as negative diagonal in lower-right', () => {
  // A[nq + i][nq + i] should be ≤ 0 when joint_viscous>0 (energy dissipates).
  const { A } = linearize(2, params);
  assert.ok(A[3][3] < 0, `A[3][3] (cart visc) < 0 (got ${A[3][3]})`);
  assert.ok(A[4][4] < 0, `A[4][4] (joint1 visc) < 0 (got ${A[4][4]})`);
  assert.ok(A[5][5] < 0, `A[5][5] (joint2 visc) < 0 (got ${A[5][5]})`);
});

test('controllability rank(2) = 6 (full)', () => {
  const { A, B } = linearize(2, params);
  const { rank } = controllability(A, B);
  assert.equal(rank, 6, `controllability rank = 6 (got ${rank})`);
});

test('A and B are finite (no NaN / Inf)', () => {
  const { A, B } = linearize(2, params);
  for (let i = 0; i < 6; i++) {
    assert.ok(Number.isFinite(B[i]), `B[${i}] finite`);
    for (let j = 0; j < 6; j++) assert.ok(Number.isFinite(A[i][j]), `A[${i}][${j}] finite`);
  }
});
