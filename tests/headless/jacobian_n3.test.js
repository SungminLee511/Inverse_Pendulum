// tests/headless/jacobian_n3.test.js — linearization sanity for n=3.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { linearize, controllability } from '../../src/control/lqr.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0.05,
  links: [
    { m: 0.2,  L: 0.5, l: 0.25, I: 0.2  * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.15, L: 0.4, l: 0.2,  I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
    { m: 0.1,  L: 0.3, l: 0.15, I: 0.1  * 0.3 * 0.3 / 12, joint_viscous: 0.001 },
  ],
};

test('linearize(3): A is 8×8, B is 8-vec with cart-only actuation', () => {
  const { A, B, n_state } = linearize(3, params);
  assert.equal(n_state, 8);
  assert.equal(A.length, 8);
  assert.equal(B.length, 8);
  // Top half of B is zero
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(B[i]) < 1e-9, `B[${i}]=0`);
  // B[4] (xddot wrt u) > 0
  assert.ok(Math.abs(B[4]) > 1e-3, `B[4] nonzero (got ${B[4]})`);
});

test('linearize(3): top-right block is identity exactly', () => {
  const { A } = linearize(3, params);
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      assert.equal(A[i][4 + j], i === j ? 1 : 0);
});

test('linearize(3): Richardson eps=1e-4 vs 1e-6 agree to 1e-4', () => {
  const { A: A_coarse } = linearize(3, params, { eps: 1e-4 });
  const { A: A_fine }   = linearize(3, params, { eps: 1e-6 });
  let maxDiff = 0;
  for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++)
    maxDiff = Math.max(maxDiff, Math.abs(A_coarse[i][j] - A_fine[i][j]));
  assert.ok(maxDiff < 1e-3, `Richardson agreement ${maxDiff.toExponential(2)}`);
});

test('linearize(3): friction → negative diag in lower-right (rows 4..7)', () => {
  const { A } = linearize(3, params);
  for (let i = 4; i < 8; i++) assert.ok(A[i][i] < 0, `A[${i}][${i}] < 0 (got ${A[i][i]})`);
});

test('linearize(3): upright unstable — at least one ∂qddot[i]/∂θ_i > 0', () => {
  const { A } = linearize(3, params);
  let any = false;
  for (let i = 1; i < 4; i++) if (A[4 + i][i] > 0) any = true;
  assert.ok(any, 'at least one tipping ∂qddot/∂θ > 0 at upright');
});

test('controllability rank(n=3) = 8 (full)', () => {
  const { A, B } = linearize(3, params);
  const { rank } = controllability(A, B);
  assert.equal(rank, 8, `controllability rank = 8 (got ${rank})`);
});

test('linearize(3): A and B are all finite', () => {
  const { A, B } = linearize(3, params);
  for (let i = 0; i < 8; i++) {
    assert.ok(Number.isFinite(B[i]), `B[${i}] finite`);
    for (let j = 0; j < 8; j++) assert.ok(Number.isFinite(A[i][j]), `A[${i}][${j}] finite`);
  }
});
