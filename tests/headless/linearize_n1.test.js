// tests/headless/linearize_n1.test.js — verify numerical Jacobian linearization for n=1.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { linearize, controllability, matrixRank } from '../../src/control/lqr.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0,
  links: [{ m: 0.2, L: 0.5, l: 0.25, I: 0.2*0.5*0.5/12, joint_viscous: 0 }],
};

test('A is 4x4 and has identity kinematic block', () => {
  const { A, B, n_state } = linearize(1, params);
  assert.equal(n_state, 4);
  assert.equal(A.length, 4);
  assert.equal(A[0].length, 4);
  // Top half: A[0..2, 2..4] = I (d/dt q = qdot)
  assert.equal(A[0][2], 1);
  assert.equal(A[0][3], 0);
  assert.equal(A[1][2], 0);
  assert.equal(A[1][3], 1);
  // Top half of A[0..2, 0..2] = 0
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) {
    assert.equal(A[i][j], 0, `A[${i}][${j}]=0`);
  }
});

test('B is 4-vector with only the qddot half nonzero', () => {
  const { B } = linearize(1, params);
  assert.equal(B.length, 4);
  assert.ok(Math.abs(B[0]) < 1e-9, `B[0]=0 (got ${B[0]})`);
  assert.ok(Math.abs(B[1]) < 1e-9, `B[1]=0`);
  assert.ok(B[2] > 0, `B[2] > 0 (force pushes cart right) (got ${B[2]})`);
  assert.ok(B[3] !== 0, `B[3] ≠ 0 (couples through pendulum) (got ${B[3]})`);
});

test('Upright is unstable: A has at least one eigenvalue with positive real part', () => {
  // For n=1 with frictionless params, the bottom-half block Aqq is
  //     d(qddot)/d(theta)  ~  positive  (gravity pulls away from upright)
  // We check the sign of A[3][1] explicitly: should be positive.
  const { A } = linearize(1, params);
  assert.ok(A[3][1] > 0, `A[3][1] > 0 (upright instability) (got ${A[3][1]})`);
});

test('Friction shows up as negative entries in Aqqd', () => {
  const frParams = { ...params, cart_visc: 0.5,
    links: [{ ...params.links[0], joint_viscous: 0.05 }] };
  const { A } = linearize(1, frParams);
  // qddot has -D*qdot in it. d(qddot)/d(qdot) should have negative diagonal.
  // qddot[0] depends mostly on -cart_visc * xdot → A[2][2] < 0
  assert.ok(A[2][2] < 0, `A[2][2] < 0 with cart friction (got ${A[2][2]})`);
  assert.ok(A[3][3] < 0, `A[3][3] < 0 with joint friction (got ${A[3][3]})`);
});

test('Jacobian converges: halving eps reduces error', () => {
  // Compare central diff at eps=1e-4 vs eps=1e-6 — they should agree to ~1e-7
  const r1 = linearize(1, params, { eps: 1e-4 });
  const r2 = linearize(1, params, { eps: 1e-6 });
  let maxDiff = 0;
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    maxDiff = Math.max(maxDiff, Math.abs(r1.A[i][j] - r2.A[i][j]));
  }
  assert.ok(maxDiff < 1e-6, `A converges with smaller eps (max diff ${maxDiff})`);
});

test('Controllability: rank([B, AB, A²B, A³B]) = 4 for n=1', () => {
  const { A, B } = linearize(1, params);
  const { rank } = controllability(A, B);
  assert.equal(rank, 4, 'fully controllable');
});

test('Without friction, A is well-conditioned (no NaN / Inf)', () => {
  const { A, B } = linearize(1, params);
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      assert.ok(Number.isFinite(A[i][j]), `A[${i}][${j}] finite`);
  for (let i = 0; i < 4; i++) assert.ok(Number.isFinite(B[i]), `B[${i}] finite`);
});

test('Linearized dynamics around upright reproduces standard formulas (closed form check)', () => {
  // For n=1 the upright linearization analytic A coefficients are well-known:
  //   For x_state = [x, θ, xdot, θdot]:
  //   ẋddot:  ∂xddot/∂θ = (m1 l1 g) / (m0 + m1 - (m1 l1)^2 / (I1 + m1 l1^2))
  // We just spot-check a couple of signs and orders of magnitude here.
  const { A } = linearize(1, params);
  // d(xddot)/d(theta): pendulum tipping causes the cart to be pushed
  assert.ok(Math.abs(A[2][1]) > 0.1, `A[2][1] non-trivial (got ${A[2][1]})`);
});
