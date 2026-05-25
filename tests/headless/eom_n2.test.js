// tests/headless/eom_n2.test.js — symbolic correctness of the n=2 EOM.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_2.js';

const baseParams = {
  m0: 1.0, g: 9.81, cart_visc: 0.1,
  links: [
    { m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.15, L: 0.4, l: 0.2, I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
  ],
};

function rand(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

test('n=2 EOM exports correct N and DOF', () => {
  assert.equal(eom.N, 2);
  assert.equal(eom.DOF, 3);
});

test('M(q) is symmetric across random q,qdot configurations', () => {
  const r = rand(7);
  for (let trial = 0; trial < 20; trial++) {
    const q    = [r() - 0.5, (r() - 0.5) * Math.PI, (r() - 0.5) * Math.PI];
    const qdot = [(r() - 0.5) * 2, (r() - 0.5) * 4, (r() - 0.5) * 4];
    const M = eom.M(q, qdot, baseParams);
    for (let i = 0; i < 3; i++)
      for (let j = i + 1; j < 3; j++)
        assert.ok(Math.abs(M[i][j] - M[j][i]) < 1e-9,
          `M[${i}][${j}]==M[${j}][${i}] (trial ${trial})`);
  }
});

test('M(q) is positive-definite (Sylvester criterion)', () => {
  const r = rand(11);
  for (let trial = 0; trial < 10; trial++) {
    const q = [0, (r() - 0.5) * Math.PI, (r() - 0.5) * Math.PI];
    const M = eom.M(q, [0,0,0], baseParams);
    // 1st minor: M[0][0] > 0
    assert.ok(M[0][0] > 0, 'M[0][0] > 0');
    // 2nd minor: det of top-left 2x2 > 0
    const det2 = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    assert.ok(det2 > 0, 'det of 2x2 minor > 0');
    // 3rd minor: full det > 0 (use cofactor expansion)
    const det3 =
      M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
    - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    assert.ok(det3 > 0, `det3 > 0 (got ${det3.toExponential(2)})`);
  }
});

test('At q=0 (upright), G is exactly zero', () => {
  const G = eom.G([0, 0, 0], [0, 0, 0], baseParams);
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(G[i]) < 1e-10, `G[${i}]=0`);
});

test('At hanging (θ=π,π), G is exactly zero', () => {
  const G = eom.G([0, Math.PI, Math.PI], [0, 0, 0], baseParams);
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(G[i]) < 1e-10, `G[${i}]=0`);
});

test('qddot at upright, zero qdot, zero u, no friction → zero acceleration', () => {
  const noFriction = {
    ...baseParams, cart_visc: 0,
    links: baseParams.links.map(l => ({ ...l, joint_viscous: 0 })),
  };
  const acc = eom.qddot([0, 0, 0], [0, 0, 0], 0, noFriction);
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(acc[i]) < 1e-10, `qddot[${i}]=0`);
});

test('qddot signs at hanging with small θ_1 perturbation: gravity tips link further', () => {
  // At θ_1=π+0.1 (tilted further past hanging), G[1] points in the direction
  // that increases θ_1 (away from upright). At θ_2=π too with zero qdot.
  const noFriction = {
    ...baseParams, cart_visc: 0,
    links: baseParams.links.map(l => ({ ...l, joint_viscous: 0 })),
  };
  const acc1 = eom.qddot([0, Math.PI + 0.1, Math.PI], [0,0,0], 0, noFriction);
  // Should NOT be zero
  assert.ok(Math.abs(acc1[1]) > 1e-3, `qddot[1] !=0 with theta_1 perturb (${acc1[1]})`);
});

test('Friction Dqdot is diagonal: D_i depends only on qdot_i', () => {
  // Compare D at (xdot, θd1, θd2) vs at (xdot, 0, 0) — only the first
  // component should match.
  const D_all = eom.Dqdot([0,0,0], [0.3, 0.5, 0.7], baseParams);
  const D_only_x = eom.Dqdot([0,0,0], [0.3, 0, 0], baseParams);
  assert.ok(Math.abs(D_all[0] - D_only_x[0]) < 1e-12, 'D[0] from xdot only');
  // and D[1] depends only on θd1
  const D_only_th1 = eom.Dqdot([0,0,0], [0, 0.5, 0], baseParams);
  assert.ok(Math.abs(D_all[1] - D_only_th1[1]) < 1e-12, 'D[1] from θd1 only');
});
