// tests/headless/eom_n3.test.js — symbolic correctness of the n=3 EOM.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_3.js';

const baseParams = {
  m0: 1.0, g: 9.81, cart_visc: 0.1,
  links: [
    { m: 0.2,  L: 0.5, l: 0.25, I: 0.2  * 0.5 * 0.5 / 12, joint_viscous: 0.001 },
    { m: 0.15, L: 0.4, l: 0.2,  I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0.001 },
    { m: 0.1,  L: 0.3, l: 0.15, I: 0.1  * 0.3 * 0.3 / 12, joint_viscous: 0.001 },
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

test('n=3 EOM exports correct N (=3) and DOF (=4)', () => {
  assert.equal(eom.N, 3);
  assert.equal(eom.DOF, 4);
});

test('M(q) is symmetric across 20 random configurations', () => {
  const r = rand(13);
  for (let trial = 0; trial < 20; trial++) {
    const q = [r()-0.5, (r()-0.5)*Math.PI, (r()-0.5)*Math.PI, (r()-0.5)*Math.PI];
    const qdot = [(r()-0.5)*2, (r()-0.5)*4, (r()-0.5)*4, (r()-0.5)*4];
    const M = eom.M(q, qdot, baseParams);
    for (let i = 0; i < 4; i++)
      for (let j = i + 1; j < 4; j++)
        assert.ok(Math.abs(M[i][j] - M[j][i]) < 1e-9, `M sym [${i}][${j}] trial ${trial}`);
  }
});

test('M(q) is positive-definite via Sylvester at multiple configurations', () => {
  // det helpers
  const det2 = (a) => a[0][0]*a[1][1] - a[0][1]*a[1][0];
  const det3 = (a) => a[0][0]*(a[1][1]*a[2][2] - a[1][2]*a[2][1])
                    - a[0][1]*(a[1][0]*a[2][2] - a[1][2]*a[2][0])
                    + a[0][2]*(a[1][0]*a[2][1] - a[1][1]*a[2][0]);
  // 4x4 det via cofactor expansion along row 0
  const det4 = (m) => {
    let d = 0;
    for (let j = 0; j < 4; j++) {
      const minor = [];
      for (let i = 1; i < 4; i++) {
        const row = [];
        for (let k = 0; k < 4; k++) if (k !== j) row.push(m[i][k]);
        minor.push(row);
      }
      d += ((j & 1) ? -1 : 1) * m[0][j] * det3(minor);
    }
    return d;
  };
  const r = rand(17);
  for (let trial = 0; trial < 10; trial++) {
    const q = [0, (r()-0.5)*Math.PI, (r()-0.5)*Math.PI, (r()-0.5)*Math.PI];
    const M = eom.M(q, [0,0,0,0], baseParams);
    assert.ok(M[0][0] > 0, `M[0][0]>0`);
    const m11 = [[M[0][0], M[0][1]], [M[1][0], M[1][1]]];
    assert.ok(det2(m11) > 0, `det(2x2)>0`);
    const m22 = [[M[0][0],M[0][1],M[0][2]],[M[1][0],M[1][1],M[1][2]],[M[2][0],M[2][1],M[2][2]]];
    assert.ok(det3(m22) > 0, `det(3x3)>0`);
    assert.ok(det4(M) > 0, `det(4x4)>0`);
  }
});

test('G is exactly zero at upright (q=0,0,0)', () => {
  const G = eom.G([0,0,0,0], [0,0,0,0], baseParams);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(G[i]) < 1e-10, `G[${i}]=0`);
});

test('G is exactly zero at hanging (θ=π,π,π)', () => {
  const G = eom.G([0, Math.PI, Math.PI, Math.PI], [0,0,0,0], baseParams);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(G[i]) < 1e-10, `G[${i}]=0`);
});

test('qddot at upright + zero qdot + zero u + no friction → zero', () => {
  const nofric = { ...baseParams, cart_visc: 0,
    links: baseParams.links.map(l => ({ ...l, joint_viscous: 0 })) };
  const acc = eom.qddot([0,0,0,0], [0,0,0,0], 0, nofric);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(acc[i]) < 1e-9, `qddot[${i}]=0`);
});

test('Friction Dqdot is diagonal in n=3', () => {
  const all = eom.Dqdot([0,0,0,0], [0.3, 0.5, 0.7, 0.9], baseParams);
  const onlyTh2 = eom.Dqdot([0,0,0,0], [0, 0, 0.7, 0], baseParams);
  assert.ok(Math.abs(all[2] - onlyTh2[2]) < 1e-12, `D[2] depends only on θd2`);
});

test('M condition: diagonal entries vs off-diagonals (sanity)', () => {
  // For n=3, M's 3,3 entry is I_3 + l_3²·m_3 ≈ 0.1·0.15² + 0.1·0.225²/12 ≈ 2.625e-3.
  // M's 0,0 = m_total ≈ 1.45. Off-diagonals are intermediate. Just verify
  // that all diagonals are positive and finite at upright.
  const M = eom.M([0,0,0,0], [0,0,0,0], baseParams);
  for (let i = 0; i < 4; i++) {
    assert.ok(M[i][i] > 0, `M[${i}][${i}] > 0`);
    assert.ok(Number.isFinite(M[i][i]), `M[${i}][${i}] finite`);
  }
});
