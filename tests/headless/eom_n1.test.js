// tests/headless/eom_n1.test.js — sanity checks on the sympy-generated n=1 EOM.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_1.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0.1,
  links: [{ m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0.001 }],
};

test('DOF and length consistency', () => {
  assert.equal(eom.N, 1);
  assert.equal(eom.DOF, 2);
});

test('M is symmetric across q,qdot sweep', () => {
  for (let theta of [-1.5, -0.5, 0, 0.5, 1.5, Math.PI]) {
    for (let thd of [-3, 0, 3]) {
      const Mm = eom.M([0, theta], [0, thd], params);
      assert.equal(Mm.length, 2);
      assert.equal(Mm[0].length, 2);
      assert.ok(Math.abs(Mm[0][1] - Mm[1][0]) < 1e-12,
        `M symmetric at theta=${theta}: M[0][1]=${Mm[0][1]}, M[1][0]=${Mm[1][0]}`);
    }
  }
});

test('M is positive-definite via determinant + diagonals', () => {
  for (let theta = -Math.PI; theta <= Math.PI; theta += 0.3) {
    const Mm = eom.M([0, theta], [0, 0], params);
    const det = Mm[0][0] * Mm[1][1] - Mm[0][1] * Mm[1][0];
    assert.ok(Mm[0][0] > 0, `M[0][0] > 0 at theta=${theta}`);
    assert.ok(Mm[1][1] > 0, `M[1][1] > 0 at theta=${theta}`);
    assert.ok(det > 0, `det(M) > 0 at theta=${theta} (got ${det})`);
  }
});

test('Gravity vector matches analytic formula', () => {
  for (const theta of [-1, 0, 0.7, Math.PI / 3]) {
    const G = eom.G([0, theta], [0, 0], params);
    assert.ok(Math.abs(G[0]) < 1e-12, 'G[cart]=0');
    const expected = -params.g * params.links[0].l * params.links[0].m * Math.sin(theta);
    assert.ok(Math.abs(G[1] - expected) < 1e-12,
      `G[theta] match at theta=${theta}: got ${G[1]}, expected ${expected}`);
  }
});

test('Coriolis vector matches analytic formula', () => {
  const thd = 2.5;
  for (const theta of [-1, 0.3, 1.2]) {
    const C = eom.Cqdot([0, theta], [0, thd], params);
    const expected = -params.links[0].l * params.links[0].m * thd * thd * Math.sin(theta);
    assert.ok(Math.abs(C[0] - expected) < 1e-12, `Cqdot[cart] match at theta=${theta}`);
    assert.ok(Math.abs(C[1]) < 1e-12, 'Cqdot[theta] = 0 for n=1');
  }
});

test('qddot solver: free fall from upright drops to 0 angular acceleration at upright', () => {
  // At theta=0 (upright), no friction, no input — gravity torque is 0, so qddot should be ~0.
  const q = [0, 0], qdot = [0, 0];
  const acc = eom.qddot(q, qdot, 0.0, { ...params, cart_visc: 0, links: [{ ...params.links[0], joint_viscous: 0 }] });
  assert.ok(Math.abs(acc[0]) < 1e-12, `xddot ~ 0 at upright (got ${acc[0]})`);
  assert.ok(Math.abs(acc[1]) < 1e-12, `thddot ~ 0 at upright (got ${acc[1]})`);
});

test('qddot at theta = pi/2 (horizontal): gravity accelerates pendulum', () => {
  // theta=pi/2: link horizontal (pointing right). Gravity should produce CCW torque.
  // With everything else zero, expect thddot > 0 (CCW from up means falling clockwise; sign convention check).
  // Actually theta increases CCW from up. At theta=+pi/2 the link points right; gravity pulls it down which is further CCW (toward theta=pi). So thddot > 0.
  const q = [0, Math.PI / 2], qdot = [0, 0];
  const noFric = { ...params, cart_visc: 0, links: [{ ...params.links[0], joint_viscous: 0 }] };
  const acc = eom.qddot(q, qdot, 0.0, noFric);
  assert.ok(acc[1] > 0, `thddot > 0 falling from horizontal (got ${acc[1]})`);
});
