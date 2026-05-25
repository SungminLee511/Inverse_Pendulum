// tests/headless/energy_n3.test.js — frictionless RK4 must conserve total
// mechanical energy of the n=3 cart-pendulum within 1% over 5 s.
// (PLAN: triple's M(q) is worse-conditioned, so allow looser tolerance than n=1,2.)

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_3.js';
import { stepRK4, totalEnergy } from '../../src/physics/integrator.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0,
  links: [
    { m: 0.2,  L: 0.5, l: 0.25, I: 0.2  * 0.5 * 0.5 / 12, joint_viscous: 0 },
    { m: 0.15, L: 0.4, l: 0.2,  I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0 },
    { m: 0.1,  L: 0.3, l: 0.15, I: 0.1  * 0.3 * 0.3 / 12, joint_viscous: 0 },
  ],
};

test('n=3 RK4 conserves energy to < 1% over 5 s (frictionless)', () => {
  let q = [0, Math.PI - 0.3, Math.PI - 0.2, Math.PI - 0.1];
  let qdot = [0, 0, 0, 0];
  const dt = 1e-4;
  const N = Math.round(5 / dt);
  const E0 = totalEnergy(q, qdot, params);
  let Emax = E0, Emin = E0;
  for (let k = 0; k < N; k++) {
    [q, qdot] = stepRK4(q, qdot, 0, dt, params, eom);
    if ((k & 0xfff) === 0) {
      const E = totalEnergy(q, qdot, params);
      if (E > Emax) Emax = E;
      if (E < Emin) Emin = E;
    }
  }
  const drift = Math.abs(Emax - Emin) / Math.max(Math.abs(E0), 1e-9);
  assert.ok(drift < 0.01, `n=3 RK4 energy drift ${drift.toExponential(2)} < 1%`);
});

test('n=3 RK4 with friction dissipates energy monotonically', () => {
  let q = [0, Math.PI - 0.3, Math.PI - 0.2, Math.PI - 0.1];
  let qdot = [0, 0, 0, 0];
  const fric = { ...params, cart_visc: 0.3,
    links: params.links.map(l => ({ ...l, joint_viscous: 0.01 })) };
  const dt = 1e-4;
  let Eprev = totalEnergy(q, qdot, fric);
  let nonmono = 0;
  for (let k = 0; k < Math.round(4 / dt); k++) {
    [q, qdot] = stepRK4(q, qdot, 0, dt, fric, eom);
    if ((k & 0xfff) === 0) {
      const E = totalEnergy(q, qdot, fric);
      if (E > Eprev + 1e-6) nonmono++;
      Eprev = E;
    }
  }
  assert.equal(nonmono, 0, `n=3 friction dissipates monotonically (${nonmono} violations)`);
});

test('n=3 qddot at upright + zero qdot + zero u + friction → zero', () => {
  const fric = { ...params, cart_visc: 0.3,
    links: params.links.map(l => ({ ...l, joint_viscous: 0.01 })) };
  const acc = eom.qddot([0,0,0,0], [0,0,0,0], 0, fric);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(acc[i]) < 1e-9, `qddot[${i}]=0`);
});

test('n=3 qddot at hanging with cart force → cart accelerates, joints respond', () => {
  // u = +10 N at hanging equilibrium should accelerate cart in +x.
  const acc = eom.qddot([0, Math.PI, Math.PI, Math.PI], [0,0,0,0], 10, params);
  assert.ok(Number.isFinite(acc[0]), 'cart acc finite');
  assert.ok(acc[0] > 0, `cart acc > 0 (got ${acc[0].toFixed(3)})`);
  // At least one joint should accelerate non-trivially through the back-reaction.
  let maxJoint = 0;
  for (let i = 1; i < 4; i++) maxJoint = Math.max(maxJoint, Math.abs(acc[i]));
  assert.ok(maxJoint > 1e-3, `some joint responds (max |acc| = ${maxJoint.toFixed(3)})`);
});
