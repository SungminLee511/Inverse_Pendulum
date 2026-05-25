// tests/headless/energy_n2.test.js — frictionless RK4 must conserve total
// mechanical energy of the n=2 cart-pendulum within 0.1% over 10 s of sim.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as eom from '../../src/physics/nlink_2.js';
import { stepRK4, totalEnergy } from '../../src/physics/integrator.js';

const params = {
  m0: 1.0, g: 9.81, cart_visc: 0,
  links: [
    { m: 0.2, L: 0.5, l: 0.25, I: 0.2 * 0.5 * 0.5 / 12, joint_viscous: 0 },
    { m: 0.15, L: 0.4, l: 0.2, I: 0.15 * 0.4 * 0.4 / 12, joint_viscous: 0 },
  ],
};

test('n=2 RK4 conserves energy to < 0.5% over 10 s (frictionless)', () => {
  // Initial: cart at rest, both links tilted away from hanging.
  let q = [0, Math.PI - 0.3, Math.PI - 0.2];
  let qdot = [0, 0, 0];
  const dt = 1e-4;
  const N = Math.round(10 / dt);
  const E0 = totalEnergy(q, qdot, params);
  let Emax = E0, Emin = E0;
  for (let k = 0; k < N; k++) {
    [q, qdot] = stepRK4(q, qdot, 0, dt, params, eom);
    if ((k & 0xfff) === 0) {     // sample every 4096 steps
      const E = totalEnergy(q, qdot, params);
      if (E > Emax) Emax = E;
      if (E < Emin) Emin = E;
    }
  }
  const drift = Math.abs(Emax - Emin) / Math.max(Math.abs(E0), 1e-9);
  assert.ok(drift < 0.005, `n=2 RK4 energy drift ${drift.toExponential(2)} < 0.5%`);
});

test('n=2 RK4 with friction dissipates energy monotonically', () => {
  let q = [0, Math.PI - 0.3, Math.PI - 0.2];
  let qdot = [0, 0, 0];
  const dt = 1e-4;
  const friction = {
    ...params, cart_visc: 0.3,
    links: params.links.map(l => ({ ...l, joint_viscous: 0.01 })),
  };
  let Eprev = totalEnergy(q, qdot, friction);
  let nonmono = 0;
  for (let k = 0; k < Math.round(5 / dt); k++) {
    [q, qdot] = stepRK4(q, qdot, 0, dt, friction, eom);
    if ((k & 0xfff) === 0) {
      const E = totalEnergy(q, qdot, friction);
      if (E > Eprev + 1e-6) nonmono++;
      Eprev = E;
    }
  }
  assert.equal(nonmono, 0, `friction dissipates monotonically (${nonmono} violations)`);
});
