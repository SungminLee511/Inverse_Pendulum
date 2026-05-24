// integrator.js — three ODE steppers for the manipulator-form EOM.
//
//   q_{k+1}, qdot_{k+1} = step(q_k, qdot_k, u_k, dt, params, eom)
//
// `eom` is the module returned from import * as eom from './nlink_N.js'.
// `u` is the cart force (scalar).
//
// Forward Euler  — fast but drifts (gains energy).
// SI Euler       — symplectic-ish: update qdot first, then q using NEW qdot.
//                  Bounded drift, good cheap default.
// RK4            — 4 evals, ~O(dt^4) accuracy, the project default.

function addScaled(a, b, s) {
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + s * b[i];
  return out;
}

function copy(a) {
  return a.slice();
}

export function stepEuler(q, qdot, u, dt, params, eom) {
  const acc = eom.qddot(q, qdot, u, params);
  const q1    = addScaled(q,    qdot, dt);
  const qdot1 = addScaled(qdot, acc,  dt);
  return [q1, qdot1];
}

export function stepSemiImplicit(q, qdot, u, dt, params, eom) {
  const acc   = eom.qddot(q, qdot, u, params);
  const qdot1 = addScaled(qdot, acc, dt);
  const q1    = addScaled(q, qdot1, dt);   // use NEW qdot — symplectic-ish
  return [q1, qdot1];
}

export function stepRK4(q, qdot, u, dt, params, eom) {
  // Treat state as z = [q; qdot]; zdot = [qdot; qddot(q, qdot, u)].
  // u is held constant across the substeps (zero-order hold).
  const f = (qa, qda) => [copy(qda), eom.qddot(qa, qda, u, params)];

  const [k1_q, k1_v] = f(q, qdot);
  const q2  = addScaled(q,    k1_q, dt / 2);
  const v2  = addScaled(qdot, k1_v, dt / 2);
  const [k2_q, k2_v] = f(q2, v2);

  const q3 = addScaled(q,    k2_q, dt / 2);
  const v3 = addScaled(qdot, k2_v, dt / 2);
  const [k3_q, k3_v] = f(q3, v3);

  const q4 = addScaled(q,    k3_q, dt);
  const v4 = addScaled(qdot, k3_v, dt);
  const [k4_q, k4_v] = f(q4, v4);

  const n = q.length;
  const q_next  = new Array(n);
  const qd_next = new Array(n);
  for (let i = 0; i < n; i++) {
    q_next[i]  = q[i]    + dt / 6 * (k1_q[i] + 2 * k2_q[i] + 2 * k3_q[i] + k4_q[i]);
    qd_next[i] = qdot[i] + dt / 6 * (k1_v[i] + 2 * k2_v[i] + 2 * k3_v[i] + k4_v[i]);
  }
  return [q_next, qd_next];
}

const STEPPERS = {
  euler: stepEuler,
  si_euler: stepSemiImplicit,
  rk4: stepRK4,
};

export function step(name, q, qdot, u, dt, params, eom) {
  const s = STEPPERS[name];
  if (!s) throw new Error(`Unknown integrator '${name}'`);
  return s(q, qdot, u, dt, params, eom);
}

// Compute total mechanical energy for an n-link pendulum.
// Used as a debug invariant.
//   E = sum_i [ 0.5 * m_i * |v_com_i|^2  +  0.5 * I_i * theta_i_dot^2 ]
//       + 0.5 * m0 * xdot^2
//       + sum_i m_i * g * y_com_i
export function totalEnergy(q, qdot, params) {
  const n = q.length - 1;
  const x = q[0], xdot = qdot[0];
  let T = 0.5 * params.m0 * xdot * xdot;
  let V = 0;
  // accumulate joint position and velocity
  let jx = x, jy = 0, jxv = xdot, jyv = 0;
  for (let i = 0; i < n; i++) {
    const lnk = params.links[i];
    const th = q[i + 1];
    const thd = qdot[i + 1];
    const s = Math.sin(th), c = Math.cos(th);
    // CoM position and velocity
    const cx = jx + lnk.l * s;
    const cy = jy + lnk.l * c;
    const cxd = jxv + lnk.l * c * thd;
    const cyd = jyv - lnk.l * s * thd;
    T += 0.5 * lnk.m * (cxd * cxd + cyd * cyd) + 0.5 * lnk.I * thd * thd;
    V += lnk.m * params.g * cy;
    // step joint to next link tip
    jx  = jx  + lnk.L * s;
    jy  = jy  + lnk.L * c;
    jxv = jxv + lnk.L * c * thd;
    jyv = jyv - lnk.L * s * thd;
  }
  return T + V;
}
