// control/lqr.js — linearization + Kleinman CARE (filled in 4.2).
//
// State vector for LQR (different from the manipulator coords):
//   x_state = [ q ; qdot ]   of length 2*(n+1)
//
// The continuous-time dynamics:
//   d/dt x_state = [ qdot ; qddot(q, qdot, u, params) ]
//
// Linearizing at the upright equilibrium (q=0, qdot=0, u=0):
//   dxdot = A * dx + B * du
//
// A has block form
//     A = [ 0      I       ]
//         [ Aqq    Aqqd    ]
// where Aqq  = ∂qddot/∂q,
//       Aqqd = ∂qddot/∂qdot,
// both evaluated numerically with central differences.
//
// B = [ 0 ; ∂qddot/∂u ]   (only the bottom half is nonzero — u acts on the cart)

import * as eom1 from '../physics/nlink_1.js';
import { getEOM } from '../physics/index.js';

// Map n → eom module. Falls back to runtime registry so future n=2/3 modules
// auto-pick up.
function eomFor(n) {
  if (n === 1) return eom1;
  const m = getEOM(n);
  if (m && !m.placeholder) return m;
  return null;
}

/** Evaluate the dynamics qddot for a 2(n+1) state vector and scalar u. */
function dyn(eom, x_state, u, params, nq) {
  const q = new Array(nq);
  const qdot = new Array(nq);
  for (let i = 0; i < nq; i++) {
    q[i] = x_state[i];
    qdot[i] = x_state[i + nq];
  }
  return eom.qddot(q, qdot, u, params);
}

/** Central-difference numerical Jacobian of `dyn` wrt x_state component k.
 *  Returns the n-vector ∂qddot/∂x_state[k]. */
function jac_x(eom, x_state, u, params, nq, k, eps = 1e-6) {
  const xp = x_state.slice(), xm = x_state.slice();
  xp[k] += eps; xm[k] -= eps;
  const fp = dyn(eom, xp, u, params, nq);
  const fm = dyn(eom, xm, u, params, nq);
  const out = new Array(nq);
  for (let i = 0; i < nq; i++) out[i] = (fp[i] - fm[i]) / (2 * eps);
  return out;
}

function jac_u(eom, x_state, u, params, nq, eps = 1e-6) {
  const fp = dyn(eom, x_state, u + eps, params, nq);
  const fm = dyn(eom, x_state, u - eps, params, nq);
  const out = new Array(nq);
  for (let i = 0; i < nq; i++) out[i] = (fp[i] - fm[i]) / (2 * eps);
  return out;
}

/** Return {A, B, n_state} for the n-link system linearized at the upright equilibrium
 *  with optional override of the operating point (default: q=0, qdot=0, u=0).        */
export function linearize(n, params, { x0 = null, u0 = 0, eps = 1e-6 } = {}) {
  const eom = eomFor(n);
  if (!eom) throw new Error(`linearize: no real EOM for n=${n}`);
  const nq = n + 1;
  const n_state = 2 * nq;
  const x = x0 ? x0.slice() : new Array(n_state).fill(0);

  // A is n_state × n_state
  const A = Array.from({ length: n_state }, () => new Array(n_state).fill(0));
  // Top half: d/dt q = qdot  →  A[0..nq, nq..n_state] = I
  for (let i = 0; i < nq; i++) A[i][nq + i] = 1.0;
  // Bottom half: A[nq+i][k] = ∂qddot[i]/∂x_state[k]
  for (let k = 0; k < n_state; k++) {
    const col = jac_x(eom, x, u0, params, nq, k, eps);
    for (let i = 0; i < nq; i++) A[nq + i][k] = col[i];
  }

  // B is n_state × 1
  const dqd_du = jac_u(eom, x, u0, params, nq, eps);
  const B = new Array(n_state).fill(0);
  for (let i = 0; i < nq; i++) B[nq + i] = dqd_du[i];

  return { A, B, n_state };
}

// ---------- Linear-algebra utilities ----------
export function matMul(A, B) {
  const m = A.length, n = A[0].length, p = B[0].length;
  const out = Array.from({ length: m }, () => new Array(p).fill(0));
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < p; j++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += A[i][k] * B[k][j];
      out[i][j] = s;
    }
  }
  return out;
}

export function matVec(A, v) {
  const m = A.length, n = A[0].length;
  const out = new Array(m).fill(0);
  for (let i = 0; i < m; i++) {
    let s = 0;
    for (let j = 0; j < n; j++) s += A[i][j] * v[j];
    out[i] = s;
  }
  return out;
}

export function transpose(A) {
  const m = A.length, n = A[0].length;
  const T = Array.from({ length: n }, () => new Array(m).fill(0));
  for (let i = 0; i < m; i++) for (let j = 0; j < n; j++) T[j][i] = A[i][j];
  return T;
}

/** Compute the rank of a (m × p) matrix by QR with column pivoting. */
export function matrixRank(A, tol = 1e-9) {
  const m = A.length;
  if (m === 0) return 0;
  const p = A[0].length;
  // Gauss-Jordan with row pivoting; works for small matrices.
  const M = A.map(r => r.slice());
  let rank = 0;
  let row = 0;
  for (let col = 0; col < p && row < m; col++) {
    // find pivot
    let piv = -1, max = tol;
    for (let i = row; i < m; i++) {
      const a = Math.abs(M[i][col]);
      if (a > max) { max = a; piv = i; }
    }
    if (piv < 0) continue;
    [M[row], M[piv]] = [M[piv], M[row]];
    const inv = 1 / M[row][col];
    for (let j = col; j < p; j++) M[row][j] *= inv;
    for (let i = 0; i < m; i++) {
      if (i === row) continue;
      const f = M[i][col];
      if (Math.abs(f) < tol) continue;
      for (let j = col; j < p; j++) M[i][j] -= f * M[row][j];
    }
    row++; rank++;
  }
  return rank;
}

/** Controllability matrix [B, AB, A²B, ..., A^(n-1) B] and rank.   */
export function controllability(A, B) {
  const n = A.length;
  // collect columns A^k B
  const cols = [];
  let v = B.slice();
  cols.push(v);
  for (let k = 1; k < n; k++) {
    v = matVec(A, v);
    cols.push(v);
  }
  // assemble m × n matrix
  const M = Array.from({ length: n }, () => new Array(cols.length).fill(0));
  for (let i = 0; i < n; i++) for (let j = 0; j < cols.length; j++) M[i][j] = cols[j][i];
  return { M, rank: matrixRank(M) };
}

// ---------- 4.2 CARE placeholder (filled next) ----------
export function solveCARE(/* A, B, Q, R */) { throw new Error('solveCARE: filled in Phase 4.2'); }
