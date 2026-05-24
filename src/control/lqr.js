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

// ---------- 4.2 CARE solver via matrix sign function ----------
//
// Solve continuous-time algebraic Riccati equation
//     A^T P + P A − P B R^{-1} B^T P + Q = 0
// for symmetric positive-definite P, given (A: n×n, B: n×1 or n×p,
// Q: n×n SPD, R: scalar or p×p SPD).
//
// Method: build Hamiltonian
//     H = [ A     -G ]    where G = B R^{-1} B^T
//         [-Q    -A^T]
// then iterate the matrix sign function
//     Z_{k+1} = ½ (Z_k + Z_k^{-1}) ,   Z_0 = H
// to converge to S = sign(H).  The stable invariant subspace of H is the
// column space of (I - S) / 2.  Take a QR decomposition, split into
// [X1; X2] (top n rows, bottom n rows), then  P = X2 * X1^{-1}.
//
// Works well for small (≤16) matrices typical of this project.

/** Invert a square matrix via Gauss-Jordan with partial pivoting. */
export function matrixInvert(A) {
  const n = A.length;
  // [A | I] augmented
  const M = Array.from({ length: n }, (_, i) => {
    const row = new Array(2 * n).fill(0);
    for (let j = 0; j < n; j++) row[j] = A[i][j];
    row[n + i] = 1;
    return row;
  });
  for (let k = 0; k < n; k++) {
    let piv = k;
    let mx = Math.abs(M[k][k]);
    for (let i = k + 1; i < n; i++) {
      const a = Math.abs(M[i][k]);
      if (a > mx) { mx = a; piv = i; }
    }
    if (mx < 1e-14) throw new Error('matrixInvert: singular');
    if (piv !== k) [M[k], M[piv]] = [M[piv], M[k]];
    const inv = 1 / M[k][k];
    for (let j = 0; j < 2 * n; j++) M[k][j] *= inv;
    for (let i = 0; i < n; i++) {
      if (i === k) continue;
      const f = M[i][k];
      if (f === 0) continue;
      for (let j = 0; j < 2 * n; j++) M[i][j] -= f * M[k][j];
    }
  }
  const inv = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) inv[i][j] = M[i][n + j];
  return inv;
}

/** Matrix sign function via Newton iteration. */
export function matrixSign(A, { maxIter = 60, tol = 1e-12 } = {}) {
  const n = A.length;
  let Z = A.map(r => r.slice());
  for (let k = 0; k < maxIter; k++) {
    const Zinv = matrixInvert(Z);
    const Znew = Array.from({ length: n }, () => new Array(n).fill(0));
    let diff = 0;
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        Znew[i][j] = 0.5 * (Z[i][j] + Zinv[i][j]);
        diff = Math.max(diff, Math.abs(Znew[i][j] - Z[i][j]));
      }
    }
    Z = Znew;
    if (diff < tol) return Z;
  }
  return Z;
}

/** Modified Gram-Schmidt QR. Returns {Q, R}.  A: m×n with m ≥ n. */
export function qrDecompose(A) {
  const m = A.length, n = A[0].length;
  const Q = Array.from({ length: m }, () => new Array(n).fill(0));
  const R = Array.from({ length: n }, () => new Array(n).fill(0));
  const V = A.map(r => r.slice());
  for (let j = 0; j < n; j++) {
    let norm = 0;
    for (let i = 0; i < m; i++) norm += V[i][j] * V[i][j];
    norm = Math.sqrt(norm);
    R[j][j] = norm;
    if (norm < 1e-14) continue;
    for (let i = 0; i < m; i++) Q[i][j] = V[i][j] / norm;
    for (let k = j + 1; k < n; k++) {
      let dot = 0;
      for (let i = 0; i < m; i++) dot += Q[i][j] * V[i][k];
      R[j][k] = dot;
      for (let i = 0; i < m; i++) V[i][k] -= dot * Q[i][j];
    }
  }
  return { Q, R };
}

/** Solve A X = B for X (n×n A, n×p B). */
export function solveLinearSystem(A, B) {
  const Ainv = matrixInvert(A);
  return matMul(Ainv, B);
}

/** Symmetrize a square matrix in-place-ish: returns 0.5(A + A^T). */
function symmetrize(P) {
  const n = P.length;
  const out = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) out[i][j] = 0.5 * (P[i][j] + P[j][i]);
  return out;
}

/** Solve continuous-time algebraic Riccati equation
 *      A^T P + P A − P B R^{-1} B^T P + Q = 0
 *  Returns { P, K }  where  K = R^{-1} B^T P  is the LQR feedback gain
 *  (control law: u = -K x_state). */
export function solveCARE(A, B, Q, R) {
  const n = A.length;
  // B may be passed as a 1-D vector (n,) or 2-D (n,p). Promote to (n,p).
  let Bm;
  if (!Array.isArray(B[0])) Bm = B.map(v => [v]);
  else Bm = B;
  const p = Bm[0].length;

  // R may be scalar or p×p.
  let Rm;
  if (typeof R === 'number') {
    Rm = Array.from({ length: p }, (_, i) => Array.from({ length: p }, (_, j) => i === j ? R : 0));
  } else Rm = R;
  const Rinv = matrixInvert(Rm);
  const BT = transpose(Bm);
  const G = matMul(matMul(Bm, Rinv), BT);   // n×n
  const AT = transpose(A);

  // Hamiltonian H (2n × 2n)
  const N2 = 2 * n;
  const H = Array.from({ length: N2 }, () => new Array(N2).fill(0));
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    H[i][j]         =  A[i][j];
    H[i][j + n]     = -G[i][j];
    H[i + n][j]     = -Q[i][j];
    H[i + n][j + n] = -AT[i][j];
  }

  const S = matrixSign(H);

  // Stable invariant subspace = column space of (I - S) / 2
  const W = Array.from({ length: N2 }, () => new Array(N2).fill(0));
  for (let i = 0; i < N2; i++) for (let j = 0; j < N2; j++) {
    W[i][j] = (i === j ? 1 : 0) - S[i][j];
  }

  // QR of W; keep first n columns of Q as the basis
  const { Q: Qmat } = qrDecompose(W);
  // Take first n columns
  const X1 = Array.from({ length: n }, () => new Array(n).fill(0));
  const X2 = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    X1[i][j] = Qmat[i][j];
    X2[i][j] = Qmat[i + n][j];
  }

  // P = X2 * X1^{-1}
  let P = matMul(X2, matrixInvert(X1));
  P = symmetrize(P);                         // numerical cleanup
  // K = R^{-1} B^T P
  const K_mat = matMul(matMul(Rinv, BT), P);
  // Flatten K if p==1
  const K = (p === 1) ? K_mat[0] : K_mat;

  return { P, K };
}
