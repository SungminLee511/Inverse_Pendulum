// control/controller.js — top-level controller dispatcher.
//
// Reads from state.sensor_last (positions) and state.sensor_vel_est (velocities),
// builds the LQR state vector x_state = [x, θ_1, ..., θ_n, xdot, θ_1_dot, ...],
// computes u via the selected controller, and writes state.u_cmd. The actuator
// then runs in the same control tick to convert u_cmd to u_applied.
//
// Phase 4: LQR-only path is live. Phase 5 adds swing-up and the switcher.

import { state, on } from '../state.js';
import { linearize, solveCARE } from './lqr.js';
import { isReal } from '../physics/index.js';
import { swingupU, resetSwingup } from './swingup.js';
import { HandoverSwitcher } from './switcher.js';

let _K = null;           // current gain vector (length n_state)
let _Kn = -1;            // mode the gain was computed for
let _dirty = true;       // recompute on next tick
let _switcher = new HandoverSwitcher();   // swingup ↔ LQR handover

function recompute() {
  if (!isReal(state.n)) { _K = null; _Kn = state.n; _dirty = false; return; }
  try {
    const { A, B } = linearize(state.n, state.params);
    const nq = state.n + 1;
    const n_state = 2 * nq;
    // Build Q diagonal from params. Defaults: Q[x]=1, Q[θ_i]=100, Q[xdot]=1, Q[θdot_i]=10.
    const Qdiag = new Array(n_state).fill(1);
    const userQ = state.params.Q_diag || [];
    for (let i = 0; i < n_state; i++) {
      if (userQ[i] !== undefined) Qdiag[i] = userQ[i];
      else if (i === 0) Qdiag[i] = 1;                 // x
      else if (i < nq) Qdiag[i] = 100;                // θ_i
      else if (i === nq) Qdiag[i] = 1;                // xdot
      else Qdiag[i] = 10;                              // θ_i_dot
    }
    const Q = Array.from({ length: n_state }, (_, i) =>
      Array.from({ length: n_state }, (_, j) => i === j ? Qdiag[i] : 0));
    const R = state.params.R || 0.05;
    const { K } = solveCARE(A, B, Q, R);
    _K = K; _Kn = state.n; _dirty = false;
  } catch (e) {
    console.warn('[controller] LQR recompute failed:', e);
    _K = null; _dirty = false;
  }
}

export function initController() {
  recompute();
  on('mode-change', () => {
    _dirty = true;
    _switcher.reset();
    resetSwingup();
  });
  on('reset', () => {
    _switcher.reset();
    resetSwingup();
  });
  on('param-change', ({ path }) => {
    // Only physical / actuator / control gain params affect K.
    if (path.startsWith('R') || path === 'g' || path === 'm0' || path === 'cart_visc'
        || path.startsWith('links') || path === 'F_max'
        || path === 'Q_diag' || path.startsWith('Q_diag.')) {
      _dirty = true;
    }
  });
}

/** Build x_state from the most recent sensor sample + velocity estimate.
 *  Falls back to true state if sensor pipeline hasn't produced a value yet. */
function readXState() {
  const nq = state.n + 1;
  const ns = 2 * nq;
  const x = new Array(ns).fill(0);
  const sl = state.sensor_last;
  const sv = state.sensor_vel_est;
  for (let i = 0; i < nq; i++) {
    x[i] = (sl && Number.isFinite(sl[i])) ? sl[i] : state.q[i];
    x[i + nq] = (sv && Number.isFinite(sv[i])) ? sv[i] : state.qdot[i];
  }
  return x;
}

function lqrU() {
  if (!_K) return 0;
  const x = readXState();
  for (let i = 1; i <= state.n; i++) {
    let a = x[i];
    while (a >  Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    x[i] = a;
  }
  let u = 0;
  for (let i = 0; i < _K.length; i++) u -= _K[i] * x[i];
  const F_max = state.params.F_max || 30;
  return Math.max(-F_max, Math.min(F_max, u));
}

function swU() {
  return swingupU(state.n, state.q, state.qdot, state.params, state.t);
}

/** Controller dispatcher.
 *  - 'off'     : leave state.u_cmd alone (tests / scripts may drive it)
 *  - 'lqr'     : raw LQR (clipped)  — for steady-state demos / lqr_live tests
 *  - 'swingup' : raw swingup (no handover) — pedagogical
 *  - 'auto'    : swingup → blend → LQR via the HandoverSwitcher
 *  - 'sysid'   : reserved for Phase 14
 */
export function controllerTick() {
  if (_dirty) recompute();
  const m = state.params.ctrl_mode;
  if (m === 'off') return;
  if (m === 'lqr') { state.u_cmd = lqrU(); return; }
  if (m === 'swingup') { state.u_cmd = swU(); return; }
  if (m === 'sysid') { state.u_cmd = 0; return; }   // Phase 14
  // 'auto' (default) — blended
  if (!_K) { state.u_cmd = swU(); return; }   // can't LQR without K
  state.u_cmd = _switcher.mix(state.t, state.q, state.qdot, state.params, swU, lqrU);
}

/** Expose the current gain for diagnostics / tests. */
export function getK() { return _K ? _K.slice() : null; }
export function markDirty() { _dirty = true; }
