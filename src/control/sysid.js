// control/sysid.js — system identification: excitation + fit.
//
// PLAN §5 / §14 sketch:
//   • Excitation: impulse / step / chirp / PRBS on the cart force u.
//   • Hang test: small-angle pendulum period gives ω_n = √(m·g·l/(I+m·l²)),
//     constraining the (m, l, I) trio for each link.
//   • Output-error fit: starting from a nominal parameter vector p_0, run a
//     local-search (coordinate descent / Nelder-Mead) over a chosen subset
//     of params to minimize Σ_t (q_sim(t; p) − q_meas(t))². Useful when the
//     "measured" data comes from a digital twin (here: the same EOM run with
//     the TRUE plant params, while the fit operates on a "model" with
//     perturbed initial-guess params).
//
// All functions are pure: they take inputs, return outputs, no module state.

import { stepRK4 } from '../physics/integrator.js';

// ---------------------- Excitation generators -----------------------------
// All generators return a function `u(t)` that yields the cart-force command
// at time `t` in seconds.

export function impulseExcitation({ t0 = 0.5, width = 0.05, amplitude = 10 } = {}) {
  return (t) => (t >= t0 && t < t0 + width) ? amplitude : 0;
}

export function stepExcitation({ t0 = 0.5, amplitude = 5 } = {}) {
  return (t) => (t >= t0) ? amplitude : 0;
}

export function chirpExcitation({ f0 = 0.1, f1 = 5.0, duration = 8.0, amplitude = 5 } = {}) {
  // Linear sweep from f0 → f1 over `duration` seconds.
  const k = (f1 - f0) / duration;
  return (t) => {
    if (t < 0 || t > duration) return 0;
    // Instantaneous phase = 2π·(f0·t + ½·k·t²)
    const phase = 2 * Math.PI * (f0 * t + 0.5 * k * t * t);
    return amplitude * Math.sin(phase);
  };
}

/** Pseudo-random binary sequence (m-sequence of length 2^n − 1). */
export function prbsExcitation({ amplitude = 5, dt_switch = 0.1, seed = 12345 } = {}) {
  // 11-bit maximal-length LFSR (length 2047). Bit indices for the XOR tap
  // (11, 9) give an m-sequence.
  let reg = seed & 0x7FF || 1;
  const seq = [];
  for (let i = 0; i < 2047; i++) {
    const out = reg & 1;
    seq.push(out);
    const newBit = ((reg >> 10) ^ (reg >> 8)) & 1;
    reg = ((reg >> 1) | (newBit << 10)) & 0x7FF;
  }
  return (t) => {
    if (t < 0) return 0;
    const idx = Math.floor(t / dt_switch) % seq.length;
    return seq[idx] ? amplitude : -amplitude;
  };
}

// ---------------------- Hang-test small-angle fit -------------------------
//
// For link i isolated (no cart drive), small-angle natural frequency is
//     ω_n² = m_i · g · l_i / (I_i + m_i · l_i²)
// → period T_n = 2π / ω_n.
// Measuring T_n (e.g. from a free-swing zero-crossing time series) gives
// one scalar constraint per link.

/** Predicted period of a single pendulum with given physical params. */
export function pendulumPeriod({ m, g, l, I }) {
  const wn2 = (m * g * l) / (I + m * l * l);
  if (wn2 <= 0) return Infinity;
  return 2 * Math.PI / Math.sqrt(wn2);
}

/** Detect period from a θ(t) signal via consecutive zero-crossings. */
export function periodFromZeroCrossings(t, theta, { thetaOffset = 0 } = {}) {
  const xs = [];     // times of upward zero-crossings of (theta - thetaOffset)
  for (let i = 1; i < theta.length; i++) {
    const a = theta[i - 1] - thetaOffset;
    const b = theta[i] - thetaOffset;
    if (a < 0 && b >= 0) {
      // linear interp
      const frac = a / (a - b);
      xs.push(t[i - 1] + frac * (t[i] - t[i - 1]));
    }
  }
  if (xs.length < 2) return null;
  let sum = 0;
  for (let i = 1; i < xs.length; i++) sum += xs[i] - xs[i - 1];
  return sum / (xs.length - 1);
}

// ---------------------- Output-error fit ----------------------------------
//
// Run the integrator forward over a known reference trajectory (q_meas[t]),
// minimize the L2 error between simulated and measured q. Local coordinate-
// descent over a list of (path, lo, hi) param entries; cheap and good
// enough for one-shot bench-top sysid.

/** Simulate the n-link plant over T seconds with a given param set and
 *  excitation function u(t). Returns { ts, qs } (ts: array of times, qs:
 *  array of q snapshots, sampled every `sample_dt`). */
export function simulateOpen(eom, params, u_fn, q0, qdot0, T, dt_sim = 1e-4, sample_dt = 0.01) {
  let q = q0.slice(), qdot = qdot0.slice();
  const N = Math.round(T / dt_sim);
  const stride = Math.max(1, Math.round(sample_dt / dt_sim));
  const ts = [], qs = [];
  for (let k = 0; k <= N; k++) {
    if (k % stride === 0) {
      ts.push(k * dt_sim);
      qs.push(q.slice());
    }
    if (k === N) break;
    const u = u_fn(k * dt_sim);
    [q, qdot] = stepRK4(q, qdot, u, dt_sim, params, eom);
  }
  return { ts, qs };
}

/** L2 error between two sampled trajectories of identical length. */
export function trajError(qs_a, qs_b) {
  let s = 0;
  const N = qs_a.length;
  const dim = qs_a[0].length;
  for (let i = 0; i < N; i++)
    for (let j = 0; j < dim; j++) {
      const d = qs_a[i][j] - qs_b[i][j];
      s += d * d;
    }
  return Math.sqrt(s / (N * dim));
}

/** Set a nested param by dotted path (mirrors state.setParam without the bus). */
function setParamLocal(params, path, value) {
  const parts = path.split('.');
  let obj = params;
  for (let i = 0; i < parts.length - 1; i++) {
    const k = parts[i];
    obj = obj[isNaN(+k) ? k : +k];
  }
  const last = parts[parts.length - 1];
  obj[isNaN(+last) ? last : +last] = value;
}
function getParamLocal(params, path) {
  const parts = path.split('.');
  let obj = params;
  for (const k of parts) {
    if (obj == null) return undefined;
    obj = obj[isNaN(+k) ? k : +k];
  }
  return obj;
}

/** Coordinate-descent output-error fit.
 *  - knobs: array of { path, lo, hi } — params to optimize.
 *  - measurement: { ts, qs } from the TRUE plant.
 *  - paramsGuess: initial guess (will be deep-cloned).
 *  Returns { params, error_final, iters }. */
export function fitOutputError({ eom, paramsGuess, knobs, measurement, q0, qdot0,
                                 u_fn, T = 4.0, dt_sim = 1e-4, sample_dt = 0.01,
                                 maxIters = 20, tol = 1e-4 }) {
  const p = structuredClone(paramsGuess);
  function err(pp) {
    const sim = simulateOpen(eom, pp, u_fn, q0, qdot0, T, dt_sim, sample_dt);
    // align lengths
    const N = Math.min(sim.qs.length, measurement.qs.length);
    return trajError(sim.qs.slice(0, N), measurement.qs.slice(0, N));
  }
  let cur = err(p);
  let iters = 0;
  // Per-knob adaptive step that shrinks when the local sweep fails to improve.
  const stepSize = new Map(knobs.map(k => [k.path, (k.hi - k.lo) * 0.25]));
  for (let it = 0; it < maxIters; it++) {
    iters++;
    let improved = false;
    for (const k of knobs) {
      const v0 = getParamLocal(p, k.path);
      const step = stepSize.get(k.path);
      const trials = [v0 + step, v0 - step,
                      v0 + 0.5 * step, v0 - 0.5 * step,
                      v0 + 0.25 * step, v0 - 0.25 * step,
                      v0 + 0.1 * step, v0 - 0.1 * step];
      let bestV = v0, bestE = cur;
      for (const tv of trials) {
        if (tv < k.lo || tv > k.hi) continue;
        setParamLocal(p, k.path, tv);
        const e = err(p);
        if (e < bestE) { bestE = e; bestV = tv; }
      }
      setParamLocal(p, k.path, bestV);
      if (bestE < cur - tol) {
        cur = bestE;
        improved = true;
        // Successful step → keep stepSize; if we kept the smallest fine
        // direction, also shrink for fine-tuning next pass.
        if (Math.abs(bestV - v0) <= 0.11 * step) stepSize.set(k.path, step * 0.5);
      } else {
        stepSize.set(k.path, step * 0.5);    // shrink and retry next iter
      }
    }
    if (!improved && Array.from(stepSize.values()).every(s => s < tol * 100)) break;
  }
  return { params: p, error_final: cur, iters };
}
