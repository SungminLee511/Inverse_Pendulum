// state.js — single source of truth for the whole app.
// Anything observable that the UI or physics or controller needs to read or
// mutate goes through here. Tiny pub/sub so panels and plots can react.
//
// Sign convention: angles measured from up, CCW positive in screen coordinates.

const DEFAULT_PARAMS = {
  // Cart
  m0: 1.0,           // [kg]
  cart_visc: 0.1, // [N s/m]
  cart_coulomb: 0.0, // [N]
  g: 9.81,           // [m/s^2]

  // Per-link defaults (filled below by n)
  links: [
    { m: 0.2, L: 0.5, l: 0.25, I: null, joint_viscous: 0.001, joint_coulomb: 0.0 },
    { m: 0.2, L: 0.4, l: 0.20, I: null, joint_viscous: 0.001, joint_coulomb: 0.0 },
    { m: 0.2, L: 0.3, l: 0.15, I: null, joint_viscous: 0.001, joint_coulomb: 0.0 },
  ],

  // Sensors
  angle_noise: 0.001745,   // 0.1 deg in rad
  cart_noise: 1e-3,        // 1 mm
  quant_bits: 12,
  sensor_period: 2e-3,     // 2 ms (500 Hz)
  sensor_delay: 2e-3,      // 2 ms

  // Actuator
  F_max: 30.0,             // [N]
  motor_tau: 5e-3,         // 5 ms
  slew_max: 5000,          // [N/s]
  force_noise: 0.0,

  // Controller
  ctrl_mode: 'auto',       // 'off'|'swingup'|'lqr'|'auto'|'sysid'
  control_period: 5e-3,    // 5 ms (200 Hz)
  Q_diag: [10, 100, 1, 10],  // grows with n; sized at use site
  R: 0.01,
  handover_theta: 0.35,    // ~20 deg
  handover_omega: 2.0,
  handover_blend_ms: 80,

  // Sim
  integrator: 'rk4',       // 'euler'|'si_euler'|'rk4'
  dt_sim: 1e-4,            // 0.1 ms (10 kHz)
  max_frame_ms: 50,
  seed: 12345,
  start_pose: 'hanging',   // 'hanging' | 'near-upright' | 'upright'

  // Sys-ID
  sysid_excitation: 'chirp',     // 'impulse' | 'step' | 'chirp' | 'prbs'
  sysid_amplitude: 5,            // [N]
};

// Fill inertia I = m L^2 / 12 for nulls
for (const lnk of DEFAULT_PARAMS.links) {
  if (lnk.I == null) lnk.I = lnk.m * lnk.L * lnk.L / 12;
}

// --- Pub/sub ---
const listeners = new Map();   // event name -> Set<fn>

export function on(evt, fn) {
  if (!listeners.has(evt)) listeners.set(evt, new Set());
  listeners.get(evt).add(fn);
  return () => listeners.get(evt).delete(fn);
}

export function emit(evt, payload) {
  const s = listeners.get(evt);
  if (!s) return;
  for (const fn of s) {
    try { fn(payload); } catch (e) { console.error(`[state] listener for ${evt} threw`, e); }
  }
}

// --- Mutable runtime state ---
export const state = {
  // configured
  n: 1,                    // active mode (1, 2, or 3)
  params: structuredClone(DEFAULT_PARAMS),

  // runtime
  running: true,
  speed: 1.0,
  t: 0,
  // q = [x, theta_1, ..., theta_n], qdot = [...]
  q: null,
  qdot: null,

  // controller scratch
  u_cmd: 0,
  u_applied: 0,

  // sensor scratch
  sensor_last: null,
  sensor_buffer: [],

  // diagnostics
  energy: 0,
  fps: 0,
};

// Build the q vector for a given mode. `start` can be:
//   'hanging'      — θ_i = π (default, demanded by PLAN)
//   'near-upright' — θ_i = 0.05 (Phase 13 fallback for n=3 — LQR catches it
//                    immediately so we sidestep the trajopt requirement).
//   'upright'      — θ_i = 0 (debug / smoke-test only).
export function freshQ(n, start = 'hanging') {
  let theta = Math.PI;
  if (start === 'near-upright') theta = 0.05;
  else if (start === 'upright') theta = 0;
  const q = new Float64Array(n + 1);
  const qd = new Float64Array(n + 1);
  for (let i = 1; i <= n; i++) q[i] = theta;
  return { q, qdot: qd };
}

// --- Setters that emit events ---
export function setMode(n) {
  if (n !== 1 && n !== 2 && n !== 3) throw new Error(`Bad mode n=${n}`);
  state.n = n;
  // UX defaults per n:
  //   n=1 → hanging (swing-up demo works).
  //   n≥2 → near-upright (PLAN §10/§13: pure energy pumping doesn't reach
  //          the LQR ROA for multi-link; near-upright lets LQR catch).
  // The user can override either via the Sim-panel start_pose select; we
  // only ROLL the default when no one has explicitly set the field yet OR
  // when the existing setting is the n=1 default and we're switching to n>1.
  if (!state.params.start_pose_user_set) {
    state.params.start_pose = (n === 1) ? 'hanging' : 'near-upright';
  }
  // F_max scales with link count — triple needs more force authority. The
  // user can still override via the Sensor/Actuator panel after.
  if (!state.params.F_max_user_set) {
    state.params.F_max = (n === 1) ? 30 : (n === 2) ? 50 : 80;
  }
  // R also scales — the default R=0.01 makes LQR saturate at F_max on
  // tiny perturbations for multi-link plants (the optimal-control penalty
  // on u is too cheap relative to the rich state-cost). The Phase 9 / 12
  // headless tests use R=0.05–0.1; mirror them here.
  if (!state.params.R_user_set) {
    state.params.R = (n === 1) ? 0.05 : (n === 2) ? 0.1 : 0.05;
  }
  // Control / sensor cadence — n=1 LQR works fine at the textbook 200 Hz,
  // but n≥2 has a much tighter unstable-mode bandwidth (≳ 5 rad/s for the
  // triple) and the velocity-FD lag at 5 ms ZOH destabilises the closed loop
  // even with idealised sensors. Bump to 1 kHz for multi-link.
  if (!state.params.control_period_user_set) {
    state.params.control_period = (n === 1) ? 5e-3 : 1e-3;
  }
  if (!state.params.sensor_period_user_set) {
    state.params.sensor_period = (n === 1) ? 2e-3 : 1e-3;
  }
  // PLAN §9 "triple sensitivity" pitfall: 0.5° sensor noise + 5 ms delay can
  // sink the triple LQR. We harden the sensor pipeline for n≥2 so the
  // velocity-FD doesn't read spurious rad/s at cold-start.
  if (!state.params.angle_noise_user_set) {
    state.params.angle_noise = (n === 1) ? 1.745e-3 : (n === 2) ? 5e-4 : 1e-4;
  }
  if (!state.params.sensor_delay_user_set) {
    state.params.sensor_delay = (n === 1) ? 2e-3 : 0;
  }
  if (!state.params.quant_bits_user_set) {
    state.params.quant_bits = (n === 1) ? 12 : 16;
  }
  const start = state.params.start_pose || 'hanging';
  const fresh = freshQ(n, start);
  state.q = fresh.q;
  state.qdot = fresh.qdot;
  state.t = 0;
  state.u_cmd = 0;
  state.u_applied = 0;
  // Re-shape Q_diag to fit the new state size [x, θ_1..θ_n, ẋ, θ̇_1..θ̇_n].
  // The default array (length 4) is correctly indexed only for n=1; for n>1
  // the index-2 entry would silently leak the n=1 ẋ weight into Q_θ_2 and
  // cripple the LQR. Defaults: Q_x=10, Q_θ=100, Q_ẋ=1, Q_θ̇=10.
  resizeQDiag(n);
  emit('mode-change', n);
  emit('reset', { reason: 'mode-change' });
}

/** Grow/shrink state.params.Q_diag to the right shape for n, preserving any
 *  user-set entries in the new layout where they still make sense.
 *  Defaults are tuned from the Phase 9/12 headless tests that successfully
 *  stabilise the FULL nonlinear EOM. n=1 uses softer θ weight since LQR
 *  catches a swing-up termination there; n≥2 needs heavier weighting on both
 *  θ and θ̇ to damp the coupled modes. */
function resizeQDiag(n) {
  const nq = n + 1;
  const n_state = 2 * nq;
  const Q_theta = (n === 1) ? 100 : 500;
  const Q_omega = (n === 1) ?  10 :  30;
  const next = new Array(n_state);
  for (let i = 0; i < n_state; i++) {
    if (i === 0) next[i] = 10;                // Q[x]
    else if (i < nq) next[i] = Q_theta;       // Q[θ_i]
    else if (i === nq) next[i] = 1;           // Q[ẋ]
    else next[i] = Q_omega;                    // Q[θ̇_i]
  }
  state.params.Q_diag = next;
}

export function setRunning(b) {
  state.running = !!b;
  emit('running-change', state.running);
}

export function setSpeed(s) {
  state.speed = Math.max(0.05, Math.min(10, Number(s) || 1));
  emit('speed-change', state.speed);
}

export function reset() {
  setMode(state.n);
}

export function setParam(path, value) {
  // dotted path like "F_max" or "links.0.m"
  const parts = path.split('.');
  let obj = state.params;
  for (let i = 0; i < parts.length - 1; i++) {
    const k = parts[i];
    obj = obj[isNaN(+k) ? k : +k];
  }
  const last = parts[parts.length - 1];
  const key = isNaN(+last) ? last : +last;
  obj[key] = value;
  // Track whether the user has explicitly chosen a start pose; once they do,
  // mode-change stops rolling the per-mode default underneath them.
  if (path === 'start_pose') state.params.start_pose_user_set = true;
  if (path === 'F_max') state.params.F_max_user_set = true;
  if (path === 'R') state.params.R_user_set = true;
  if (path === 'control_period') state.params.control_period_user_set = true;
  if (path === 'sensor_period')  state.params.sensor_period_user_set = true;
  if (path === 'angle_noise')    state.params.angle_noise_user_set = true;
  if (path === 'sensor_delay')   state.params.sensor_delay_user_set = true;
  if (path === 'quant_bits')     state.params.quant_bits_user_set = true;
  emit('param-change', { path, value });
}

export function getParam(path) {
  const parts = path.split('.');
  let obj = state.params;
  for (const k of parts) {
    if (obj == null) return undefined;
    obj = obj[isNaN(+k) ? k : +k];
  }
  return obj;
}

// Initialise q/qdot for the default mode
setMode(state.n);
