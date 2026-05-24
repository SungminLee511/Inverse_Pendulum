// actuator.js — models the chain between controller-commanded force and the
// force that actually reaches the cart in the EOM.
//
//   u_cmd  →  saturate  →  slew  →  first-order lag  →  +noise  →  u_applied
//   F_friction_coulomb = -F_c * tanh(xdot / eps)   (opposes motion)
//   u_effective = u_applied - F_friction_coulomb
//
// The EOM's `D` term already handles VISCOUS friction (proportional to xdot).
// This module adds Coulomb (static-like) friction, since the EOM as derived has
// no sign(xdot) term.
//
// State held: last_u_cmd_pre_sat (for slew), u_applied (for lag).

import { state } from './state.js';

function mulberry32(seed) {
  let s = seed >>> 0;
  return function () {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gaussianFn(rng) {
  let cached = null;
  return function () {
    if (cached != null) { const r = cached; cached = null; return r; }
    let u1, u2;
    do { u1 = rng(); } while (u1 === 0);
    u2 = rng();
    const mag = Math.sqrt(-2 * Math.log(u1));
    cached = mag * Math.sin(2 * Math.PI * u2);
    return mag * Math.cos(2 * Math.PI * u2);
  };
}

export class Actuator {
  constructor({ F_max, slew_max, motor_tau, force_noise, cart_coulomb, rng }) {
    this.F_max = F_max;
    this.slew_max = slew_max;
    this.motor_tau = motor_tau;
    this.force_noise = force_noise || 0;
    this.cart_coulomb = cart_coulomb || 0;
    this.gauss = rng ? gaussianFn(rng) : null;
    this.u_pre_lag = 0;     // post-saturation, post-slew, pre-lag
    this.u_applied = 0;     // post-lag, pre-noise (state mirror)
  }

  /** One actuator update: maps u_cmd to (u_applied, u_effective).
   *  dt is the controller period (or whatever sub-step you want — the lag uses dt). */
  step(u_cmd, dt, xdot = 0) {
    // 1) saturate
    let u = Math.max(-this.F_max, Math.min(this.F_max, u_cmd));
    // 2) slew limit (relative to the prior u_pre_lag)
    if (this.slew_max > 0 && dt > 0) {
      const maxDelta = this.slew_max * dt;
      const delta = u - this.u_pre_lag;
      if (delta >  maxDelta) u = this.u_pre_lag + maxDelta;
      if (delta < -maxDelta) u = this.u_pre_lag - maxDelta;
    }
    this.u_pre_lag = u;
    // 3) first-order lag: u_applied += (u - u_applied) * dt / tau
    if (this.motor_tau > 0 && dt > 0) {
      const alpha = dt / (this.motor_tau + dt);
      this.u_applied += alpha * (u - this.u_applied);
    } else {
      this.u_applied = u;
    }
    // 4) additive force noise
    const noise = (this.gauss && this.force_noise > 0) ? this.gauss() * this.force_noise : 0;
    const u_with_noise = this.u_applied + noise;
    // 5) Coulomb friction on cart (smoothed via tanh)
    const eps = 0.01;
    const f_coulomb = this.cart_coulomb * Math.tanh(xdot / eps);
    const u_effective = u_with_noise - f_coulomb;
    return { u_applied: this.u_applied, u_with_noise, u_effective, f_coulomb };
  }

  reset() {
    this.u_pre_lag = 0;
    this.u_applied = 0;
  }
}

// ---- Bank wired to global state ----
let _actuator = null;

export function initActuator(seed = state.params.seed || 42) {
  const p = state.params;
  _actuator = new Actuator({
    F_max: p.F_max,
    slew_max: p.slew_max,
    motor_tau: p.motor_tau,
    force_noise: p.force_noise,
    cart_coulomb: p.cart_coulomb,
    rng: mulberry32(seed),
  });
}

/** Apply the actuator to state.u_cmd → state.u_applied. Called from the loop's
 *  control step (so the cadence is the controller's). */
export function actuatorTick(dt = state.params.control_period) {
  if (!_actuator) return;
  // Keep tunable params live (panel slider may have changed them since init)
  const p = state.params;
  _actuator.F_max        = p.F_max;
  _actuator.slew_max     = p.slew_max;
  _actuator.motor_tau    = p.motor_tau;
  _actuator.force_noise  = p.force_noise;
  _actuator.cart_coulomb = p.cart_coulomb;

  const xdot = state.qdot[0];
  const out = _actuator.step(state.u_cmd, dt, xdot);
  state.u_applied   = out.u_applied;
  state.u_with_noise = out.u_with_noise;
  state.u_effective = out.u_effective;
  state.f_coulomb   = out.f_coulomb;
}

export function _resetActuator() { _actuator && _actuator.reset(); }

export const _internal = { mulberry32, gaussianFn, getActuator: () => _actuator };
