// control/switcher.js — picks between swing-up and LQR.
//
// Region-of-attraction proxy (single):
//   |θ_i (wrapped to (-π,π])| < θ_thresh    (default 0.35 rad ≈ 20°)
//   |θ̇_i|                     < ω_thresh    (default 2 rad/s)
//   |x|, |ẋ|                  bounded by sensible defaults.
//
// To avoid a step in u when crossing the boundary, we linearly blend
// u = α·u_LQR + (1-α)·u_swingup over `handover_blend_ms` after entering the ROA.
//
// State held: latched_in_roa flag, blend_start_t.

const TWO_PI = 2 * Math.PI;

function wrap(a) {
  let x = a;
  while (x >  Math.PI) x -= TWO_PI;
  while (x < -Math.PI) x += TWO_PI;
  return x;
}

export class HandoverSwitcher {
  constructor() {
    this.in_roa = false;
    this.blend_start_t = -Infinity;
    this._everSwung = false;     // has u_swing() ever been called?
  }

  inROA(q, qdot, params) {
    const theta_t = params.handover_theta || 0.35;
    const omega_t = params.handover_omega || 2.0;
    // angles
    for (let i = 1; i < q.length; i++) {
      if (Math.abs(wrap(q[i])) > theta_t) return false;
      if (Math.abs(qdot[i]) > omega_t) return false;
    }
    if (Math.abs(q[0])    > 1.5)  return false;
    if (Math.abs(qdot[0]) > 2.5)  return false;
    return true;
  }

  /** Compute u given access to both swing-up and LQR producers.
   *  - u_swing : () => number  (swing-up command)
   *  - u_lqr   : () => number  (LQR command from same state)
   *  - t       : sim time (seconds)
   *
   *  Behaviour:
   *    - If we ENTER the ROA (false → true), start a linear blend.
   *    - If we LEAVE the ROA, drop back to pure swing-up immediately.
   *    - If the FIRST call already finds us in the ROA (e.g. user starts the
   *      sim near upright with the "near-upright start" fallback), skip the
   *      blend and go straight to LQR. The blend exists to soften swing-up→
   *      LQR transients; if we never ran swing-up there's nothing to soften.
   */
  mix(t, q, qdot, params, u_swing, u_lqr) {
    const inside = this.inROA(q, qdot, params);
    if (inside && !this.in_roa) {
      this.in_roa = true;
      // First-ever ROA entry with no swing-up history → skip blend.
      this.blend_start_t = this._everSwung ? t : -Infinity;
    } else if (!inside && this.in_roa) {
      this.in_roa = false;
      this.blend_start_t = -Infinity;
    }
    if (!this.in_roa) {
      this._everSwung = true;
      return u_swing();
    }
    const blend_ms = params.handover_blend_ms || 80;
    const elapsed_ms = Math.max(0, (t - this.blend_start_t) * 1000);
    const alpha = Math.min(1, elapsed_ms / blend_ms);
    if (!Number.isFinite(this.blend_start_t)) return u_lqr();    // skipped blend
    const us = u_swing();
    const ul = u_lqr();
    return alpha * ul + (1 - alpha) * us;
  }

  reset() {
    this.in_roa = false;
    this.blend_start_t = -Infinity;
    this._everSwung = false;
  }
}
