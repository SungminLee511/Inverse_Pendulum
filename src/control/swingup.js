// control/swingup.js — energy-based swing-up controllers.
//
// Single (n=1) — Åström–Furuta:
//   E_p  = pendulum mechanical energy (cart-relative frame; cart KE excluded so
//          feedback that adds cart KE doesn't fool the pumping law).
//   E_p* = upright pendulum energy = sum_i m_i g l_i .
//   Ẽ    = E_p − E_p*.
//   u    = −k_E · Ẽ · σ(θ̇·cosθ)   with σ(.) a soft sign (tanh-saturated).
//
// Properties:
//   - When Ẽ<0 (under-energized) and θ̇·cosθ ≠ 0, this pushes the cart in the
//     direction that injects mechanical energy into the pendulum.
//   - When Ẽ>0 (over-energized), it removes energy. So Ẽ → 0.
//   - In the homoclinic limit (Ẽ=0 along the separatrix), the trajectory
//     passes through θ=0, θ̇=0, which is the LQR catch point.
//
// Practical fixes for cart-pendulum:
//   - SOFT centering on x and ẋ to prevent the cart sliding off the rail (the
//     law alone has no cart-position regulation).
//   - BOOTSTRAP kick when the system is at rest exactly at hanging (the
//     pumping law is identically zero there).
//   - Energy reference is shifted by the cart KE when |x|>x_soft so the law
//     doesn't keep pumping when energy is already "spent" on cart motion.
//
// Sign convention: θ_i from up, CCW positive. Hanging = π. Upright = 0.
//
// Double (n=2) — pumps using the sum of per-link θ̇·cosθ contributions.
// Triple (n=3) — same fallback; Phase 13's trajopt + TVLQR is the real plan.

// ---------- Pendulum-only energy (cart-relative) ----------
function pendulumEnergy(q, qdot, params) {
  const n = q.length - 1;
  let T = 0, V = 0;
  let jx = 0, jy = 0, jxv = 0, jyv = 0;   // cart x ≡ 0 in this frame
  for (let i = 0; i < n; i++) {
    const lnk = params.links[i];
    const th = q[i + 1], thd = qdot[i + 1];
    const s = Math.sin(th), c = Math.cos(th);
    const cx = jx + lnk.l * s;
    const cy = jy + lnk.l * c;
    const cxd = jxv + lnk.l * c * thd;
    const cyd = jyv - lnk.l * s * thd;
    T += 0.5 * lnk.m * (cxd * cxd + cyd * cyd) + 0.5 * lnk.I * thd * thd;
    V += lnk.m * params.g * cy;
    jx  += lnk.L * s;
    jy  += lnk.L * c;
    jxv += lnk.L * c * thd;
    jyv -= lnk.L * s * thd;
  }
  return T + V;
}

function uprightEnergy(n, params) {
  return pendulumEnergy(new Array(n + 1).fill(0), new Array(n + 1).fill(0), params);
}

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

// Module-level state. One pendulum at a time.
let _bootStartT = null;        // when bootstrap began
let _bootDone = false;
let _lastT = -Infinity;

/** Compute swing-up u_cmd. Takes n, q, qdot, params, sim time t.
 *  Two-stage controller:
 *    Stage 1 (bootstrap): drive cart with a square wave near the pendulum's
 *      natural frequency f_n = (1/2π) sqrt(m·g·l / (I + m·l²)). This forces
 *      a sustained oscillation regardless of initial conditions and breaks the
 *      pumping law's degeneracy at hanging-with-zero-velocity.
 *    Stage 2 (Aström-Furuta pumping): once the pendulum is visibly swinging
 *      (|θ̇| above threshold or Ẽ above hanging by some margin), switch to the
 *      energy-based law u = −k_E·Ẽ·σ(θ̇·cosθ) + soft cart centering.
 */
export function swingupU(n, q, qdot, params, t = 0) {
  const E = pendulumEnergy(q, qdot, params);
  const E_star = uprightEnergy(n, params);
  const E_tilde = E - E_star;
  const F_max = params.F_max || 30;
  // Use nullish coalescing so a tuned-to-zero gain like swingup_kxP=0 is
  // honoured instead of silently picking up the default.
  const k_E      = params.swingup_kE     ?? 80.0;
  const k_xP     = params.swingup_kxP    ?? 0.6;
  const k_xD     = params.swingup_kxD    ?? 0.8;
  const eps_om   = params.swingup_epsOm  ?? 0.4;
  const omega_min= params.swingup_omegaMin ?? 3.0;   // [rad/s] required to exit boot
  const boot_min = params.swingup_bootMin  ?? 1.5;   // [s] min bootstrap time

  // dt bookkeeping (unused here but kept for future)
  _lastT = t;

  // Decide stage. We treat "bootstrap" as a latched mode: once we leave it,
  // we don't re-enter (avoids thrashing if the pendulum briefly stalls at the
  // top of a swing).
  let max_om = 0;
  for (let i = 1; i <= n; i++) max_om = Math.max(max_om, Math.abs(qdot[i]));

  if (!_bootDone) {
    if (_bootStartT == null) _bootStartT = t;
    const elapsed = t - _bootStartT;
    // Exit when pendulum is genuinely swinging (|θ̇| above omega_min) and we've
    // run at least boot_min seconds (so a single tip past omega_min doesn't
    // trip us out prematurely).
    if (elapsed > boot_min && max_om > omega_min) {
      _bootDone = true;
    } else {
      // Square wave at f ≈ f_natural of first link.
      const lnk = params.links[0];
      const omega_n = Math.sqrt(lnk.m * params.g * lnk.l / (lnk.I + lnk.m * lnk.l * lnk.l));
      const T_n = 2 * Math.PI / omega_n;
      const phase = ((t - _bootStartT) % T_n) / T_n;     // [0,1)
      const dir = (phase < 0.5) ? +1 : -1;
      // Soft cart centering even during bootstrap (don't let cart fly off).
      const u_boot = dir * F_max - k_xP * q[0] - 0.5 * k_xD * qdot[0];
      return clamp(u_boot, -F_max, F_max);
    }
  }

  // ----- Aström-Furuta pumping ----------------------------------------------
  // Derivation (cart frame, ignoring back-reaction on cart):
  //   dE_p/dt = -m·l·θ̇·cosθ·ẍ_cart
  // To ADD energy (Ẽ<0) we need -m·l·θ̇·cosθ·ẍ > 0  ⇔  u and θ̇·cosθ
  // have opposite signs (since ẍ_cart has the sign of u).
  // To REMOVE energy (Ẽ>0) we want u and θ̇·cosθ to have the same sign.
  // Both cases unified by   u_pump = +k_E · Ẽ · σ(θ̇·cosθ) .
  //
  // For n>1 we weight each link's contribution by m_i · l_i — that's the
  // physically-correct prefactor in dE_p/dt = Σ m_i l_i θ̇_i cosθ_i · ẍ_cart.
  let sm = 0;
  let totalW = 0;
  for (let i = 1; i <= n; i++) {
    const lnk = params.links[i - 1];
    const w = lnk.m * lnk.l;
    sm += w * Math.tanh((qdot[i] * Math.cos(q[i])) / eps_om);
    totalW += w;
  }
  if (totalW > 0) sm /= totalW;
  const u_pump = +k_E * E_tilde * sm;
  const u_cart = -k_xP * q[0] - k_xD * qdot[0];
  return clamp(u_pump + u_cart, -F_max, F_max);
}

/** Reset internal swing-up state (mode change / sim reset). */
export function resetSwingup() {
  _bootStartT = null;
  _bootDone = false;
  _lastT = -Infinity;
}

/** Diagnostic handle so tests can probe (E, E*, Ẽ). */
export function swingupDiag(n, q, qdot, params) {
  const E = pendulumEnergy(q, qdot, params);
  const E_star = uprightEnergy(n, params);
  return { E, E_star, E_tilde: E - E_star };
}

export { pendulumEnergy };
