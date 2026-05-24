// physics/index.js — registry mapping mode n → EOM module.
// n=1 is real (sympy-generated). n=2, n=3 fall back to a placeholder until
// Phases 8 and 11 fill them in.

import * as eom1 from './nlink_1.js';

const placeholder = {
  N: null, DOF: null,
  M: () => null,
  Cqdot: () => null,
  G: () => null,
  Dqdot: () => null,
  // Damped oscillation around hanging — visually alive but not physical.
  qddot: (q, qdot /*, u, params */) => {
    const n = q.length - 1;
    const out = new Array(n + 1);
    out[0] = 0;
    for (let i = 1; i <= n; i++) {
      out[i] = - (q[i] - Math.PI) * 4 - 0.5 * qdot[i];
    }
    return out;
  },
  placeholder: true,
};

const MODULES = {
  1: eom1,
  2: placeholder,    // Phase 8 replaces this
  3: placeholder,    // Phase 11 replaces this
};

export function getEOM(n) {
  const m = MODULES[n];
  if (!m) throw new Error(`No EOM for n=${n}`);
  return m;
}

export function setEOM(n, mod) {
  MODULES[n] = mod;
}

export function isReal(n) {
  return MODULES[n] && !MODULES[n].placeholder;
}
