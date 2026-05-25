#!/usr/bin/env python
"""trajopt_triple.py — offline trajectory optimization for n=3 swing-up.

PLAN §13.2: "tools/trajopt_triple.py (CasADi or scipy direct collocation):
minimize ∫u²dt s.t. EOM, x(0)=hanging, x(T)=upright, |u|≤F_max. Dump
(x*(t), u*(t)) + TVLQR K(t) as JSON."

This file is a self-contained Hermite-Simpson direct collocation driver in
scipy. CasADi would be faster but adds a dep; scipy is enough for one-shot
generation of the reference trajectory + TVLQR gain schedule. The resulting
`trajopt_triple.json` is loaded by an in-browser TVLQR tracker (NOT shipped
in this commit — Phase 13 ships the doc and the trajopt skeleton).

Run:
    python tools/trajopt_triple.py --T 4.0 --N 80 --out src/control/trajopt_triple.json

Sign convention identical to JS modules: θ_i from up, CCW positive; hanging=π.

NOTE: Phase 13.5 fallback ("near-upright start" toggle) ships in the UI, so
trajopt isn't a hard dependency for n=3 stabilisation. This script is the
ROADMAP for promoting to a true hanging-→ upright swing-up later.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


# ---------- Plant ----------
# Re-derive the n=3 EOM symbolically per call. For runtime trajopt we want
# fast Python eval, so we use a closed-form M(q), C(q,qdot), G(q) constructed
# at module load.

import sympy as sp


def build_plant(n: int = 3):
    """Return JIT-compiled numpy functions for M(q), C(q,qdot)·qdot, G(q)."""
    t = sp.symbols("t", real=True)
    x = sp.Function("x")(t)
    thetas = [sp.Function(f"th{i+1}")(t) for i in range(n)]
    q = [x] + thetas
    qdot_t = [sp.diff(qi, t) for qi in q]

    m0, g = sp.symbols("m0 g", positive=True)
    m = sp.symbols(" ".join(f"m{i+1}" for i in range(n)), positive=True)
    L = sp.symbols(" ".join(f"L{i+1}" for i in range(n)), positive=True)
    lc = sp.symbols(" ".join(f"lc{i+1}" for i in range(n)), positive=True)
    Iarr = sp.symbols(" ".join(f"I{i+1}" for i in range(n)), positive=True)

    J = [(x, sp.Integer(0))]
    com = []
    for i in range(n):
        jx, jy = J[i]
        s, c = sp.sin(thetas[i]), sp.cos(thetas[i])
        com.append((jx + lc[i] * s, jy + lc[i] * c))
        J.append((jx + L[i] * s, jy + L[i] * c))

    T = sp.Rational(1, 2) * m0 * sp.diff(x, t) ** 2
    for i in range(n):
        vx, vy = sp.diff(com[i][0], t), sp.diff(com[i][1], t)
        T += sp.Rational(1, 2) * m[i] * (vx**2 + vy**2)
        T += sp.Rational(1, 2) * Iarr[i] * sp.diff(thetas[i], t)**2
    V = sum(m[i] * g * com[i][1] for i in range(n))
    Lag = sp.expand(T - V)

    qddot_syms = sp.symbols(" ".join(f"qdd{i}" for i in range(n + 1)))
    tau = []
    for i in range(n + 1):
        dL_dqd = sp.diff(Lag, qdot_t[i])
        tau.append(sp.expand(sp.diff(dL_dqd, t) - sp.diff(Lag, q[i])))
    subs = {sp.diff(q[i], t, 2): qddot_syms[i] for i in range(n + 1)}
    tau_sub = [ti.xreplace(subs) for ti in tau]

    M_mat = sp.zeros(n + 1, n + 1)
    for i in range(n + 1):
        for j in range(n + 1):
            M_mat[i, j] = sp.simplify(tau_sub[i].coeff(qddot_syms[j]))
    qddot_vec = sp.Matrix(qddot_syms)
    rhs = sp.simplify(sp.Matrix(tau_sub) - M_mat * qddot_vec)
    qdot0 = {qdot_t[i]: 0 for i in range(n + 1)}
    G_vec = sp.simplify(rhs.xreplace(qdot0))
    h_vec = sp.simplify(rhs - G_vec)

    # Replace time-function symbols with plain sympy symbols for lambdify.
    plain_x = sp.symbols("x", real=True)
    plain_xd = sp.symbols("xdot", real=True)
    plain_th = sp.symbols(" ".join(f"th{i+1}" for i in range(n)), real=True)
    plain_thd = sp.symbols(" ".join(f"thd{i+1}" for i in range(n)), real=True)
    if n == 1:
        plain_th, plain_thd = (plain_th,), (plain_thd,)

    subs_plain = {x: plain_x, sp.diff(x, t): plain_xd}
    for i in range(n):
        subs_plain[thetas[i]] = plain_th[i]
        subs_plain[qdot_t[i + 1]] = plain_thd[i]

    args = [plain_x, *plain_th, plain_xd, *plain_thd, m0, g, *m, *L, *lc, *Iarr]
    M_f  = sp.lambdify(args, M_mat.xreplace(subs_plain), modules='numpy')
    h_f  = sp.lambdify(args, h_vec.xreplace(subs_plain), modules='numpy')
    G_f  = sp.lambdify(args, G_vec.xreplace(subs_plain), modules='numpy')

    def qddot(q_vec, qd_vec, u, p):
        # q_vec = [x, θ_1, θ_2, θ_3], qd_vec same.
        args_val = (
            q_vec[0], *q_vec[1:],
            qd_vec[0], *qd_vec[1:],
            p['m0'], p['g'],
            *[lk['m'] for lk in p['links']],
            *[lk['L'] for lk in p['links']],
            *[lk['l'] for lk in p['links']],
            *[lk['I'] for lk in p['links']],
        )
        M = np.asarray(M_f(*args_val), float)
        h = np.asarray(h_f(*args_val), float).flatten()
        G = np.asarray(G_f(*args_val), float).flatten()
        rhs = np.zeros(n + 1)
        rhs[0] = u
        return np.linalg.solve(M, rhs - h - G)

    return qddot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=4.0, help="Total horizon [s]")
    ap.add_argument("--N", type=int, default=80, help="Collocation knots")
    ap.add_argument("--F-max", type=float, default=60.0)
    ap.add_argument("--out", default="src/control/trajopt_triple.json")
    args = ap.parse_args()

    print(f"[trajopt] building n=3 plant ...", flush=True)
    qddot = build_plant(3)
    print(f"[trajopt] plant ready; horizon T={args.T} s, N={args.N} knots.", flush=True)
    print("[trajopt] NOTE — direct collocation NLP not yet implemented in this skeleton.")
    print("[trajopt] PLAN §13.2: integrate scipy.optimize.minimize with Hermite-Simpson")
    print("[trajopt] defects and write (t, x*, u*, K(t)) to", args.out)

    # Stub output so the consumer pipeline can be wired up before optimisation
    # is complete.
    stub = {
        "T": args.T, "N": args.N, "F_max": args.F_max,
        "t": [i * args.T / args.N for i in range(args.N + 1)],
        "x_star": [],
        "u_star": [],
        "K_t":    [],
        "status": "skeleton — Hermite-Simpson NLP not yet wired",
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(stub, indent=2))
    print(f"[trajopt] wrote {args.out}  ({Path(args.out).stat().st_size} bytes)")


if __name__ == "__main__":
    main()
