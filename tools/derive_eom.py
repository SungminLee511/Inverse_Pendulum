#!/usr/bin/env python
"""Derive equations of motion for an n-link inverted pendulum on a cart.

Usage:
    python tools/derive_eom.py 1        # emits src/physics/nlink_1.js
    python tools/derive_eom.py 2        # emits src/physics/nlink_2.js
    python tools/derive_eom.py 3        # emits src/physics/nlink_3.js
    python tools/derive_eom.py all      # all three

Sign convention (must match src/state.js):
    angles theta_i measured from the upward vertical, CCW positive in screen
    coordinates (so x-axis right, y-axis up). theta=0 means link points straight up.
"""
from __future__ import annotations
import sys
import re
import textwrap
import argparse
from pathlib import Path

import sympy as sp


# ----------------------------------------------------------------------------
# Symbolic derivation
# ----------------------------------------------------------------------------
def derive(n: int):
    """Return (M, h, G, D, params_syms) symbolic matrices/vectors for n links.

    EOM form:        M(q) qddot + h(q, qdot) + G(q) + D qdot = B u
    where h(q, qdot) = C(q, qdot) qdot is the Coriolis/centripetal vector.
    """
    t = sp.symbols("t", real=True)

    # Generalized coordinates
    x = sp.Function("x")(t)
    thetas = [sp.Function(f"theta_{i+1}")(t) for i in range(n)]
    q = [x] + thetas
    qdot = [sp.diff(qi, t) for qi in q]
    qddot = [sp.diff(qi, t, 2) for qi in q]

    # Parameters (per-link arrays)
    m0 = sp.symbols("m0", positive=True)
    g = sp.symbols("g", positive=True)
    cart_visc = sp.symbols("cart_visc", nonnegative=True)

    def _vec(prefix, nonneg=False):
        kw = {"nonnegative": True} if nonneg else {"positive": True}
        if n == 1:
            return (sp.symbols(f"{prefix}_1", **kw),)
        return sp.symbols(" ".join(f"{prefix}_{i+1}" for i in range(n)), **kw)

    m = _vec("m")
    L = _vec("L")
    l = _vec("l")
    I = _vec("I")
    joint_visc = _vec("joint_visc", nonneg=True)

    # Kinematics: joint positions and CoM positions
    # Joint 0 (cart pivot): (x, 0)
    # Joint i (top of link i): J_{i-1} + L_{i-1} * (sin theta_{i-1}, cos theta_{i-1})
    # CoM_i: J_{i-1} + l_{i-1} * (sin theta_{i-1}, cos theta_{i-1})
    # (using zero-based link index in the loop, but reading symbols as 1-based)
    J = [(x, sp.Integer(0))]
    com = []
    for i in range(n):
        jx, jy = J[i]
        s = sp.sin(thetas[i])
        c = sp.cos(thetas[i])
        com.append((jx + l[i] * s, jy + l[i] * c))
        J.append((jx + L[i] * s, jy + L[i] * c))

    # Kinetic energy
    # Cart
    T = sp.Rational(1, 2) * m0 * sp.diff(x, t) ** 2
    # Each link: translational + rotational
    for i in range(n):
        vx = sp.diff(com[i][0], t)
        vy = sp.diff(com[i][1], t)
        omega = sp.diff(thetas[i], t)  # absolute angular velocity
        T += sp.Rational(1, 2) * m[i] * (vx ** 2 + vy ** 2)
        T += sp.Rational(1, 2) * I[i] * omega ** 2

    # Potential energy (y_c is "up")
    V = sum(m[i] * g * com[i][1] for i in range(n))

    # Lagrangian
    Lag = sp.simplify(T - V)

    # Euler-Lagrange RHS: tau = d/dt(dL/dqdot) - dL/dq
    # which equals M*qddot + h + G  (with no friction; friction added separately)
    tau = []
    for i in range(n + 1):
        dL_dqdot = sp.diff(Lag, qdot[i])
        d_dt = sp.diff(dL_dqdot, t)
        dL_dq = sp.diff(Lag, q[i])
        tau.append(sp.expand(d_dt - dL_dq))

    # Extract M(q) by collecting qddot coefficients
    qddot_syms = sp.symbols(" ".join(f"_qdd{i}" for i in range(n + 1)))
    if n == 0:
        qddot_syms = (qddot_syms,)
    if not isinstance(qddot_syms, (tuple, list)):
        qddot_syms = (qddot_syms,)
    subs_acc = {qddot[i]: qddot_syms[i] for i in range(n + 1)}
    tau_sub = [ti.xreplace(subs_acc) for ti in tau]

    M = sp.zeros(n + 1, n + 1)
    for i in range(n + 1):
        for j in range(n + 1):
            M[i, j] = sp.simplify(tau_sub[i].coeff(qddot_syms[j]))

    # Residual = tau - M*qddot = h + G
    qddot_vec = sp.Matrix(qddot_syms)
    residual = sp.Matrix(tau_sub) - M * qddot_vec
    residual = sp.simplify(residual)

    # Split residual into gravity G (set qdot=0) and Coriolis h
    qdot0 = {qdot[i]: 0 for i in range(n + 1)}
    G_vec = sp.simplify(residual.xreplace(qdot0))
    h_vec = sp.simplify(residual - G_vec)

    # Friction (diagonal viscous damping)
    D_diag = [cart_visc] + list(joint_visc)
    D_vec = sp.Matrix([D_diag[i] * qdot[i] for i in range(n + 1)])

    # Replace function-of-t variables with plain symbols for JS emission
    plain_x = sp.symbols("x", real=True)
    plain_theta = [sp.symbols(f"theta_{i+1}", real=True) for i in range(n)]
    plain_xd = sp.symbols("xdot", real=True)
    plain_td = [sp.symbols(f"theta_{i+1}_dot", real=True) for i in range(n)]

    subs_plain = {x: plain_x, sp.diff(x, t): plain_xd}
    for i in range(n):
        subs_plain[thetas[i]] = plain_theta[i]
        subs_plain[qdot[i + 1]] = plain_td[i]

    M_p = M.xreplace(subs_plain)
    h_p = h_vec.xreplace(subs_plain)
    G_p = G_vec.xreplace(subs_plain)
    D_p = D_vec.xreplace(subs_plain)

    syms = dict(
        x=plain_x, xdot=plain_xd,
        theta=plain_theta, theta_dot=plain_td,
        m0=m0, g=g, cart_visc=cart_visc,
        m=m, L=L, l=l, I=I, joint_visc=joint_visc,
    )
    return M_p, h_p, G_p, D_p, syms


# ----------------------------------------------------------------------------
# JS code emission
# ----------------------------------------------------------------------------
def _sym_to_js(s: sp.Symbol) -> str:
    """Map symbolic names to JS variable references."""
    name = s.name
    if name == "x": return "x"
    if name == "xdot": return "xdot"
    m = re.fullmatch(r"theta_(\d+)", name)
    if m: return f"th{m.group(1)}"
    m = re.fullmatch(r"theta_(\d+)_dot", name)
    if m: return f"thd{m.group(1)}"
    if name == "m0": return "m0"
    if name == "g": return "g"
    if name == "cart_visc": return "cart_visc"
    m = re.fullmatch(r"m_(\d+)", name)
    if m: return f"m{m.group(1)}"
    m = re.fullmatch(r"L_(\d+)", name)
    if m: return f"L{m.group(1)}"
    m = re.fullmatch(r"l_(\d+)", name)
    if m: return f"lc{m.group(1)}"   # 'l' alone is hard to read in JS
    m = re.fullmatch(r"I_(\d+)", name)
    if m: return f"I{m.group(1)}"
    m = re.fullmatch(r"joint_visc_(\d+)", name)
    if m: return f"cv{m.group(1)}"
    return name


def _expr_to_js(e) -> str:
    """Compact JS expression from a sympy expr (no Math.* simplifications beyond sin/cos)."""
    e = sp.simplify(e)
    code = sp.jscode(e)
    # sympy jscode uses Math.sin/cos already; nothing else needed.
    # Replace identifier names with our JS variable mapping.
    for s in e.free_symbols:
        code = re.sub(rf"\b{re.escape(s.name)}\b", _sym_to_js(s), code)
    return code


def emit_js(n: int) -> str:
    M, h, G_vec, D_vec, syms = derive(n)
    nq = n + 1

    # Per-link unpacking lines for the JS function bodies
    unpack_lines = [
        "const x    = q[0];",
        "const xdot = qdot[0];",
    ]
    for i in range(n):
        unpack_lines.append(f"const th{i+1}  = q[{i+1}];")
        unpack_lines.append(f"const thd{i+1} = qdot[{i+1}];")
    unpack_lines.append("const m0 = params.m0, g = params.g, cart_visc = params.cart_visc;")
    for i in range(n):
        unpack_lines.append(
            f"const m{i+1} = params.links[{i}].m, "
            f"L{i+1} = params.links[{i}].L, "
            f"lc{i+1} = params.links[{i}].l, "
            f"I{i+1} = params.links[{i}].I, "
            f"cv{i+1} = params.links[{i}].joint_viscous;"
        )
    unpack = "  " + "\n  ".join(unpack_lines)

    # M matrix entries
    M_rows = []
    for i in range(nq):
        row = ", ".join(_expr_to_js(M[i, j]) for j in range(nq))
        M_rows.append(f"    [{row}]")
    M_body = ",\n".join(M_rows)

    # h, G, D vectors
    def vec_body(V):
        return ",\n".join(f"    {_expr_to_js(V[i])}" for i in range(nq))

    h_body = vec_body(h)
    G_body = vec_body(G_vec)
    D_body = vec_body(D_vec)

    js = f"""// Generated by tools/derive_eom.py for n={n} — DO NOT EDIT BY HAND.
// Equations of motion for the inverted {n}-link pendulum on a cart.
//
// Sign convention: angles theta_i measured from the upward vertical, CCW positive.
// q     = [x, theta_1, ..., theta_{n}]  (length {nq})
// qdot  = [xdot, theta_1_dot, ..., theta_{n}_dot]
// params = {{ m0, g, cart_visc, links: [{{ m, L, l, I, joint_viscous }}, ...] }}
//
// EOM (manipulator form):
//     M(q) * qddot + h(q, qdot) + G(q) + D(qdot) = B u
// where B = [1, 0, ..., 0]^T  (force u acts only on the cart).
//
// Solve via qddot = M^{{-1}} (B u - h - G - D).

export const N = {n};
export const DOF = {nq};

export function M(q, qdot, params) {{
{unpack}
  return [
{M_body}
  ];
}}

export function Cqdot(q, qdot, params) {{
{unpack}
  return [
{h_body}
  ];
}}

export function G(q, qdot, params) {{
{unpack}
  return [
{G_body}
  ];
}}

export function Dqdot(q, qdot, params) {{
{unpack}
  return [
{D_body}
  ];
}}

// Convenience: full RHS qddot = M^{{-1}} (B u - C - G - D).
// Pulls in a small Gaussian-elimination solver for symmetric SPD M.
export function qddot(q, qdot, u, params) {{
  const Mm = M(q, qdot, params);
  const C  = Cqdot(q, qdot, params);
  const Gg = G(q, qdot, params);
  const Dd = Dqdot(q, qdot, params);
  const n = Mm.length;
  // RHS vector
  const rhs = new Array(n);
  for (let i = 0; i < n; i++) {{
    rhs[i] = (i === 0 ? u : 0) - C[i] - Gg[i] - Dd[i];
  }}
  return solveSPD(Mm, rhs);
}}

// In-place Gaussian elimination with partial pivoting for an n x n linear solve.
// Cheap for n in {{2, 3, 4}}.
function solveSPD(A, b) {{
  const n = A.length;
  const M2 = A.map(r => r.slice());
  const v  = b.slice();
  for (let k = 0; k < n; k++) {{
    let piv = k;
    let max = Math.abs(M2[k][k]);
    for (let i = k + 1; i < n; i++) {{
      const a = Math.abs(M2[i][k]);
      if (a > max) {{ max = a; piv = i; }}
    }}
    if (max < 1e-14) throw new Error("M singular");
    if (piv !== k) {{ [M2[k], M2[piv]] = [M2[piv], M2[k]]; [v[k], v[piv]] = [v[piv], v[k]]; }}
    for (let i = k + 1; i < n; i++) {{
      const f = M2[i][k] / M2[k][k];
      if (f === 0) continue;
      for (let j = k; j < n; j++) M2[i][j] -= f * M2[k][j];
      v[i] -= f * v[k];
    }}
  }}
  const out = new Array(n);
  for (let i = n - 1; i >= 0; i--) {{
    let s = v[i];
    for (let j = i + 1; j < n; j++) s -= M2[i][j] * out[j];
    out[i] = s / M2[i][i];
  }}
  return out;
}}
"""
    return js


# ----------------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------------
def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("which", help="1, 2, 3, or 'all'")
    args = ap.parse_args(argv)

    root = Path(__file__).resolve().parent.parent
    out_dir = root / "src" / "physics"
    out_dir.mkdir(parents=True, exist_ok=True)

    targets = [1, 2, 3] if args.which == "all" else [int(args.which)]
    for n in targets:
        print(f"[derive_eom] n={n} ...", flush=True)
        js = emit_js(n)
        p = out_dir / f"nlink_{n}.js"
        p.write_text(js)
        print(f"[derive_eom] wrote {p.relative_to(root)}  ({len(js)} bytes)")


if __name__ == "__main__":
    main(sys.argv[1:])
