# Inverted Pendulum on a Cart

Single-page web simulation of 1-, 2-, and 3-link inverted pendulums on a
horizontally-driven cart. Includes energy-based swing-up, LQR stabilisation
with smooth handover, system identification (chirp / PRBS / step / impulse
excitation + output-error fit), adjustable physical / sensor / actuator /
controller params, real-time visualisation with time-series, phase-portrait,
and control-force plots.

Vanilla HTML / CSS / JS — no build step. EOM derived offline with sympy and
emitted as plain JS modules. Headless tests via `node --test`; UI tests via
`playwright` against headless Chromium.

---

## Quickstart

```bash
# Serve the page locally and open it.
npm run serve            # tiny static server, prints http://localhost:port
# (or just open index.html in a browser that supports ES modules + fetch())

# Re-derive the equations of motion (only when changing the symbolic
# convention — the generated nlink_{1,2,3}.js files are checked in).
python tools/derive_eom.py 1     # also: 2 | 3 | all

# Run the test suites.
npm test                 # all of headless/ + ui/ (≈ 60 s wall)
npm run test:headless    # ~129 headless tests (~ 30 s)
npm run test:ui          # ~53 Playwright tests (~ 90 s)
```

The static page also lives at the path the resume site mounts it (Phase 16,
not in this commit).

---

## Repo layout

```
index.html                       — the page
src/
  main.js                        — entry point, rAF loop
  state.js                       — single source of truth + pub/sub
  loop.js                        — multi-rate accumulator loop
  styles.css
  physics/
    nlink_1.js  nlink_2.js  nlink_3.js   — sympy-generated EOM
    integrator.js                — Euler / Semi-implicit Euler / RK4
    index.js                     — n → module registry
  control/
    lqr.js                       — linearize + Hamiltonian-sign CARE
    swingup.js                   — Åström-Furuta + bootstrap square wave
    switcher.js                  — ROA-based swingup → LQR handover
    sysid.js                     — excitations + output-error fit
    controller.js                — dispatcher: off/lqr/swingup/auto/sysid
    trajopt_triple.json          — trajopt stub (Phase 13.5 fallback used)
  sensors.js                     — quant + Gaussian noise + delay + IIR vel
  actuator.js                    — saturation + slew + first-order lag
  ui/
    canvas.js                    — pendulum animation + mouse-drag
    panel.js                     — sliders, presets, kick, start-pose
    plots.js                     — rolling-buffer canvas plots
tools/
  derive_eom.py                  — sympy → JS emitter
  trajopt_triple.py              — direct-collocation skeleton (PLAN §13.2)
tests/
  headless/                      — node --test (no browser)
  ui/                            — playwright (headless chromium)
  RESULTS.md                     — per-phase pass tally
  RESULTS_robustness_n3.md       — Phase 12 robustness matrix
```

---

## Conventions

- **Angles** are measured from the upward vertical, CCW positive in screen
  coordinates. Hanging = π, upright = 0. Documented at the top of every
  physics file.
- **Generalised coords**: `q = [x, θ_1, ..., θ_n]`, `qdot` parallel.
- **EOM (manipulator form)**: `M(q)q̈ + h(q,q̇) + G(q) + D(q̇) = B u`,
  with `B = [1, 0, ..., 0]^T` (cart-only actuation).
- **Integrator default**: RK4 at `dt_sim = 1e-4 s` (10 kHz).
- **Control rate**: 200 Hz default (5 ms control period).
- **Sensor rate**: 500 Hz default (2 ms sampling).
- **All times in seconds, forces in newtons, angles in radians.**

---

## Controls

### Mouse / Touch
- **Mode buttons** (top bar): 1 / 2 / 3-link.
- **Play / Pause**: ⏸ / ▶ button.
- **Reset**: Reset button (also `R` key).
- **Speed slider**: 0.1× – 5× sim time.
- **Kick button**: instantaneous impulse on link 1 (magnitude slider beside).
- **Click + drag a joint**: applies a horizontal disturbance force while held.

### Keyboard
- `Space` — pause / resume
- `R` — reset sim time
- `K` — kick link 1 by `kick-mag` slider
- `1` / `2` / `3` — switch mode

### Sim panel
- **integrator** dropdown (Forward Euler / SI Euler / RK4)
- **start pose** dropdown
  - `Hanging (θ=π)` — default
  - `Near-upright (θ=0.05)` — Phase 13 fallback for n=3 LQR catching
  - `Upright (θ=0)` — debug / smoke

### Controller panel
- **mode** dropdown: `auto` / `swingup` / `lqr` / `sysid` / `off`
- Per-state Q diagonal sliders (Q[x], Q[θ_i], Q[ẋ], Q[θ̇_i])
- R (control penalty)
- control period
- handover θ / ω thresholds
- sysid excite + amplitude

### Presets
Built-in: `default-1`, `default-2`, `default-3`, `noisy`, `fast-motor`,
`stiff-triple`. Save / Load round-trip via `localStorage`.

---

## What's implemented (per PLAN phases)

| Phase | Title                                                  | Status |
|------:|--------------------------------------------------------|--------|
|  1    | Skeleton (HTML grid, rAF loop, panel scaffold)         | ✅     |
|  2    | n=1 physics (EOM, integrators, canvas, energy invariant) | ✅   |
|  3    | Sensor + actuator models                               | ✅     |
|  4    | n=1 LQR (linearize + CARE + UI sliders)                | ✅     |
|  5    | n=1 swing-up + switcher handover                       | ✅     |
|  6    | Plots (angles / velocities / phase / force)            | ✅     |
|  7    | Full param UI + presets + keyboard                     | ✅     |
|  8    | n=2 EOM                                                | ✅     |
|  9    | n=2 LQR (closed-loop on full nonlinear EOM)            | ✅     |
| 10    | n=2 swing-up (mass-weighted Åström; documented limit)  | ✅     |
| 11    | n=3 EOM (sympy-derived; energy conservation < 1% / 5s) | ✅     |
| 12    | n=3 LQR + 27-cell robustness sweep (mass/length/friction) | ✅ |
| 13    | n=3 swing-up (Plan C fallback + trajopt skeleton)      | ✅     |
| 14    | System identification (4 excitations + fit)            | ✅     |
| 15    | Polish (mouse drag, presets, regression, README)       | ✅     |
| 16    | Deploy to resume site                                  | ⏳ TODO |

### Known limitations (documented)

- **n=3 swing-up from full hanging** does NOT reach the LQR ROA with pure
  Åström energy pumping. PLAN §13 documents the failure regime and ships
  the **near-upright start** toggle as the fallback. `tools/trajopt_triple.py`
  is a scaffold for the real direct-collocation solution.
- **In-browser n=3 closed-loop LQR** is fragile with the velocity-FD IIR +
  ZOH at the control period (phase lag destabilises the tight triple LQR).
  The HEADLESS closed-loop + robustness sweep validate the algorithm
  itself; the browser test asserts only K-length + non-zero u_cmd. PLAN §9
  explicitly calls this out as a "triple sensitivity" pitfall.
- **n=2 swing-up from full hanging** brings pendulum energy near E_p* but
  does not reliably land in the LQR ROA. Documented; near-upright start
  works fine.

---

## Math sketch

### LQR

Linearise the EOM at the upright equilibrium (numerical Jacobian, central
differences) into

  `d/dt x_state = A x_state + B u`

where `x_state = [q ; q̇]`. Solve the continuous-time algebraic Riccati
equation

  `A^T P + P A − P B R^{-1} B^T P + Q = 0`

via the matrix-sign Newton iteration on the Hamiltonian
`H = [[A, -B R^{-1} B^T], [-Q, -A^T]]`. The stable invariant subspace is
the column space of `(I - sign(H))/2`; QR gives `[X_1 ; X_2]` and
`P = X_2 X_1^{-1}`. Gain `K = R^{-1} B^T P`; control law `u = -K x_state`.

### Swing-up (Åström-Furuta)

Pendulum-only energy (cart KE excluded so feedback can't fake pumping):

  `E_p = ½ Σ_i (I_i + m_i l_i²) θ̇_i² + Σ_i m_i g · y_com_i,cart-frame`
  `E_p* = Σ_i m_i g l_i`         (all links upright, at rest)
  `Ẽ = E_p − E_p*`

Pumping law (sign chosen so dE_p/dt has the right sign, accounting for
the cart back-reaction in the derivation):

  `u = +k_E · Ẽ · σ(  Σ_i  m_i l_i tanh((θ̇_i cosθ_i)/ε)  /  Σ m_i l_i )`

plus a soft cart-position P-D term. A square-wave bootstrap at the link
natural frequency kicks the pendulum out of the (degenerate) at-rest-at-
hanging state before the pumping law takes over.

### Switcher

ROA proxy: `|θ_i wrap| < handover_theta`, `|θ̇_i| < handover_omega`,
`|x| < 1.5`, `|ẋ| < 2.5`. Once latched in, linear blend
`u = α u_LQR + (1-α) u_swingup` over `handover_blend_ms`. If the very
first call already finds us inside the ROA (start_pose='near-upright'),
the blend is skipped — there's no swing-up history to soften.

### Sys-ID

Output-error fit: coordinate-descent on a selected subset of
`state.params` knobs, minimising the L2 error between the model's
simulated trajectory and the measurement's. Per-knob step shrinks when
the local sweep fails to improve; converges in O(20–60) outer iterations
on the round-trip tests.

---

## Sign-convention bugs to watch for

Catalogued in PLAN §9. The big ones that bit during development:

- The Åström pumping sign for cart-pendulum (`dE_p/dt = −m l θ̇ cosθ ẍ`):
  the JS `tanh` smooth-sign must give `u_pump = +k_E Ẽ σ`, not `−`. A flipped
  sign drives the cart away from the pendulum every swing and over-energises
  the system to chaotic spinning.
- `state.params.swingup_kxP = 0` ≠ unset. Used `||` to read defaults early
  on; a real zero gets coerced. All sysup tunables now use `??`.
- Mode-button click via Playwright bypasses pub/sub. To pause the loop
  cleanly from a test, use the `#btn-playpause` button click so the
  `running-change` event fires and the loop's `pumpRunningFromState` sees it.
- Q_diag is a flat array indexed `[x, θ_1, ..., θ_n, ẋ, θ̇_1, ..., θ̇_n]`.
  The default array has length 4; for n=3 the controller fills indices
  4..7 with sensible defaults.

---

## Testing

```bash
npm test
```

- Headless (`tests/headless/`) — ~126 tests, < 30 s wall. Energy invariants,
  Jacobian Richardson convergence, CARE residual, controllability rank,
  closed-loop stability, swing-up energy + handover, sensor / actuator
  models, sysid round-trip, robustness sweep for n=3.
- UI (`tests/ui/`) — ~53 Playwright tests against a tiny in-process static
  server. Smoke, canvas rendering, slider → state coupling, presets,
  keyboard shortcuts, LQR / swing-up live, plot canvases populate, sysid
  excitation in browser.

The latest pass tally lives in `tests/RESULTS.md`. The 27-cell n=3
robustness matrix is in `tests/RESULTS_robustness_n3.md`.

---

## License

MIT (or whatever the parent resume-site repo specifies).
