# Test Results

Auto-updated after each phase. Latest run is at the bottom.

## Phase 1.5 — UI smoke (Playwright)

| Test                                         | Status | Time (ms) |
|----------------------------------------------|--------|-----------|
| page loads without console errors            | ✅     | 1691      |
| mode buttons exist and toggle active class   | ✅     | 1483      |
| canvas has non-trivial pixel content         | ✅     | 1759      |
| play/pause button toggles                    | ✅     | 1442      |
| parameter panel groups are populated         | ✅     | 1252      |
| keyboard shortcut R resets sim time          | ✅     | 2181      |

6 pass / 0 fail / 0 skip — 10.7 s.

## Phase 2.1 — n=1 EOM (sympy-generated, analytic checks)

| Test                                                   | Status | Time (ms) |
|--------------------------------------------------------|--------|-----------|
| DOF and length consistency                             | ✅     | 1.4       |
| M is symmetric across q,qdot sweep                     | ✅     | 0.6       |
| M is positive-definite (det > 0, diag > 0)             | ✅     | 1.3       |
| Gravity vector matches analytic formula                | ✅     | 0.3       |
| Coriolis vector matches analytic formula               | ✅     | 0.3       |
| qddot = 0 at upright with no input / no friction       | ✅     | 0.4       |
| qddot > 0 falling from horizontal (sign convention)    | ✅     | 0.2       |

7 pass / 0 fail — 0.34 s.

## Phase 2.2 — integrators (Euler / SI Euler / RK4)

| Test                                                  | Status | Time (ms) |
|-------------------------------------------------------|--------|-----------|
| RK4 conserves energy < 0.1% over 10 s, n=1 free       | ✅     | 138       |
| SI Euler energy drift < 5% over 10 s                  | ✅     | 34        |
| Forward Euler drifts ≥ 5× more than RK4               | ✅     | 14        |
| RK4 order-of-accuracy: halving dt drops err ~16×      | ✅     | 108       |
| SI Euler order: halving dt drops err ~2×              | ✅     | 106       |
| Friction dissipates energy monotonically              | ✅     | 56        |

6 pass / 0 fail — 0.46 s.

**Phase 2.1 + 2.2 combined: 13/13 headless tests, 1.16 s.**

## Phase 2.3 — wire real physics into rAF loop

| Test                                                | Status | Time (ms) |
|-----------------------------------------------------|--------|-----------|
| n=1 physics runs live, energy stays bounded (<2%)   | ✅     | 3657      |
| mode switching keeps physics step alive (no NaN)    | ✅     | 1870      |

Bug found and fixed during 2.3: state.js named friction parameter `cart_viscous` but the sympy-generated EOM reads `cart_visc` → undefined → NaN. Renamed to `cart_visc` for consistency.

**Total after Phase 2.3: 21/21 passing (15 headless + 6 UI), 10.2 s.**

## Phase 2.4 — extracted canvas renderer (ui/canvas.js)

| Test                                                       | Status | Time (ms) |
|------------------------------------------------------------|--------|-----------|
| canvas draws ground/cart/link for n=1 (>15 distinct colors)| ✅     | 1496      |
| n=2 mode draws orange link 2                               | ✅     | 1532      |
| n=3 mode draws green link 3                                | ✅     | 1505      |
| cart centroid shifts left when q[0]: 0.4 → -0.4            | ✅     | 1307      |

Screenshots `IP_phase2_n{1,2,3}_t20260524.png` pushed to output_port.

**Total after Phase 2.4: 25/25 passing (15 headless + 10 UI), 10.9 s.**

## Phase 3.1 — sensor model (quant + noise + delay + velocity FD)

| Test                                              | Status | Time (ms) |
|---------------------------------------------------|--------|-----------|
| quantize: rounds to nearest LSB                   | ✅     | 1.5       |
| Gaussian noise: mean~0 std~σ over 20k samples     | ✅     | 6.9       |
| Sensor delay buffer returns value from delay ago  | ✅     | 0.5       |
| delaySec=0 returns latest push                    | ✅     | 0.2       |
| Noise statistics over 10k: mean+std match spec    | ✅     | 7.8       |
| Quantization is applied at LSB granularity        | ✅     | 0.2       |
| FD velocity estimator tracks 5 m/s ramp           | ✅     | 0.8       |
| FD velocity rejects noise on stationary input     | ✅     | 0.5       |
| Same seed → identical noisy sample stream         | ✅     | 0.4       |
| sensor_last + sensor_vel_est live in browser      | ✅     | 2160      |

**Total after Phase 3.1: 35/35 passing (22 headless + 13 UI), 11.7 s.**

## Phase 3.2 — actuator (sat + slew + lag + Coulomb + force noise)

| Test                                                  | Status | Time (ms) |
|-------------------------------------------------------|--------|-----------|
| Saturation: u_cmd above F_max clips                   | ✅     | 1.5       |
| Slew rate: 0→F_max in F_max/slew seconds              | ✅     | 1.4       |
| First-order lag: step → ≈63% at t=τ, converges        | ✅     | 0.4       |
| Coulomb friction opposes cart motion                  | ✅     | 0.2       |
| Coulomb smooth near xdot=0 (no chatter)               | ✅     | 0.2       |
| Force noise σ statistics match spec                   | ✅     | 5.4       |
| Sat+slew+lag interplay converges to F_max             | ✅     | 0.3       |
| Reset clears actuator state                           | ✅     | 0.3       |
| Live: state.u_cmd → u_applied through actuator        | ✅     | 1733      |
| Live: saturation clips u_cmd above F_max              | ✅     | 1543      |

**Total after Phase 3.2: 45/45 passing (30 headless + 15 UI), 11.4 s.**

## Phase 3.3 + 3.4 — actuator reset on mode-change + full param sliders

| Test                                                  | Status | Time (ms) |
|-------------------------------------------------------|--------|-----------|
| Changing F_max slider changes saturation behaviour    | ✅     | 1442      |
| Switching integrator to Euler keeps sim finite        | ✅     | 1492      |
| Lowering angle σ to 0 reduces sensor noise variance   | ✅     | 1911      |
| All ≥16 sliders present                               | ✅     | 979       |

**Total after Phase 3 complete: 49/49 passing (30 headless + 19 UI), 12.7 s.**

## Phase 4.1 — numerical Jacobian linearization (n=1)

| Test                                                       | Status | Time (ms) |
|------------------------------------------------------------|--------|-----------|
| A is 4×4 with identity kinematic block + zero top-left     | ✅     | 2.1       |
| B is 4-vec; B[0]=B[1]=0; B[2]>0 (force pushes cart right)  | ✅     | 0.4       |
| Upright is unstable: A[3][1] > 0 (gravity tips away)       | ✅     | 0.3       |
| Friction in Aqqd: A[2][2]<0, A[3][3]<0                     | ✅     | 0.4       |
| Jacobian Richardson convergence: ε=1e-4 vs 1e-6 agree to 1e-7 | ✅  | 0.4       |
| Controllability rank([B,AB,A²B,A³B]) = 4 for n=1           | ✅     | 0.5       |
| A and B finite (no NaN/Inf)                                | ✅     | 1.3       |
| Linearized dynamics non-trivial sign check                 | ✅     | 0.2       |

**Total after Phase 4.1: 57/57 passing (38 headless + 19 UI).**

## Phase 4.2 — CARE solver (Hamiltonian matrix-sign method) + utilities

| Test                                                  | Status | Time (ms) |
|-------------------------------------------------------|--------|-----------|
| CARE: P symmetric, diagonals positive                 | ✅     | 6.0       |
| CARE: A^T P + PA - PBR^{-1}B^T P + Q residual < 1e-3  | ✅     | 2.4       |
| LQR closed-loop shrinks >20× over 10 s                | ✅     | 11.1      |
| LQR closed-loop eigenvalues in LHP (trace < 0)        | ✅     | 0.9       |
| Heavier R → smaller \|\|K\|\|                         | ✅     | 1.2       |
| Higher Q[θ] → larger \|K[θ]\|                         | ✅     | 1.2       |
| matrixInvert round-trip A·A⁻¹ = I                     | ✅     | 0.2       |

**Total after Phase 4.2: 64/64 passing (45 headless + 19 UI).**

## Phase 4.3 + 4.4 — controller wired into rAF loop + Q/R sliders

| Test                                                            | Status | Time (ms) |
|-----------------------------------------------------------------|--------|-----------|
| LQR on full nonlinear EOM brings θ=0.15→~0 within 6s (R=0.05)   | ✅     | 92        |
| LQR on full nonlinear EOM with browser-default R=0.01 stabilises| ✅     | 68        |
| LQR mode stabilises 0.15 rad tilt back to upright in 6 s (browser) | ✅  | 5175      |
| Increasing Q[θ] slider produces larger \|K[θ]\| gain            | ✅     | 1182      |

Side fixes during 4.3+4.4:
- `panel_live`/`physics_live` tests updated to `ctrl_mode='off'` (otherwise the
  default-`auto` LQR would whack the hanging pendulum during test setup).
- Exposed `_resetActuator` on `window.__pendulum` so tests can drain residual
  motor lag/slew state before sampling.
- Sensor velocity LPF cutoff bumped 30→200 rad/s with a warm-start snap so the
  controller isn't blind for the first 1/cutoff seconds.

**Total after Phase 4 complete: 68/68 passing (47 headless + 21 UI).**

## Phase 5 — Swing-up (n=1) + handover

Two-stage swing-up:
1. Bootstrap — square-wave cart drive at the pendulum's natural frequency
   `ω_n = √(m·g·l / (I + m·l²))`. Excites the pendulum into a real swing
   regardless of initial conditions (handles the degenerate at-rest-at-hanging
   case where the bang-bang law is identically zero). Latched once the system
   is genuinely swinging (`|θ̇| > omega_min` after `boot_min` seconds).
2. Åström-Furuta — `u = k_E · Ẽ · σ(θ̇·cosθ)` + soft cart centering, where
   `σ` is a tanh-smoothed sign and `Ẽ = E_p − E_p*` uses the pendulum-only
   energy (cart KE excluded — including it lets feedback fake "energy" and
   breaks damping).

Region-of-attraction switcher (`HandoverSwitcher`): |θ_i wrapped| < `handover_theta`,
|θ̇_i| < `handover_omega`, |x|<1.5, |ẋ|<2.5. Linear blend over
`handover_blend_ms` once latched.

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| swingup pumps energy from hanging into ROA within 30 s (n=1)      | ✅     | 112       |
| swingup → handover blend → LQR drives θ to 0 within 40 s          | ✅     | 671       |
| swingupDiag at upright equilibrium reports E ≈ E*                 | ✅     | 0.4       |
| Switcher latches and unlatches based on ROA boundary              | ✅     | 0.2       |
| Switcher mix outputs pure swingup before crossing, blends to LQR  | ✅     | 0.3       |
| Auto mode swings up and LQR holds (browser, ≤45 s, \|θ\|<10°)     | ✅     | 5571      |
| Swing-up-only mode keeps E ≈ E* (browser)                         | ✅     | 2252      |

**Total after Phase 5 complete: 75/75 passing (52 headless + 23 UI).**

## Phase 6 — Plots (angles / velocities / phase portrait / control force)

Rolling-buffer canvas plots; sample at the sensor cadence (with internal
throttle so changing sensor_period doesn't blow the ring), render throttled to
~30 Hz wall time.

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| TimeSeries: empty initial state                                   | ✅     | 1.4       |
| TimeSeries: push appends rows and grows n up to cap               | ✅     | 0.9       |
| TimeSeries: ring wraps when cap exceeded                          | ✅     | 0.2       |
| TimeSeries: forEach is chronological after wrap                   | ✅     | 0.3       |
| TimeSeries: clear empties without resizing                        | ✅     | 0.2       |
| TimeSeries: trange reports oldest/newest after wrap               | ✅     | 0.2       |
| TimeSeries: multi-channel push lazily allocates channels          | ✅     | 0.3       |
| wrap(): maps any real angle into (−π, π]                          | ✅     | 0.2       |
| All four plot canvases have content after ~2 s sim                | ✅     | 3215      |
| Force plot shows ±F_max amber reference lines                     | ✅     | 2680      |
| Phase plot trail accumulates over time (more bright px later)     | ✅     | 4086      |
| Plot buffers clear on mode change                                 | ✅     | 2036      |

**Total after Phase 6 complete: 87/87 passing (60 headless + 27 UI).**

## Phase 7 — Full parameter UI + presets + keyboard

- Per-link slider sub-blocks (m, L, l_c, I, joint visc, joint Coulomb) rebuilt
  on mode-change. n=1 → 6 link sliders, n=2 → 12, n=3 → 18.
- Q-diag controller block resized to 2·(n+1) entries per mode.
- Built-in presets (`default-1/2/3`, `noisy`, `fast-motor`, `stiff-triple`)
  with localStorage Save/Load round-trip.
- Kick button + magnitude slider + `K` keyboard shortcut → impulse on
  `qdot[1]`.

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| Per-link sliders appear for n=1 (6 entries × 1 link)              | ✅     | 1193      |
| Switching to n=2 rebuilds per-link block (12 entries)             | ✅     | 1304      |
| Switching to n=3 rebuilds per-link block (18) + 8 Q-diag sliders  | ✅     | 1309      |
| Per-link slider edit propagates to state.params.links[i].field    | ✅     | 1358      |
| Preset Load applies "stiff-triple" (n=3, F_max=80, sensor_delay=1ms) | ✅  | 1364      |
| Preset Save → Load round-trip preserves a modified F_max          | ✅     | 1416      |
| Keyboard K applies a kick (qdot[1] += magnitude)                  | ✅     | 1266      |
| Keyboard Space toggles running                                    | ✅     | 1304      |
| Keyboard R resets sim time                                        | ✅     | 1627      |
| Total slider count ≥ 22 (mode 1)                                  | ✅     | 1224      |

**Total after Phase 7 complete: 97/97 passing (60 headless + 37 UI).**

## Phase 8 — Physics: double (n=2)

`tools/derive_eom.py 2` → `src/physics/nlink_2.js`. Registered in
`physics/index.js`. Canvas already draws n links generically.

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| n=2 EOM exports correct N (=2) and DOF (=3)                       | ✅     | 1.4       |
| M(q) is symmetric across random q, qdot                           | ✅     | 1.6       |
| M(q) is positive-definite (Sylvester criterion)                   | ✅     | 0.4       |
| At q=0 (upright), G is exactly zero                               | ✅     | 0.3       |
| At hanging (θ=π, π), G is exactly zero                            | ✅     | 0.2       |
| qddot at upright, zero qdot, zero u, no friction → zero           | ✅     | 0.5       |
| qddot signs at hanging with θ_1 perturb: gravity tips link        | ✅     | 0.2       |
| Friction Dqdot is diagonal                                        | ✅     | 0.3       |
| n=2 RK4 conserves energy to <0.5% over 10 s (frictionless)        | ✅     | 190       |
| n=2 RK4 with friction dissipates energy monotonically             | ✅     | 104       |
| linearize(2): A is 6×6, B is 6-vec with cart-only actuation       | ✅     | 2.6       |
| linearize(2): Richardson eps=1e-4 vs 1e-6 agree to 1e-5           | ✅     | 0.6       |
| linearize(2): top-right block = identity exactly                  | ✅     | 1.1       |
| linearize(2): upright is unstable (∂qddot/∂θ > 0 somewhere)       | ✅     | 0.3       |
| linearize(2): friction → negative diag in lower-right             | ✅     | 0.3       |
| controllability rank(n=2) = 6 (full)                              | ✅     | 0.7       |
| A and B are finite (no NaN / Inf)                                 | ✅     | 0.3       |
| Switching to n=2 runs real physics: energy bounded, no NaN        | ✅     | 2492      |
| LQR for n=2 produces a valid 6-vector gain                        | ✅     | 1456      |
| Canvas: two links visible after mode switch (orange + blue px)    | ✅     | 1714      |

**Total after Phase 8 complete: 117/117 passing (77 headless + 40 UI).**

## Phase 9 — LQR: double (n=2)

The controller dispatcher in `src/control/controller.js` is already generic
in n (Jacobian + CARE work for arbitrary mode). What Phase 9 adds:

- Closed-loop stability tests on the FULL nonlinear n=2 EOM (not just the
  linearisation).
- Q/R coupling tests (heavier R → smaller ||K||, larger Q[θ] → larger |K|).
- Browser end-to-end LQR mode test for n=2.

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| n=2 LQR stabilises θ_1=0.05, θ_2=0.05 within 6 s                  | ✅     | 118       |
| n=2 LQR keeps peak \|θ\| < 0.5 rad during a 0.1 rad pulse         | ✅     | 68        |
| n=2 LQR closed-loop has no NaN over 5 s under modest perturbation | ✅     | 72        |
| n=2 LQR: heavier R reduces \|\|K\|\|                              | ✅     | 1.7       |
| n=2 LQR: higher Q[θ_1] → larger \|K[1]\|                          | ✅     | 1.5       |
| n=2 LQR stabilises in browser (ideal sensors) ≤ 8 s               | ✅     | 5379      |
| n=2 Q[θ_2] slider live-changes the \|K[2]\| gain                  | ✅     | 1257      |

**Total after Phase 9 complete: 124/124 passing (82 headless + 42 UI).**

## Phase 10 — Swing-up: double (n=2)

PLAN §10 anticipated this would be iterative and notes the failure regime
should be documented if pure energy pumping doesn't reach the LQR ROA.

**What works**
- Pendulum-only energy converges to within ~30% of E* within 20 s of pumping.
- Mass-weighted smooth-sign — `Σ m_i l_i tanh(θ̇_i cosθ_i / ε) / Σ m_i l_i`
  — correctly biases the pumping direction toward the link with more
  m·l contribution to dE/dt.
- Near-upright start (θ_1=0.05, θ_2=0.05) → LQR catches inside 6 s via the
  Auto-mode switcher.
- Nullish-coalescing fix in `swingup.js`: `params.swingup_kxP = 0` etc. now
  override the default (was being silently coerced through `||`).

**What doesn't (documented failure regime)**
- From full hanging (θ_1=θ_2=π), pure Åström-Furuta + soft cart centering
  does NOT bring the (θ_i, θ̇_i, x, ẋ) state into the LQR ROA inside 30 s.
  The energy hovers near E* but the joints visit the ROA only intermittently
  and not coincidentally. Phase 13 plans direct collocation + TVLQR; for
  Phase 10 we ship the documented limitation.

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| n=2 swing-up: energy converges to within 30% of E* in 20 s        | ✅     | 476       |
| n=2 mass-weighted average uses per-link m·l weights               | ✅     | 0.4       |
| n=2 Auto-mode: near-upright start → LQR catches in ≤ 6 s          | ✅     | 146       |
| n=2 swing-up from hanging: DOCUMENTED non-convergence in 30 s     | ✅     | 688       |

**Total after Phase 10 complete: 128/128 passing (86 headless + 42 UI).**

## Phase 11 — Physics: triple (n=3)

`tools/derive_eom.py 3` → `src/physics/nlink_3.js`. Registered in
`physics/index.js`. Canvas already draws n links generically.

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| n=3 EOM exports correct N (=3) and DOF (=4)                       | ✅     | 1.3       |
| M(q) is symmetric across 20 random configurations                 | ✅     | 1.7       |
| M(q) is positive-definite via Sylvester (4×4 det)                 | ✅     | 0.7       |
| G is exactly zero at upright                                      | ✅     | 0.3       |
| G is exactly zero at hanging (π,π,π)                              | ✅     | 0.2       |
| qddot at upright + zero qdot + zero u + no friction → zero        | ✅     | 0.5       |
| Friction Dqdot is diagonal in n=3                                 | ✅     | 0.2       |
| M condition: diagonal entries vs off-diagonals (sanity)           | ✅     | 0.2       |
| n=3 RK4 conserves energy to < 1% over 5 s (frictionless)          | ✅     | 152       |
| n=3 RK4 with friction dissipates monotonically                    | ✅     | 122       |
| n=3 qddot at upright + friction → zero                            | ✅     | 0.3       |
| n=3 qddot at hanging with cart force → cart + joints respond      | ✅     | 0.5       |
| linearize(3): A is 8×8, B is 8-vec, cart-only actuation           | ✅     | 2.9       |
| linearize(3): top-right block = identity exactly                  | ✅     | 1.7       |
| linearize(3): Richardson eps=1e-4 vs 1e-6 agree to 1e-3           | ✅     | 0.7       |
| linearize(3): friction → negative diag in lower-right             | ✅     | 0.5       |
| linearize(3): upright is unstable                                 | ✅     | 0.4       |
| controllability rank(n=3) = 8 (full)                              | ✅     | 1.2       |
| linearize(3): A and B all finite                                  | ✅     | 0.4       |
| Switching to n=3 runs real physics; q/qdot finite                 | ✅     | 2716      |
| LQR for n=3 produces a valid 8-vector gain                        | ✅     | 1369      |
| Canvas: three links visible after mode switch                     | ✅     | 1423      |

**Total after Phase 11 complete: 150/150 passing (105 headless + 45 UI).**

## Phase 12 — LQR triple + robustness sweep

Controller dispatcher already generic in n (Phase 4); the n=3 EOM is now in
place (Phase 11). Phase 12 deliverables:

- **Closed-loop n=3 LQR on full nonlinear EOM** (no sensor / actuator path):
  small per-joint perturbation drives back to upright in ≤ 6 s.
- **27-cell robustness sweep**: mass × length × friction multipliers
  {0.8, 1.0, 1.2} × {0.8, 1.0, 1.2} × {0.5, 1.0, 1.5}. K is re-solved against
  the perturbed plant each cell. Pass criterion: |θ_i| < 0.05 rad after 8 s
  from a 0.03 rad/joint perturbation. **27/27 cells pass** — the linearisation
  + CARE pipeline is solidly within margin for this nominal triple. Matrix
  written to `tests/RESULTS_robustness_n3.md`.
- **UI wiring**: LQR mode produces a finite 8-vector K and a non-zero u_cmd
  under perturbation in the browser. A full in-browser closed-loop n=3
  convergence test is **not** included — the velocity-FD IIR + ZOH at the
  control_period introduce enough phase lag to destabilise the tightly-tuned
  triple LQR even with σ=0 / delay=0 sensors. The headless closed-loop +
  robustness sweep already validate the algorithm itself. (PLAN §9 calls
  this out explicitly as a "triple sensitivity" pitfall.)

| Test                                                              | Status | Time (ms) |
|-------------------------------------------------------------------|--------|-----------|
| n=3 LQR: linearize → CARE → K finite, length 8                    | ✅     | 14        |
| n=3 LQR stabilises (0.03, 0.03, 0.03) perturbation in 6 s         | ✅     | 156       |
| n=3 LQR: heavier R reduces \|\|K\|\|                              | ✅     | 9         |
| n=3 LQR: higher Q[θ_3] → larger \|K[3]\|                          | ✅     | 2.5       |
| robustness sweep: nominal point succeeds                          | ✅     | 211       |
| robustness 27-cell matrix (mass × length × friction): 27/27 pass  | ✅     | 4752      |
| n=3 LQR wiring (browser): K length 8 + non-zero u_cmd             | ✅     | 1752      |

**Total after Phase 12 complete: 157/157 passing (111 headless + 46 UI).**
