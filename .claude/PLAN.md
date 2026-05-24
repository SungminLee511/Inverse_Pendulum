# Inverted Pendulum on a Cart — Build Plan

A single-page web app simulating 1-, 2-, and 3-link inverted pendulums on a horizontally-driven cart. Includes energy-based swing-up, LQR stabilization with smooth handover, system identification, adjustable physical/sensor/actuator/controller parameters, sensor and motor noise/delay/saturation, real-time visualization with time-series, phase-portrait, and control-force plots.

Final delivery: static site mirrored into `TRASHCAN/SungminLee511.github.io/projects/inverse-pendulum/` and linked from the resume index.

---

## 0. Build target & decisions (locked in)

- **Scope**: All 15 numeric phases + deployment phase (16 total).
- **Subrepo**: Static copy into resume site after sim is finished (no git submodule).
- **Resume mount point**: `projects/inverse-pendulum/` (linked from resume index).
- **sympy derivation**: Run locally, emit JS files for M/C/G.
- **Build stack**: Vanilla HTML/CSS/JS, no build step. `math.js` for matrix ops only if needed; otherwise hand-rolled.
- **Testing**: Fully autonomous. Headless Node.js for physics/control, Playwright (headless Chromium) for UI. User reviews only at end of each phase (optional screenshot drop to output_port) and final live URL.

---

## 1. Tech stack & repo layout

**Stack**:

- Vanilla HTML / CSS / JS (no React, no Vite, no build step).
- `<canvas>` for pendulum animation and all plots.
- Optional `math.js` for matrix ops (CARE solver, eigendecomp); hand-rolled fallback for 4×4-ish.
- No server. Pure static `index.html` + JS modules (ES modules via `<script type="module">`).
- `sympy` (Python) offline to derive EOMs → emits JS code → pasted into `src/physics/nlink_N.js`.
- Node.js + `vitest` (or plain `node --test`) for headless tests.
- `playwright` for UI tests + screenshots.

**Repo layout**:

```
index.html
src/
  main.js                — entry, rAF loop, mode switching
  state.js               — global params + sim state
  presets.js
  physics/
    nlink_1.js           — EOM for n=1 (sympy-generated)
    nlink_2.js           — EOM for n=2 (sympy-generated)
    nlink_3.js           — EOM for n=3 (sympy-generated)
    integrator.js        — Forward Euler, Semi-Implicit Euler, RK4
  control/
    lqr.js               — linearization, Kleinman CARE solver, K
    swingup.js           — energy-based (Aström)
    tvlqr.js             — time-varying LQR for triple
    pid.js               — single only
    switcher.js          — region check + smooth blend
    sysid.js
  sensors.js
  actuator.js
  ui/
    panel.js             — parameter controls
    plots.js             — canvas plotting
    canvas.js            — pendulum animation
tools/
  derive_eom.py          — sympy script that emits nlink_*.js
  trajopt_triple.py      — offline trajectory optimization for triple swing-up
tests/
  energy.test.js
  integrator_convergence.test.js
  jacobian.test.js
  lqr_stability.test.js
  lqr_eigenvalues.test.js
  handover.test.js
  sensors.test.js
  actuator.test.js
  sysid.test.js
  ui.spec.js             — Playwright
  RESULTS.md             — generated test report
.claude/
  PLAN.md
  SKILL.md
package.json             — test deps only (node test runner + playwright)
README.md
```

---

## 2. Mathematical foundations

### 2.1 Coordinates

For an n-link pendulum on a cart:

- `x` — cart position [m]
- `θᵢ` for i = 1..n — angle of link i from upward vertical [rad]; θᵢ = 0 means link i points straight up.

State vector `q = [x, θ₁, ..., θₙ]ᵀ`. Full state `[q, q̇]` is 2(n+1)-D (8-D for triple).

**Sign convention** (one-time decision, never deviate): angles measured from up, CCW positive in screen coordinates. Documented at the top of every physics file.

### 2.2 Lagrangian → manipulator form

For each link i with mass `mᵢ`, full length `Lᵢ`, CoM offset `lᵢ`, inertia `Iᵢ` about CoM:

- `T = ½ m₀ ẋ² + Σᵢ [½ mᵢ (ẋ_cᵢ² + ẏ_cᵢ²) + ½ Iᵢ ω_iᵢ²]`
- `V = Σᵢ mᵢ g y_cᵢ`
- `L = T − V`

Euler–Lagrange → manipulator form:

```
M(q) q̈ + C(q, q̇) q̇ + G(q) = B u + F_friction(q̇)
```

- `M(q)` symmetric positive-definite (n+1)×(n+1) inertia matrix
- `C(q, q̇) q̇` Coriolis + centripetal
- `G(q)` gravity
- `B = [1, 0, ..., 0]ᵀ` — cart-only actuation
- `u` — horizontal force on cart [N] (force, not velocity; see §4.3)

Solve: `q̈ = M⁻¹ (B u − C q̇ − G − F_friction)`.

### 2.3 Derivation workflow

`tools/derive_eom.py` runs sympy → emits `src/physics/nlink_{1,2,3}.js`. Each file exports plain JS functions:

```js
export function M(q, params) { ... }
export function C(q, qdot, params) { ... }
export function G(q, params) { ... }
```

### 2.4 Integration

Implement all three. Default RK4 at `dt_sim = 1e-4 s` (10 kHz):

| Integrator        | Cost/step | Energy behavior                 | Use         |
|-------------------|-----------|---------------------------------|-------------|
| Forward Euler     | 1 eval    | drifts upward (anti-gravity)    | demo only   |
| Semi-implicit     | 1 eval    | bounded drift                   | OK fallback |
| RK4               | 4 evals   | small drift, accurate           | default     |

Expose integrator choice in UI for pedagogical side-by-side.

### 2.5 Energy invariant

In no-friction, no-input runs, total mechanical energy must be conserved. Used as headless test pass/fail.

---

## 3. Simulation architecture

### 3.1 Multi-rate timing

| Rate           | Default Hz | Period   | Role                          |
|----------------|------------|----------|-------------------------------|
| Physics        | 10000      | 0.1 ms   | ODE accuracy                  |
| Sensor sample  | 500        | 2 ms     | Discretize state to controller|
| Control update | 200        | 5 ms     | Compute motor command         |
| Render         | 60         | 16.7 ms  | Canvas + plots                |

All four user-adjustable. Per-rAF accumulator pattern (PLAN v1 §3.1). Cap `MAX_FRAME = 50 ms` to prevent backgrounded-tab catch-up.

### 3.2 Sensor model

Per channel (joint angles + cart position; velocities not directly measured):

- Quantization to nearest LSB (configurable bits; default 12-bit encoder).
- Gaussian noise (σ per channel; defaults 0.1° angle, 1 mm cart).
- Delay ring buffer (default 1–5 ms).
- Optional bias / drift (low priority).

Velocity estimation in controller: filtered finite-difference by default; observer (Kalman) as opt-in.

### 3.3 Actuator model

Commanded force → applied force passes through:

- Saturation `[-F_max, +F_max]`.
- Slew rate limit (motor current-loop bandwidth proxy).
- First-order lag `τ_motor` (1–20 ms).
- Cart friction (viscous + Coulomb).
- Optional additive force noise.

### 3.4 Disturbances

- Kick button (impulse on chosen link, slider magnitude).
- Click-and-drag a link (applied force while held).
- Step disturbance (constant lateral force for N seconds).

---

## 4. Control system design

### 4.1 LQR stabilization

Linearize EOM at `q = 0, q̇ = 0` (numerical Jacobian, finite-difference; recomputed on parameter change). Solve continuous-time algebraic Riccati equation with Kleinman iteration (warm-start from previous K). Gain `K = R⁻¹ Bᵀ P`. Law `u = −K x_state`.

UI: Q diagonal sliders + scalar R + optional Q/R auto-tune button.

### 4.2 Swing-up — single (Åström–Furuta)

`u = k · sign(θ̇ cosθ) · Ẽ` where `Ẽ = E − E*` and `E* = 2 m g l`. Pumps energy when below upright energy, removes when above.

### 4.3 Swing-up — double

Energy-based + partial feedback linearization. Expect iteration.

### 4.4 Swing-up — triple (research-grade — see §6)

Plan A: energy-based (likely fails).
Plan B: offline trajectory optimization in Python → TVLQR tracker.
Plan C fallback: ship "near-upright start" mode and document.

### 4.5 Handover

Region-of-attraction proxy: `|θᵢ| < θ_thresh`, `|θ̇ᵢ| < ω_thresh`, `|x|, |ẋ|` bounded. Linear blend `u_swingup ↔ u_LQR` over 50–100 ms to avoid discontinuous force step.

---

## 5. System identification

- **Hang test** — small-angle period fit → constraint on m, l, I.
- **Excitation** — chirp / PRBS / step / impulse on cart; log I/O.
- **Fit** — output-error least squares against linear model.
- **UI** — choose excitation, run, see estimated vs ground truth, "Apply to controller" / "Revert".
- **Stretch** — online EKF for joint state+param estimation.

---

## 6. Testing strategy

Two layers. Fully autonomous — no manual smoke tests required from user.

### 6.1 Layer 1 — Headless Node.js tests (`tests/*.test.js`)

Run via `node --test tests/` (or `npm test`). Each test imports the relevant module(s) directly (ES modules), runs sim, asserts.

| Test                              | Asserts                                                                |
|-----------------------------------|------------------------------------------------------------------------|
| `energy.test.js`                  | n=1,2,3 zero-friction zero-input runs conserve E to <0.1% over 10 s    |
| `integrator_convergence.test.js`  | Halving dt drops RK4 error by ~16× (order-4 convergence)               |
| `jacobian.test.js`                | Numerical Jacobian matches sympy-analytical to 1e-6                    |
| `lqr_stability.test.js`           | Start at perturbation → ‖state‖ → 0 within bound                       |
| `lqr_eigenvalues.test.js`         | All closed-loop (A−BK) eigenvalues in LHP, damping ratio > 0.3         |
| `controllability.test.js`         | rank([B, AB, A²B, ...]) = full for n=1,2,3                             |
| `handover.test.js`                | Single from hanging → upright in 30 s with smooth blend                |
| `sensors.test.js`                 | Noise / quant / delay statistics match spec                            |
| `actuator.test.js`                | Saturation, slew, lag respond as designed                              |
| `sysid.test.js`                   | Fit from known params recovers within 5% tolerance                     |
| `robustness_sweep.test.js`        | Parameter sweep heatmap of LQR success across noise/delay/mass/length  |

Total runtime target: < 30 s.

### 6.2 Layer 2 — Playwright UI tests (`tests/ui.spec.js`)

- Load `index.html` in headless Chromium.
- DOM assertions: every slider exists and emits input events; canvas non-empty after 1 s.
- Mode switching 1 → 2 → 3 → 1, no console errors.
- Click "Kick" → force plot shows spike.
- Screenshot animation and plots → save to `tests/screenshots/`.
- (Optional) Push screenshots to output_port at end of each UI phase.

### 6.3 Triple-pendulum specific validation (Phases 12 & 13)

**Stabilization (Phase 12)**:
1. Closed-loop eigenvalues all in LHP with damping > 0.3 (else model bug).
2. Controllability rank = 8.
3. Robustness sweep: mass ±20%, length ±20%, friction ±50%, sensor noise σ ∈ {0, 0.1°, 0.5°, 1°}, delay ∈ {0, 2, 5, 10 ms}. Headless run per combo → success matrix written to `tests/RESULTS.md`.
4. Q/R auto-tune: scripted diagonal-Q search; pick widest robustness margin.

**Swing-up (Phase 13)**:
1. Plan A energy-based — try, log failure mode.
2. Plan B `tools/trajopt_triple.py` (CasADi or scipy direct collocation): minimize ∫u²dt s.t. EOM, x(0)=hanging, x(T)=upright, |u|≤F_max. Dump `(x*(t), u*(t))` + TVLQR `K(t)` as JSON.
3. In-browser tracker: `u = u*(t) − K(t)·(x − x*(t))`.
4. Validation: perturb initial state, headless run, basin-of-attraction report.
5. Plan C fallback: near-upright-start UI toggle if even TVLQR is flaky. Document honestly.

### 6.4 Test report

`tests/RESULTS.md` is regenerated each test run. Contains per-test pass/fail, runtime, and triple-pendulum robustness matrix. Committed at end of each phase.

---

## 7. UI / UX

### 7.1 Layout

```
┌─────────────────────────────────────────────────────────────┐
│ Mode: [1-link] [2-link] [3-link]    [▶/⏸] [Reset] [Speed ×] │
├──────────────────────────┬──────────────────────────────────┤
│   Pendulum canvas        │ Parameters                       │
│                          │   ▾ Physical / Cart              │
│                          │   ▾ Sensor / Actuator            │
│                          │   ▾ Controller                   │
│                          │   ▾ Sim                          │
│                          ├──────────────────────────────────┤
│                          │ Mode: Swing-up / Stabilize / SysID│
├──────────────────────────┴──────────────────────────────────┤
│  Plots: angles | velocities | phase portrait | control force │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Parameter panel groups (collapsible, slider + numeric per param)

**Physical** (per link): m, L, l, I, joint friction (viscous + Coulomb).
**Cart**: m₀, friction, gravity g.
**Sensors**: noise σ per channel, quant bits, sample period, delay.
**Actuator**: F_max, τ_motor, slew, force noise σ.
**Controller**: mode (Swing-up / LQR / Auto), Q diagonals + R, PID Kp/Ki/Kd (single), handover thresholds, control period.
**Sim**: integrator, dt_sim, speed multiplier, random seed.

**Presets dropdown**: "Default 1-link", "Default 3-link", "Noisy sensors", "Fast motor", "Stiff triple", etc. Save/load to localStorage.

### 7.3 Plots (canvas, rolling 10 s, render at 30 Hz)

- Angles vs t (one line per joint, color-coded).
- Velocities vs t.
- Phase portrait (θ vs θ̇ per joint, fading trail).
- Control force vs t (with ±F_max dashed reference).

### 7.4 Interactions

- Mouse drag a link → apply force while held.
- "Kick" button (magnitude / direction / target-link).
- Drag cart to set initial position.
- Keyboard: `Space` pause, `R` reset, `K` kick, `1/2/3` switch modes.

---

## 8. Development phases

16 phases. Each ends with: code committed + tests passing + `RESULTS.md` updated. No phase advances until previous passes its tests.

### Phase 1 — Skeleton
**Steps**: 1.1 `index.html` layout + CSS grid. 1.2 mode selector (1/2/3) wired to state. 1.3 blank canvas + rAF loop. 1.4 parameter panel scaffolding (empty groups). 1.5 `package.json` + node test runner + playwright install.
**Tests**: Playwright DOM smoke (`ui.spec.js` minimal): page loads, mode buttons clickable, canvas exists.

### Phase 2 — Physics: single
**Steps**: 2.1 `tools/derive_eom.py` for n=1, emit `nlink_1.js`. 2.2 `integrator.js` (Euler, SI-Euler, RK4). 2.3 wire physics into rAF loop. 2.4 draw cart + link on canvas.
**Tests**: `energy.test.js` (n=1), `integrator_convergence.test.js`, `jacobian.test.js` (n=1).

### Phase 3 — Sensor + actuator models
**Steps**: 3.1 `sensors.js` (quant + Gaussian noise + delay buffer + finite-diff velocity). 3.2 `actuator.js` (saturation + slew + first-order lag + cart friction). 3.3 wire into rAF loop. 3.4 UI sliders for σ, F_max, τ, slew.
**Tests**: `sensors.test.js`, `actuator.test.js`.

### Phase 4 — LQR: single
**Steps**: 4.1 numerical Jacobian for A,B at upright. 4.2 Kleinman CARE solver (warm-start). 4.3 controller wiring (control update rate). 4.4 Q/R sliders.
**Tests**: `lqr_eigenvalues.test.js` (n=1), `lqr_stability.test.js` (n=1), `controllability.test.js` (n=1).

### Phase 5 — Swing-up: single + handover
**Steps**: 5.1 `swingup.js` Aström–Furuta law. 5.2 `switcher.js` region check + linear blend. 5.3 mode dropdown (Swing-up / LQR / Auto).
**Tests**: `handover.test.js` (n=1): start hanging, finish upright within 30 s, ‖θ‖<5° final.

### Phase 6 — Plots
**Steps**: 6.1 `plots.js` rolling-buffer canvas plot primitive. 6.2 angles, velocities, phase portrait, control force panels. 6.3 30 Hz render throttle.
**Tests**: Playwright: each plot canvas has non-zero pixels after 2 s sim.

### Phase 7 — Parameter UI
**Steps**: 7.1 all collapsible groups populated with sliders + numeric inputs. 7.2 preset dropdown wired to localStorage. 7.3 keyboard shortcuts (Space, R, K, 1/2/3).
**Tests**: Playwright: every slider mutates state; preset round-trip works; keyboard events fire.

### Phase 8 — Physics: double
**Steps**: 8.1 `derive_eom.py` for n=2, emit `nlink_2.js`. 8.2 canvas draws two links. 8.3 sympy-vs-numerical Jacobian sanity.
**Tests**: `energy.test.js` (n=2), `jacobian.test.js` (n=2).

### Phase 9 — LQR: double
**Steps**: 9.1 Jacobian + CARE for n=2. 9.2 Q diagonals updated.
**Tests**: `lqr_eigenvalues.test.js` (n=2), `lqr_stability.test.js` (n=2), `controllability.test.js` (n=2).

### Phase 10 — Swing-up: double
**Steps**: 10.1 energy-based + partial feedback linearization for n=2. 10.2 handover thresholds tightened. 10.3 iterate.
**Tests**: `handover.test.js` (n=2): start hanging, finish upright. If swing-up cannot stabilize reliably, document specific failure regime.

### Phase 11 — Physics: triple
**Steps**: 11.1 `derive_eom.py` for n=3 (hardest derivation). 11.2 canvas draws three links. 11.3 verify energy conservation extra-carefully (numerical condition number of M for triple is worse).
**Tests**: `energy.test.js` (n=3), `jacobian.test.js` (n=3).

### Phase 12 — LQR: triple
**Steps**: 12.1 Jacobian + CARE for n=3. 12.2 controllability check (rank = 8). 12.3 Q/R auto-tune script. 12.4 robustness sweep (`robustness_sweep.test.js`) over mass/length/friction/noise/delay.
**Tests**: `lqr_eigenvalues.test.js` (n=3), `lqr_stability.test.js` (n=3), `controllability.test.js` (n=3), robustness matrix in `RESULTS.md`.

### Phase 13 — Swing-up: triple
**Steps**: 13.1 try energy-based, log failure. 13.2 `tools/trajopt_triple.py` direct collocation → JSON of `(x*(t), u*(t), K(t))`. 13.3 in-browser TVLQR tracker. 13.4 basin-of-attraction validation. 13.5 if all fail, ship "near-upright start" toggle + documented limitation.
**Tests**: `triple_swingup.test.js`: from hanging → upright with TVLQR tracker; OR documented near-upright fallback in `RESULTS.md`.

### Phase 14 — System identification
**Steps**: 14.1 `sysid.js` excitation generators (impulse / chirp / PRBS / step). 14.2 fit routines (hang-test small-angle + output-error). 14.3 UI panel (choose excitation, run, see estimate vs truth, Apply / Revert).
**Tests**: `sysid.test.js`: known-params round-trip within 5% tolerance.

### Phase 15 — Polish
**Steps**: 15.1 kick + click-drag disturbance. 15.2 full preset library. 15.3 keyboard shortcuts finalized. 15.4 optional Web Worker for physics (only if profiling demands). 15.5 README.md with quickstart + GIFs.
**Tests**: Playwright full-flow regression: hanging → swing-up → LQR → kick → recover.

### Phase 16 — Deployment to resume site
**Steps**: 16.1 final Playwright screenshot pass → push samples to output_port for user review. 16.2 static copy `Inverse_Pendulum/` (everything except `.claude/`, `tests/`, `tools/`) → `TRASHCAN/SungminLee511.github.io/projects/inverse-pendulum/`. 16.3 add card on resume index linking to it. 16.4 verify path on GitHub Pages (relative asset URLs). 16.5 push resume; wait for Pages deploy; me curl the live URL and report 200 + sanity-check HTML.
**Tests**: Live-URL fetch + grep for expected DOM markers; final screenshot from live URL pushed to output_port.

---

## 9. Pitfalls (carried from v1)

- **Sign conventions** — pick once, document at top of every physics file. Most physics bugs are sign errors.
- **Integrator drift** — even RK4 drifts over minutes; don't run forever between resets when checking energy.
- **Handover transients** — discontinuous control output excites the pendulum; blend over 50–100 ms.
- **LQR + saturation** — when LQR commands beyond F_max it can wind up; either add anti-windup or rely on the slew/lag to mask it.
- **Triple sensitivity** — linearized triple is hair-trigger. 0.5° sensor noise + 5 ms delay can sink LQR. Tighten sensor params before blaming the controller.
- **M(q) conditioning** — SPD by construction, but if NaNs appear check all m, L, I positive and no sign typos.
- **Tab throttling** — backgrounded `requestAnimationFrame` slows down. Cap MAX_FRAME or reset accumulators on resume.
- **Premature optimization** — n=3 doesn't need a Web Worker. Don't add complexity until profiling says so.

---

## 10. Open questions / decide-later

- Controller velocity estimate: filtered finite-difference (default) vs state observer (opt-in).
- Replay / run save-load (not v1 unless requested).
- Online EKF parameter estimation (stretch).
- MPC option for saturation-aware control (easy to add post-v1).

---

## 11. Phase / step summary

- **Total phases**: 16 (15 build + 1 deployment).
- **Total numbered steps across phases**: 67.
- **Total headless tests**: 11 numeric + Playwright UI spec + robustness sweep + triple-swing-up dedicated.
- **Live deliverable**: GitHub-Pages-hosted page at `SungminLee511.github.io/projects/inverse-pendulum/`, linked from resume index, with screenshots in output_port.
