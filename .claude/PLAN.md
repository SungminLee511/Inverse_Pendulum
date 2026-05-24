# Inverted Pendulum on a Cart — Simulation Design Document

A single HTML/CSS/JS app simulating 1-, 2-, and 3-link inverted pendulums on a horizontally-driven cart. Includes energy-based swing-up, LQR stabilization with smooth handover, system identification via swing tests, adjustable physical/sensor/actuator/controller parameters, sensor and motor noise/delay/saturation, and real-time visualization with time-series, phase-portrait, and control-force plots.

-----

## 1. Project Goals

- **High-fidelity physics** for n ∈ {1, 2, 3} link inverted pendulums on a cart.
- **Multi-rate timing**: sim step ≪ sensor sample ≪ control update ≪ render.
- **Controllers**: energy-based swing-up + LQR stabilization with smooth handover.
- **System identification**: excite the pendulum (swing/chirp/impulse), fit physical parameters from the response.
- **Adjustable parameters**: masses, lengths, CoMs, inertias, gravity, friction, sensor noise, actuator dynamics, controller gains.
- **Real-time visualization**: pendulum animation, angle/velocity time series, phase portraits, control-force plot.
- **One HTML page** with a mode selector for 1/2/3 links. Pure frontend.

-----

## 2. Mathematical Foundations

### 2.1 Generalized coordinates

For an n-link pendulum on a cart, use:

- `x` — horizontal cart position [m]
- `θᵢ` for i = 1..n — angle of link i from upward vertical [rad]; θᵢ = 0 means link i points straight up.

State vector: `q = [x, θ₁, ..., θₙ]ᵀ`. Full state `[q, q̇]` is 2(n+1)-dimensional (so 8-D for the triple).

**Sign conventions** (decide once, never deviate): angles measured from up, CCW positive in screen coordinates. Document at top of every file. Most physics bugs in this kind of project are sign errors.

### 2.2 Lagrangian mechanics

For each link i — mass `mᵢ`, full length `Lᵢ`, CoM at distance `lᵢ` from the joint, inertia `Iᵢ` about CoM — compute the CoM position by accumulating through preceding links:

- Kinetic energy `T = ½ m₀ ẋ² + Σᵢ [½ mᵢ (ẋ_cᵢ² + ẏ_cᵢ²) + ½ Iᵢ ω_iᵢ²]`
- Potential energy `V = Σᵢ mᵢ g y_cᵢ`
- Lagrangian `L = T − V`

Euler–Lagrange yields the standard manipulator form:

```
M(q) q̈ + C(q, q̇) q̇ + G(q) = B u + F_friction(q̇)
```

- `M(q)` — (n+1)×(n+1) inertia matrix, symmetric positive-definite
- `C(q, q̇) q̇` — Coriolis & centripetal terms
- `G(q)` — gravity terms
- `B = [1, 0, ..., 0]ᵀ` — actuation acts only on the cart
- `u` — horizontal force on the cart [N]  ← **the control input is force, not velocity** (see §3.3)

Solve: `q̈ = M⁻¹ (B u − C q̇ − G − F_friction)`.

### 2.3 Practical derivation strategy

Hand-deriving M, C, G for n = 2 is painful, for n = 3 nearly intolerable. Recommended workflow:

1. Use `sympy` (offline, in Python) to derive M(q), C(q, q̇), G(q) symbolically for each n.
1. Have sympy emit JS/JSON code for the matrix entries.
1. Paste those into the JS physics module as plain functions: `(q, q̇, params) → matrices`.
1. **Sanity check**: with zero friction and zero input, total energy must be conserved (RK4 will drift slowly; visible drift over seconds means a derivation or integration bug).

Alternative: a generic recursive Newton–Euler implementation that loops over links. Cleaner code, but harder to debug if something is off. For this project, the hardcoded-per-n approach is more transparent.

### 2.4 Numerical integration

Choice of integrator drives realism. Implement and expose:

|Integrator                      |Cost / step|Energy behavior                  |Use        |
|--------------------------------|-----------|---------------------------------|-----------|
|Forward Euler                   |1 eval     |drifts upward (fake anti-gravity)|demo only  |
|Semi-implicit (symplectic) Euler|1 eval     |bounded drift                    |OK fallback|
|RK4                             |4 evals    |small drift, very accurate       |**default**|

**Default**: RK4 at sim step `dt_sim = 1 × 10⁻⁴ s` (10 kHz). The math is cheap enough — even n=3 at 10 kHz is well under a millisecond of CPU per render frame.

Expose integrator choice in the UI so the user can demonstrate Euler vs RK4 quality side by side — useful pedagogically.

### 2.5 Energy as a debug invariant

In a no-friction, no-input run, total mechanical energy must be conserved. Plot E(t) optionally (hidden by default since you didn’t request it, but keep the computation available for debugging).

-----

## 3. Simulation Architecture

### 3.1 Multi-rate timing

Four independent rates:

|Rate               |Default Hz|Default period|Role                          |
|-------------------|----------|--------------|------------------------------|
|Physics integration|10 000    |0.1 ms        |ODE accuracy                  |
|Sensor sampling    |500       |2 ms          |Discretize state to controller|
|Control update     |200       |5 ms          |Compute motor command         |
|Render             |60        |16.7 ms       |Draw canvas + plots           |

All four are independently adjustable.

Pattern per `requestAnimationFrame` tick:

```
deltaWallTime = now − lastFrame
simAdvance = min(deltaWallTime, MAX_FRAME) × speedMultiplier
while simAdvance > 0:
    physicsStep(dt_sim)
    sensorAccum += dt_sim
    controlAccum += dt_sim
    if sensorAccum ≥ dt_sensor:
        sample sensors (noise + quantization + delay); sensorAccum -= dt_sensor
    if controlAccum ≥ dt_control:
        u_cmd = controller(latest_sensor_sample); controlAccum -= dt_control
    simAdvance -= dt_sim
render()
```

`speedMultiplier` (0.1× to 5×) gives slow-mo and fast-forward. Cap MAX_FRAME (e.g. 50 ms) so a backgrounded tab doesn’t try to catch up an enormous chunk of sim time on resume.

### 3.2 Sensor model

Each measurable channel (joint angles and cart position; velocities are *not* directly measured — controller estimates them by differencing or with an observer):

- **Quantization** — round to nearest LSB. E.g. 12-bit encoder = 4096 counts/rev = ~0.088°/count.
- **Gaussian noise** — σ adjustable (defaults: 0.1° per angle, 1 mm cart).
- **Delay** — small ring buffer; defaults 1–5 ms.
- **Bias / drift** (optional, low priority).

The controller never sees the true state — it sees the noised, quantized, delayed version. This is what makes the sim a meaningful pre-test of a real control loop.

Velocity estimation options for the controller:

- **Finite difference** of consecutive samples (noisy; needs a filter).
- **Kalman filter / state observer** over the linearized model (cleaner, more code).
- Default to filtered finite-difference; expose option to use an observer.

### 3.3 Actuator model

The commanded force `u_cmd` (controller output) passes through:

- **Saturation**: clip to `[−F_max, F_max]`.
- **Slew-rate limit**: `|du/dt| ≤ slew_max` (models motor current loop bandwidth).
- **First-order lag**: `u_applied += (u_cmd − u_applied) · dt / τ_motor` (motor time constant ~1–20 ms).
- **Static + viscous friction on the cart**: subtract a friction force opposing motion.
- Optional: additive Gaussian force noise.

**On force vs velocity as the control input**: real motors produce torque/force, not velocity. Commanding velocity assumes a perfect inner velocity loop, which hides motor dynamics and friction from the outer controller — fine for a textbook demo, dishonest for sim-to-real. Use force as the input; users who want a velocity command can build a velocity controller as a thin wrapper.

### 3.4 Disturbances

- **Kick button** — impulse on chosen link, magnitude slider.
- **Click-and-drag** a link → applied force while held.
- **Step disturbance** — constant lateral force for N seconds.

Essential for evaluating robustness and visualizing recovery behavior.

-----

## 4. Control System Design

### 4.1 Stabilization: LQR

Linearize the EOM around the upright equilibrium `q = 0, q̇ = 0`:

```
ẋ_state = A x_state + B u
```

where `x_state = [x, θ₁, ..., θₙ, ẋ, θ̇₁, ..., θ̇ₙ]ᵀ`.

Compute A, B by symbolic differentiation of the EOM (offline, again with sympy) or by numerical Jacobian (finite differences). Numerical is fine since A, B must be recomputed whenever physical parameters change.

Solve the continuous-time algebraic Riccati equation (CARE):

```
AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0
```

Feedback gain `K = R⁻¹ Bᵀ P`. Control law: `u = −K x_state`.

**CARE solver in JS**:

- **Kleinman iteration** — fixed-point given an initial stabilizing K. Simple to implement. Warm-start from the previous K when parameters change slightly.
- **Schur method** — Hamiltonian matrix eigendecomposition. More robust but needs eigenvalue routines; pull in `math.js` or `numeric.js`.

Recommend Kleinman; bootstrap an initial K by hand (or by pole placement) for the defaults.

UI: expose `Q` diagonal (one slider per state weight) and a scalar `R`.

### 4.2 Swing-up: energy-based (single)

Canonical method (Åström–Furuta). Pendulum total energy `E`; desired energy `E* = 2 m g l` (energy at the upright). Error `Ẽ = E − E*`.

Sketch:

```
u = k · sign( θ̇ cosθ ) · Ẽ
```

Pumps energy when `Ẽ < 0`, removes when `Ẽ > 0`. Add saturation. Switch to LQR once near upright.

### 4.3 Swing-up: double and triple

Significantly harder. Options:

- **Partial feedback linearization** — feedback-linearize the actuated coordinate, design a controller for the underactuated remainder. Works for some double-pendulum configurations.
- **Trajectory optimization** — solve offline for a feasible swing-up trajectory (direct collocation in Python). Track with **TVLQR** (time-varying LQR) or feedforward + LQR.
- **RL** — the sim is an environment; train a policy. Out of scope but feasible.

**Honest expectations**:

- **Double**: energy-based shaping + partial feedback linearization is plausible. Expect to iterate.
- **Triple**: research-grade. If energy-based fails (likely), fall back to a precomputed trajectory tracked with TVLQR. Stretch goal: “start from near upright” if full swing-up is too hard.

### 4.4 Handover from swing-up to LQR

“Region of attraction” proxy:

- All `|θᵢ| < θ_thresh` (e.g. 20° single, ~10° double, ~5° triple).
- All `|θ̇ᵢ| < ω_thresh`.
- `|x|, |ẋ|` bounded.

Inside region → switch to LQR. To avoid bang switching: linearly blend `u_swingup` and `u_LQR` over a short window (~50–100 ms). This avoids a discontinuous force step that would otherwise itself excite the pendulum.

### 4.5 Per-mode notes

- **Single (1-link)** — PID can also stabilize. LQR is one-line. Swing-up reliable.
- **Double (2-link)** — PID alone won’t stabilize (two underactuated coordinates). LQR works. Swing-up is the interesting problem.
- **Triple (3-link)** — LQR can stabilize from near upright if the model is accurate and noise is low. Sensitivity is high: 0.5° sensor noise and 5 ms delay can sink it. Swing-up is genuinely hard.

-----

## 5. System Identification

You asked specifically for “swing tests to find parameters like mass and length.” Approach:

### 5.1 Hang test (small-angle linearization)

Hang the pendulum in the stable downward equilibrium. Give a small impulse. Observe the oscillation.

For a single pendulum, period for small angles:

```
T = 2π √( I_eff / (m g l) )
```

Measure T from zero crossings → constraint relating m, l, I.

Add a known reference mass and re-measure → break the ambiguity. Or weigh the cart known.

For n > 1, the small-angle response is a linear ODE with n modes. Fit by least squares of the analytic mode solution to the observed angle traces.

### 5.2 Excitation tests

Drive the cart with a designed input — chirp (e.g. 0.1 → 5 Hz over 30 s), PRBS, or step. Log cart position and joint angles. Fit linear-model parameters from input/output by output-error or prediction-error methods.

### 5.3 Online estimation (stretch)

Augment the state with unknown parameters (slowly varying), use an Extended Kalman Filter to estimate state + params jointly.

### 5.4 Sys-ID UI

- Choose excitation: impulse / chirp / PRBS / step.
- Run duration.
- “Run sys-ID” → log data, fit, display estimated vs true (you have ground truth — perfect for validating the method).
- “Apply fitted params to controller” / “Revert”.

This is also useful as a teaching tool: shows how much excitation you need to identify which parameters.

-----

## 6. UI / UX Design

### 6.1 Layout (single page)

```
┌─────────────────────────────────────────────────────────────┐
│ Mode: [1-link] [2-link] [3-link]    [▶/⏸] [Reset] [Speed ×] │
├──────────────────────────┬──────────────────────────────────┤
│                          │ Parameters                       │
│   Pendulum canvas        │  ▾ Physical                      │
│   (the animation)        │  ▾ Sensor / Actuator             │
│                          │  ▾ Controller                    │
│                          │  ▾ Sim                           │
│                          ├──────────────────────────────────┤
│                          │ Mode: Swing-up | Stabilize | SysID│
├──────────────────────────┴──────────────────────────────────┤
│  Plots: angles | velocities | phase portrait | control force │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 Parameters panel (collapsible groups, slider + numeric for each)

**Physical (per link i)**

- mᵢ [kg] · Lᵢ [m] · lᵢ (CoM offset, default Lᵢ/2) · Iᵢ (default m L²/12) · joint viscous friction · joint Coulomb friction

**Cart**

- m₀ · cart viscous friction · cart Coulomb friction · gravity g

**Sensors**

- angle noise σ · cart position noise σ · quantization bits · sample period · delay

**Actuator**

- F_max · motor time constant τ · slew rate · force noise σ

**Controller**

- mode: Swing-up only / LQR only / Auto (swing-up → LQR)
- LQR Q diagonals + R
- (optional, single only) PID Kp/Ki/Kd
- Handover thresholds
- Control period

**Sim**

- Integrator (Euler / Semi-Implicit / RK4)
- Sim step
- Speed multiplier
- Random seed (reproducible noise)

**Presets dropdown**: “Default 1-link”, “Default 3-link”, “Noisy sensors”, “Fast motor”, “Stiff triple”, etc. Save/load to localStorage.

### 6.3 Plots

Custom canvas-drawn (don’t pull in chart.js — overkill, and harder to make smooth). Rolling 10-second window. Render at 30 Hz (every other frame).

- **Angles vs t** — one line per joint, color-coded.
- **Velocities vs t** — same.
- **Phase portrait** — θ vs θ̇ per joint, fading trail (older points more transparent). Beautiful and informative.
- **Control force vs t** — line plot with dashed `±F_max` reference lines.

### 6.4 Interactions

- Mouse drag a link → apply force while held (great for stress-testing).
- “Kick” button with magnitude / direction / target-link selectors.
- Drag the cart to set initial position.
- Keyboard: `Space` pause, `R` reset, `K` kick, `1/2/3` switch modes.

-----

## 7. Tech Stack & Implementation

### 7.1 Stack

- Vanilla HTML/CSS/JS (no framework needed; pick React/Svelte if more comfortable, but the rewards are small here).
- `<canvas>` for both the pendulum animation and the plots.
- Optional: `math.js` for matrix inverse, eigendecomposition. For 4×4 inverses and the CARE solver, hand-rolling is also fine.
- No build step needed; can be a single `index.html` + a few JS files.

### 7.2 Module layout

```
index.html
src/
  main.js           — entry, rAF loop, mode switching
  physics/
    nlink_1.js      — EOM for n=1 (from sympy)
    nlink_2.js      — EOM for n=2
    nlink_3.js      — EOM for n=3
    integrator.js   — Euler, SI-Euler, RK4
  control/
    lqr.js          — A,B linearization, CARE solver, K
    swingup.js      — energy-based
    pid.js          — single only
    switcher.js     — region check + blend
    sysid.js
  sensors.js
  actuator.js
  ui/
    panel.js        — parameter controls
    plots.js        — canvas plotting
    canvas.js       — pendulum animation
  presets.js
  state.js          — global params + sim state
```

### 7.3 Performance

n=3 at 10 kHz sim, 60 Hz render → ~30 000 RK4 evals/sec, each a 4×4 matrix solve. Comfortably under a few ms per render frame on a laptop. If you ever hit a wall: move physics into a Web Worker, communicate state via `postMessage` or `SharedArrayBuffer`.

### 7.4 Math helpers

- 4×4 matrix solve (LU or direct formula).
- Continuous-time Riccati solver (Kleinman iteration recommended; warm-start from previous gain).
- Small linear-algebra utilities (vec ops, transpose, multiply).

-----

## 8. Development Phases

Build in order. Don’t move to phase n+1 until n is solid.

1. **Skeleton** — HTML layout, mode selector, blank canvas, parameter panel scaffolding.
1. **Physics — single** — derive (sympy), RK4 integrator, free-dynamics animation, verify energy conservation.
1. **Sensor + actuator models** — noise, quantization, delay, saturation, slew.
1. **LQR — single** — linearize, solve CARE, stabilize from near-upright.
1. **Swing-up — single** — energy-based, smooth handover to LQR.
1. **Plots** — angles, velocities, phase portrait, control force.
1. **Parameter UI** — all sliders, presets, save/load.
1. **Physics — double** — derive, integrate, verify.
1. **LQR — double** — stabilize.
1. **Swing-up — double** — energy + partial feedback linearization. Expect iteration.
1. **Physics — triple** — derive (this is the hardest derivation).
1. **LQR — triple** — stabilize from near-upright.
1. **Swing-up — triple** — try energy-based; on failure, build a precomputed trajectory + TVLQR tracker.
1. **System identification** — drop/chirp tests, fitting, ground-truth comparison UI.
1. **Polish** — keyboard shortcuts, presets library, optional Web Worker.

-----

## 9. Pitfalls & Lessons

- **Sign conventions** — pick once, document at the top of every physics file, never deviate. Most physics bugs are sign errors.
- **Integrator drift** — even RK4 drifts over minutes. Don’t run forever between resets if you’re checking energy.
- **Handover transients** — discontinuous control output at swing-up → LQR switch *excites* the pendulum. Blend over ~50–100 ms.
- **LQR + saturation** — when LQR commands beyond F_max, the controller “thinks” it’s pushing harder than it is. Add anti-windup or treat the saturated case explicitly (e.g. MPC handles this naturally; LQR does not).
- **Triple sensitivity** — linearized triple is hair-trigger. 0.5° sensor noise + 5 ms delay can sink LQR. Tighten sensor params before blaming the controller.
- **M(q) conditioning** — symmetric positive-definite by construction, but if you see NaNs check that L’s, m’s, I’s are all positive and that you didn’t mistype a sign.
- **Tab throttling** — backgrounded tabs slow `requestAnimationFrame`. Cap MAX_FRAME or reset accumulators on resume so you don’t try to “catch up” several seconds of sim in one frame.
- **Float precision** — JS doubles are fine.
- **Premature optimization** — n=3 doesn’t need a Web Worker. Don’t add complexity until profiling says so.

-----

## 10. Looking Ahead (post-sim)

A brief note since you said not to focus on hardware now, but worth remembering:

- The sim’s value for the hardware build depends entirely on how honestly it models reality. Keep friction, motor lag, sensor noise, and quantization realistic — those are the usual sim-to-real gaps.
- The sim is also where you safely test what you wouldn’t on hardware: maximum disturbance, sensor dropouts, controller bugs, parameter mismatches.

-----

## Open questions / things to decide later

- Should the controller’s velocity estimate come from finite differences or a state observer? (Default: filtered finite difference; observer as opt-in.)
- Save/load runs for replay or controller comparison? (Not v1 unless you want it.)
- Online Kalman parameter estimation? (Stretch.)
- Optional MPC, since you didn’t pick it but it would handle saturation more gracefully than LQR? (Easy to add later if curiosity strikes.)