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
