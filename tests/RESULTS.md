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
