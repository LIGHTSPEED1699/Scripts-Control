---
name: Scripts-Control
description: "Control systems simulation library for web-based PID tuning, process modeling, and adaptive control."
metadata:
  author: "Spot (hongbinli-website)"
  version: "1.3.0"
  homepage: "https://github.com/LIGHTSPEED1699/Scripts-Control"
  license: "MIT"
  keywords: ["pid", "control-systems", "simulation", "tuning", "tf2ss", "state-space", "lqr", "pole-placement"]
  source_repo: "https://github.com/LIGHTSPEED1699/hongbinli-website"
allowed-tools: ["read", "write", "edit", "exec"]
user-invocable: false
---

# Control Systems Web Library

Extracted and modularized simulation algorithms from the [hongbinli-website](https://github.com/LIGHTSPEED1699/hongbinli-website) interactive control systems tools.

## What's Included

| Module | Source | Description |
|--------|--------|-------------|
| `control-systems-lib.js` | `ManualPIDTuner.astro`, `PIDTuner.astro`, `RelayFeedbackPID.astro` | Core algorithms: TF↔SS, PID controller (derivative-on-measurement + back-calculation anti-windup), process models, tuning rules (ZN, CC, CHR, IMC, AMIGO), relay feedback |
| `mrac-lib.js` | `MRAC.astro` | Model Reference Adaptive Control: Lyapunov + MIT rule |
| `bode-plot-lib.js` | `PIDTuner.astro` | Frequency response, stability margins, Nyquist |
| `canvas-plot-utils.js` | `PIDTuner.astro`, `MRAC.astro` | Shared 2D-canvas drawing utilities |
| `process-model-lib.js` | `process-model/index.astro` | FOLPD / SOPDT / 2nd order step response and Bode |
| `combustion-lib.js` | `combustion-calculator/index.astro` | Natural gas / hydrogen combustion, exact dry-basis O₂, three-tier status |
| `loop-load-lib.js` | `loop-load/index.astro` | 4-20 mA loop load, cable resistance, voltage margin |
| `tool-utils.js` | `public/tool-utils.js` | Shared DOM utilities (formatNum, parseInput, debounce, getColors, observeDarkMode) |
| `pole-roots-lib.js` | `src/lib/closed-loop-poles.js` | Closed-form polynomial root solver (deg 1–4), characteristic poly, closed-loop / state-feedback poles |
| `linear-algebra-lib.js` | `public/lqr.js`, `src/lib/lqr.js` | Matrix ops, Lyapunov, continuous-time LQR (Kleinman), Ackermann pole placement, cart-pole wrapper |
| `anim-engine-lib.js` ⚠️ | `public/anim-engine.js` | RK4 integrator, history buffer, animation loop, MSD/tank drawing, 8 derivative functions (experimental) |

⚠️ = experimental. The anim-engine surface is broader than the rest of the repo's; provided as-is.

## Core Capabilities

### Transfer Function ↔ State Space
- `tfToStateSpace(num, den)` — Observable canonical form conversion
- `evaluateTF(num, den, sRe, sIm)` — Complex frequency evaluation
- Validated against Python `scipy.signal.tf2ss`

### PID Controller
- `PIDController(kp, ki, kd, tau_d, dt, limits)` — Full PID with derivative-on-measurement, one-pole derivative filter, and back-calculation anti-windup
- Industrial-standard derivative convention (no setpoint kicks)
- Smoother saturation recovery than the older clamp-and-correct-integral approach

### Process Models
- `createFOLPD(K, T, L)` — First-order lag plus dead time
- `createSOPDT(K, T1, T2, L)` — Second-order plus dead time
- `createArbitraryTF(num, den, L)` — Arbitrary polynomial transfer function

### Simulation Engine
- `simulateSetpoint(plant, kp, ki, kd, tau_d, dt)` — Closed-loop step response
- `simulateDisturbance(plant, kp, ki, kd, tau_d, dt)` — Load rejection response
- Forward Euler ODE integration with transport delay buffer

### Tuning Rules
- `zieglerNichols(K, T, L)` — Reaction curve method
- `cohenCoon(K, T, L)` — 1953 controllability-based
- `chienHronesReswick(K, T, L)` — 0% overshoot, disturbance rejection
- `imcTuning(K, T, L, tau_c)` — Internal Model Control PID (returns `{kp, ki, kd}` using the half-dead-time approximation)
- `imcPI(K, T, L, tau_c)` — Skogestad simplified IMC-PI
- `amigoTuning(K, T, L)` — Åström & Hägglund "Almost Minimum Input-Output" (returns `{kp, ti, td}`)

### Relay Feedback
- `relayFeedbackSimulation(plant, amplitude, dt, simTime)` — Auto-tuning via limit cycle
- Identifies ultimate gain (Ku) and period (Pu)

### Linear Algebra & Control (`linear-algebra-lib.js`)
- `solveLyapunov(A, C)` — `A^T X + X A = C` via Smith's reduced symmetric form
- `solveLQR(A, B, Q, R, opts)` — continuous-time LQR via Kleinman iteration; auto-picks a stabilizing Ackermann-based K_init for single-input plants
- `ackerStabilizing(A, B, desiredPoles)` — single-input Ackermann pole placement
- `polyFromRoots(roots)`, `lqrForCartPole(M, m, l, g, b_cart, Q, R)`, plus the standard matrix ops (matAdd, matMul, matTranspose, matFrobNorm, invertMatrix, identity, solveLinearSystem)

### Polynomial Roots (`pole-roots-lib.js`)
- `polynomialRoots(coeffs)` — closed-form root finder for degree 1–4
- `solveQuadratic`, `solveCubic` (Cardano with trig fallback), `solveQuartic` (Ferrari)
- `characteristicPoly`, `closedLoopPoles`, `stateFeedbackPoles` — top-level helpers for the inverted-pendulum simulator and similar 4-state plants

### Anim Engine (`anim-engine-lib.js`, experimental)
- `rk4`, `step`, `weighted` — RK4 integrator and helpers
- `createHistoryBuffer`, `startAnimLoop` — throttled time-series buffer and rAF loop
- `clamp`, `drawMSD`, `drawTankSVG` — UI primitives
- `deriv`, `paramPresets`, `initialStates` — physics primitives for msd, tank, dcMotor, rlcSeries, simplePendulum, invertedPendulum, pvtol, bioreactor

## Validation

Cross-checked against Python `scipy.signal.tf2ss` and `lsim`:
- DC gain: exact match
- Poles/eigenvalues: exact match
- Step response: <0.1% error (forward Euler discretization)

LQR validated against scipy.linalg.solve_continuous_are for the cart-pole example (M=2, m=0.2, l=0.5, g=9.81, K = [-4.47, -82.55, -10.10, -16.21]): matches the LQR-optimal K to 1e-3 in 6 iterations of Kleinman.

## Usage

```javascript
import {
  tfToStateSpace, PIDController, createFOLPD,
  simulateSetpoint, zieglerNichols, amigoTuning
} from './scripts/control-systems-lib.js';

// Create a process model
const plant = createFOLPD(2.0, 5.0, 1.0);

// Get tuning parameters
const { kp, ti, td } = zieglerNichols(2.0, 5.0, 1.0);
const amigo = amigoTuning(2.0, 5.0, 1.0);

// Simulate
const result = simulateSetpoint(plant, kp, 1/ti, kp*td, 0.1, 0.01);
// result.t, result.y, result.u, result.sp
```

```javascript
// LQR for the cart-pole
import { lqrForCartPole } from './linear-algebra-lib.js';

const { K, iterations, residual } = lqrForCartPole(
  2,     // M
  0.2,   // m
  0.5,   // l
  9.81,  // g
  0.1,   // b_cart
  [10, 100, 1, 1],  // Q diagonal
  0.1    // R (scalar)
);
// K = [[-10, -95.537, -12.385, -20.128]]
```

## References

- `references/api-reference.md` — Full API documentation
- `references/validation-report.md` — Cross-check results vs Python scipy

## Related Tools

These algorithms power the following interactive tools on [www.hongbinli.ca](https://www.hongbinli.ca):
- [PID Tuner](https://www.hongbinli.ca/tools/pid-tuner)
- [Manual PID Tuner](https://www.hongbinli.ca/tools/manual-pid-tuner)
- [Relay Feedback PID Tuner](https://www.hongbinli.ca/tools/relay-pid-tuner)
- [Process Model Simulator](https://www.hongbinli.ca/tools/process-model)
- [MRAC Demo](https://www.hongbinli.ca/tools/mrac-demo)
- [Combustion Calculator](https://www.hongbinli.ca/tools/combustion-calculator)
- [Loop Load Calculator](https://www.hongbinli.ca/tools/loop-load)

## Contributing

When updating algorithms in the website components, sync changes back to this skill:
1. Edit the relevant `*-lib.js` file
2. Update version in `SKILL.md`
3. Run validation: `node scripts/validate.js` (or call the relevant module's tests)
4. Commit and push to the dedicated repo

## Changelog

### v1.3.0 (2026-07-01)
**Bug fixes**
- `bode-plot-lib.js`: replaced broken `evaluateTFRealPart` / `evaluateTFImagPart` stubs with calls to the local `evaluateTF` (complex Horner-method evaluator).
- `canvas-plot-utils.js`: `drawDashedLine` now requires `yMin` / `yMax` as parameters; the old hardcoded `yMin=0, yMax=1.2` made setpoint lines land in the wrong place for non-default data ranges.
- `combustion-lib.js`: stack O₂ now uses exact closed-form formulas (`o2DryNG`, `o2DryH2`); the previous approximation was algebraically identical for natural gas but materially wrong for hydrogen. LHV corrected from 1010 → 909 BTU/scf for natural gas. Status is now three-tier (red < 5% / green 5-25% / yellow > 25%).
- `control-systems-lib.js`: `PIDController` rewritten to use derivative-on-measurement with back-calculation anti-windup.

**New tuning rules**
- `amigoTuning(K, T, L)` (Åström & Hägglund).
- `imcTuning` now uses the half-dead-time approximation; return shape changed from `{kp, ti, td}` to `{kp, ki, kd}` to match the website.

**New modules extracted from the website**
- `tool-utils.js` — shared DOM utilities.
- `pole-roots-lib.js` — closed-form polynomial root solver + characteristic poly / closed-loop pole helpers.
- `linear-algebra-lib.js` — matrix ops, Lyapunov, LQR (Kleinman), Ackermann pole placement.
- `anim-engine-lib.js` ⚠️ — RK4, history buffer, animation loop, drawing layer. Experimental; broader surface than the rest of the repo.

### v1.2.0 (2026-05)
- Added Process Model Simulator, Combustion Calculator, Loop Load Calculator

### v1.1.0
- Added MRAC, Bode analysis, canvas plotting modules

### v1.0.0 (2026-05-28)
- Initial extraction from hongbinli-website
- Modularized core algorithms: tf2ss, PID, tuning rules, relay feedback
