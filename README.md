# Scripts-Control

JavaScript library for control systems simulation and industrial calculators, extracted from the interactive tools at [www.hongbinli.ca](https://www.hongbinli.ca).

## Modules

| File | Description | Source |
|------|-------------|--------|
| `control-systems-lib.js` | Core: TF↔SS, PID, tuning rules, relay feedback | `PIDTuner.astro`, `ManualPIDTuner.astro` |
| `mrac-lib.js` | Model Reference Adaptive Control (Lyapunov + MIT) | `MRAC.astro` |
| `bode-plot-lib.js` | Frequency response, stability margins, Nyquist | `PIDTuner.astro` |
| `canvas-plot-utils.js` | Shared canvas drawing utilities | `PIDTuner.astro`, `MRAC.astro` |
| `process-model-lib.js` | FOLPD/SOPTD/2nd order step response and Bode | `process-model/index.astro` |
| `combustion-lib.js` | Natural gas/hydrogen combustion, AFR, heat input | `combustion-calculator/index.astro` |
| `loop-load-lib.js` | 4-20mA loop load, cable resistance, voltage margin | `loop-load/index.astro` |
| `tool-utils.js` | Shared DOM utilities: `formatNum`, `parseInput`, `debounce`, `getColors`, `observeDarkMode` | `public/tool-utils.js` |
| `pole-roots-lib.js` | Closed-form polynomial root solver (deg 1–4), characteristic poly, closed-loop poles | `src/lib/closed-loop-poles.js` |
| `linear-algebra-lib.js` | Matrix ops, Lyapunov, LQR (Kleinman), Ackermann pole placement | `public/lqr.js`, `src/lib/lqr.js` |
| `anim-engine-lib.js` ⚠️ | RK4 integrator, history buffer, animation loop, MSD/tank drawing, 8 derivative functions | `public/anim-engine.js` |

⚠️ = experimental. Anim-engine has a working library of physics primitives beyond what the rest of the repo uses; the broader surface is provided as-is.

## What's Included

### Control Systems (`control-systems-lib.js`)
- **Transfer Function ↔ State Space** conversion (validated against Python scipy)
- **PID Controller** with derivative-on-measurement, derivative filter, and back-calculation anti-windup
- **Process Models**: FOLPD, SOPDT, Arbitrary TF
- **Simulation Engine**: Forward Euler with transport delay
- **Tuning Rules**: Ziegler-Nichols, Cohen-Coon, CHR, IMC PID (half-dead-time), IMC-PI, AMIGO
- **Relay Feedback**: Auto-tuning via limit cycle analysis

### MRAC (`mrac-lib.js`)
- **Lyapunov-based** gradient adaptation (stable, guaranteed convergence)
- **MIT Rule** adaptation (Åström & Wittenmark)
- First-order SISO plant with reference model following
- Convergence checking and ideal parameter computation

### Bode Analysis (`bode-plot-lib.js`)
- Open-loop frequency response computation
- Gain and phase margin calculation
- Closed-loop stability assessment
- Bandwidth estimation
- Nyquist plot data generation

### Canvas Plotting (`canvas-plot-utils.js`)
- High-DPI canvas setup
- Grid and axis drawing with auto-scaling
- Step response trace plotting
- MRAC sliding-window scope rendering
- Dark/light theme support
- `drawDashedLine(ctx, yValue, width, height, yMin, yMax, color, label)` — setpoint/reference lines (yMin/yMax now required as parameters; production code should inline per the website pattern)

### Process Model Simulator (`process-model-lib.js`)
- **FOLPD**: Closed-form step response and Bode with dead time
- **SOPTD**: Two real poles plus dead time
- **Standard 2nd Order**: ζ, ωn parameterization (underdamped, critically damped, overdamped)
- Bandwidth calculation from -3 dB point

### Combustion Calculator (`combustion-lib.js`)
- Natural gas (CH₄) and hydrogen (H₂) combustion
- Stoichiometric and excess-air airflow
- Stack O₂ forecast (dry basis) — exact closed-form formulas (`o2DryNG`, `o2DryH2`)
- Heat input calculation (MMBTU/h) using LHV (909 BTU/scf for natural gas, 275 BTU/scf for H₂)
- Three-tier status (red < 5% / green 5-25% / yellow > 25%)
- Reverse calculation: excess air from measured O₂ (closed-form inverse of `o2DryNG` / `o2DryH2`)

### Loop Load Calculator (`loop-load-lib.js`)
- 4-20 mA loop voltage drop analysis
- Cable resistance by AWG gauge
- Receiver burden (250Ω / 50Ω / custom)
- Voltage margin at transmitter
- Maximum cable length calculation
- Current-to-process value conversion

### Tool Utilities (`tool-utils.js`)
- `formatNum(n, decimals)` — locale-aware number formatting
- `parseInput(el, opts)` — safe numeric input parsing with optional validation feedback
- `debounce(fn, ms)` — debounce helper
- `getColors()` / `setActive()` / `maxCount()` — UI helpers
- `observeDarkMode(callback)` — auto-disconnecting MutationObserver on `<html class="dark">`

### Pole Roots (`pole-roots-lib.js`)
- `polynomialRoots(coeffs)` — closed-form root finder for degree 1–4
- `solveQuadratic`, `solveCubic` (Cardano with trig fallback), `solveQuartic` (Ferrari)
- `cbrtReal`, `cbrtComplex` — robust real / principal complex cube root
- `characteristicPoly({A, B, N, cS2, cS1, cS0})` — build `1 + L(s)` coefficients for the inverted-pendulum PID loop, with automatic `s = 0` root factoring
- `closedLoopPoles(params)` and `stateFeedbackPoles({M, m, l, g, Kx, Ktheta, Kv, Komega})` — top-level helpers for displaying the closed-loop pole positions
- `durandKerner` — BC alias for `polynomialRoots`

### Linear Algebra (`linear-algebra-lib.js`)
- `matAdd`, `matSub`, `matScale`, `matMul`, `matFrobNorm`, `transpose`, `invertMatrix`, `identity`
- `solveLinearSystem(L, rhs)` — dense Gaussian elimination with partial pivoting
- `solveLyapunov(A, C)` — `A^T X + X A = C` via Smith's reduced symmetric form
- `ackerStabilizing(A, B, desiredPoles)` — single-input Ackermann pole placement
- `polyFromRoots(roots)` — build polynomial coefficients from roots
- `solveLQR(A, B, Q, R, opts)` — continuous-time LQR via Kleinman iteration with automatic Ackermann-based stabilizing `K_init`
- `lqrForCartPole(M, m, l, g, b_cart, Q, R)` — cart-pole-specific wrapper that builds the linearized 4-state A/B matrices and calls `solveLQR`
- Re-exports `polynomialRoots` from `pole-roots-lib.js`

### Anim Engine (`anim-engine-lib.js`, experimental)
- `rk4(state, h, deriv)` — classical 4th-order Runge-Kutta
- `step`, `weighted` — low-level RK4 helpers (exported for custom integrators)
- `createHistoryBuffer(sampleHz, maxSamples)` — throttled sliding-window time-series buffer
- `startAnimLoop(opts)` — `requestAnimationFrame` loop with substep + history throttle; returns `{ stop() }`
- `clamp(x, lo, hi)` — numeric clamp
- `drawMSD(ctx, state, params, opts)` — mass-spring-damper 2D canvas schematic
- `drawTankSVG(state, params, opts)` — tank SVG markup (returns string for caller to inject)
- `deriv` table with 8 system derivative functions: `msd`, `tank`, `dcMotor`, `rlcSeries`, `simplePendulum`, `invertedPendulum`, `pvtol`, `bioreactor`
- `paramPresets` and `initialStates` — default starting points for each system

## Quick Start

```javascript
// Core control systems
import {
  tfToStateSpace, PIDController, createFOLPD,
  simulateSetpoint, zieglerNichols
} from './control-systems-lib.js';

// Create a process model
const plant = createFOLPD(2.0, 5.0, 1.0); // K=2, T=5s, L=1s

// Get tuning parameters
const { kp, ti, td } = zieglerNichols(2.0, 5.0, 1.0);

// Simulate closed-loop response
const result = simulateSetpoint(plant, kp, 1/ti, kp*td, 0.1, 0.01);
// result.t, result.y, result.u, result.sp
```

```javascript
// MRAC adaptive control
import { createMRAC, simulateMRAC, idealParameters } from './mrac-lib.js';

const mrac = createMRAC('lyapunov');
mrac.a = 2.0;    // Plant pole
mrac.b = 1.0;    // Plant gain
mrac.am = 1.0;   // Reference model pole
mrac.bm = 1.0;   // Reference model gain

const result = simulateMRAC(mrac, 30, 0.005);
console.log('Final error:', result.final.error);
console.log('Ideal params:', idealParameters(2.0, 1.0, 1.0, 1.0));
```

```javascript
// Bode analysis
import { logspace, computeOpenLoopBode, findMargins } from './bode-plot-lib.js';

const omega = logspace(1e-3, 1e2, 500);
const { mag, phase } = computeOpenLoopBode(2, 5, 1, 2.5, 0.5, 1.25, 0.1, omega);
const margins = findMargins(omega, mag, phase);
console.log('Phase margin:', margins.pm, 'degrees');
console.log('Gain margin:', margins.gm_dB, 'dB');
```

```javascript
// Process model simulation
import { simulateProcessModel } from './process-model-lib.js';

const result = simulateProcessModel('so2', { K: 1, zeta: 0.5, wn: 2 });
console.log('Rise time metrics:', result.metrics);
// result.t, result.y, result.omega, result.mag, result.phase
```

```javascript
// Combustion calculation
import { calculateCombustion } from './combustion-lib.js';

const result = calculateCombustion('natural_gas', 1000, 'scfh', 15);
console.log('Heat input:', result.heatInput.mmbtuPerHour, 'MMBTU/h');
console.log('Stack O₂:', result.stackO2.percent, '%');
```

```javascript
// Loop load analysis
import { calculateLoopLoad } from './loop-load-lib.js';

const result = calculateLoopLoad({
  supplyVoltage: 24,
  txMinVoltage: 10.5,
  burden: 250,
  awg: 18,
  cableLength: 500,
  lengthUnit: 'ft',
  extraResistance: 0
});
console.log('Voltage margin:', result.loop.voltageMargin, 'V');
console.log('Status:', result.status.message);
```

## Validation

Cross-checked against Python `scipy.signal.tf2ss`:
- DC gain: exact match
- Poles/eigenvalues: exact match  
- Step response: <0.1% error (forward Euler discretization)

## Related Interactive Tools

These algorithms power:
- [PID Tuner](https://www.hongbinli.ca/tools/pid-tuner)
- [Manual PID Tuner](https://www.hongbinli.ca/tools/manual-pid-tuner)
- [Relay Feedback PID Tuner](https://www.hongbinli.ca/tools/relay-pid-tuner)
- [Process Model Simulator](https://www.hongbinli.ca/tools/process-model)
- [MRAC Demo](https://www.hongbinli.ca/tools/mrac-demo)
- [Combustion Calculator](https://www.hongbinli.ca/tools/combustion-calculator)
- [Loop Load Calculator](https://www.hongbinli.ca/tools/loop-load)

## Changelog

### v1.3.0 (2026-07-01)
- **Bug fixes**
  - `bode-plot-lib.js`: replaced broken `evaluateTFRealPart` / `evaluateTFImagPart` stubs with calls to the local `evaluateTF` (complex Horner-method evaluator). `computeTFOpenLoopBode` was returning garbage for arbitrary TFs.
  - `canvas-plot-utils.js`: `drawDashedLine` now requires `yMin` / `yMax` as parameters; the old hardcoded `yMin=0, yMax=1.2` made setpoint lines land in the wrong place for any data range that wasn't the default.
  - `combustion-lib.js`: stack O₂ now uses exact closed-form formulas (`o2DryNG`, `o2DryH2`). The previous approximation was algebraically identical for natural gas but materially wrong for hydrogen (~20% error at 20% excess). LHV corrected from 1010 → 909 BTU/scf for natural gas (1010 was closer to HHV). Status is now three-tier (red < 5% / green 5-25% / yellow > 25%) matching the website.
  - `control-systems-lib.js`: `PIDController` rewritten to use derivative-on-measurement with back-calculation anti-windup. The previous derivative-on-error form produced setpoint "kicks"; back-calculation gives smoother saturation recovery.
- **New tuning rules**: `amigoTuning(K, T, L)` (Åström & Hägglund, Advanced PID Control, Eq. 7.7, p. 233).
- **More accurate IMC**: `imcTuning` now uses the half-dead-time approximation `Kp = (T + L/2) / (K * (τc + L/2))` instead of the simplified `T / (K * (L + τc))`. Return shape changed from `{kp, ti, td}` to `{kp, ki, kd}` to match the website.
- **New modules extracted from the website**:
  - `tool-utils.js` — shared DOM utilities (formatNum, parseInput, debounce, getColors, observeDarkMode, setActive, maxCount).
  - `pole-roots-lib.js` — closed-form polynomial root solver for degree 1–4 plus characteristic-polynomial and closed-loop-pole helpers.
  - `linear-algebra-lib.js` — matrix operations, Lyapunov, continuous-time LQR (Kleinman iteration), Ackermann pole placement, plus the `lqrForCartPole` wrapper.
  - `anim-engine-lib.js` ⚠️ — RK4 integrator, history buffer, animation loop, and a 2D canvas / SVG drawing layer (mark as experimental; broader physics-primitives surface than the rest of the repo).

### v1.2.0 (2026-05)
- Added Process Model Simulator, Combustion Calculator, Loop Load Calculator

### v1.1.0
- Added MRAC, Bode analysis, canvas plotting modules

## License

MIT
