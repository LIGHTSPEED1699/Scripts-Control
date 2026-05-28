---
name: Scripts-Control
description: "Control systems simulation library for web-based PID tuning, process modeling, and adaptive control."
metadata:
  author: "Spot (hongbinli-website)"
  version: "1.0.0"
  homepage: "https://github.com/LIGHTSPEED1699/Scripts-Control"
  license: "MIT"
  keywords: ["pid", "control-systems", "simulation", "tuning", "tf2ss", "state-space"]
  source_repo: "https://github.com/LIGHTSPEED1699/hongbinli-website"
allowed-tools: ["read", "write", "edit", "exec"]
user-invocable: false
---

# Control Systems Web Library

Extracted and modularized simulation algorithms from the [hongbinli-website](https://github.com/LIGHTSPEED1699/hongbinli-website) interactive control systems tools.

## What's Included

| Module | Source | Description |
|--------|--------|-------------|
| `control-systems-lib.js` | `ManualPIDTuner.astro`, `PIDTuner.astro`, `RelayFeedbackPID.astro` | Core algorithms: TF↔SS, PID controller, process models, tuning rules, relay feedback |

## Core Capabilities

### Transfer Function ↔ State Space
- `tfToStateSpace(num, den)` — Observable canonical form conversion
- `evaluateTF(num, den, sRe, sIm)` — Complex frequency evaluation
- Validated against Python `scipy.signal.tf2ss`

### PID Controller
- `PIDController(kp, ki, kd, tau_d, dt, limits)` — Full PID with derivative filter and anti-windup
- Form-agnostic: works with parallel or series parameters

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
- `imcTuning(K, T, L, tau_c)` — Internal Model Control PID
- `imcPI(K, T, L, tau_c)` — Skogestad simplified IMC-PI

### Relay Feedback
- `relayFeedbackSimulation(plant, amplitude, dt, simTime)` — Auto-tuning via limit cycle
- Identifies ultimate gain (Ku) and period (Pu)

## Validation

Cross-checked against Python `scipy.signal.tf2ss` and `lsim`:
- DC gain: exact match
- Poles/eigenvalues: exact match
- Step response: <0.1% error (forward Euler discretization)

## Usage

```javascript
import {
  tfToStateSpace, PIDController, createFOLPD,
  simulateSetpoint, zieglerNichols
} from './scripts/control-systems-lib.js';

// Create a process model
const plant = createFOLPD(2.0, 5.0, 1.0);

// Get tuning parameters
const { kp, ti, td } = zieglerNichols(2.0, 5.0, 1.0);

// Simulate
const result = simulateSetpoint(plant, kp, 1/ti, kp*td, 0.1, 0.01);
// result.t, result.y, result.u, result.sp
```

## References

- `references/api-reference.md` — Full API documentation
- `references/validation-report.md` — Cross-check results vs Python scipy

## Related Tools

These algorithms power the following interactive tools on [hongbinli.com](https://hongbinli.com):
- [PID Tuner](https://hongbinli.com/tools/pid-tuner)
- [Manual PID Tuner](https://hongbinli.com/tools/manual-pid-tuner)
- [Relay Feedback PID Tuner](https://hongbinli.com/tools/relay-pid-tuner)
- [Process Model Simulator](https://hongbinli.com/tools/process-model)
- [MRAC Demo](https://hongbinli.com/tools/mrac-demo)

## Contributing

When updating algorithms in the website components, sync changes back to this skill:
1. Edit `scripts/control-systems-lib.js`
2. Update version in `SKILL.md`
3. Run validation: `node scripts/validate.js`
4. Commit and push to the dedicated repo

## Changelog

### v1.0.0 (2026-05-28)
- Initial extraction from hongbinli-website
- Modularized core algorithms: tf2ss, PID, tuning rules, relay feedback
- Added validation suite against Python scipy
