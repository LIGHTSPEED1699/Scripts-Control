# Scripts-Control

JavaScript library for control systems simulation and industrial calculators, extracted from the interactive tools at [hongbinli.com](https://hongbinli.com).

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

## What's Included

### Control Systems (`control-systems-lib.js`)
- **Transfer Function ↔ State Space** conversion (validated against Python scipy)
- **PID Controller** with derivative filter and anti-windup
- **Process Models**: FOLPD, SOPDT, Arbitrary TF
- **Simulation Engine**: Forward Euler with transport delay
- **Tuning Rules**: Ziegler-Nichols, Cohen-Coon, CHR, IMC PID, IMC-PI
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

### Process Model Simulator (`process-model-lib.js`)
- **FOLPD**: Closed-form step response and Bode with dead time
- **SOPTD**: Two real poles plus dead time
- **Standard 2nd Order**: ζ, ωn parameterization (underdamped, critically damped, overdamped)
- Bandwidth calculation from -3 dB point

### Combustion Calculator (`combustion-lib.js`)
- Natural gas (CH₄) and hydrogen (H₂) combustion
- Stoichiometric and excess-air airflow
- Stack O₂ forecast (dry basis)
- Heat input calculation (MMBTU/h)
- Reverse calculation: excess air from measured O₂

### Loop Load Calculator (`loop-load-lib.js`)
- 4-20 mA loop voltage drop analysis
- Cable resistance by AWG gauge
- Receiver burden (250Ω / 50Ω / custom)
- Voltage margin at transmitter
- Maximum cable length calculation
- Current-to-process value conversion

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
- [PID Tuner](https://hongbinli.com/tools/pid-tuner)
- [Manual PID Tuner](https://hongbinli.com/tools/manual-pid-tuner)
- [Relay Feedback PID Tuner](https://hongbinli.com/tools/relay-pid-tuner)
- [Process Model Simulator](https://hongbinli.com/tools/process-model)
- [MRAC Demo](https://hongbinli.com/tools/mrac-demo)
- [Combustion Calculator](https://hongbinli.com/tools/combustion-calculator)
- [Loop Load Calculator](https://hongbinli.com/tools/loop-load)

## License

MIT
