# Control Systems Web Library

JavaScript library for control systems simulation, extracted from the interactive tools at [hongbinli.com](https://hongbinli.com).

## What's Included

- **Transfer Function ↔ State Space** conversion (validated against Python scipy)
- **PID Controller** with derivative filter and anti-windup
- **Process Models**: FOLPD, SOPDT, Arbitrary TF
- **Simulation Engine**: Forward Euler with transport delay
- **Tuning Rules**: Ziegler-Nichols, Cohen-Coon, CHR, IMC PID, IMC-PI
- **Relay Feedback**: Auto-tuning via limit cycle analysis

## Quick Start

```javascript
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

## Validation

Cross-checked against Python `scipy.signal.tf2ss`:
- DC gain: exact match
- Poles/eigenvalues: exact match  
- Step response: <0.1% error (forward Euler discretization)

See [references/validation-report.md](references/validation-report.md) for full results.

## Related Interactive Tools

These algorithms power:
- [PID Tuner](https://hongbinli.com/tools/pid-tuner)
- [Manual PID Tuner](https://hongbinli.com/tools/manual-pid-tuner)
- [Relay Feedback PID Tuner](https://hongbinli.com/tools/relay-pid-tuner)
- [Process Model Simulator](https://hongbinli.com/tools/process-model)
- [MRAC Demo](https://hongbinli.com/tools/mrac-demo)

## License

MIT
