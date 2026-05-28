# Control Systems Web Library — API Reference

## Table of Contents
- [Transfer Functions](#transfer-functions)
- [PID Controller](#pid-controller)
- [Process Models](#process-models)
- [Simulation](#simulation)
- [Tuning Rules](#tuning-rules)
- [Relay Feedback](#relay-feedback)
- [Utilities](#utilities)

---

## Transfer Functions

### `tfToStateSpace(num, den)`

Convert transfer function to observable canonical state-space form.

**Parameters:**
- `num` (`number[]`): Numerator coefficients, highest power first
- `den` (`number[]`): Denominator coefficients, highest power first

**Returns:** `{A, B, C, D, n}` or `null` if invalid

**Example:**
```javascript
const ss = tfToStateSpace([2], [5, 1]);
// ss = { A: [[-0.2]], B: [1], C: [[0.4]], D: [0], n: 1 }
```

**Mathematical Background:**
Given transfer function G(s) = (b₀sⁿ + ... + bₙ) / (a₀sⁿ + ... + aₙ)

The observable canonical form yields:
```
A = [0  1  0  ...  0]
    [0  0  1  ...  0]
    [...]
    [-aₙ/a₀  -aₙ₋₁/a₀  ...  -a₁/a₀]

B = [0 ... 0  1]ᵀ
C = [bₙ-b₀aₙ/a₀  bₙ₋₁-b₀aₙ₋₁/a₀  ...  b₁-b₀a₁/a₀]
D = b₀/a₀
```

### `evaluateTF(num, den, sRe, sIm)`

Evaluate transfer function at complex frequency s = σ + jω.

**Parameters:**
- `num`, `den`: Coefficient arrays
- `sRe`: Real part of s
- `sIm`: Imaginary part of s

**Returns:** `{re, im}` — Complex result

### `parseCoeffs(str)`

Parse comma-separated coefficient string.

**Example:**
```javascript
parseCoeffs("1, 3, 2"); // [1, 3, 2]  represents s² + 3s + 2
```

---

## PID Controller

### `new PIDController(kp, ki, kd, tau_d, dt, limits)`

**Constructor Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `kp` | `number` | Proportional gain |
| `ki` | `number` | Integral gain |
| `kd` | `number` | Derivative gain |
| `tau_d` | `number` | Derivative filter time constant (s) |
| `dt` | `number` | Sample time (s) |
| `limits` | `[number, number]` | Output clamp [min, max] |

**Methods:**
- `update(setpoint, measurement)` — Compute control output
- `reset()` — Clear internal state

**Anti-windup:** Back-calculates integral when output is clamped.

**Derivative Filtering:** First-order filter on raw derivative:
```
α = τ_d / (τ_d + dt)
D_filtered = α·D_prev + (1-α)·D_raw
```

---

## Process Models

### `createFOLPD(K, T, L)`

First-Order Lag Plus Dead Time: G(s) = K·e^(-Ls)/(Ts + 1)

### `createSOPDT(K, T1, T2, L)`

Second-Order Plus Dead Time: G(s) = K·e^(-Ls)/((T₁s+1)(T₂s+1))

If T₂ ≈ 0, degrades to FOLPD.

### `createArbitraryTF(num, den, L)`

Arbitrary polynomial transfer function with delay.

---

## Simulation

### `simulateSetpoint(plant, kp, ki, kd, tau_d, dt)`

Closed-loop setpoint step response (unit step at t=0).

**Parameters:**
- `plant`: Process model from `createFOLPD`/`createSOPDT`/`createArbitraryTF`
- PID parameters
- `dt`: Integration time step

**Returns:** `{t, y, u, sp}`
- `t`: Time array
- `y`: Process output
- `u`: Control signal
- `sp`: Setpoint value (1.0)

**Simulation Duration:** Automatically calculated as `5·T + 5·L + 5` seconds.

### `simulateDisturbance(plant, kp, ki, kd, tau_d, dt)`

Closed-loop disturbance step response (unit step at process input).

**Returns:** `{t, y, u, sp, d}`

---

## Tuning Rules

All rules return `{kp, ti, td}` in **parallel form**.

| Method | Function | Best For |
|--------|----------|----------|
| Ziegler-Nichols | `zieglerNichols(K, T, L)` | Aggressive response |
| Cohen-Coon | `cohenCoon(K, T, L)` | Controllability ratio tuning |
| CHR (0% overshoot) | `chienHronesReswick(K, T, L)` | No overshoot, disturbance rejection |
| IMC PID | `imcTuning(K, T, L, tau_c)` | Robust performance tradeoff |
| IMC-PI | `imcPI(K, T, L, tau_c)` | Simplified IMC, no derivative |

### Conversion: Series ↔ Parallel

**Series to Parallel:**
```
Kp_parallel = Kp_series · (1 + Td/Ti)
Ti_parallel = Ti_series · (1 + Td/Ti)
Td_parallel = Td_series / (1 + Td/Ti)
```

**Parallel to Series:**
```
τ = 0.5·Ti·(1 ± √(1 - 4·Td/Ti))
Kp_series = Kp_parallel · Ti_parallel / τ
Ti_series = τ
Td_series = Ti_parallel - τ
```

---

## Relay Feedback

### `relayFeedbackSimulation(plant, amplitude, dt, simTime)`

Simulate relay feedback test to identify ultimate parameters.

**Parameters:**
- `amplitude`: Relay output magnitude (±amplitude)
- `dt`: Time step
- `simTime`: Total simulation time

**Returns:** `{Ku, Pu, converged}`
- `Ku`: Ultimate gain = 4d/(πa)
- `Pu`: Ultimate period (average zero-crossing interval)
- `converged`: Boolean

**Usage with Ziegler-Nichols Closed-Loop:**
```javascript
const { Ku, Pu } = relayFeedbackSimulation(plant, 1.0, 0.01, 100);
if (Ku) {
  const kp = 0.6 * Ku;
  const ti = 0.5 * Pu;
  const td = 0.125 * Pu;
}
```

---

## Utilities

### `getProcessTimeConstant(plant)`

Estimate dominant time constant from process model.

### `logspace(start, stop, n)`

Generate `n` logarithmically spaced points from `start` to `stop`.

---

## TypeScript Declarations (Optional)

```typescript
interface StateSpace {
  A: number[][];
  B: number[];
  C: number[][];
  D: number[];
  n: number;
}

interface Plant {
  num: number[];
  den: number[];
  L: number;
  model: 'folpd' | 'sopdt' | 'arbitrary';
}

interface SimulationResult {
  t: Float64Array;
  y: Float64Array;
  u: Float64Array;
  sp: number;
}

interface TuningResult {
  kp: number;
  ti: number;
  td: number;
}
```
