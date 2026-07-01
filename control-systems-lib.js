/**
 * Control Systems Web Library
 * Extracted from hongbinli-website components
 * Core algorithms for PID tuning, process simulation, and adaptive control
 */

// ============================================================================
// TF2SS - Transfer Function to State Space Conversion
// ============================================================================

/**
 * Convert transfer function coefficients to controllable canonical form
 * @param {number[]} num - Numerator coefficients (highest power first)
 * @param {number[]} den - Denominator coefficients (highest power first)
 * @returns {Object|null} State-space matrices {A, B, C, D, n} or null if invalid
 */
function tfToStateSpace(num, den) {
  if (den.length === 0) den = [1];
  const an = den[0];
  const n = den.length - 1;
  if (an === 0) {
    console.error('Leading denominator coefficient is zero');
    return null;
  }

  // Normalize
  const a = den.map(v => v / an);
  const b = num.length <= den.length
    ? [...num]
    : num.slice(num.length - den.length);
  while (b.length < den.length) b.unshift(0);
  const bn = b.map(v => v / an);

  // Build companion matrix A (observable canonical form)
  const A = [];
  for (let i = 0; i < n; i++) {
    const row = new Array(n).fill(0);
    if (i < n - 1) {
      row[i + 1] = 1;
    } else {
      for (let j = 0; j < n; j++) {
        row[j] = -a[n - j];
      }
    }
    A.push(row);
  }

  const B = new Array(n).fill(0);
  B[n - 1] = 1;

  const C = new Array(n).fill(0);
  for (let j = 0; j < n; j++) {
    C[j] = bn[n - j] - bn[0] * a[n - j];
  }

  const D = [bn[0]];

  return { A, B, C: [C], D, n };
}

/**
 * Evaluate transfer function at complex frequency s = σ + jω
 * @param {number[]} num - Numerator coefficients
 * @param {number[]} den - Denominator coefficients
 * @param {number} sRe - Real part of s
 * @param {number} sIm - Imaginary part of s
 * @returns {Object} Complex result {re, im}
 */
function evaluateTF(num, den, sRe, sIm) {
  let numRe = 0, numIm = 0, denRe = 0, denIm = 0;

  for (let i = 0; i < num.length; i++) {
    const power = num.length - 1 - i;
    let pRe = 1, pIm = 0;
    for (let k = 0; k < power; k++) {
      const tRe = pRe * sRe - pIm * sIm;
      const tIm = pRe * sIm + pIm * sRe;
      pRe = tRe; pIm = tIm;
    }
    numRe += num[i] * pRe;
    numIm += num[i] * pIm;
  }

  for (let i = 0; i < den.length; i++) {
    const power = den.length - 1 - i;
    let pRe = 1, pIm = 0;
    for (let k = 0; k < power; k++) {
      const tRe = pRe * sRe - pIm * sIm;
      const tIm = pRe * sIm + pIm * sRe;
      pRe = tRe; pIm = tIm;
    }
    denRe += den[i] * pRe;
    denIm += den[i] * pIm;
  }

  const magSq = denRe * denRe + denIm * denIm;
  if (magSq < 1e-30) return { re: 0, im: 0 };

  return {
    re: (numRe * denRe + numIm * denIm) / magSq,
    im: (numIm * denRe - numRe * denIm) / magSq
  };
}

/**
 * Parse comma-separated coefficient string into array
 * @param {string} str - Comma-separated values
 * @returns {number[]} Parsed coefficients
 */
function parseCoeffs(str) {
  return str.split(',').map(s => parseFloat(s.trim())).filter(v => !isNaN(v));
}

// ============================================================================
// PID Controller
// ============================================================================

/**
 * PID Controller with derivative filter and output clamping
 *
 * v1.3.0 update: uses **derivative-on-measurement** (industrial best practice)
 * with **back-calculation anti-windup**, matching the website's ManualPIDTuner.astro.
 *
 * Why derivative-on-measurement:
 *   - Setpoint changes don't produce a "derivative kick" because the derivative
 *     term depends only on the measurement (which is smooth) rather than the
 *     setpoint (which can step abruptly).
 *
 * Why back-calculation anti-windup:
 *   - When the output saturates, the integral is corrected by the saturation
 *     error divided by ki, then the output is re-computed. This is preferred
 *     over the simpler "clamp-and-clamp-integral" approach because it gives
 *     smooth windup recovery when the actuator comes back into range.
 *
 * @param {number} kp - Proportional gain
 * @param {number} ki - Integral gain
 * @param {number} kd - Derivative gain
 * @param {number} tau_d - Derivative filter time constant (s)
 * @param {number} dt - Sample time (s)
 * @param {number[]} limits - Output clamp limits [min, max]
 */
class PIDController {
  constructor(kp, ki, kd, tau_d, dt, limits) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.tau_d = tau_d;
    this.dt = dt;
    this.limits = limits || [-Infinity, Infinity];

    this.integral = 0;
    this.prevMeas = null;
    this.prevDeriv = 0;
  }

  reset() {
    this.integral = 0;
    this.prevMeas = null;
    this.prevDeriv = 0;
  }

  update(setpoint, measurement) {
    const error = setpoint - measurement;
    const pTerm = this.kp * error;

    // Integral (with back-calculation anti-windup applied later if saturated)
    this.integral += error * this.dt;
    let iTerm = this.ki * this.integral;

    // Derivative on MEASUREMENT (not error) — avoids setpoint kicks.
    //   dRaw = -(measurement - prevMeas) / dt   (sign chosen so that a rising
    //                                            measurement increases the
    //                                            control action, matching the
    //                                            "error derivative" convention
    //                                            under steady setpoint)
    //   alpha = dt / (tau_d + dt)  (one-pole low-pass on dRaw)
    let dTerm = 0;
    if (this.prevMeas !== null) {
      const dRaw = -(measurement - this.prevMeas) / this.dt;
      const alpha = this.dt / (this.tau_d + this.dt);
      const dFiltered = alpha * dRaw + (1 - alpha) * this.prevDeriv;
      this.prevDeriv = dFiltered;
      dTerm = this.kd * dFiltered;
    }
    this.prevMeas = measurement;

    // Output, with saturation
    const outputUnsat = pTerm + iTerm + dTerm;
    let output = Math.max(this.limits[0], Math.min(this.limits[1], outputUnsat));

    // Back-calculation anti-windup: if saturated, correct the integral
    // by satErr/ki and re-compute the output. This is the conditional
    // form used by the website: skip the back-correction when ki === 0
    // to avoid division by zero (pure P / PD controller).
    if (this.ki !== 0) {
      const satErr = output - outputUnsat;
      this.integral += satErr / this.ki;
      iTerm = this.ki * this.integral;
      output = Math.max(this.limits[0], Math.min(this.limits[1], pTerm + iTerm + dTerm));
    }

    return output;
  }
}

// ============================================================================
// Process Models
// ============================================================================

/**
 * Create FOLPD (First-Order Lag Plus Dead Time) model
 * @param {number} K - Process gain
 * @param {number} T - Time constant (s)
 * @param {number} L - Dead time (s)
 * @returns {Object} Model {num, den, L, model: 'folpd'}
 */
function createFOLPD(K, T, L) {
  return { num: [K], den: [T, 1], L, model: 'folpd' };
}

/**
 * Create SOPDT (Second-Order Plus Dead Time) model
 * @param {number} K - Process gain
 * @param {number} T1 - First time constant (s)
 * @param {number} T2 - Second time constant (s)
 * @param {number} L - Dead time (s)
 * @returns {Object} Model {num, den, L, model: 'sopdt'}
 */
function createSOPDT(K, T1, T2, L) {
  if (Math.abs(T2) < 0.01) {
    return { num: [K], den: [T1, 1], L, model: 'sopdt' };
  }
  return {
    num: [K],
    den: [T1 * T2, T1 + T2, 1],
    L,
    model: 'sopdt'
  };
}

/**
 * Create arbitrary transfer function model
 * @param {number[]} num - Numerator coefficients
 * @param {number[]} den - Denominator coefficients
 * @param {number} L - Dead time (s)
 * @returns {Object} Model {num, den, L, model: 'arbitrary'}
 */
function createArbitraryTF(num, den, L) {
  return { num, den, L, model: 'arbitrary' };
}

// ============================================================================
// Simulation Engine
// ============================================================================

/**
 * Simulate closed-loop setpoint step response
 * @param {Object} plant - Plant model {num, den, L}
 * @param {number} kp - Proportional gain
 * @param {number} ki - Integral gain
 * @param {number} kd - Derivative gain
 * @param {number} tau_d - Derivative filter (s)
 * @param {number} dt - Time step (s)
 * @returns {Object} Simulation results {t, y, u, sp}
 */
function simulateSetpoint(plant, kp, ki, kd, tau_d, dt) {
  const { num, den, L } = plant;
  const Tc = getProcessTimeConstant(plant);
  const tFinal = 5 * Tc + 5 * L + 5;
  const n = Math.floor(tFinal / dt) + 1;
  const t = new Float64Array(n);
  const y = new Float64Array(n);
  const u = new Float64Array(n);
  const sp = 1.0;

  const pid = new PIDController(kp, ki, kd, tau_d, dt, [-100, 100]);
  const ss = tfToStateSpace(num, den);
  if (!ss) return { t, y, u, sp };
  const { A, B, C, D, n: nState } = ss;
  let x = new Float64Array(nState).fill(0);

  const delaySamples = Math.max(1, Math.round(L / dt));
  const delayBuf = new Float64Array(delaySamples);
  let delayIdx = 0;

  for (let i = 0; i < n; i++) {
    t[i] = i * dt;
    const meas = (i > 0) ? y[i - 1] : 0;
    const control = pid.update(sp, meas);
    u[i] = control;
    const delayedU = delayBuf[delayIdx];
    delayBuf[delayIdx] = control;
    delayIdx = (delayIdx + 1) % delaySamples;

    const dx = new Float64Array(nState);
    for (let j = 0; j < nState; j++) {
      dx[j] = A[j].reduce((sum, aij, k) => sum + aij * x[k], 0) + B[j] * delayedU;
    }
    for (let j = 0; j < nState; j++) x[j] += dx[j] * dt;
    y[i] = C[0].reduce((sum, cj, j) => sum + cj * x[j], 0) + D[0] * delayedU;
  }

  return { t, y, u, sp };
}

/**
 * Simulate closed-loop disturbance step response
 * @param {Object} plant - Plant model {num, den, L}
 * @param {number} kp - Proportional gain
 * @param {number} ki - Integral gain
 * @param {number} kd - Derivative gain
 * @param {number} tau_d - Derivative filter (s)
 * @param {number} dt - Time step (s)
 * @returns {Object} Simulation results {t, y, u, sp, d}
 */
function simulateDisturbance(plant, kp, ki, kd, tau_d, dt) {
  const { num, den, L } = plant;
  const Tc = getProcessTimeConstant(plant);
  const tFinal = 5 * Tc + 5 * L + 5;
  const n = Math.floor(tFinal / dt) + 1;
  const t = new Float64Array(n);
  const y = new Float64Array(n);
  const u = new Float64Array(n);
  const sp = 0.0;
  const d = 1.0;

  const pid = new PIDController(kp, ki, kd, tau_d, dt, [-100, 100]);
  const ss = tfToStateSpace(num, den);
  if (!ss) return { t, y, u, sp, d };
  const { A, B, C, D, n: nState } = ss;
  let x = new Float64Array(nState).fill(0);

  const delaySamples = Math.max(1, Math.round(L / dt));
  const delayBuf = new Float64Array(delaySamples);
  let delayIdx = 0;

  for (let i = 0; i < n; i++) {
    t[i] = i * dt;
    const meas = (i > 0) ? y[i - 1] : 0;
    const control = pid.update(sp, meas);
    u[i] = control;
    const effectiveU = control + d;
    const delayedU = delayBuf[delayIdx];
    delayBuf[delayIdx] = effectiveU;
    delayIdx = (delayIdx + 1) % delaySamples;

    const dx = new Float64Array(nState);
    for (let j = 0; j < nState; j++) {
      dx[j] = A[j].reduce((sum, aij, k) => sum + aij * x[k], 0) + B[j] * delayedU;
    }
    for (let j = 0; j < nState; j++) x[j] += dx[j] * dt;
    y[i] = C[0].reduce((sum, cj, j) => sum + cj * x[j], 0) + D[0] * delayedU;
  }

  return { t, y, u, sp, d };
}

// ============================================================================
// Utility Functions
// ============================================================================

function getProcessTimeConstant(plant) {
  const { num, den, model } = plant;
  if (model === 'folpd') {
    // den = [T, 1] -> T = den[0]
    return den[0];
  }
  if (model === 'sopdt') {
    // den = [T1*T2, T1+T2, 1]
    if (den.length === 2) return den[0]; // Degraded to FOLPD
    return Math.max(den[1] / den[0], 0); // Approximate
  }
  // For arbitrary TF, use dominant time constant
  if (den.length >= 2) return den[1] / den[0];
  return 1;
}

/**
 * Generate logarithmically spaced array
 * @param {number} start - Start value
 * @param {number} stop - Stop value
 * @param {number} n - Number of points
 * @returns {number[]} Log-spaced array
 */
function logspace(start, stop, n) {
  const arr = new Array(n);
  const logStart = Math.log10(start);
  const logStop = Math.log10(stop);
  for (let i = 0; i < n; i++) {
    arr[i] = Math.pow(10, logStart + (logStop - logStart) * i / (n - 1));
  }
  return arr;
}

// ============================================================================
// Tuning Rules
// ============================================================================

/**
 * Ziegler-Nichols open-loop tuning (reaction curve method)
 * @param {number} K - Process gain
 * @param {number} T - Time constant
 * @param {number} L - Dead time
 * @returns {Object} PID parameters {kp, ti, td}
 */
function zieglerNichols(K, T, L) {
  const kp = 1.2 * T / (K * L);
  const ti = 2 * L;
  const td = L / 2;
  return { kp, ti, td };
}

/**
 * Cohen-Coon tuning
 * @param {number} K - Process gain
 * @param {number} T - Time constant
 * @param {number} L - Dead time
 * @returns {Object} PID parameters {kp, ti, td}
 */
function cohenCoon(K, T, L) {
  const r = L / T;
  const kp = (1 / K) * (1.33 / r + 0.25);
  const ti = L * (32 + 6 * r) / (13 + 8 * r);
  const td = 4 * L / (11 + 2 * r);
  return { kp, ti, td };
}

/**
 * Chien-Hrones-Reswick tuning (0% overshoot, load disturbance)
 * @param {number} K - Process gain
 * @param {number} T - Time constant
 * @param {number} L - Dead time
 * @returns {Object} PID parameters {kp, ti, td}
 */
function chienHronesReswick(K, T, L) {
  const kp = 0.6 * T / (K * L);
  const ti = T;
  const td = 0.5 * L;
  return { kp, ti, td };
}

/**
 * Internal Model Control (IMC) PID tuning
 *
 * v1.3.0 update: use the more accurate half-dead-time (L/2) approximation
 * from PIDTuner.astro:1184-1189. For a first-order-plus-dead-time plant
 * with non-negligible dead time, the simple `T / (K * (L + tau_c))`
 * formula over-estimates Kp by ignoring that some of the dead time is
 * already absorbed by the plant pole.
 *
 * Numerical comparison at K=1, T=5, L=1, tau_c=1:
 *   v1.2.0 (simple):   kp = 5 / (1 * (1 + 1)) = 2.5
 *   v1.3.0 (half-L):   kp = (5 + 0.5) / (1 * (1 + 0.5)) = 2.667  (+6.7%)
 *
 * @param {number} K - Process gain
 * @param {number} T - Time constant
 * @param {number} L - Dead time
 * @param {number} tau_c - Closed-loop time constant (filter tuning knob)
 * @returns {Object} PID parameters {kp, ki, kd}
 */
function imcTuning(K, T, L, tau_c) {
  const kp = (T + L / 2) / (K * (tau_c + L / 2));
  const ki = kp / (T + L / 2);
  const kd = kp * T * L / (2 * (T + L / 2));
  return { kp, ki, kd };
}

/**
 * AMIGO tuning rule (Åström & Hägglund, Advanced PID Control, Eq. 7.7, p. 233).
 *
 * New in v1.3.0. AMIGO is the "Almost Minimum Input-Output" rule, derived
 * for robust performance on first-order-plus-dead-time plants. It typically
 * gives less aggressive gains than Ziegler-Nichols while still being simple
 * to compute from open-loop step-test data.
 *
 * @param {number} K - Process gain
 * @param {number} T - Time constant
 * @param {number} L - Dead time
 * @returns {Object} PID parameters {kp, ti, td}
 */
function amigoTuning(K, T, L) {
  if (K === 0) return { kp: 0, ti: 0, td: 0 };
  const kp = (1 / K) * (0.2 + 0.45 * T / L);
  const ti = ((0.4 * L + 0.8 * T) / (L + 0.1 * T)) * L;
  const td = (0.5 * L * T) / (0.3 * L + T);
  return { kp, ti, td };
}

/**
 * IMC-PI tuning (Skogestad simplified)
 * @param {number} K - Process gain
 * @param {number} T - Time constant
 * @param {number} L - Dead time
 * @param {number} tau_c - Closed-loop time constant
 * @returns {Object} PI parameters {kp, ti, td: 0}
 */
function imcPI(K, T, L, tau_c) {
  const kp = T / (K * tau_c);
  const ti = Math.min(T, 4 * (tau_c + L));
  return { kp, ti, td: 0 };
}

// ============================================================================
// Relay Feedback Methods
// ============================================================================

/**
 * Simulate relay feedback to identify ultimate gain and period
 * @param {Object} plant - Plant model {num, den, L}
 * @param {number} amplitude - Relay amplitude
 * @param {number} dt - Time step
 * @param {number} simTime - Simulation duration
 * @returns {Object} {Ku, Pu, converged}
 */
function relayFeedbackSimulation(plant, amplitude, dt, simTime) {
  const { num, den, L } = plant;
  const ss = tfToStateSpace(num, den);
  if (!ss) return { Ku: null, Pu: null, converged: false };
  const { A, B, C, D, n: nState } = ss;

  const n = Math.floor(simTime / dt) + 1;
  const t = new Float64Array(n);
  const y = new Float64Array(n);
  const u = new Float64Array(n);

  let x = new Float64Array(nState).fill(0);
  const delaySamples = Math.max(1, Math.round(L / dt));
  const delayBuf = new Float64Array(delaySamples);
  let delayIdx = 0;

  // Detect oscillations
  const crossings = [];
  let prevY = 0;

  for (let i = 0; i < n; i++) {
    t[i] = i * dt;

    // Relay output
    const relayOut = y[i - 1] >= 0 ? -amplitude : amplitude;
    u[i] = relayOut;

    // Delay
    const delayedU = delayBuf[delayIdx];
    delayBuf[delayIdx] = relayOut;
    delayIdx = (delayIdx + 1) % delaySamples;

    // State update
    const dx = new Float64Array(nState);
    for (let j = 0; j < nState; j++) {
      dx[j] = A[j].reduce((sum, aij, k) => sum + aij * x[k], 0) + B[j] * delayedU;
    }
    for (let j = 0; j < nState; j++) x[j] += dx[j] * dt;
    y[i] = C[0].reduce((sum, cj, j) => sum + cj * x[j], 0) + D[0] * delayedU;

    // Zero crossing detection
    if (i > 0 && prevY * y[i] < 0) {
      crossings.push(t[i]);
    }
    prevY = y[i];
  }

  if (crossings.length < 2) {
    return { Ku: null, Pu: null, converged: false };
  }

  // Calculate period from crossings
  const periods = [];
  for (let i = 1; i < crossings.length; i++) {
    periods.push(crossings[i] - crossings[i - 1]);
  }
  const Pu = periods.reduce((a, b) => a + b, 0) / periods.length;

  // Ultimate gain: Ku = 4 * d / (π * a)
  // Approximate amplitude from peak-to-peak
  const yMax = Math.max(...y);
  const yMin = Math.min(...y);
  const a = (yMax - yMin) / 2;
  const Ku = (4 * amplitude) / (Math.PI * a);

  return { Ku, Pu, converged: true };
}

// ============================================================================
// Module Exports
// ============================================================================

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    tfToStateSpace,
    evaluateTF,
    parseCoeffs,
    PIDController,
    createFOLPD,
    createSOPDT,
    createArbitraryTF,
    simulateSetpoint,
    simulateDisturbance,
    getProcessTimeConstant,
    logspace,
    zieglerNichols,
    cohenCoon,
    chienHronesReswick,
    imcTuning,
    imcPI,
    amigoTuning,
    relayFeedbackSimulation
  };
}
