/**
 * Process Model Simulator Library
 * Extracted from process-model/index.astro
 *
 * Closed-form step response and Bode analysis for:
 * - FOLPD (First-Order Lag Plus Dead Time)
 * - SOPTD (Second-Order with Two real poles Plus Dead Time)
 * - Standard 2nd Order (ζ, ωn)
 */

// ============================================================================
// Math Helpers
// ============================================================================

/**
 * Linearly spaced array
 * @param {number} start - Start value
 * @param {number} stop - Stop value
 * @param {number} n - Number of points
 * @returns {number[]}
 */
function linspace(start, stop, n) {
  const arr = new Array(n);
  const step = (stop - start) / (n - 1);
  for (let i = 0; i < n; i++) arr[i] = start + step * i;
  return arr;
}

/**
 * Logarithmically spaced array
 * @param {number} start - Start value
 * @param {number} stop - Stop value
 * @param {number} n - Number of points
 * @returns {number[]}
 */
function logspace(start, stop, n) {
  const arr = new Array(n);
  const startLog = Math.log10(start);
  const stopLog = Math.log10(stop);
  const step = (stopLog - startLog) / (n - 1);
  for (let i = 0; i < n; i++) arr[i] = Math.pow(10, startLog + step * i);
  return arr;
}

// ============================================================================
// Step Response Functions
// ============================================================================

/**
 * FOLPD step response: G(s) = K/(Ts+1)·e^(-Ls)
 * @param {number} t - Time (s)
 * @param {number} K - Gain
 * @param {number} T - Time constant (s)
 * @param {number} L - Dead time (s)
 * @returns {number} Output y(t)
 */
function folpdStep(t, K, T, L) {
  if (t < L) return 0;
  return K * (1 - Math.exp(-(t - L) / T));
}

/**
 * SOPTD step response: G(s) = K/((T1s+1)(T2s+1))·e^(-Ls)
 * Two real poles only
 * @param {number} t - Time (s)
 * @param {number} K - Gain
 * @param {number} T1 - Time constant 1 (s)
 * @param {number} T2 - Time constant 2 (s)
 * @param {number} L - Dead time (s)
 * @returns {number} Output y(t)
 */
function soptdStep(t, K, T1, T2, L) {
  if (t < L) return 0;
  const td = t - L;
  // Equal poles approximation
  if (Math.abs(T1 - T2) < 0.001) {
    const T = (T1 + T2) / 2;
    return K * (1 - (1 + td / T) * Math.exp(-td / T));
  } else {
    return K * (1 - (T1 * Math.exp(-td / T1) - T2 * Math.exp(-td / T2)) / (T1 - T2));
  }
}

/**
 * Standard 2nd order step response: G(s) = K·ωn²/(s² + 2ζωns + ωn²)
 * @param {number} t - Time (s)
 * @param {number} K - Gain
 * @param {number} zeta - Damping ratio
 * @param {number} wn - Natural frequency (rad/s)
 * @returns {number} Output y(t)
 */
function so2Step(t, K, zeta, wn) {
  if (zeta > 1.001) {
    // Overdamped: two distinct real poles
    const p1 = -wn * (zeta - Math.sqrt(zeta * zeta - 1));
    const p2 = -wn * (zeta + Math.sqrt(zeta * zeta - 1));
    return K * (1 - (p2 * Math.exp(p1 * t) - p1 * Math.exp(p2 * t)) / (p2 - p1));
  } else if (Math.abs(zeta - 1) < 0.001) {
    // Critically damped
    return K * (1 - (1 + wn * t) * Math.exp(-wn * t));
  } else if (zeta > 0) {
    // Underdamped
    const wd = wn * Math.sqrt(1 - zeta * zeta);
    return K * (1 - Math.exp(-zeta * wn * t) * (Math.cos(wd * t) + (zeta / Math.sqrt(1 - zeta * zeta)) * Math.sin(wd * t)));
  } else {
    // Undamped (zeta = 0)
    return K * (1 - Math.cos(wn * t));
  }
}

// ============================================================================
// Bode Response Functions
// ============================================================================

/**
 * FOLPD Bode: G(s) = K/(Ts+1)·e^(-Ls)
 * @param {number[]} omega - Frequency array (rad/s)
 * @param {number} K - Gain
 * @param {number} T - Time constant (s)
 * @param {number} L - Dead time (s)
 * @returns {Object} {mag, phase} in dB and degrees
 */
function folpdBode(omega, K, T, L) {
  const mag = new Array(omega.length);
  const phase = new Array(omega.length);
  for (let i = 0; i < omega.length; i++) {
    const w = omega[i];
    const wT = w * T;
    mag[i] = 20 * Math.log10(K / Math.sqrt(1 + wT * wT));
    phase[i] = (-Math.atan(wT) - w * L) * 180 / Math.PI;
  }
  return { mag, phase };
}

/**
 * SOPTD Bode: G(s) = K/((T1s+1)(T2s+1))·e^(-Ls)
 * @param {number[]} omega - Frequency array (rad/s)
 * @param {number} K - Gain
 * @param {number} T1 - Time constant 1 (s)
 * @param {number} T2 - Time constant 2 (s)
 * @param {number} L - Dead time (s)
 * @returns {Object} {mag, phase} in dB and degrees
 */
function soptdBode(omega, K, T1, T2, L) {
  const mag = new Array(omega.length);
  const phase = new Array(omega.length);
  for (let i = 0; i < omega.length; i++) {
    const w = omega[i];
    mag[i] = 20 * Math.log10(K) - 10 * Math.log10(1 + (w * T1) ** 2) - 10 * Math.log10(1 + (w * T2) ** 2);
    phase[i] = (-Math.atan(w * T1) - Math.atan(w * T2) - w * L) * 180 / Math.PI;
  }
  return { mag, phase };
}

/**
 * Standard 2nd order Bode: G(s) = K·ωn²/(s² + 2ζωns + ωn²)
 * @param {number[]} omega - Frequency array (rad/s)
 * @param {number} K - Gain
 * @param {number} zeta - Damping ratio
 * @param {number} wn - Natural frequency (rad/s)
 * @returns {Object} {mag, phase} in dB and degrees
 */
function so2Bode(omega, K, zeta, wn) {
  const mag = new Array(omega.length);
  const phase = new Array(omega.length);
  for (let i = 0; i < omega.length; i++) {
    const w = omega[i];
    const r = w / wn;
    const denom = Math.sqrt((1 - r * r) ** 2 + (2 * zeta * r) ** 2);
    mag[i] = 20 * Math.log10(K) - 20 * Math.log10(denom);
    phase[i] = -Math.atan2(2 * zeta * r, 1 - r * r) * 180 / Math.PI;
  }
  return { mag, phase };
}

// ============================================================================
// Metrics
// ============================================================================

/**
 * Find -3 dB bandwidth from computed magnitude array
 * @param {number[]} omega - Frequency array (rad/s)
 * @param {number[]} mag - Magnitude in dB
 * @param {number} K - DC gain
 * @returns {number} Bandwidth frequency (rad/s)
 */
function findBandwidth(omega, mag, K) {
  if (K <= 0) return omega[omega.length - 1];
  const target = 20 * Math.log10(K) - 3;
  let idx = mag.findIndex(m => m <= target);
  if (idx === -1) return omega[omega.length - 1];
  if (idx === 0) return omega[0];
  // Linear interpolation
  const m0 = mag[idx - 1], m1 = mag[idx];
  const w0 = omega[idx - 1], w1 = omega[idx];
  if (m0 === m1) return w0;
  const t = (target - m0) / (m1 - m0);
  return w0 + t * (w1 - w0);
}

/**
 * Calculate damping ratio from two time constants
 * @param {number} T1 - Time constant 1
 * @param {number} T2 - Time constant 2
 * @returns {number} Equivalent damping ratio
 */
function zetaFromTimeConstants(T1, T2) {
  return (T1 + T2) / (2 * Math.sqrt(T1 * T2));
}

/**
 * Get pole description for 2nd order system
 * @param {number} zeta - Damping ratio
 * @returns {string} Pole description
 */
function poleDescription(zeta) {
  if (zeta < 1) return 'complex conj.';
  if (Math.abs(zeta - 1) < 0.01) return 'repeated real';
  return 'two real poles';
}

// ============================================================================
// Simulation Runner
// ============================================================================

/**
 * Run complete simulation for any model type
 * @param {string} model - 'folpd', 'soptd', or 'so2'
 * @param {Object} params - Model parameters
 * @param {Object} options - Simulation options
 * @returns {Object} {t, y, omega, mag, phase, metrics}
 */
function simulateProcessModel(model, params, options = {}) {
  const nStep = options.nStep || 300;
  const nBode = options.nBode || 500;
  const wStart = options.wStart || 1e-3;
  const wStop = options.wStop || 1e2;

  let t, y, omega, mag, phase, metrics;

  switch (model) {
    case 'folpd': {
      const { K, T, L } = params;
      const tEnd = 5 * Math.max(T, 1) + 3 * L + 2;
      t = linspace(0, tEnd, nStep);
      y = t.map(ti => folpdStep(ti, K, T, L));
      omega = logspace(wStart, wStop, nBode);
      ({ mag, phase } = folpdBode(omega, K, T, L));
      metrics = {
        gain: K,
        riseTime: T,
        deadTime: L,
        ratio: L / T,
        bandwidth: findBandwidth(omega, mag, K)
      };
      break;
    }
    case 'soptd': {
      const { K, T1, T2, L } = params;
      const tEnd = 5 * Math.max(T1, T2, 1) + 3 * L + 2;
      t = linspace(0, tEnd, nStep);
      y = t.map(ti => soptdStep(ti, K, T1, T2, L));
      omega = logspace(wStart, wStop, nBode);
      ({ mag, phase } = soptdBode(omega, K, T1, T2, L));
      metrics = {
        gain: K,
        T1, T2,
        deadTime: L,
        zeta: zetaFromTimeConstants(T1, T2),
        bandwidth: findBandwidth(omega, mag, K)
      };
      break;
    }
    case 'so2': {
      const { K, zeta, wn } = params;
      const tEnd = 8 / (zeta * wn + 0.01);
      t = linspace(0, tEnd, nStep);
      y = t.map(ti => so2Step(ti, K, zeta, wn));
      omega = logspace(wStart, wStop, nBode);
      ({ mag, phase } = so2Bode(omega, K, zeta, wn));
      metrics = {
        gain: K,
        zeta,
        wn,
        poles: poleDescription(zeta),
        bandwidth: findBandwidth(omega, mag, K)
      };
      break;
    }
    default:
      throw new Error(`Unknown model: ${model}`);
  }

  return { t, y, omega, mag, phase, metrics };
}

// ============================================================================
// Module Exports
// ============================================================================

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    linspace,
    logspace,
    folpdStep,
    soptdStep,
    so2Step,
    folpdBode,
    soptdBode,
    so2Bode,
    findBandwidth,
    zetaFromTimeConstants,
    poleDescription,
    simulateProcessModel
  };
}
