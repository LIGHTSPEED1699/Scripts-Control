/**
 * Bode Plot Computation Library
 * Extracted from PIDTuner.astro
 * 
 * Frequency response analysis for control systems
 */

// ============================================================================
// Frequency Response Computation
// ============================================================================

/**
 * Generate logarithmically spaced frequency array
 * @param {number} start - Start frequency (rad/s)
 * @param {number} stop - Stop frequency (rad/s)
 * @param {number} n - Number of points
 * @returns {number[]} Log-spaced frequencies
 */
function logspace(start, stop, n) {
  const arr = new Array(n);
  const startLog = Math.log10(start);
  const stopLog = Math.log10(stop);
  const step = (stopLog - startLog) / (n - 1);
  for (let i = 0; i < n; i++) {
    arr[i] = Math.pow(10, startLog + step * i);
  }
  return arr;
}

/**
 * Compute open-loop Bode response for FOLPD plant with PID controller
 * 
 * Plant: G(s) = K·e^(-Ls)/(Ts+1)
 * Controller: Gc(s) = Kp + Ki/s + Kd·s/(τd·s+1)
 * Open loop: L(s) = Gc(s)·G(s)
 * 
 * @param {number} K - Plant gain
 * @param {number} T - Plant time constant (s)
 * @param {number} L - Plant dead time (s)
 * @param {number} kp - Proportional gain
 * @param {number} ki - Integral gain
 * @param {number} kd - Derivative gain
 * @param {number} tau_d - Derivative filter time constant (s)
 * @param {number[]} omega - Frequency array (rad/s)
 * @returns {Object} {mag, phase} arrays in dB and degrees
 */
function computeOpenLoopBode(K, T, L, kp, ki, kd, tau_d, omega) {
  const mag = new Array(omega.length);
  const phase = new Array(omega.length);
  
  for (let i = 0; i < omega.length; i++) {
    const w = omega[i];
    
    if (w < 1e-10) {
      mag[i] = 200;
      phase[i] = -90;
      continue;
    }
    
    // PID controller frequency response
    const wTd = w * tau_d;
    const denomFilter = 1 + wTd * wTd;
    
    // Gc(jω) = kp + ki/(jω) + kd·(jω)/(1 + jω·τd)
    const reGc = kp + kd * w * w * tau_d / denomFilter;
    const imGc = kd * w / denomFilter - ki / w;
    
    const magGc = Math.sqrt(reGc * reGc + imGc * imGc);
    const phaseGc = Math.atan2(imGc, reGc) * 180 / Math.PI;
    
    // Plant frequency response: G(jω) = K·e^(-jωL)/(1 + jωT)
    const wT = w * T;
    const magG = K / Math.sqrt(1 + wT * wT);
    const phaseG = (-Math.atan(wT) - w * L) * 180 / Math.PI;
    
    // Open loop
    mag[i] = 20 * Math.log10(magGc * magG);
    phase[i] = phaseGc + phaseG;
  }
  
  return { mag, phase };
}

/**
 * Compute Bode response for arbitrary transfer function
 * @param {number[]} num - Numerator coefficients
 * @param {number[]} den - Denominator coefficients
 * @param {number} L - Dead time (s)
 * @param {number[]} omega - Frequency array (rad/s)
 * @returns {Object} {mag, phase} arrays
 */
function computeTFOpenLoopBode(num, den, L, kp, ki, kd, tau_d, omega) {
  const mag = new Array(omega.length);
  const phase = new Array(omega.length);
  
  for (let i = 0; i < omega.length; i++) {
    const w = omega[i];
    
    if (w < 1e-10) {
      mag[i] = 200;
      phase[i] = -90;
      continue;
    }
    
    // Evaluate plant TF at s = jω using the canonical evaluateTF from control-systems-lib.js
    // evaluateTF returns {re, im} (real and imaginary parts of the complex result).
    const plantResult = evaluateTF(num, den, 0, w);
    const plantMag = Math.sqrt(plantResult.re * plantResult.re + plantResult.im * plantResult.im);
    const plantPhase = Math.atan2(plantResult.im, plantResult.re) * 180 / Math.PI - w * L * 180 / Math.PI;
    
    // PID controller
    const wTd = w * tau_d;
    const denomFilter = 1 + wTd * wTd;
    const reGc = kp + kd * w * w * tau_d / denomFilter;
    const imGc = kd * w / denomFilter - ki / w;
    const magGc = Math.sqrt(reGc * reGc + imGc * imGc);
    const phaseGc = Math.atan2(imGc, reGc) * 180 / Math.PI;
    
    // Combined
    mag[i] = 20 * Math.log10(magGc * plantMag);
    phase[i] = phaseGc + plantPhase;
  }
  
  return { mag, phase };
}

// ============================================================================
// Stability Margins
// ============================================================================

/**
 * Find gain and phase margins from Bode data
 * @param {number[]} omega - Frequency array
 * @param {number[]} mag_dB - Magnitude in dB
 * @param {number[]} phase - Phase in degrees
 * @returns {Object} {w_gc, pm, w_180, gm_dB}
 */
function findMargins(omega, mag_dB, phase) {
  let w_gc = null, pm = null;
  let w_180 = null, gm_dB = null;
  
  // Find gain crossover (where |L(jω)| = 1, i.e., 0 dB)
  for (let i = 1; i < omega.length; i++) {
    if (mag_dB[i - 1] > 0 && mag_dB[i] <= 0) {
      const t = -mag_dB[i - 1] / (mag_dB[i] - mag_dB[i - 1]);
      w_gc = omega[i - 1] + t * (omega[i] - omega[i - 1]);
      const ph = phase[i - 1] + t * (phase[i] - phase[i - 1]);
      pm = 180 + ph;
      break;
    }
  }
  
  // Find phase crossover (where ∠L(jω) = -180°)
  for (let i = 1; i < omega.length; i++) {
    if (phase[i - 1] > -180 && phase[i] <= -180) {
      const t = (-180 - phase[i - 1]) / (phase[i] - phase[i - 1]);
      w_180 = omega[i - 1] + t * (omega[i] - omega[i - 1]);
      const m = mag_dB[i - 1] + t * (mag_dB[i] - mag_dB[i - 1]);
      gm_dB = -m;
      break;
    }
  }
  
  return { w_gc, pm, w_180, gm_dB };
}

/**
 * Check closed-loop stability from margins
 * @param {Object} margins - From findMargins()
 * @returns {boolean} True if stable
 */
function isStable(margins) {
  // Stable if:
  // 1. Gain margin > 0 dB (or null if no phase crossover)
  // 2. Phase margin > 0° (or null if no gain crossover)
  if (margins.pm !== null && margins.pm <= 0) return false;
  if (margins.gm_dB !== null && margins.gm_dB <= 0) return false;
  return true;
}

// ============================================================================
// Bandwidth Estimation
// ============================================================================

/**
 * Estimate closed-loop bandwidth from open-loop Bode
 * @param {number[]} omega - Frequency array
 * @param {number[]} mag_dB - Open-loop magnitude
 * @returns {number|null} Bandwidth frequency (rad/s)
 */
function estimateBandwidth(omega, mag_dB) {
  // Bandwidth is where open-loop gain drops to ~-3 to -7 dB
  // Approximate: find where |L(jω)| ≈ 0.5 (-6 dB)
  for (let i = 1; i < omega.length; i++) {
    if (mag_dB[i - 1] > -6 && mag_dB[i] <= -6) {
      const t = (-6 - mag_dB[i - 1]) / (mag_dB[i] - mag_dB[i - 1]);
      return omega[i - 1] + t * (omega[i] - omega[i - 1]);
    }
  }
  return null;
}

// ============================================================================
// Nyquist Analysis
// ============================================================================

/**
 * Compute Nyquist plot data
 * @param {number[]} num - Numerator coefficients
 * @param {number[]} den - Denominator coefficients
 * @param {number} L - Dead time
 * @param {number[]} omega - Frequency array
 * @returns {Object} {re, im} arrays
 */
function computeNyquist(num, den, L, omega) {
  const re = new Array(omega.length);
  const im = new Array(omega.length);
  
  for (let i = 0; i < omega.length; i++) {
    const w = omega[i];
    const result = evaluateTF(num, den, 0, w);
    const delayPhase = -w * L;
    const cosPhase = Math.cos(delayPhase);
    const sinPhase = Math.sin(delayPhase);
    
    // Apply dead time rotation
    re[i] = result.re * cosPhase - result.im * sinPhase;
    im[i] = result.re * sinPhase + result.im * cosPhase;
  }
  
  return { re, im };
}

// ============================================================================
// Helper: Evaluate Transfer Function
// ============================================================================

/**
 * Evaluate transfer function at complex frequency s = σ + jω
 * (Same as in control-systems-lib.js)
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

// ============================================================================
// Module Exports
// ============================================================================

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    logspace,
    computeOpenLoopBode,
    computeTFOpenLoopBode,
    findMargins,
    isStable,
    estimateBandwidth,
    computeNyquist,
    evaluateTF
  };
}
