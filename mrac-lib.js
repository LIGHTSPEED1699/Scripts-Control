/**
 * MRAC (Model Reference Adaptive Control) Library
 * Extracted from MRAC.astro
 * 
 * Supports:
 * - Lyapunov-based gradient adaptation
 * - MIT Rule adaptation (Åström & Wittenmark)
 * - First-order SISO plants with reference model following
 */

// ============================================================================
// Base MRAC Class
// ============================================================================

class BaseMRAC {
  constructor() {
    this.reset();
  }

  reset() {
    // Plant parameters
    this.a = 2.0;    // Plant pole
    this.b = 1.0;    // Plant gain
    
    // Reference model parameters
    this.am = 1.0;   // Reference model pole
    this.bm = 1.0;   // Reference model gain
    
    // Adaptive parameters
    this.theta1 = 0.0;
    this.theta2 = 0.0;
    this.gamma1 = 0.5;
    this.gamma2 = 0.5;
    
    // States
    this.y = 0.0;    // Plant output
    this.ym = 0.0;   // Reference model output
    this.t = 0.0;    // Current time
    
    // Reference signal
    this.refType = 'square';
    this.refPeriod = 10.0;
    
    // History for plotting
    this.history = [];
    this.maxHistory = 2000;
  }

  /**
   * Generate reference signal r(t)
   * @param {number} t - Current time
   * @returns {number} Reference value
   */
  reference(t) {
    const p = this.refPeriod;
    switch (this.refType) {
      case 'sine':
        return Math.sin(2 * Math.PI * t / p);
      case 'square':
        return Math.sign(Math.sin(2 * Math.PI * t / p));
      case 'step':
        return t > 1 ? 1 : 0;
      default:
        return 0;
    }
  }

  /**
   * Integrate plant dynamics: ẏ = -a·y + b·u
   * @param {number} u - Control input
   * @param {number} dt - Time step
   */
  integratePlant(u, dt) {
    const dy = -this.a * this.y + this.b * u;
    this.y += dy * dt;
  }

  /**
   * Integrate reference model: ẏm = -am·ym + bm·r
   * @param {number} r - Reference signal
   * @param {number} dt - Time step
   */
  integrateRefModel(r, dt) {
    const dym = -this.am * this.ym + this.bm * r;
    this.ym += dym * dt;
  }

  /**
   * Record simulation history
   * @param {number} r - Reference
   * @param {number} u - Control input
   * @param {number} e - Error
   */
  recordHistory(r, u, e) {
    this.history.push({
      t: this.t,
      y: this.y,
      ym: this.ym,
      u: u,
      e: e,
      r: r,
      theta1: this.theta1,
      theta2: this.theta2
    });
    if (this.history.length > this.maxHistory) {
      this.history.shift();
    }
  }

  /**
   * Compute control law (to be implemented by subclasses)
   * @param {number} r - Reference signal
   * @returns {number} Control input
   */
  computeControl(r) {
    throw new Error('Must be implemented by subclass');
  }

  /**
   * Single simulation step (to be implemented by subclasses)
   * @param {number} dt - Time step
   * @returns {Object} Simulation results
   */
  step(dt) {
    throw new Error('Must be implemented by subclass');
  }
}

// ============================================================================
// Lyapunov-based MRAC
// ============================================================================

/**
 * Lyapunov-based gradient adaptation
 * 
 * Control law: u = θ₁·r + θ₂·y
 * Error: e = ym - y
 * Adaptation: θ̇₁ = γ₁·e·r, θ̇₂ = γ₂·e·y
 * 
 * This guarantees stability via Lyapunov function V = ½e² + (θ₁-θ₁*)²/(2γ₁) + (θ₂-θ₂*)²/(2γ₂)
 */
class LyapunovMRAC extends BaseMRAC {
  computeControl(r) {
    return this.theta1 * r + this.theta2 * this.y;
  }

  step(dt) {
    const r = this.reference(this.t);
    const u = this.computeControl(r);
    
    // Integrate plant and reference model
    this.integratePlant(u, dt);
    this.integrateRefModel(r, dt);
    
    // Compute error
    const e = this.ym - this.y;
    
    // Lyapunov adaptation (gradient of error squared)
    this.theta1 += this.gamma1 * e * r * dt;
    this.theta2 += this.gamma2 * e * this.y * dt;
    
    this.t += dt;
    this.recordHistory(r, u, e);
    
    return {
      y: this.y,
      ym: this.ym,
      u: u,
      e: e,
      theta1: this.theta1,
      theta2: this.theta2
    };
  }
}

// ============================================================================
// MIT Rule MRAC
// ============================================================================

/**
 * MIT Rule adaptation (Åström & Wittenmark, Example 5.2)
 * 
 * Control law: u = θ₁·r - θ₂·y
 * Error: e = y - ym
 * Filters: ẋ₁ = -am·x₁ + am·r, ẋ₂ = -am·x₂ + am·y
 * Adaptation: θ̇₁ = -γ₁·e·x₁, θ̇₂ = γ₂·e·x₂
 */
class MitMRAC extends BaseMRAC {
  constructor() {
    super();
    this.filter1 = 0.0;  // x₁ = am/(p+am)·r
    this.filter2 = 0.0;  // x₂ = am/(p+am)·y
  }

  reset() {
    super.reset();
    this.filter1 = 0.0;
    this.filter2 = 0.0;
  }

  computeControl(r) {
    return this.theta1 * r - this.theta2 * this.y;
  }

  step(dt) {
    const r = this.reference(this.t);
    const u = this.computeControl(r);
    
    // Step 1: Integrate plant and reference model
    this.integratePlant(u, dt);
    this.integrateRefModel(r, dt);
    
    // Step 2: Update filters with current y
    const dx1 = -this.am * this.filter1 + this.am * r;
    this.filter1 += dx1 * dt;
    
    const dx2 = -this.am * this.filter2 + this.am * this.y;
    this.filter2 += dx2 * dt;
    
    // Step 3: Compute MIT error
    const e = this.y - this.ym;
    
    // MIT adaptation
    this.theta1 += -this.gamma1 * e * this.filter1 * dt;
    this.theta2 += this.gamma2 * e * this.filter2 * dt;
    
    this.t += dt;
    this.recordHistory(r, u, e);
    
    return {
      y: this.y,
      ym: this.ym,
      u: u,
      e: e,
      theta1: this.theta1,
      theta2: this.theta2,
      filter1: this.filter1,
      filter2: this.filter2
    };
  }
}

// ============================================================================
// MRAC Factory
// ============================================================================

/**
 * Create MRAC instance by method name
 * @param {string} method - 'lyapunov' or 'mit'
 * @returns {BaseMRAC} MRAC instance
 */
function createMRAC(method) {
  switch (method) {
    case 'lyapunov':
      return new LyapunovMRAC();
    case 'mit':
      return new MitMRAC();
    default:
      throw new Error(`Unknown MRAC method: ${method}`);
  }
}

// ============================================================================
// Simulation Runner
// ============================================================================

/**
 * Run MRAC simulation for specified duration
 * @param {BaseMRAC} mrac - MRAC instance
 * @param {number} duration - Simulation duration (s)
 * @param {number} dt - Time step (s)
 * @returns {Object} Complete history and final parameters
 */
function simulateMRAC(mrac, duration, dt) {
  const steps = Math.floor(duration / dt);
  
  for (let i = 0; i < steps; i++) {
    mrac.step(dt);
  }
  
  return {
    history: mrac.history,
    final: {
      theta1: mrac.theta1,
      theta2: mrac.theta2,
      y: mrac.y,
      ym: mrac.ym,
      error: mrac.ym - mrac.y
    }
  };
}

// ============================================================================
// Stability Analysis
// ============================================================================

/**
 * Check if MRAC parameters have converged
 * @param {Object[]} history - Simulation history
 * @param {number} window - Number of samples to check
 * @param {number} threshold - Convergence threshold
 * @returns {boolean} True if converged
 */
function checkConvergence(history, window = 100, threshold = 0.01) {
  if (history.length < window) return false;
  
  const recent = history.slice(-window);
  const theta1Var = variance(recent.map(h => h.theta1));
  const theta2Var = variance(recent.map(h => h.theta2));
  
  return theta1Var < threshold && theta2Var < threshold;
}

function variance(arr) {
  const mean = arr.reduce((a, b) => a + b, 0) / arr.length;
  return arr.reduce((sum, val) => sum + (val - mean) ** 2, 0) / arr.length;
}

// ============================================================================
// Ideal Parameters (for comparison)
// ============================================================================

/**
 * Compute ideal controller parameters for perfect model matching
 * @param {number} a - Plant pole
 * @param {number} b - Plant gain
 * @param {number} am - Reference model pole
 * @param {number} bm - Reference model gain
 * @returns {Object} Ideal {theta1, theta2}
 */
function idealParameters(a, b, am, bm) {
  return {
    theta1: bm / b,
    theta2: (am - a) / b
  };
}

// ============================================================================
// Module Exports
// ============================================================================

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    BaseMRAC,
    LyapunovMRAC,
    MitMRAC,
    createMRAC,
    simulateMRAC,
    checkConvergence,
    idealParameters
  };
}
