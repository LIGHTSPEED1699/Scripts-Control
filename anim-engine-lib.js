// anim-engine-lib.js — animation + physics primitives for the repo.
//
// EXPERIMENTAL (v1.3.0): this module was extracted from the website's
// public/anim-engine.js and is published as-is. The API is stable for
// the documented exports (rk4, createHistoryBuffer, startAnimLoop,
// clamp, drawMSD, drawTankSVG, the deriv / paramPresets / initialStates
// tables) but the broader surface is a working library of physics
// primitives rather than a curated control-systems interface. The
// helper functions `step` and `weighted` are now part of the public
// API even though they were private in the original (they are useful
// building blocks for custom integrators).
//
// Pure ES module, zero dependencies, browser-only at the edges
// (canvas 2D context, requestAnimationFrame). The core math (RK4,
// history buffer, derivative functions) is environment-agnostic.
//
// Design notes (vs. TURIX Lab):
//   - 500-700 lines, no React, no shadcn, no Recharts, no build step
//   - RK4 integrator is the same textbook 4-stage form TURIX uses
//   - Substep pattern (dt ≤ 0.5 ms) and chart throttle (50 Hz) are TURIX's exact pattern
//   - Drawing code is hand-rolled 2D canvas, like TURIX (no Three.js, no WebGL)
//   - State contract: every system includes a `time: 1` derivative so RK4 advances the clock
//   - Variables named in English, not Hungarian. Less clever than the minified original.
//
// Public API:
//   - rk4(state, h, derivFn)                 — single RK4 step
//   - step(state, dState, scale)             — low-level state advance
//   - weighted(k1, k2, k3, k4, h)            — low-level RK4 weighted sum
//   - startAnimLoop(opts)                    — rAF loop with substep + history throttle; returns stop()
//   - createHistoryBuffer(sampleHz, maxSamples)
//   - clamp(x, lo, hi)                       — numeric clamp
//   - drawMSD(ctx, state, params, opts)      — mass-spring-damper visual
//   - drawTankSVG(state, params, opts)       — returns SVG string for the tank
//   - deriv.msd / deriv.tank / deriv.dcMotor / deriv.rlcSeries /
//     deriv.simplePendulum / deriv.invertedPendulum / deriv.pvtol / deriv.bioreactor
//   - paramPresets / initialStates

// ─────────────────────────────────────────────────────────────────────
// RK4 INTEGRATOR
// ─────────────────────────────────────────────────────────────────────

/**
 * One step of classical 4th-order Runge-Kutta.
 * @param {object} state  - current state object (any shape, must include `time`)
 * @param {number} h      - time step in seconds
 * @param {Function} deriv - (state, params, t) → dState/dt. Must return a state-like object
 *                           whose `time` derivative is 1 (so the integrator advances the clock).
 * @returns {object} new state at t + h
 */
export function rk4(state, h, deriv) {
  const params = state._params; // params carried through state; cleaner than passing separately
  const t = state.time;

  const k1 = deriv(state, params, t);
  const k2 = deriv(step(state, k1, h / 2), params, t + h / 2);
  const k3 = deriv(step(state, k2, h / 2), params, t + h / 2);
  const k4 = deriv(step(state, k3, h), params, t + h);

  return step(state, weighted(k1, k2, k3, k4, h), 1);
}

function step(state, dState, scale) {
  const out = { ...state };
  for (const k in dState) {
    if (k === '_params') continue;
    out[k] = state[k] + dState[k] * scale;
  }
  return out;
}

function weighted(k1, k2, k3, k4, h) {
  const out = {};
  for (const k in k1) {
    if (k === '_params') continue;
    out[k] = h * (k1[k] + 2 * k2[k] + 2 * k3[k] + k4[k]) / 6;
  }
  return out;
}

/**
 * Re-export the low-level RK4 helpers. They were private in the
 * website's anim-engine.js (rk4 is the only documented entry point),
 * but they're useful as building blocks for custom integrators.
 * The repo's public API surface exposes them.
 */
export { step, weighted };

// ─────────────────────────────────────────────────────────────────────
// HISTORY BUFFER
// ─────────────────────────────────────────────────────────────────────

/**
 * Throttled history buffer for chart data. Pushes at most `sampleHz` samples/second,
 * caps total stored samples at `maxSamples` (oldest get dropped first).
 */
export function createHistoryBuffer(sampleHz = 50, maxSamples = 500) {
  let lastSampleTime = -Infinity;
  const data = [];

  return {
    /**
     * Maybe push a sample. Returns true if the sample was stored.
     * @param {number} t  - current sim time
     * @param {object} pt - sample object (whatever shape your chart expects)
     */
    push(t, pt) {
      const bucket = Math.floor(t * sampleHz);
      if (bucket === lastSampleTime) return false;
      lastSampleTime = bucket;
      data.push(pt);
      if (data.length > maxSamples) data.splice(0, data.length - maxSamples);
      return true;
    },
    clear() { data.length = 0; lastSampleTime = -Infinity; },
    get all() { return data; },
    get length() { return data.length; },
  };
}

// ─────────────────────────────────────────────────────────────────────
// ANIMATION LOOP
// ─────────────────────────────────────────────────────────────────────

/**
 * Vanilla-JS animation loop with TURIX's exact pattern:
 *   - dt capped at `maxDtPerFrame` (20 ms) to recover from tab unfocus
 *   - physics substepped so each step ≤ `substepMaxH` (0.5 ms) for RK4 stability
 *   - chart history pushed at `historyHz` (50 Hz) regardless of frame rate
 *
 * Drawing bypasses any framework by writing directly to the canvas via `drawFn`.
 *
 * @param {object} opts
 * @param {HTMLCanvasElement} opts.canvas
 * @param {Function} opts.getState    - () → current state
 * @param {Function} opts.setState    - (newState) → void
 * @param {Function} opts.derivFn     - (state, params, t) → dState
 * @param {Function} opts.drawFn      - (ctx, state, params, opts) → void
 * @param {Function} [opts.historySample] - (state, params) → chartPoint | null
 * @param {Function} [opts.onHistory]     - (chartPoint) → void
 * @param {Function} [opts.running]       - () → boolean (defaults to true)
 * @param {object}    opts.params
 * @param {number}    [opts.maxDtPerFrame=0.02]   - max wallclock dt per frame (s)
 * @param {number}    [opts.substepMaxH=0.0005]   - max substep size (s)
 * @param {number}    [opts.historyHz=50]
 * @param {number}    [opts.historyMaxSamples=500]
 * @returns {{ stop: () => void }}
 */
export function startAnimLoop(opts) {
  const {
    canvas, getState, setState, derivFn, drawFn, params,
    historySample, onHistory,
    running = () => true,
    maxDtPerFrame = 0.02,
    substepMaxH = 0.0005,
    historyHz = 50,
    historyMaxSamples = 500,
  } = opts;

  const history = createHistoryBuffer(historyHz, historyMaxSamples);
  const ctx = canvas.getContext('2d');
  let lastWallTime = null;
  let rafId = null;

  function loop(now) {
    if (lastWallTime === null) {
      lastWallTime = now;
      rafId = requestAnimationFrame(loop);
      return;
    }

    const isRunning = running();

    if (isRunning) {
      const wallDt = (now - lastWallTime) / 1000;
      lastWallTime = now;
      const dt = Math.min(wallDt, maxDtPerFrame);

      // Substep physics so each RK4 step is ≤ substepMaxH
      const substeps = Math.max(1, Math.ceil(dt / substepMaxH));
      const h = dt / substeps;

      let state = getState();
      for (let i = 0; i < substeps; i++) {
        state = rk4(state, h, derivFn);
      }
      setState(state);

      // Throttled history push
      if (historySample && onHistory) {
        const pt = historySample(state, params);
        if (pt) history.push(state.time, pt);
      }
    } else {
      // Paused: don't advance physics, but DO redraw the canvas so
      // the user sees the frozen state (and any UI changes the
      // paused state is still responsive to). Also reset
      // lastWallTime so the first frame after resume doesn't see
      // a huge accumulated dt and step the physics way forward.
      lastWallTime = now;
    }

    // Direct canvas redraw (no framework in the hot path). Always
    // redraw, whether running or paused, so the user always sees
    // the current state.
    if (ctx) {
      const state = getState();
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawFn(ctx, state, params, { canvas });
    }

    // Schedule the next frame unconditionally so a paused state
    // can be resumed just by flipping running() back to true.
    // The full stop() method below is the only way to kill the
    // loop entirely.
    rafId = requestAnimationFrame(loop);
  }

  // Always schedule the first frame, even if `running()` is false
  // at init. The loop itself is cheap when paused (just a canvas
  // redraw + `running()` check), and scheduling it unconditionally
  // means a later toggle of `running` from false -> true takes
  // effect on the next frame. The previous version had
  // `if (running()) rafId = requestAnimationFrame(loop)`, which
  // meant a paused-at-startup page would never get a frame and
  // would never notice when the user clicked Start. (The loop
  // re-schedules itself from inside via the unconditional
  // `rafId = requestAnimationFrame(loop)` at the end of the loop
  // body, so this initial kickoff is the only place the gate
  // matters.)
  rafId = requestAnimationFrame(loop);

  return {
    stop() {
      if (rafId !== null) cancelAnimationFrame(rafId);
      rafId = null;
    },
  };
}

// ─────────────────────────────────────────────────────────────────────
// DERIVATIVE FUNCTIONS — one per system
// Convention: every deriv returns dState/dt; `time` derivative is always 1.
// ─────────────────────────────────────────────────────────────────────

export const deriv = {
  /**
   * Mass-spring-damper with augmented PID book-keeping (TURIX's exact pattern).
   * State: { x, v, z_int, d_f, time }
   * Params: { m, c, k, F_ext, Kp, Ki, Kd, N, beta, F_max, controlled }
   */
  msd: (s, p, t) => {
    const F = p.controlled
      ? clamp(
          p.Kp * ((p.reference ?? 0) - s.x)
            + p.Ki * s.z_int
            - p.Kd * s.d_f,
          -p.F_max, p.F_max
        )
      : (p.F_ext ?? 0);
    const e = (p.reference ?? 0) - s.x;
    const F_clamped = p.controlled ? clamp(p.Kp * e + p.Ki * s.z_int - p.Kd * s.d_f, -p.F_max, p.F_max) : F;

    return {
      x: s.v,
      v: (F_clamped - p.c * s.v - p.k * s.x) / p.m,
      // Set-point weighting on integral: z_int accumulates the *clamped* error
      z_int: p.controlled ? (e + p.beta * (F_clamped - (p.Kp * e + p.Ki * s.z_int - p.Kd * s.d_f))) : 0,
      // Derivative on measurement with filter time constant
      d_f: p.controlled ? p.N * (s.v - s.d_f) : 0,
      time: 1,
    };
  },

  /**
   * Tank with Torricelli outflow + first-order lag on inflow.
   * State: { L, Q, time }
   * Params: { A, a, g, Q_target, tau, valve_gain, controlled, Q_max, reference }
   */
  tank: (s, p, t) => {
    const L = Math.max(s.L, 0);
    const outflow = p.a * Math.sqrt(2 * p.g * L);
    const Q_target = p.controlled
      ? clamp(p.valve_gain * ((p.reference ?? 1.0) - L), 0, p.Q_max ?? 100)
      : (p.Q_in ?? 0);
    return {
      L: (s.Q - outflow) / p.A,
      Q: (Q_target - s.Q) / p.tau,
      time: 1,
    };
  },

  /**
   * Separately-excited DC motor. State: { theta, omega, ia, time }
   * Params: { J, b, Kt, Kb, R, L, V, load_torque }
   *   V is the supply voltage (or set by controller).
   */
  dcMotor: (s, p, t) => {
    const V = p.controlled ? clamp(p.V_cmd(s.theta, s.omega, t), -p.V_max, p.V_max) : (p.V ?? 0);
    return {
      theta: s.omega,
      omega: (p.Kt * s.ia - p.b * s.omega - (p.load_torque ?? 0)) / p.J,
      ia: (V - p.R * s.ia - p.Kb * s.omega) / p.L,
      time: 1,
    };
  },

  /**
   * Series RLC circuit driven by a voltage source. State: { q, iq, time }
   *   q is capacitor charge, iq is loop current.
   * Params: { R, L, C, V, controlled }
   *   (For a current source input, use V_cmd function via p.V_cmd.)
   */
  rlcSeries: (s, p, t) => {
    const V = p.controlled ? p.V_cmd(t) : (p.V ?? 0);
    return {
      q: s.iq,
      iq: (V - p.R * s.iq - s.q / p.C) / p.L,
      time: 1,
    };
  },

  /**
   * Simple pendulum. State: { theta, omega, time } (theta in radians, 0 = down)
   * Params: { L, m, g, b, controlled, theta_cmd, Kp, Kd }
   *   Controlled applies a torque T = Kp*(theta_cmd - theta) - Kd*omega
   */
  simplePendulum: (s, p, t) => {
    const T = p.controlled
      ? clamp(
          p.Kp * ((p.theta_cmd ?? 0) - s.theta) - p.Kd * s.omega,
          -p.T_max, p.T_max
        )
      : 0;
    return {
      theta: s.omega,
      omega: -(p.g / p.L) * Math.sin(s.theta) - (p.b / (p.m * p.L * p.L)) * s.omega + T / (p.m * p.L * p.L),
      time: 1,
    };
  },

  /**
   * Inverted pendulum on a cart. State: { x, v, theta, omega, z_int, d_f, time }
   *   x: cart position; theta: pendulum angle from vertical (0 = up)
   * Params: { M, m, L, g, b_cart, b_pivot, F_max, Kp, Ki, Kd, N, beta, controlled,
   *           reference, controller_mode, Kx, Ktheta, Kv, Komega }
   *
   *   controller_mode:
   *     'pid'  — scalar PID on θ: F = Kp·θ + Ki·∫θ - Kd·d_f (default)
   *     'sf'   — full 4-state linear feedback:
   *                F = -(Kx·x + Ktheta·θ + Kv·v + Komega·ω)
   *              Active cart-position feedback; gains typically negative
   *              (LQR K = [-4.47, -82.55, -10.10, -16.21] stabilizes the
   *              M=2, m=0.2, L=0.5, g=9.81 plant). PID internals (z_int,
   *              d_f) are unused in 'sf' mode but kept in the state vector
   *              for storage reuse.
   *
   *   Full nonlinear equations of motion (sin/cos/omega^2 coupling terms kept).
   *   Linearization (sinθ→θ, drop ω^2) only valid for small angles near upright.
   *   Derivation: Spong, Hutchinson, & Vidyasagar, Robot Modeling and Control
   *   (Wiley, 2006), Ch. 5.
   *
   *   Control law (PID, 2026-06-05):
   *     Kp/Ki/Kd act on the pole angle θ. The control goal is to keep
   *     the pole upright; the cart position is an uncontrolled state
   *     and will settle to whatever offset the balance requires.
   *     Sign convention: when θ > 0 (pole falling right), F > 0
   *     (cart accelerates right) so the pivot moves under the
   *     falling CoM. Kp is positive in this control law.
   *
   *   Control law (state feedback, added 2026-06-13):
   *     F = -(Kx·x + Ktheta·θ + Kv·v + Komega·ω). The four gains
   *     multiply (x, θ, v, ω) directly. Sign convention: a *negative*
   *     Ktheta means "if θ > 0 (pole falling right), push the cart
   *     right" — same physical action as a positive Kp in PID mode.
   *     The LQR default K = [-4.47, -82.55, -10.10, -16.21] from the
   *     §8a tutorial is the canonical starting point. See
   *     src/pages/tools/inverted-pendulum-simulator/index.astro
   *     §3.4 for the in-browser walk-through.
   */
  invertedPendulum: (s, p, t) => {
    // Compute the control force. PID mode (default) and SF mode
    // (4-state feedback) both clamp to ±F_max. In SF mode the
    // controller is a pure linear function of the four states
    // (p, θ, v, ω) — no integral action, no derivative filter.
    // z_int and d_f are unused in SF mode but kept in the state
    // vector so the integrator state shape is unchanged.
    let F_pid;
    const e_theta = s.theta;  // PID error; used in z_int update below
    if (p.controlled) {
      if (p.controller_mode === 'sf') {
        F_pid = -(p.Kx * s.p + p.Ktheta * s.theta + p.Kv * s.v + p.Komega * s.omega);
      } else {
        // PID on θ. Sign convention: when θ > 0 (falling right),
        // push the cart right so the pivot moves under the CoM.
        // Kp > 0 stabilizes the pole.
        F_pid = p.Kp * e_theta + p.Ki * s.z_int - p.Kd * s.d_f;
      }
    } else {
      F_pid = 0;
    }
    const F_clamped = clamp(F_pid, -p.F_max, p.F_max);
    // Full nonlinear EOM for cart-pendulum. Pre-solved (decoupled) form
    // following Spong, Hutchinson, & Vidyasagar, Robot Modeling and
    // Control (Wiley, 2006), eq. 5.6–5.7 with point-mass pole (mass m
    // concentrated at the CoM a distance L from the pivot, so I = mL²).
    // Centripetal (m L ω² sinθ) and coupling (m L θ̈ cosθ) terms are kept
    // — the linearized form drops these. b_pivot is the pivot viscous
    // torque coefficient (N·m·s/rad), so it divides by I = mL².
    const denom = p.M + p.m;
    const sin_t = Math.sin(s.theta);
    const cos_t = Math.cos(s.theta);
    const pivot_drag = p.b_pivot / (p.m * p.L * p.L);  // 1/s per (rad/sec)
    // ddot(p) — from Spong 5.6, θ̈ substituted from 5.7
    const p_dd = (F_pid + p.m * p.L * s.omega * s.omega * sin_t - p.b_cart * s.v) / denom
                 - (p.m * (p.g * sin_t - pivot_drag * s.omega) * cos_t) / denom;
    // ddot(theta) — from Spong 5.7
    const theta_dd = (p.g * sin_t - pivot_drag * s.omega) * cos_t / p.L
                     - (F_pid + p.m * p.L * s.omega * s.omega * sin_t - p.b_cart * s.v) * cos_t / (denom * p.L);
    return {
      p: s.v,
      v: p_dd,
      theta: s.omega,
      omega: theta_dd,
      // z_int / d_f are only meaningful in PID mode. Keep them
      // frozen in SF mode (a 0 derivative) so toggling back to
      // PID mode does not introduce a stale integrator or
      // derivative-filter transient.
      z_int: p.controlled && p.controller_mode !== 'sf' ? e_theta : 0,
      d_f:   p.controlled && p.controller_mode !== 'sf' ? p.N * (s.omega - s.d_f) : 0,
      time: 1,
    };
  },

  /**
   * Planar Vertical Take-Off and Landing aircraft (simplified).
   * State: { x, y, theta, vx, vy, omega, time }
   *   (x, y): planar position; theta: pitch angle; vx, vy: velocities; omega: pitch rate
   * Params: { m, Ixx, g, T_left_max, T_right_max, T_left_cmd, T_right_cmd, L_arm }
   *   Two thrust vectors at the wingtips, angled inward.
   */
  pvtol: (s, p, t) => {
    const Tl = p.T_left_cmd ? p.T_left_cmd(t) : (p.T_left ?? 0);
    const Tr = p.T_right_cmd ? p.T_right_cmd(t) : (p.T_right ?? 0);
    const T = Tl + Tr;                  // total thrust (up in body frame)
    const tau = (Tr - Tl) * p.L_arm;    // pitch torque (L_arm is half the wingtip distance)
    return {
      x: s.vx,
      y: s.vy,
      theta: s.omega,
      vx: (T / p.m) * Math.sin(s.theta),
      vy: (T / p.m) * Math.cos(s.theta) - p.g,
      omega: tau / p.Ixx,
      time: 1,
    };
  },

  /**
   * Continuous stirred-tank bioreactor (Monod kinetics).
   * State: { X, S, P, time }
   *   X: biomass conc, S: substrate conc, P: product conc
   * Params: { mu_max, Ks, Yxs, Ypx, Sf, D, controlled, S_setpoint, Kp }
   */
  bioreactor: (s, p, t) => {
    const mu = p.mu_max * s.S / (p.Ks + s.S);
    const D = p.controlled
      ? clamp(p.D_cmd(s.X, s.S, t), 0, p.D_max)
      : (p.D ?? 0.1);
    return {
      X: mu * s.X - D * s.X,
      S: -mu * s.X / p.Yxs + D * (p.Sf - s.S),
      P: p.Ypx * mu * s.X - D * s.P,
      time: 1,
    };
  },
};

export function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }

// ─────────────────────────────────────────────────────────────────────
// DEFAULTS — useful starting points
// ─────────────────────────────────────────────────────────────────────

export const paramPresets = {
  msd:           { m: 1, c: 0.5, k: 2, F_max: 50, Kp: 5, Ki: 0.5, Kd: 2, N: 10, beta: 1, controlled: true, reference: 1, F_ext: 0 },
  tank:          { A: 1, a: 0.05, g: 9.81, tau: 0.5, valve_gain: 0.5, Q_max: 5, controlled: true, reference: 1.0, Q_in: 0 },
  dcMotor:       { J: 0.01, b: 0.001, Kt: 0.05, Kb: 0.05, R: 1, L: 0.01, V_max: 24, V: 12, load_torque: 0, controlled: false, V_cmd: () => 12 },
  rlcSeries:     { R: 10, L: 0.1, C: 1e-4, V: 5, controlled: false, V_cmd: () => 5 },
  simplePendulum:{ L: 1, m: 1, g: 9.81, b: 0.1, T_max: 5, Kp: 10, Kd: 2, controlled: false, theta_cmd: 0 },
  invertedPendulum:{ M: 2, m: 0.2, L: 0.5, g: 9.81, b_cart: 0.1, b_pivot: 0.01, F_max: 30, Kp: 30, Ki: 0, Kd: 0.5, N: 20, beta: 1, controlled: true, reference: 0, controller_mode: 'pid', Kx: 0, Ktheta: 0, Kv: 0, Komega: 0, Qp: 1, Qtheta: 50, Qv: 1, Qomega: 1, R: 0.05, p1: -1, p2: -2, p3: -3, p4: -4, sf_design_method: 'lqr', kick_scale: 1 },
  pvtol:         { m: 1, Ixx: 0.1, g: 9.81, L_arm: 0.3, T_left: 5, T_right: 5, T_left_cmd: null, T_right_cmd: null },
  bioreactor:    { mu_max: 0.5, Ks: 0.1, Yxs: 0.5, Ypx: 0.2, Sf: 10, D: 0.1, D_max: 0.5, controlled: false, D_cmd: () => 0.1 },
};

export const initialStates = {
  msd:              { x: 0, v: 0, z_int: 0, d_f: 0, time: 0 },
  tank:             { L: 0.1, Q: 0, time: 0 },
  dcMotor:          { theta: 0, omega: 0, ia: 0, time: 0 },
  rlcSeries:        { q: 0, iq: 0, time: 0 },
  simplePendulum:   { theta: 0.1, omega: 0, time: 0 },
  invertedPendulum: { p: 0, v: 0, theta: 0.05, omega: 0, z_int: 0, d_f: 0, time: 0 },
  pvtol:            { x: 0, y: 0, theta: 0, vx: 0, vy: 0, omega: 0, time: 0 },
  bioreactor:       { X: 0.1, S: 9, P: 0, time: 0 },
};

// ─────────────────────────────────────────────────────────────────────
// DRAWING — hand-rolled 2D Canvas / SVG
// Pattern from TURIX: brown ground, brick wall, orange zigzag spring, gray damper.
// ─────────────────────────────────────────────────────────────────────

/**
 * Draw a mass-spring-damper visual to a 2D canvas.
 * Layout mirrors TURIX's original: left wall, ground, spring zigzagging to a cart,
 * damper below the spring, optional reference line.
 *
 * @param {CanvasRenderingContext2D} ctx
 * @param {object} state  - { x, v, ... } in SI units (meters, seconds)
 * @param {object} params - { reference, scale, etc. }
 * @param {object} [opts] - { canvas }
 */
export function drawMSD(ctx, state, params = {}, opts = {}) {
  const canvas = opts.canvas || ctx.canvas;
  const W = canvas.width;
  const H = canvas.height;

  const scale = params.scale ?? 150;     // pixels per meter
  const homeX = W / 2;                   // cart center when x = 0
  const groundY = H - 60;
  const wallX = 40;
  const wallW = 18;
  const cartW = 80;
  const cartH = 50;
  const cartY = groundY - cartH;
  const springY = groundY - cartH * 1.4;

  const cartX = homeX + state.x * scale;

  // Ground
  ctx.fillStyle = '#8B4513';
  ctx.fillRect(0, groundY, W, 8);
  ctx.fillStyle = '#654321';
  ctx.fillRect(0, groundY + 8, W, 30);

  // Wall (left)
  ctx.fillStyle = '#D3D3D3';
  ctx.fillRect(wallX, 0, wallW, groundY);
  ctx.strokeStyle = '#000';
  ctx.lineWidth = 2;
  ctx.strokeRect(wallX, 0, wallW, groundY);
  // Brick lines
  ctx.strokeStyle = '#999';
  ctx.lineWidth = 1;
  for (let y = 30; y < groundY; y += 30) {
    ctx.beginPath();
    ctx.moveTo(wallX + 3, y);
    ctx.lineTo(wallX + wallW - 3, y);
    ctx.stroke();
  }

  // Spring (zigzag from wall to cart)
  const springStartX = wallX + wallW;
  const springEndX = cartX - cartW / 2;
  const segs = 14;
  const amp = 12;
  ctx.strokeStyle = '#FF6B35';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(springStartX, springY);
  for (let i = 1; i <= segs; i++) {
    const frac = i / segs;
    const x = springStartX + frac * (springEndX - springStartX);
    const y = springY + (i % 2 === 0 ? -amp : amp);
    ctx.lineTo(x, y);
  }
  ctx.lineTo(springEndX, springY);
  ctx.stroke();
  // Spring label
  ctx.fillStyle = '#FF6B35';
  ctx.font = 'bold 14px system-ui, sans-serif';
  ctx.fillText('k', springStartX + 20, springY - 14);

  // Damper (piston + cylinder, below the spring)
  const damperY = groundY - 25;
  ctx.fillStyle = '#606060';
  ctx.fillRect(springStartX, damperY - 18, 26, 36);
  ctx.strokeStyle = '#000';
  ctx.lineWidth = 2;
  ctx.strokeRect(springStartX, damperY - 18, 26, 36);
  // Piston rod
  const pistonEndX = cartX - cartW / 2;
  ctx.strokeStyle = '#404040';
  ctx.lineWidth = 6;
  ctx.beginPath();
  ctx.moveTo(springStartX + 26, damperY);
  ctx.lineTo(pistonEndX, damperY);
  ctx.stroke();
  ctx.fillStyle = '#4A90E2';
  ctx.font = 'bold 14px system-ui, sans-serif';
  ctx.fillText('c', springStartX + 8, damperY + 28);

  // Cart
  ctx.fillStyle = '#E67E22';
  ctx.fillRect(cartX - cartW / 2, cartY, cartW, cartH);
  ctx.strokeStyle = '#000';
  ctx.lineWidth = 2;
  ctx.strokeRect(cartX - cartW / 2, cartY, cartW, cartH);
  // Mass label
  ctx.fillStyle = '#fff';
  ctx.font = 'bold 16px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('m', cartX, cartY + cartH * 0.65);
  ctx.textAlign = 'left';

  // Reference line (dashed red, only if PID active and reference != 0)
  if (params.controlled && params.reference) {
    const refX = homeX + params.reference * scale;
    ctx.strokeStyle = '#E74C3C';
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 4]);
    ctx.beginPath();
    ctx.moveTo(refX, 30);
    ctx.lineTo(refX, groundY);
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

/**
 * Render the tank visual as an SVG string. The caller can inject it into the DOM.
 * @param {object} state - { L, Q, time } in SI units (meters, m³/s, seconds)
 * @param {object} params
 * @param {object} [opts] - { width, height, maxHeight, levelMin, levelMax }
 * @returns {string} SVG markup
 */
export function drawTankSVG(state, params = {}, opts = {}) {
  const W = opts.width ?? 400;
  const H = opts.height ?? 300;
  const maxH = opts.maxHeight ?? 2;     // physical tank height in meters
  const tankW = 160;
  const tankH = 220;
  const tankX = (W - tankW) / 2;
  const tankY = H - tankH - 20;

  // Clamp water level to the visible range
  const levelFrac = Math.max(0, Math.min(1, state.L / maxH));
  const waterH = levelFrac * tankH;
  const waterY = tankY + tankH - waterH;

  return `
<svg width="${W}" height="${H}" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <linearGradient id="water" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#3498db" stop-opacity="0.7"/>
      <stop offset="100%" stop-color="#2980b9" stop-opacity="0.9"/>
    </linearGradient>
  </defs>

  <!-- tank shell -->
  <rect x="${tankX}" y="${tankY}" width="${tankW}" height="${tankH}"
        fill="none" stroke="hsl(220, 15%, 35%)" stroke-width="3" rx="4"/>

  <!-- water -->
  <rect x="${tankX + 2}" y="${waterY}" width="${tankW - 4}" height="${waterH - 2}"
        fill="url(#water)"/>

  <!-- reference line (target level) -->
  ${params.reference != null ? `
    <line x1="${tankX - 10}" y1="${tankY + tankH - (params.reference / maxH) * tankH}"
          x2="${tankX + tankW + 10}" y2="${tankY + tankH - (params.reference / maxH) * tankH}"
          stroke="#E74C3C" stroke-width="2" stroke-dasharray="6,3"/>
  ` : ''}

  <!-- inflow arrow (top) -->
  <line x1="${tankX + tankW / 2}" y1="0" x2="${tankX + tankW / 2}" y2="${tankY - 4}"
        stroke="#27AE60" stroke-width="3"/>
  <polygon points="${tankX + tankW / 2 - 6},${tankY - 10}
                    ${tankX + tankW / 2 + 6},${tankY - 10}
                    ${tankX + tankW / 2},${tankY - 2}"
           fill="#27AE60"/>

  <!-- outflow arrow (bottom right) -->
  <line x1="${tankX + tankW}" y1="${tankY + tankH / 2}"
        x2="${W}" y2="${tankY + tankH / 2}"
        stroke="#E67E22" stroke-width="3"/>
  <polygon points="${W - 8},${tankY + tankH / 2 - 6}
                    ${W - 8},${tankY + tankH / 2 + 6}
                    ${W - 1},${tankY + tankH / 2}"
           fill="#E67E22"/>

  <!-- level readout -->
  <text x="${tankX - 20}" y="${waterY + 4}" text-anchor="end"
        font-family="ui-monospace, monospace" font-size="14" fill="hsl(220, 15%, 25%)">
    ${state.L.toFixed(2)} m
  </text>
</svg>`.trim();
}

// ============================================================================
// CommonJS / script-tag interop
//
// EXPERIMENTAL module. The script-tag / require() fallback is included for
// consistency with the rest of the repo's bower-style loaders, but the
// browser-only functions (startAnimLoop, drawMSD) will throw at runtime in
// Node — that's intentional, since they depend on canvas + rAF.
// ============================================================================

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    rk4,
    step,
    weighted,
    createHistoryBuffer,
    startAnimLoop,
    clamp,
    drawMSD,
    drawTankSVG,
    deriv,
    paramPresets,
    initialStates,
  };
}
