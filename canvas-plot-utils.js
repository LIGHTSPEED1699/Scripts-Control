/**
 * Canvas Plotting Utilities
 * Shared plotting functions for control system simulations
 * Extracted from PIDTuner.astro and MRAC.astro
 */

// ============================================================================
// Canvas Setup
// ============================================================================

/**
 * Setup high-DPI canvas
 * @param {HTMLCanvasElement} canvas - Canvas element
 * @param {number} heightPx - Desired height in CSS pixels
 * @returns {Object} {ctx, width, height} with proper scaling
 */
function setupCanvas(canvas, heightPx = 360) {
  const ctx = canvas.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const parent = canvas.parentElement;
  const rect = parent ? parent.getBoundingClientRect() : canvas.getBoundingClientRect();
  
  const style = parent ? getComputedStyle(parent) : getComputedStyle(canvas);
  const padLeft = parseFloat(style.paddingLeft) || 0;
  const padRight = parseFloat(style.paddingRight) || 0;
  const borderLeft = parseFloat(style.borderLeftWidth) || 0;
  const borderRight = parseFloat(style.borderRightWidth) || 0;
  const availW = Math.max(0, rect.width - padLeft - padRight - borderLeft - borderRight);
  
  canvas.width = Math.floor(availW * dpr);
  canvas.height = heightPx * dpr;
  canvas.style.width = Math.floor(availW) + 'px';
  canvas.style.height = heightPx + 'px';
  
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  
  return {
    ctx,
    width: Math.floor(availW),
    height: heightPx
  };
}

/**
 * Check if dark mode is active
 * @returns {boolean}
 */
function isDarkMode() {
  return document.documentElement.classList.contains('dark');
}

/**
 * Get color palette based on current theme
 * @returns {Object} {bg, grid, text, line}
 */
function getColors() {
  const dark = isDarkMode();
  return dark
    ? { grid: '#334155', text: '#94a3b8', bg: '#1e293b', line: '#0ea5e9' }
    : { grid: '#e2e8f0', text: '#475569', bg: '#f8fafc', line: '#0ea5e9' };
}

// ============================================================================
// Grid Drawing
// ============================================================================

/**
 * Draw plot grid
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} width - Canvas width
 * @param {number} height - Canvas height
 * @param {Object} pad - {top, right, bottom, left}
 */
function drawGrid(ctx, width, height, pad = { top: 20, right: 20, bottom: 40, left: 50 }) {
  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;
  
  ctx.strokeStyle = isDarkMode() ? '#334155' : '#e2e8f0';
  ctx.lineWidth = 1;
  
  // Vertical grid lines
  for (let i = 0; i <= 10; i++) {
    const x = pad.left + (i / 10) * plotW;
    ctx.beginPath();
    ctx.moveTo(x, pad.top);
    ctx.lineTo(x, height - pad.bottom);
    ctx.stroke();
  }
  
  // Horizontal grid lines
  for (let i = 0; i <= 6; i++) {
    const y = pad.top + (i / 6) * plotH;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
  }
}

// ============================================================================
// Axis Drawing
// ============================================================================

/**
 * Draw axes with labels
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} width - Canvas width
 * @param {number} height - Canvas height
 * @param {Object} pad - Padding
 * @param {number} tMax - Max time value
 * @param {number} yMin - Min y value
 * @param {number} yMax - Max y value
 */
function drawAxes(ctx, width, height, pad, tMax, yMin, yMax) {
  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;
  
  ctx.strokeStyle = isDarkMode() ? '#94a3b8' : '#475569';
  ctx.lineWidth = 1.5;
  
  // X axis
  ctx.beginPath();
  ctx.moveTo(pad.left, height - pad.bottom);
  ctx.lineTo(width - pad.right, height - pad.bottom);
  ctx.stroke();
  
  // Y axis
  ctx.beginPath();
  ctx.moveTo(pad.left, pad.top);
  ctx.lineTo(pad.left, height - pad.bottom);
  ctx.stroke();
  
  // Labels
  ctx.fillStyle = isDarkMode() ? '#94a3b8' : '#475569';
  ctx.font = '12px Inter, sans-serif';
  
  // X label
  ctx.textAlign = 'center';
  ctx.fillText('Time (s)', width / 2, height - 8);
  
  // Y ticks
  ctx.textAlign = 'right';
  const nTicks = 6;
  const range = yMax - yMin;
  for (let i = 0; i <= nTicks; i++) {
    const v = yMin + range * i / nTicks;
    const yPos = pad.top + plotH * (1 - i / nTicks);
    ctx.fillText(v.toFixed(1), pad.left - 6, yPos + 4);
  }
  
  // X ticks
  ctx.textAlign = 'center';
  for (let i = 0; i <= 5; i++) {
    const tVal = (i / 5) * tMax;
    const x = pad.left + plotW * i / 5;
    ctx.fillText(tVal.toFixed(0) + 's', x, height - pad.bottom + 18);
  }
}

// ============================================================================
// Trace Drawing
// ============================================================================

/**
 * Draw a trace on the canvas
 * @param {CanvasRenderingContext2D} ctx
 * @param {Object[]} history - Array of {t, y} points
 * @param {string} key - Property to plot ('y', 'u', 'r', etc.)
 * @param {string} color - Trace color
 * @param {number} width - Canvas width
 * @param {number} height - Canvas height
 * @param {number} tStart - Start time
 * @param {number} tEnd - End time
 * @param {number} yMin - Min y value
 * @param {number} yMax - Max y value
 * @param {number} lineWidth - Line width
 */
function drawTrace(ctx, history, key, color, width, height, 
  tStart, tEnd, yMin, yMax, lineWidth = 2) {
  
  const pad = { top: 20, right: 20, bottom: 40, left: 50 };
  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;
  
  const tRange = tEnd - tStart;
  const yRange = yMax - yMin || 1;
  
  const mapX = (t) => pad.left + plotW * (t - tStart) / tRange;
  const mapY = (y) => pad.top + plotH * (yMax - y) / yRange;
  
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  
  let started = false;
  for (const p of history) {
    if (p.t < tStart) continue;
    const x = mapX(p.t);
    const y = mapY(p[key]);
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else {
      ctx.lineTo(x, y);
    }
  }
  ctx.stroke();
}

/**
 * Draw horizontal dashed line (e.g., setpoint)
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} yValue - Y value to draw at
 * @param {number} width - Canvas width
 * @param {number} height - Canvas height
 * @param {string} color - Line color
 * @param {string} label - Optional label
 */
function drawDashedLine(ctx, yValue, width, height, color = '#f1f5f9', label = '') {
  const pad = { top: 20, right: 20, bottom: 40, left: 50 };
  const plotH = height - pad.top - pad.bottom;
  
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  
  // We need yMin/yMax for proper mapping - use defaults if not tracking
  const yMin = 0, yMax = 1.2; // These should be passed as parameters
  const yPos = pad.top + plotH * (yMax - yValue) / (yMax - yMin);
  
  ctx.moveTo(pad.left, yPos);
  ctx.lineTo(width - pad.right, yPos);
  ctx.stroke();
  ctx.setLineDash([]);
  
  if (label) {
    ctx.fillStyle = color;
    ctx.font = '11px Inter, sans-serif';
    ctx.fillText(label, width - pad.right - 80, yPos - 6);
  }
}

// ============================================================================
// Step Response Plot
// ============================================================================

/**
 * Draw complete step response plot
 * @param {HTMLCanvasElement} canvas - Canvas element
 * @param {Object} sim - Simulation result {t, y, u, sp}
 * @param {Object} options - Plot options
 */
function drawStepResponse(canvas, sim, options = {}) {
  const { ctx, width, height } = setupCanvas(canvas, options.height || 360);
  const pad = options.pad || { top: 20, right: 20, bottom: 40, left: 50 };
  
  // Background
  ctx.fillStyle = isDarkMode() ? '#1e293b' : '#f8fafc';
  ctx.fillRect(0, 0, width, height);
  
  // Grid
  drawGrid(ctx, width, height, pad);
  
  // Calculate scales
  const tMax = sim.t[sim.t.length - 1];
  const allVals = [...sim.y, ...sim.u, sim.sp || 1.0];
  const yMax = Math.max(...allVals) * 1.1;
  const yMin = Math.min(0, ...allVals) * 1.1;
  const rangeMin = Math.min(yMin, 0);
  const rangeMax = Math.max(yMax, 0.1);
  
  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;
  const xScale = plotW / tMax;
  const yScale = plotH / (rangeMax - rangeMin);
  
  const xc = (tVal) => pad.left + tVal * xScale;
  const yc = (v) => pad.top + (rangeMax - v) * yScale;
  
  // Setpoint line
  if (sim.sp !== undefined) {
    const spColor = isDarkMode() ? '#f1f5f9' : '#0f172a';
    ctx.strokeStyle = spColor;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(xc(0), yc(sim.sp));
    ctx.lineTo(xc(tMax), yc(sim.sp));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = spColor;
    ctx.font = '11px Inter, sans-serif';
    ctx.fillText('Setpoint = ' + sim.sp.toFixed(1), xc(tMax) - 80, yc(sim.sp) - 6);
  }
  
  // Process output
  ctx.strokeStyle = '#0ea5e9';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  const step = Math.max(1, Math.floor(sim.t.length / 2000));
  ctx.moveTo(xc(sim.t[0]), yc(sim.y[0]));
  for (let i = step; i < sim.t.length; i += step) {
    ctx.lineTo(xc(sim.t[i]), yc(sim.y[i]));
  }
  ctx.stroke();
  
  // Control output
  ctx.strokeStyle = '#f97316';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(xc(sim.t[0]), yc(sim.u[0]));
  for (let i = step; i < sim.t.length; i += step) {
    ctx.lineTo(xc(sim.t[i]), yc(sim.u[i]));
  }
  ctx.stroke();
  
  // Axes
  drawAxes(ctx, width, height, pad, tMax, rangeMin, rangeMax);
}

// ============================================================================
// MRAC Scope Plot
// ============================================================================

/**
 * Draw MRAC scope (sliding window)
 * @param {HTMLCanvasElement} canvas - Canvas element
 * @param {Object[]} history - MRAC history array
 * @param {Object} options - Plot options
 */
function drawMRACScope(canvas, history, options = {}) {
  const { ctx, width, height } = setupCanvas(canvas, options.height || 320);
  const pad = options.pad || { top: 20, right: 30, bottom: 30, left: 50 };
  
  if (history.length < 2) return;
  
  const tEnd = history[history.length - 1].t;
  const tStart = Math.max(0, tEnd - (options.window || 10));
  
  // Background
  ctx.fillStyle = isDarkMode() ? '#1e293b' : '#f8fafc';
  ctx.fillRect(0, 0, width, height);
  
  // Find y range
  let yMin = Infinity, yMax = -Infinity;
  for (const p of history) {
    if (p.t >= tStart) {
      yMin = Math.min(yMin, p.y, p.u, -1.5);
      yMax = Math.max(yMax, p.y, p.u, 1.5);
    }
  }
  const yRange = yMax - yMin || 1;
  
  // Grid
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.15)';
  ctx.lineWidth = 1;
  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;
  
  for (let i = 0; i <= 4; i++) {
    const y = pad.top + (plotH * i) / 4;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
  }
  
  // Labels
  ctx.fillStyle = isDarkMode() ? '#94a3b8' : '#475569';
  ctx.font = '11px monospace';
  ctx.textAlign = 'center';
  for (let i = 0; i <= 5; i++) {
    const t = tStart + (tEnd - tStart) * i / 5;
    const x = pad.left + (plotW * i) / 5;
    ctx.fillText(t.toFixed(1) + 's', Math.min(x, width - pad.right - 10), height - 8);
  }
  
  ctx.textAlign = 'right';
  for (let i = 0; i <= 4; i++) {
    const y = pad.top + (plotH * i) / 4;
    const val = yMax - (yRange * i) / 4;
    ctx.fillText(val.toFixed(1), pad.left - 8, y + 4);
  }
  
  // Map functions
  const mapX = (t) => pad.left + plotW * (t - tStart) / (tEnd - tStart);
  const mapY = (y) => pad.top + plotH * (yMax - y) / yRange;
  
  // Clip to plot area
  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.left, pad.top, plotW, plotH);
  ctx.clip();
  
  // Draw traces
  const drawTrace = (key, color, lw = 2) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = lw;
    ctx.beginPath();
    let started = false;
    for (const p of history) {
      if (p.t < tStart) continue;
      const x = mapX(p.t);
      const y = mapY(p[key]);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  };
  
  drawTrace('r', '#a855f7', 1.5);
  drawTrace('y', '#22c55e', 2);
  drawTrace('u', '#f97316', 1.5);
  
  ctx.restore();
}

// ============================================================================
// Module Exports
// ============================================================================

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    setupCanvas,
    isDarkMode,
    getColors,
    drawGrid,
    drawAxes,
    drawTrace,
    drawDashedLine,
    drawStepResponse,
    drawMRACScope
  };
}
