/**
 * Shared utilities for engineering tools.
 *
 * Extracted from the hongbinli-website public/tool-utils.js (v1.3.0).
 * Originally wrapped in an IIFE that registered a `ToolUtils` global; this
 * version is a plain ES module with named exports.
 *
 * The DOM-touching functions (`getColors`, `parseInput` markup feedback,
 * `observeDarkMode`) keep their browser dependencies — that's the whole
 * point of this module. Callers that need a Node-only build should shim
 * `document` and `MutationObserver`.
 */

const _darkColors  = { grid: '#334155', text: '#94a3b8', bg: '#1e293b', line: '#0ea5e9', line2: '#f97316', line3: '#22c55e', accent: '#0ea5e9' };
const _lightColors = { grid: '#e2e8f0', text: '#475569', bg: '#f8fafc', line: '#0ea5e9', line2: '#f97316', line3: '#22c55e', accent: '#0ea5e9' };

/**
 * Return dark/light color palette based on `<html class="dark">`.
 * @returns {{grid: string, text: string, bg: string, line: string, line2: string, line3: string, accent: string}}
 */
export function getColors() {
  return document.documentElement.classList.contains('dark') ? _darkColors : _lightColors;
}

/**
 * Format a number with fixed decimals and locale separators.
 * @param {number} n
 * @param {number} decimals
 * @returns {string} formatted number, or "—" for non-finite input
 */
export function formatNum(n, decimals) {
  if (typeof n !== 'number' || !isFinite(n)) return '—';
  return n.toLocaleString('en-US', {
    minimumFractionDigits: decimals,
    maximumFractionDigits: decimals
  });
}

/**
 * Set tab/button active/inactive styling using CSS variables.
 * @param {HTMLElement} tab
 * @param {boolean} active
 */
export function setActive(tab, active) {
  const accentBg = 'var(--color-accent)';
  const accentText = '#ffffff';
  const defBg = 'var(--color-tag-bg)';
  const defText = 'var(--color-tag-text)';
  tab.style.backgroundColor = active ? accentBg : defBg;
  tab.style.color = active ? accentText : defText;
}

/**
 * Safe 2^n - 1 for any bit depth (avoids 32-bit shift overflow).
 * @param {number} bits
 * @returns {number}
 */
export function maxCount(bits) {
  return Math.pow(2, bits) - 1;
}

/**
 * Parse a numeric input safely. Returns parsed value or null on error.
 * Optionally shows visual feedback on the element by toggling
 * the `input-invalid` class.
 *
 * @param {HTMLInputElement|string|number} el - DOM element with `.value`, or a raw string/number
 * @param {Object} [opts]
 * @param {boolean} [opts.markInvalid=false] - toggle `input-invalid` class on the element
 * @param {boolean} [opts.clamp=false] - clamp to [min, max] instead of returning null
 * @param {number} [opts.min] - inclusive lower bound
 * @param {number} [opts.max] - inclusive upper bound
 * @param {*} [opts.fallback] - value to return when invalid (defaults to null)
 * @returns {number|null|*} parsed number, clamped value, fallback, or null
 */
export function parseInput(el, opts = {}) {
  const raw = (el && typeof el === 'object' && 'value' in el) ? el.value : el;
  const v = parseFloat(raw);
  if (isNaN(v)) {
    if (opts.markInvalid && el?.classList) el.classList.add('input-invalid');
    return opts.fallback !== undefined ? opts.fallback : null;
  }
  if (opts.min !== undefined && v < opts.min) {
    if (opts.clamp) return opts.min;
    if (opts.markInvalid && el?.classList) el.classList.add('input-invalid');
    return opts.fallback !== undefined ? opts.fallback : null;
  }
  if (opts.max !== undefined && v > opts.max) {
    if (opts.clamp) return opts.max;
    if (opts.markInvalid && el?.classList) el.classList.add('input-invalid');
    return opts.fallback !== undefined ? opts.fallback : null;
  }
  if (opts.markInvalid && el?.classList) el.classList.remove('input-invalid');
  return v;
}

/**
 * Debounce helper for slider redraws.
 * @param {Function} fn
 * @param {number} ms
 * @returns {Function}
 */
export function debounce(fn, ms) {
  let t;
  return (...args) => {
    clearTimeout(t);
    t = setTimeout(() => fn(...args), ms);
  };
}

/**
 * Safe MutationObserver that auto-disconnects on page unload.
 * @param {MutationCallback} callback
 * @returns {MutationObserver}
 */
export function observeDarkMode(callback) {
  const observer = new MutationObserver(callback);
  observer.observe(document.documentElement, { attributes: true, attributeFilter: ['class'] });
  window.addEventListener('beforeunload', () => observer.disconnect());
  return observer;
}

// Default export: grouped namespace for callers that prefer a single import.
// Mirrors the old `ToolUtils` global shape for ergonomic migration.
export default {
  getColors,
  formatNum,
  setActive,
  maxCount,
  parseInput,
  debounce,
  observeDarkMode
};

// CommonJS / script-tag interop so this file also works in the existing
// Node/bower-style loaders that ship the rest of the repo.
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    getColors,
    formatNum,
    setActive,
    maxCount,
    parseInput,
    debounce,
    observeDarkMode
  };
}
