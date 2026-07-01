/**
 * Combustion Calculator Library
 * Extracted from combustion-calculator/index.astro
 *
 * Air/fuel ratio calculator for natural gas and hydrogen firing.
 * Computes stoichiometric and excess-air airflow, forecasted stack O₂ (dry basis),
 * and heat input (MMBTU/h).
 *
 * v1.3.0 update:
 *   - O₂ formula: closed-form (o2DryNG / o2DryH2) replaces the approximation.
 *     Algebraically identical for natural gas; materially different for hydrogen.
 *   - LHV: natural gas corrected from 1010 (closer to HHV) → 909 BTU/scf
 *     (pipeline natural gas LHV; matches the website).
 *   - Status: three-tier (red < 5%, green 5-25%, yellow > 25%) replaces the
 *     two-tier (red/green) bands; yellow warns of lean blowout / efficiency loss.
 *   - Public API keeps the verbose fuel keys (`natural_gas`, `hydrogen`) for
 *     backward compatibility; short aliases (`ng`, `h2`) are added.
 */

// ============================================================================
// Fuel Properties
// ============================================================================

const FUELS = {
  natural_gas: {
    name: 'Natural Gas (CH₄)',
    formula: 'CH₄',
    stoichAFR: 9.52,       // Volume air / volume fuel
    moleWeightFuel: 16.04, // g/mol
    lhv: 909,              // BTU/scf (lower heating value; matches website)
    heatValue: 909,        // Alias for backward compatibility
    co2PerFuel: 1.0,       // mol CO₂ / mol fuel
    h2oPerFuel: 2.0,       // mol H₂O / mol fuel
    o2PerFuel: 2.0         // mol O₂ / mol fuel
  },
  hydrogen: {
    name: 'Hydrogen (H₂)',
    formula: 'H₂',
    stoichAFR: 2.38,       // Volume air / volume fuel
    moleWeightFuel: 2.016, // g/mol
    lhv: 275,              // BTU/scf (LHV of H₂)
    heatValue: 275,        // Alias for backward compatibility
    co2PerFuel: 0.0,
    h2oPerFuel: 1.0,
    o2PerFuel: 0.5
  }
};

// Short aliases (website convention) — forward to the verbose keys
FUELS.ng = FUELS.natural_gas;
FUELS.h2 = FUELS.hydrogen;

// Air composition: 21% O₂, 79% N₂ by volume
const AIR_O2_FRACTION = 0.21;
const AIR_N2_FRACTION = 0.79;

// ============================================================================
// Exact Dry-Basis O₂ Formulas
// ============================================================================
//
// From complete combustion balance:
//
// Natural gas (CH₄ + 2O₂ → CO₂ + 2H₂O):
//   stoich air = 9.52 vol/vol, water in flue = 2 vol/vol
//   O₂_dry(x) = 2x / (8.52 + 9.52x) × 100
//
// Hydrogen (H₂ + 0.5O₂ → H₂O):
//   stoich air = 2.38 vol/vol, water in flue = 1 vol/vol
//   O₂_dry(x) = x / (3.76 + 4.76x) × 100
//
// x = excess air as a fraction (e.g. 20% excess → x = 0.20)

/**
 * Exact dry-basis O₂ for natural gas (%)
 * @param {number} x - Excess air as a fraction (0.20 = 20% excess)
 * @returns {number} Stack O₂ on a dry basis (%)
 */
function o2DryNG(x) {
  return (2 * x) / (8.52 + 9.52 * x) * 100;
}

/**
 * Exact dry-basis O₂ for hydrogen (%)
 * @param {number} x - Excess air as a fraction (0.20 = 20% excess)
 * @returns {number} Stack O₂ on a dry basis (%)
 */
function o2DryH2(x) {
  return x / (3.76 + 4.76 * x) * 100;
}

// ============================================================================
// Core Calculations
// ============================================================================

/**
 * Calculate combustion parameters
 * @param {string} fuelType - 'natural_gas' or 'hydrogen'
 * @param {number} fuelFlow - Fuel flow rate
 * @param {string} unit - 'scfh' or 'scmh'
 * @param {number} excessAirPercent - Excess air percentage (0-100)
 * @returns {Object} Complete combustion results
 */
function calculateCombustion(fuelType, fuelFlow, unit, excessAirPercent) {
  const fuel = FUELS[fuelType];
  if (!fuel) throw new Error(`Unknown fuel type: ${fuelType}`);

  // Convert to scfh if needed
  let fuelFlowScfh = fuelFlow;
  if (unit === 'scmh') {
    // 1 scm = 35.3147 scf
    fuelFlowScfh = fuelFlow * 35.3147;
  }

  // Stoichiometric air
  const stoichAirScfh = fuelFlowScfh * fuel.stoichAFR;

  // Total air with excess
  const excessFraction = 1 + excessAirPercent / 100;
  const totalAirScfh = stoichAirScfh * excessFraction;
  const totalAirScmh = totalAirScfh / 35.3147;

  // Actual AFR
  const actualAFR = fuel.stoichAFR * excessFraction;

  // Stack O₂ calculation (dry basis) — use the exact closed-form formula
  // for whichever fuel is selected. The repo's previous approximation
  // (excessO2 / dryFlueGas × 100) was algebraically identical for natural gas
  // but materially wrong for hydrogen.
  const x = excessAirPercent / 100;
  const stackO2 = fuelType === 'hydrogen' || fuelType === 'h2'
    ? o2DryH2(x)
    : o2DryNG(x);

  // Heat input
  const heatInput = (fuelFlowScfh * fuel.heatValue) / 1e6; // MMBTU/h

  // Three-tier status (matches the website):
  //   < 5%  : red    — insufficient air, incomplete combustion risk
  //   5-25% : green  — normal operation
  //   > 25% : yellow — lean, efficiency loss / blowout risk
  let status;
  if (excessAirPercent < 5) {
    status = {
      level: 'unsafe',
      isNormal: false,
      message: 'Unsafe — incomplete combustion',
      color: '#ef4444'
    };
  } else if (excessAirPercent <= 25) {
    status = {
      level: 'normal',
      isNormal: true,
      message: 'Normal operation',
      color: '#22c55e'
    };
  } else {
    status = {
      level: 'wasteful',
      isNormal: false,
      message: 'Wasteful — lean blowout risk',
      color: '#eab308'
    };
  }

  return {
    fuel: {
      type: fuelType,
      name: fuel.name,
      flowRate: fuelFlow,
      unit,
      flowScfh: fuelFlowScfh
    },
    stoichAFR: fuel.stoichAFR,
    actualAFR,
    stoichAir: {
      scfh: stoichAirScfh,
      scmh: stoichAirScfh / 35.3147
    },
    totalAir: {
      scfh: totalAirScfh,
      scmh: totalAirScmh
    },
    stackO2: {
      percent: stackO2,
      dryBasis: true
    },
    heatInput: {
      mmbtuPerHour: heatInput,
      btuPerHour: heatInput * 1e6
    },
    airComposition: {
      stoichPercent: (stoichAirScfh / totalAirScfh * 100),
      excessPercent: (excessAirPercent)
    },
    status
  };
}

// ============================================================================
// Reverse Calculations
// ============================================================================

/**
 * Calculate excess air from measured stack O₂
 * @param {number} stackO2 - Measured O₂ in flue gas (dry basis, %)
 * @param {string} fuelType - 'natural_gas' or 'hydrogen' (or 'ng'/'h2' alias)
 * @returns {number} Excess air percentage
 */
function excessAirFromO2(stackO2, fuelType) {
  const y = stackO2; // already in % units
  let x;
  if (fuelType === 'hydrogen' || fuelType === 'h2') {
    // Inverse of o2DryH2: x = 3.76y / (100 - 4.76y)
    const denom = 100 - 4.76 * y;
    if (denom <= 0) return Infinity;
    x = (3.76 * y) / denom;
  } else {
    // Inverse of o2DryNG: x = 8.52y / (200 - 9.52y)
    const denom = 200 - 9.52 * y;
    if (denom <= 0) return Infinity;
    x = (8.52 * y) / denom;
  }
  return Math.max(0, x * 100);
}

/**
 * Calculate required fuel flow for target heat input
 * @param {number} targetHeatInput - Target heat input (MMBTU/h)
 * @param {string} fuelType - 'natural_gas' or 'hydrogen'
 * @returns {number} Required fuel flow (scfh)
 */
function fuelFlowForHeatInput(targetHeatInput, fuelType) {
  const fuel = FUELS[fuelType];
  return (targetHeatInput * 1e6) / fuel.heatValue;
}

// ============================================================================
// Unit Conversions
// ============================================================================

const CONVERSIONS = {
  scfh_to_scmh: 1 / 35.3147,
  scmh_to_scfh: 35.3147,
  scfh_to_m3h: 0.0283168,
  m3h_to_scfh: 35.3147
};

/**
 * Convert between flow units
 * @param {number} value - Input value
 * @param {string} fromUnit - Input unit
 * @param {string} toUnit - Output unit
 * @returns {number} Converted value
 */
function convertFlow(value, fromUnit, toUnit) {
  const key = `${fromUnit}_to_${toUnit}`;
  if (CONVERSIONS[key]) return value * CONVERSIONS[key];
  // Same unit
  if (fromUnit === toUnit) return value;
  throw new Error(`Unknown conversion: ${fromUnit} to ${toUnit}`);
}

// ============================================================================
// Module Exports
// ============================================================================

// ESM exports — mirror the CJS block below for `import { ... }` consumers.
export {
  FUELS,
  AIR_O2_FRACTION,
  AIR_N2_FRACTION,
  o2DryNG,
  o2DryH2,
  calculateCombustion,
  excessAirFromO2,
  fuelFlowForHeatInput,
  convertFlow,
  CONVERSIONS
};

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    FUELS,
    AIR_O2_FRACTION,
    AIR_N2_FRACTION,
    o2DryNG,
    o2DryH2,
    calculateCombustion,
    excessAirFromO2,
    fuelFlowForHeatInput,
    convertFlow,
    CONVERSIONS
  };
}
