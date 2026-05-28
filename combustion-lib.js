/**
 * Combustion Calculator Library
 * Extracted from combustion-calculator/index.astro
 *
 * Air/fuel ratio calculator for natural gas and hydrogen firing.
 * Computes stoichiometric and excess-air airflow, forecasted stack O₂ (dry basis),
 * and heat input (MMBTU/h).
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
    heatValue: 1010,       // BTU/scf
    co2PerFuel: 1.0,       // mol CO₂ / mol fuel
    h2oPerFuel: 2.0,       // mol H₂O / mol fuel
    o2PerFuel: 2.0         // mol O₂ / mol fuel
  },
  hydrogen: {
    name: 'Hydrogen (H₂)',
    formula: 'H₂',
    stoichAFR: 2.38,       // Volume air / volume fuel
    moleWeightFuel: 2.016, // g/mol
    heatValue: 325,        // BTU/scf
    co2PerFuel: 0.0,
    h2oPerFuel: 1.0,
    o2PerFuel: 0.5
  }
};

// Air composition: 21% O₂, 79% N₂ by volume
const AIR_O2_FRACTION = 0.21;
const AIR_N2_FRACTION = 0.79;

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

  // Stack O₂ calculation (dry basis)
  // O₂ in flue gas = (excess O₂) / (total flue gas - water)
  const excessO2 = stoichAirScfh * AIR_O2_FRACTION * (excessAirPercent / 100);
  const totalFlueGas = fuelFlowScfh + totalAirScfh;
  const waterInFlue = fuelFlowScfh * fuel.h2oPerFuel;
  const dryFlueGas = totalFlueGas - waterInFlue;
  const stackO2 = (excessO2 / dryFlueGas) * 100;

  // Heat input
  const heatInput = (fuelFlowScfh * fuel.heatValue) / 1e6; // MMBTU/h

  // Status check
  const isNormal = excessAirPercent >= 10 && excessAirPercent <= 25;
  const status = {
    isNormal,
    message: isNormal ? 'Normal operation' : excessAirPercent < 10 ? 'Insufficient air - incomplete combustion risk' : 'Excess air - efficiency loss',
    color: isNormal ? '#22c55e' : '#ef4444'
  };

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
 * @param {string} fuelType - 'natural_gas' or 'hydrogen'
 * @returns {number} Excess air percentage
 */
function excessAirFromO2(stackO2, fuelType) {
  const fuel = FUELS[fuelType];
  // Approximate: %EA ≈ %O₂ / (21 - %O₂) × 100 (for natural gas)
  // More accurate calculation:
  const o2Fraction = stackO2 / 100;
  const excessAir = (o2Fraction * (1 + fuel.stoichAFR)) / (AIR_O2_FRACTION - o2Fraction) * 100;
  return Math.max(0, excessAir);
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

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    FUELS,
    calculateCombustion,
    excessAirFromO2,
    fuelFlowForHeatInput,
    convertFlow,
    CONVERSIONS
  };
}
