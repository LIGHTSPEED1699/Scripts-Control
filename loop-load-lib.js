/**
 * 4–20 mA Loop Load Calculator Library
 * Extracted from loop-load/index.astro
 *
 * Verifies that a DC supply can drive 20 mA through cable resistance,
 * receiver burden, and any barriers. Checks voltage margin at the transmitter.
 */

// ============================================================================
// Cable Resistance Data (Ω per 1000 ft / 1000 m)
// ============================================================================

const CABLE_RESISTANCE = {
  14: { ohmsPer1000ft: 2.525, ohmsPer1000m: 8.282 },
  16: { ohmsPer1000ft: 4.016, ohmsPer1000m: 13.17 },
  18: { ohmsPer1000ft: 6.385, ohmsPer1000m: 20.95 },
  20: { ohmsPer1000ft: 10.15, ohmsPer1000m: 33.31 },
  22: { ohmsPer1000ft: 16.14, ohmsPer1000m: 52.96 }
};

// ============================================================================
// Core Calculations
// ============================================================================

/**
 * Calculate loop load parameters
 * @param {Object} params - Input parameters
 * @returns {Object} Complete loop analysis
 */
function calculateLoopLoad(params) {
  const {
    supplyVoltage,      // V
    txMinVoltage,       // V (transmitter minimum operating voltage)
    burden,             // Ω
    awg,                // AWG wire gauge
    cableLength,        // ft or m
    lengthUnit,         // 'ft' or 'm'
    extraResistance,    // Ω (barriers, etc.)
    numReceivers        // Number of receivers in series (default 1)
  } = params;

  // Cable resistance
  const cableData = CABLE_RESISTANCE[awg];
  let cableResistance;
  if (lengthUnit === 'ft') {
    cableResistance = (cableData.ohmsPer1000ft / 1000) * cableLength * 2; // Two-way
  } else {
    cableResistance = (cableData.ohmsPer1000m / 1000) * cableLength * 2; // Two-way
  }

  // Total loop resistance
  const totalResistance = burden + cableResistance + extraResistance;

  // Current at 20 mA (maximum)
  const maxCurrent = 0.020; // A

  // Voltage drop at max current
  const voltageDrop = totalResistance * maxCurrent;

  // Voltage at transmitter at 20 mA
  const txVoltage = supplyVoltage - voltageDrop;

  // Voltage margin
  const voltageMargin = txVoltage - txMinVoltage;

  // Percentage of supply used
  const supplyUsedPercent = (voltageDrop / supplyVoltage) * 100;

  // Status
  const isValid = txVoltage >= txMinVoltage;
  const status = {
    isValid,
    message: isValid
      ? `OK — ${voltageMargin.toFixed(2)} V margin at 20 mA`
      : `FAIL — ${Math.abs(voltageMargin).toFixed(2)} V shortfall at 20 mA`,
    color: isValid ? '#22c55e' : '#ef4444'
  };

  // Maximum cable length for this configuration
  const maxLength = calculateMaxLength({
    supplyVoltage,
    txMinVoltage,
    burden,
    awg,
    lengthUnit,
    extraResistance
  });

  return {
    input: params,
    cable: {
      awg,
      resistancePerUnit: lengthUnit === 'ft'
        ? cableData.ohmsPer1000ft / 1000
        : cableData.ohmsPer1000m / 1000,
      length: cableLength,
      lengthUnit,
      totalResistance: cableResistance,
      maxLength
    },
    loop: {
      burden,
      extraResistance,
      totalResistance,
      maxCurrent: maxCurrent * 1000, // mA
      voltageDrop,
      txVoltage,
      voltageMargin,
      supplyUsedPercent
    },
    status
  };
}

/**
 * Calculate maximum cable length for given parameters
 * @param {Object} params
 * @returns {number} Maximum length in specified unit
 */
function calculateMaxLength(params) {
  const {
    supplyVoltage,
    txMinVoltage,
    burden,
    awg,
    lengthUnit,
    extraResistance
  } = params;

  const cableData = CABLE_RESISTANCE[awg];
  const resistancePerUnit = lengthUnit === 'ft'
    ? cableData.ohmsPer1000ft / 1000
    : cableData.ohmsPer1000m / 1000;

  // Available voltage for cable: supply - txMin - burdenDrop
  const availableVoltage = supplyVoltage - txMinVoltage - (burden + extraResistance) * 0.020;

  if (availableVoltage <= 0) return 0;

  // Max cable resistance = availableVoltage / current / 2 (two-way)
  const maxCableResistance = availableVoltage / 0.020;
  const maxLength = maxCableResistance / (resistancePerUnit * 2);

  return Math.max(0, maxLength);
}

/**
 * Calculate loop current from transmitter output (4-20 mA mapping)
 * @param {number} processValue - Process value (engineering units)
 * @param {number} rangeMin - Range minimum
 * @param {number} rangeMax - Range maximum
 * @returns {number} Loop current (mA)
 */
function processToCurrent(processValue, rangeMin, rangeMax) {
  const span = rangeMax - rangeMin;
  if (span === 0) return 4;
  const fraction = (processValue - rangeMin) / span;
  return 4 + fraction * 16; // 4-20 mA
}

/**
 * Calculate process value from loop current
 * @param {number} current - Loop current (mA)
 * @param {number} rangeMin - Range minimum
 * @param {number} rangeMax - Range maximum
 * @returns {number} Process value (engineering units)
 */
function currentToProcess(current, rangeMin, rangeMax) {
  const span = rangeMax - rangeMin;
  const fraction = (current - 4) / 16;
  return rangeMin + fraction * span;
}

/**
 * Calculate voltage across a resistor at given current
 * @param {number} resistance - Resistance (Ω)
 * @param {number} current - Current (mA)
 * @returns {number} Voltage (V)
 */
function voltageAcrossResistor(resistance, current) {
  return resistance * (current / 1000);
}

// ============================================================================
// Unit Conversions
// ============================================================================

/**
 * Convert between AWG and metric wire sizes
 * @param {number} awg - AWG gauge
 * @returns {number} Diameter in mm
 */
function awgToMm(awg) {
  // AWG diameter formula: d = 0.127 × 92^((36-AWG)/39) mm
  return 0.127 * Math.pow(92, (36 - awg) / 39);
}

/**
 * Convert feet to meters
 * @param {number} ft
 * @returns {number} meters
 */
function ftToM(ft) {
  return ft * 0.3048;
}

/**
 * Convert meters to feet
 * @param {number} m
 * @returns {number} feet
 */
function mToFt(m) {
  return m / 0.3048;
}

// ============================================================================
// Module Exports
// ============================================================================

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    CABLE_RESISTANCE,
    calculateLoopLoad,
    calculateMaxLength,
    processToCurrent,
    currentToProcess,
    voltageAcrossResistor,
    awgToMm,
    ftToM,
    mToFt
  };
}
