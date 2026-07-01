/**
 * Linear-algebra and control primitives for small (n ≤ ~8) dense
 * systems. Used by the inverted pendulum simulator and any other
 * tool that needs LQR, pole placement, Lyapunov solves, or
 * matrix arithmetic.
 *
 * Extracted from hongbinli-website public/lqr.js (v1.3.0).
 * Originally a browser script that assigned helpers to window.*;
 * this version is a plain ES module with named exports and a
 * CommonJS fallback.
 *
 * Root-finding dependencies (cubic, quartic, characteristic
 * polynomial) are imported from pole-roots-lib.js. The two
 * modules together give you the full numerical-algorithms surface
 * area needed for state-space controller design.
 *
 * ──────────────────────────────────────────────────────────────────
 */

/**
 * Continuous-time LQR solver for small (n ≤ ~8) state, m-input
 * systems. Used by the inverted pendulum simulator to turn the
 * user's Q, R weighting matrices into a state-feedback gain
 * K = R^{-1} B^T P, where P solves the algebraic Riccati
 * equation (ARE)
 *
 *     A^T P + P A − P B R^{-1} B^T P + Q = 0
 *
 * Algorithm: Kleinman iteration. Initialize P_0 = Q and iterate
 *
 *     P_{k+1} solves the Lyapunov equation
 *         A_k^T P_{k+1} + P_{k+1} A_k = −D_k
 *     where A_k = A − B R^{-1} B^T P_k
 *           D_k = Q + P_k B R^{-1} B^T P_k
 *
 * Converges monotonically from above (P_0 ≥ P_1 ≥ P_2 ≥ …
 * ≥ P) when the closed-loop is stabilizable and the ARE has a
 * unique positive-semidefinite solution. For our 4-state
 * cart-pole this converges in 15-40 iterations to 1e-10
 * residual in well under 1 ms.
 *
 * The Lyapunov solve uses the vectorized form
 *
 *     ((I ⊗ A^T) + (A^T ⊗ I)) vec(X) = vec(C)
 *
 * which is a 16×16 linear system for a 4×4 Lyapunov equation.
 * Solved by Gaussian elimination with partial pivoting — fine
 * for the small n we care about. The matrix is well
 * conditioned when A is Hurwitz, which Kleinman's iteration
 * guarantees by construction (A_k is the closed-loop
 * dynamics at step k, and P_0 = Q stabilizes the pair in
 * practice for our plants).
 *
 * References:
 *   - Kleinman (1968), "On an iterative technique for
 *     Riccati equation computations", IEEE Trans. Auto. Ctrl.
 *   - Lancaster & Rodman, "Algebraic Riccati Equations"
 *     (Oxford, 1995), Ch. 5 for the Lyapunov–vectorization
 *     link.
 */

import { polynomialRoots } from './pole-roots-lib.js';

/**
 * Solve the continuous-time Lyapunov equation
 *   A^T X + X A = C
 * for symmetric X, given symmetric C and square A.
 *
 * Returns X. Throws if A is not Hurwitz (no unique
 * positive-semidefinite solution).
 *
 * Implementation: Smith's reduced form. The standard
 * vectorization gives an n²×n² system that is rank-deficient
 * for symmetric X (n² unknowns, but only n(n+1)/2 unique
 * entries). Instead we keep only the lower-triangular
 * entries of X (10 unknowns for n=4) and write a reduced
 * n(n+1)/2 × n(n+1)/2 system. The system is invertible iff
 * A is Hurwitz, which is exactly the case we want to
 * detect — if the system is singular, A has eigenvalues on
 * the imaginary axis and there's no unique solution.
 *
 * The reduced system: enumerate the pairs (i, j) with i ≤ j;
 * for each pair, write out the (i, j) row of A^T X + X A
 * using X's symmetric structure. The result is a 10×10
 * linear system (for n=4) that we solve with Gaussian
 * elimination.
 *
 * @param {number[][]} A - square matrix (n×n)
 * @param {number[][]} C - square matrix (n×n, symmetric)
 * @returns {number[][]} X (n×n, symmetric)
 */
export function solveLyapunov(A, C) {
  const n = A.length;
  if (C.length !== n || A.some((row) => row.length !== n)) {
    throw new Error('solveLyapunov: A and C must be square n×n matrices');
  }
  if (C.some((row, i) => row.length !== n)) {
    throw new Error('solveLyapunov: C must be n×n');
  }
  // Map (i, j) with i ≤ j to a flat index 0..M-1 where
  // M = n(n+1)/2. Use row-major: index = i*n - i*(i-1)/2 + (j - i).
  // This walks the lower triangle row by row.
  function svecIndex(i, j) {
    if (i > j) [i, j] = [j, i];
    return i * n - (i * (i - 1)) / 2 + (j - i);
  }
  const M = (n * (n + 1)) / 2;
  // Build reduced L_red (M×M) and rhs_red (M).
  // For each row (i, j) with i ≤ j, the (i, j) entry of
  // A^T X + X A is:
  //   Σ_k A[k][i] X[k][j] + Σ_k X[i][k] A[k][j]  (no transpose needed)
  //   = Σ_k A[k][i] X[k][j] + Σ_k A[k][j] X[i][k]   (rearranged)
  // Each term X[a][b] appears with a ≤ b (by symmetry).
  const Lred = [];
  for (let r = 0; r < M; r++) Lred.push(new Float64Array(M));
  const rhs = new Float64Array(M);
  for (let i = 0; i < n; i++) {
    for (let j = i; j < n; j++) {
      const row = svecIndex(i, j);
      // RHS: C[i][j].
      rhs[row] = C[i][j];
      // First sum: Σ_k A[k][i] X[k][j]. When k > j, X[k][j] =
      // X[j][k], so we use svecIndex(j, k). When k ≤ j,
      // svecIndex(k, j).
      for (let k = 0; k < n; k++) {
        const col1 = svecIndex(k, j);
        Lred[row][col1] += A[k][i];
      }
      // Second sum: Σ_k A[k][j] X[i][k]. When i > k, X[i][k] =
      // X[k][i], so svecIndex(k, i). When i ≤ k, svecIndex(i, k).
      for (let k = 0; k < n; k++) {
        const col2 = svecIndex(i, k);
        Lred[row][col2] += A[k][j];
      }
    }
  }
  // Solve Lred * svecX = rhs.
  const svecX = solveLinearSystem(Lred, rhs);
  // Reconstruct X from svecX.
  const X = [];
  for (let i = 0; i < n; i++) X.push(new Float64Array(n));
  for (let i = 0; i < n; i++) {
    for (let j = i; j < n; j++) {
      const v = svecX[svecIndex(i, j)];
      X[i][j] = v;
      X[j][i] = v;
    }
  }
  return X;
}

/**
 * Solve a dense linear system L x = rhs via Gaussian
 * elimination with partial pivoting. L is N×N, rhs is N×1.
 * Returns x as Float64Array(N). Throws on singular system.
 *
 * @param {number[][]} L
 * @param {Float64Array|number[]} rhs
 * @returns {Float64Array}
 */
export function solveLinearSystem(L, rhs) {
  const N = L.length;
  // Augment: copy L into a working matrix, append rhs as
  // column N.
  const M = [];
  for (let r = 0; r < N; r++) {
    const row = new Float64Array(N + 1);
    for (let c = 0; c < N; c++) row[c] = L[r][c];
    row[N] = rhs[r];
    M.push(row);
  }
  // Forward elimination with partial pivoting.
  for (let k = 0; k < N; k++) {
    // Find the row with the largest |M[i][k]| for i >= k.
    let pivRow = k;
    let pivAbs = Math.abs(M[k][k]);
    for (let i = k + 1; i < N; i++) {
      const v = Math.abs(M[i][k]);
      if (v > pivAbs) { pivAbs = v; pivRow = i; }
    }
    if (pivAbs < 1e-14) {
      throw new Error('solveLinearSystem: singular matrix (pivot < 1e-14 at row ' + k + ')');
    }
    if (pivRow !== k) {
      const tmp = M[k]; M[k] = M[pivRow]; M[pivRow] = tmp;
    }
    // Eliminate below.
    const piv = M[k][k];
    for (let i = k + 1; i < N; i++) {
      const f = M[i][k] / piv;
      if (f === 0) continue;
      for (let c = k; c <= N; c++) M[i][c] -= f * M[k][c];
    }
  }
  // Back-substitution.
  const x = new Float64Array(N);
  for (let i = N - 1; i >= 0; i--) {
    let s = M[i][N];
    for (let j = i + 1; j < N; j++) s -= M[i][j] * x[j];
    x[i] = s / M[i][i];
  }
  return x;
}

/**
 * Matrix operations used by the Kleinman iteration. Kept
 * here (not in a separate util module) because they're
 * specific to small dense matrices and the ARE use case.
 */

export function matAdd(A, B) {
  const n = A.length;
  const C = [];
  for (let i = 0; i < n; i++) {
    const row = new Float64Array(n);
    for (let j = 0; j < n; j++) row[j] = A[i][j] + B[i][j];
    C.push(row);
  }
  return C;
}

export function matSub(A, B) {
  const n = A.length;
  const C = [];
  for (let i = 0; i < n; i++) {
    const row = new Float64Array(n);
    for (let j = 0; j < n; j++) row[j] = A[i][j] - B[i][j];
    C.push(row);
  }
  return C;
}

export function matScale(A, s) {
  const n = A.length;
  const C = [];
  for (let i = 0; i < n; i++) {
    const row = new Float64Array(n);
    for (let j = 0; j < n; j++) row[j] = A[i][j] * s;
    C.push(row);
  }
  return C;
}

/**
 * C = A · B for general rectangular matrices.
 *   A is m × k
 *   B is k × p
 *   C is m × p
 * Works for square too; the existing call sites in solveLQR
 * (B R^{-1}, B^T P, etc.) pass 4×1, 1×1, 4×4 — all sizes
 * supported.
 */
export function matMul(A, B) {
  const m = A.length;
  const k = A[0].length;
  if (B.length !== k) {
    throw new Error('matMul: inner dim mismatch (' + k + ' vs ' + B.length + ')');
  }
  const p = B[0].length;
  const C = [];
  for (let i = 0; i < m; i++) {
    const row = new Float64Array(p);
    for (let kk = 0; kk < k; kk++) {
      const aik = A[i][kk];
      if (aik === 0) continue;
      const Bk = B[kk];
      for (let j = 0; j < p; j++) row[j] += aik * Bk[j];
    }
    C.push(row);
  }
  return C;
}

/**
 * Frobenius norm ‖A‖_F = sqrt(sum |A_ij|²).
 */
export function matFrobNorm(A) {
  let s = 0;
  for (let i = 0; i < A.length; i++) {
    for (let j = 0; j < A[i].length; j++) {
      const v = A[i][j];
      s += v * v;
    }
  }
  return Math.sqrt(s);
}

/**
 * Compute a stabilizing state-feedback gain K for the
 * single-input plant (A, B) using Ackermann's pole-placement
 * formula. The desired closed-loop poles are passed as
 * `desiredPoles` (an array of n complex numbers; real
 * implementations pass n real poles or 2 real + 1
 * complex-conjugate pair, etc.).
 *
 * Ackermann (single-input):
 *   K = e_n^T · C^{-1} · α(A)
 * where C = [B, AB, A^2 B, ..., A^{n-1} B] is the
 * controllability matrix, α(s) = ∏ (s − p_i) is the desired
 * characteristic polynomial, and α(A) is α evaluated at A
 * (Cayley-Hamilton).
 *
 * Returns a 1×n row vector K.
 *
 * Exported so the page can call it directly for the
 * Ackermann "design by closed-loop eigenvalues" UX (the
 * user picks 4 desired pole locations, we compute K). The
 * LQR solver also uses this internally to find a
 * stabilizing K_init for the Kleinman iteration.
 *
 * Throws if the plant is uncontrollable (singular
 * controllability matrix) or if any desired pole is at
 * the same location as an uncontrollable mode.
 *
 * @param {number[][]} A
 * @param {number[][]} B - column vector (n×1)
 * @param {(number|{real:number, imag:number})[]} desiredPoles
 * @returns {number[][]} K as a 1×n row vector
 */
export function ackerStabilizing(A, B, desiredPoles) {
  const n = A.length;
  if (B.length !== n || B[0].length !== 1) {
    throw new Error('ackerStabilizing: only single-input plants supported');
  }
  if (desiredPoles.length !== n) {
    throw new Error('ackerStabilizing: need exactly n desired poles');
  }
  // Build controllability matrix C = [B, AB, A^2 B, A^3 B].
  const C = [];
  for (let i = 0; i < n; i++) C.push(new Float64Array(n));
  let Ab = B;  // current A^k B
  for (let k = 0; k < n; k++) {
    for (let i = 0; i < n; i++) C[i][k] = Ab[i][0];
    // Ab := A · Ab
    const next = new Array(n);
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let j = 0; j < n; j++) s += A[i][j] * Ab[j][0];
      next[i] = s;
    }
    Ab = next.map((v) => [v]);
  }
  // C^{-1} via Gaussian elimination.
  const Cinv = invertMatrix(C);
  // α(s) coefficients: α(s) = s^n + c_{n-1} s^{n-1} + ... + c_0.
  const coeffs = polyFromRoots(desiredPoles);
  // Compute A^k for k = 0..n-1.
  const Apow = [identity(n)];
  for (let k = 1; k < n; k++) Apow.push(matMul(Apow[k - 1], A));
  // α(A) = sum_{k=0..n} c_k A^k, where c_n = 1 (the polynomial
  // is monic) and coeffs[k] = c_k for k = 0..n-1. We
  // include the leading A^n term here — the Ackermann
  // formula uses the symbolic α(A), not the Cayley-Hamilton
  // reduction. The leading term matters: with c_0 = 1 and
  // c_1 = ... = c_{n-1} = 0 (e.g. for n=1, α(s) = s + 1,
  // so α(A) = A + I — the I is the c_0 term and A is the
  // leading A^1 term).
  let alphaA = matScale(Apow[0], coeffs[0]);
  for (let k = 1; k < n; k++) {
    alphaA = matAdd(alphaA, matScale(Apow[k], coeffs[k]));
  }
  // Add the leading A^n term (coefficient 1). Build A^n =
  // A * A^{n-1} by matMul.
  const An = matMul(Apow[n - 1], A);
  alphaA = matAdd(alphaA, An);
  // K = e_n^T · C^{-1} · α(A). Equivalently, K is the last
  // row of M = C^{-1} · α(A).
  const M = matMul(Cinv, alphaA);
  const K = [new Float64Array(n)];
  for (let j = 0; j < n; j++) K[0][j] = M[n - 1][j];
  return K;
}

/**
 * Given a list of n roots (real or complex), return the
 * coefficients [c_0, c_1, ..., c_{n-1}] such that
 * α(s) = s^n + c_{n-1} s^{n-1} + ... + c_1 s + c_0.
 * Complex roots must come in conjugate pairs for the
 * coefficients to be real.
 *
 * Builds the polynomial incrementally: start with 1,
 * then multiply by (s − p_i) for i=0..n-1.
 *
 * @param {(number|{real:number, imag:number})[]} roots
 * @returns {number[]}
 */
export function polyFromRoots(roots) {
  // coeffs[k] = coefficient of s^k
  let coeffs = [1];
  for (const r of roots) {
    // Detect a real root: either a plain number, or a complex
    // object whose imaginary part is (numerically) zero.
    const isReal = typeof r === 'number' ||
      (typeof r === 'object' && r !== null &&
       (r.imag === undefined || r.imag === null ||
        Math.abs(r.imag) < 1e-12));
    if (isReal) {
      const re = typeof r === 'number' ? r : r.real;
      const next = new Array(coeffs.length + 1).fill(0);
      for (let k = 0; k < coeffs.length; k++) {
        next[k + 1] += coeffs[k];             // s · c_k s^k
        next[k]     -= coeffs[k] * re;        // −r · c_k s^k
      }
      coeffs = next;
    } else {
      // Complex root: take it AND its conjugate as a real
      // quadratic (s − r)(s − r̄) = s² − 2 Re(r) s + |r|².
      // [1, -2a, a²+b²] (constant, s, s² coefficients).
      const a = r.real;
      const b = r.imag;
      const q = [a * a + b * b, -2 * a, 1];
      const next = new Array(coeffs.length + 2).fill(0);
      for (let i = 0; i < q.length; i++) {
        for (let k = 0; k < coeffs.length; k++) {
          next[i + k] += q[i] * coeffs[k];
        }
      }
      coeffs = next;
    }
  }
  // coeffs is [c_0, c_1, ..., c_n]. Trim to [c_0..c_{n-1}]
  // (drop the leading 1).
  return coeffs.slice(0, coeffs.length - 1);
}

/**
 * n×n identity matrix.
 * @param {number} n
 * @returns {number[][]}
 */
export function identity(n) {
  const I = [];
  for (let i = 0; i < n; i++) {
    const row = new Float64Array(n);
    row[i] = 1;
    I.push(row);
  }
  return I;
}

/**
 * Compute P solving the continuous-time algebraic Riccati
 * equation A^T P + P A − P B R^{-1} B^T P + Q = 0 via
 * Kleinman iteration, and the corresponding LQR gain
 * K = R^{-1} B^T P.
 *
 * Inputs:
 *   A  — n×n state matrix
 *   B  — n×m input matrix
 *   Q  — n×n state-weighting matrix (symmetric positive
 *        semidefinite in the standard LQR formulation; we
 *        only require symmetry here)
 *   R  — m×m input-weighting matrix (symmetric positive
 *        definite; for the single-input case it's a 1×1
 *        scalar). We branch on m and use either the
 *        closed-form R inverse (m=1) or a small linear
 *        solve (m>1).
 *
 * Returns { P, K, iterations, residual }.
 *
 * Iteration (Kleinman, 1968):
 *   Pick any stabilizing K_init such that A_cl0 = A − B K_init
 *   is Hurwitz. The plain Kleinman starting from P_0 = Q
 *   requires this; for plants where the open-loop A is
 *   unstable and Q doesn't stabilize it on its own, the
 *   first Lyapunov solve would be singular. (Our cart-pole
 *   is exactly this case: A has eigenvalue +4.65.)
 *
 *   To get a stabilizing K_init for the single-input case we
 *   use Ackermann's formula to place the closed-loop poles
 *   at (default) {−2, −3, −4, −5}. This is a one-shot
 *   calculation; the iteration quickly forgets this initial
 *   gain and converges to the LQR-optimal P in 20-50 steps.
 *
 *   With K_init in hand, set
 *     P_0 solves (A − B K_init)^T P_0 + P_0 (A − B K_init)
 *                = −(Q + K_init^T R K_init)
 *   Then iterate
 *     A_k  = A − B R^{-1} B^T P_k
 *     D_k  = Q + P_k B R^{-1} B^T P_k
 *     P_{k+1} solves  A_k^T P_{k+1} + P_{k+1} A_k = −D_k
 *   until ‖P_{k+1} − P_k‖_F < tol.
 *
 *   Returns K = R^{-1} B^T P (so the stabilizing law is
 *   u = −K x). The simulator's SF branch reads
 *   −(Kx·s.p + Ktheta·s.theta + ...) which matches.
 *
 * For multi-input plants (m > 1) the Ackermann fallback is
 * not implemented; the caller should pass a stabilizing
 * K_init via opts.KInit, or the function will throw.
 */
export function solveLQR(A, B, Q, R, opts = {}) {
  const tol = opts.tol ?? 1e-9;
  const maxIter = opts.maxIter ?? 200;
  const n = A.length;
  const m = B[0].length;
  // Sanity-check shapes.
  if (B.length !== n) throw new Error('solveLQR: A and B row counts differ');
  if (Q.length !== n || Q.some((r) => r.length !== n)) {
    throw new Error('solveLQR: Q must be n×n');
  }
  if (R.length !== m || R.some((r) => r.length !== m)) {
    throw new Error('solveLQR: R must be m×m');
  }
  // Compute R^{-1}.
  let Rinv;
  if (m === 1) {
    Rinv = [[1 / R[0][0]]];
  } else {
    // m×m inverse via Gaussian elimination on augmented
    // matrix. For m=1 we already handled, but support the
    // general case so this works for multi-input plants.
    Rinv = invertMatrix(R);
  }
  // Initialize P_0 from a stabilizing K_init. For
  // single-input plants we use Ackermann. For multi-input,
  // the caller must pass opts.KInit.
  let P;
  const BRinv = matMul(B, Rinv);
  if (opts.P0) {
    P = opts.P0.map((r) => Float64Array.from(r));
  } else {
    let Kinit;
    if (opts.KInit) {
      Kinit = opts.KInit;
    } else if (m === 1) {
      // Default Ackermann poles: all at −1, −2, ..., −n
      // (so a 1-state plant gets −1, a 2-state plant gets
      // {−1, −2}, a 4-state gets {−1, −2, −3, −4}). These
      // are not magic — any set of stable poles will do;
      // the iteration quickly forgets the initial gain.
      const defaultPoles = [];
      for (let k = 1; k <= n; k++) defaultPoles.push(-k);
      const desiredPoles = opts.initPoles ?? defaultPoles;
      Kinit = ackerStabilizing(A, B, desiredPoles);
    } else {
      throw new Error('solveLQR: multi-input plants require opts.KInit');
    }
    // Compute A_cl0 = A − B Kinit (Hurwitz by construction).
    const Acl0 = matSub(A, matMul(B, Kinit));
    // Compute P_0 from the ARE-residual-at-Kinit Lyapunov
    // equation. This P_0 is positive definite (the right-hand
    // side Q + Kinit^T R Kinit is positive definite) and
    // gives a starting point for Kleinman where A_cl0 is
    // Hurwitz, so the first iteration's Lyapunov is well
    // conditioned.
    const KTK = matMul(transpose(Kinit), matMul(R, Kinit));
    const rhs0 = matAdd(Q, KTK);
    const negRhs0 = matScale(rhs0, -1);
    P = solveLyapunov(Acl0, negRhs0);
  }
  for (let iter = 0; iter < maxIter; iter++) {
    // A_k = A − B R^{-1} B^T P_k
    //     = A − (B R^{-1}) (B^T P_k)
    //     = A − (B R^{-1}) ((P_k^T B)^T)  (symmetric P_k)
    // For symmetric P_k: B^T P_k is just matMul(B^T, P_k).
    const BTP = matMul(transpose(B), P);
    const BRinvBTP = matMul(BRinv, BTP);
    const Ak = matSub(A, BRinvBTP);
    // D_k = Q + P_k B R^{-1} B^T P_k = Q + (P_k B R^{-1}) (B^T P_k)
    //     = Q + (P_k (B R^{-1})) (B^T P_k)
    const PBRinv = matMul(P, BRinv);
    const PBRinvBTP = matMul(PBRinv, BTP);
    const Dk = matAdd(Q, PBRinvBTP);
    // Solve A_k^T P_{k+1} + P_{k+1} A_k = −D_k via Lyapunov
    // solver. Note the Lyapunov solver is written for
    // A^T X + X A = C; here we want A_k^T X + X A_k = -D_k,
    // so we call solveLyapunov(Ak, negDk).
    const negDk = matScale(Dk, -1);
    const Pnext = solveLyapunov(Ak, negDk);
    // Check convergence: ‖P_{k+1} − P_k‖_F.
    const diff = matSub(Pnext, P);
    const err = matFrobNorm(diff);
    P = Pnext;
    if (err < tol) {
      // K = R^{-1} B^T P
      const K = matMul(Rinv, matMul(transpose(B), P));
      return { P, K, iterations: iter + 1, residual: err };
    }
  }
  // Did not converge within maxIter; return the last P so
  // the caller can still see something. (Better to log this
  // than to throw — the simulator's slider UX shouldn't
  // crash if the user picks a pathological Q, R.)
  const K = matMul(Rinv, matMul(transpose(B), P));
  return { P, K, iterations: maxIter, residual: matFrobNorm(P) };
}

/**
 * Transpose of a rectangular matrix.
 * @param {number[][]} A
 * @returns {number[][]}
 */
export function transpose(A) {
  const m = A.length;
  const n = A[0].length;
  const T = [];
  for (let i = 0; i < n; i++) {
    const row = new Float64Array(m);
    for (let j = 0; j < m; j++) row[j] = A[j][i];
    T.push(row);
  }
  return T;
}

/**
 * Invert a small (m×m) dense matrix via Gaussian elimination
 * with partial pivoting. For m=1, this is a no-op; the
 * single-input LQR case uses a scalar inverse in solveLQR.
 *
 * @param {number[][]} A
 * @returns {number[][]}
 */
export function invertMatrix(A) {
  const m = A.length;
  // Augment with identity.
  const M = [];
  for (let i = 0; i < m; i++) {
    const row = new Float64Array(2 * m);
    for (let j = 0; j < m; j++) row[j] = A[i][j];
    row[m + i] = 1;
    M.push(row);
  }
  // Forward elimination.
  for (let k = 0; k < m; k++) {
    let pivRow = k;
    let pivAbs = Math.abs(M[k][k]);
    for (let i = k + 1; i < m; i++) {
      const v = Math.abs(M[i][k]);
      if (v > pivAbs) { pivAbs = v; pivRow = i; }
    }
    if (pivAbs < 1e-14) {
      throw new Error('invertMatrix: singular (row ' + k + ')');
    }
    if (pivRow !== k) { const t = M[k]; M[k] = M[pivRow]; M[pivRow] = t; }
    const piv = M[k][k];
    for (let i = k + 1; i < m; i++) {
      const f = M[i][k] / piv;
      if (f === 0) continue;
      for (let j = k; j < 2 * m; j++) M[i][j] -= f * M[k][j];
    }
  }
  // Back-substitution.
  const inv = [];
  for (let i = 0; i < m; i++) inv.push(new Float64Array(m));
  for (let i = m - 1; i >= 0; i--) {
    const piv = M[i][i];
    for (let j = m; j < 2 * m; j++) {
      let s = M[i][j];
      for (let k = i + 1; k < m; k++) s -= M[i][k] * inv[k][j - m];
      inv[i][j - m] = s / piv;
    }
  }
  return inv;
}

// ============================================================================
// Domain-specific wrappers
// ============================================================================
//
// lqrForCartPole is the inverted-pendulum-specific glue: it
// builds the linearized 4-state A/B matrices for the cart-pole
// plant and hands them to solveLQR. State order x = (p, θ, v, ω),
// input u = F.
//
//   A = [ 0    0       1     0  ]
//       [ 0    0       0     1  ]
//       [ 0  -mg/M     0     0  ]
//       [ 0   g(M+m)/(Ml)  0  0  ]
//
//   B = [ 0,           0,    1/M,   -1/(Ml) ]ᵀ
//
// With cart friction b_cart > 0 the cart-velocity state
// picks up a damping term, which is essential for the
// linearized plant to be controllable (without friction the
// cart position is a free integrator and uncontrollable by
// the single input). If b_cart is not passed we default to
// 0.1 (a small but stabilizing value), which is the value
// the page-default preset uses. The closed-loop poles and
// K returned by the LQR solver depend on b_cart through
// A, so the simulator should pass the live b_cart slider
// value, not a hard-coded number.
//
// The char-poly and closed-loop-pole helpers for this plant
// live in pole-roots-lib.js (characteristicPoly, closedLoopPoles,
// stateFeedbackPoles); lqrForCartPole is the matching
// gain-computation wrapper here, kept together with the
// numerical-algorithms surface that supports it.

/**
 * Convenience wrapper for the inverted-pendulum simulator:
 * given the slider values, build A and B for the linearized
 * cart-pole, then call solveLQR.
 *
 * Inputs:
 *   M, m, l, g   — physical parameters
 *   b_cart       — cart friction (default 0.1 if not
 *                  passed; pass the page's live value to
 *                  match the simulator's integrated plant)
 *   Q            — 4×1 (or 4×4) state weighting; if a 1×4 or
 *                  4×1 array is passed, the function promotes
 *                  it to a 4×4 diagonal matrix.
 *   R            — scalar or 1×1; for the single-input
 *                  cart-pole that's all we have.
 *
 * @returns {{P:number[][], K:number[][], iterations:number, residual:number}}
 */
export function lqrForCartPole(M, m, l, g, b_cart, Q, R) {
  // Promote Q to a 4×4 diagonal if passed as a flat array.
  let Qmat;
  if (Array.isArray(Q) && Q.length === 4 && typeof Q[0] === 'number') {
    Qmat = [
      [Q[0], 0,    0,    0   ],
      [0,    Q[1], 0,    0   ],
      [0,    0,    Q[2], 0   ],
      [0,    0,    0,    Q[3]],
    ];
  } else {
    Qmat = Q;
  }
  let Rmat;
  if (typeof R === 'number') {
    Rmat = [[R]];
  } else {
    Rmat = R;
  }
  // If b_cart wasn't passed (e.g., older call sites), default
  // to 0.1 to make the plant controllable.
  if (typeof b_cart !== 'number' || !isFinite(b_cart)) {
    b_cart = 0.1;
  }
  const A = [
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, -m * g / M, -b_cart / M, 0],
    [0,  g * (M + m) / (M * l), 0, 0],
  ];
  const B = [
    [0],
    [0],
    [1 / M],
    [-1 / (M * l)],
  ];
  return solveLQR(A, B, Qmat, Rmat);
}

// polynomialRoots is imported above and re-exported below so
// callers can `import { polynomialRoots } from 'linear-algebra-lib.js'`
// if they prefer the umbrella module. (The canonical home for
// root-finding is pole-roots-lib.js.)
export { polynomialRoots };

// CommonJS / script-tag interop so this file also works in the
// existing Node/bower-style loaders that ship the rest of the repo.
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    solveLyapunov,
    solveLinearSystem,
    matAdd,
    matSub,
    matScale,
    matMul,
    matFrobNorm,
    ackerStabilizing,
    polyFromRoots,
    identity,
    solveLQR,
    transpose,
    invertMatrix,
    lqrForCartPole,
    polynomialRoots,
  };
}
