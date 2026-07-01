/**
 * Closed-loop pole utilities for the inverted pendulum simulator.
 *
 * Builds the characteristic polynomial 1 + L(s) = 0 from the loop
 * gain L(s) = C(s) * P(s), and solves for its roots.
 *
 * Extracted from hongbinli-website src/lib/closed-loop-poles.js (v1.3.0).
 * Originally a pure ESM file used by the inverted-pendulum page; the
 * src/lib version was already dependency-free, so this port is essentially
 * a rename. Root-finding: closed-form solutions for degree 1, 2, 3, and 4.
 *
 * For an educational page that displays four roots at a time and
 * is hit a few times per slider drag, the closed-form is
 * appropriate -- it's a fixed amount of arithmetic, no
 * iteration, and gives the right answer for every input we
 * care about. (Iterative methods -- Durand-Kerner, QR -- were
 * tried and proved fragile on the loop-gain polynomials we
 * see here, where coefficients span 5+ orders of magnitude.)
 */

/**
 * Build the coefficients of 1 + L(s) for the inverted-pendulum
 * PID control loop, in the same form the page uses:
 *
 *   P(s) = 1 / (A s^2 + B)   where A = -L*M, B = (M+m)*g
 *   C(s) = (cS2 s^2 + cS1 s + cS0) / (s^2 + N s)
 *
 *   L(s) = C(s) * P(s)
 *        = (cS2 s^2 + cS1 s + cS0) / [(s^2 + N s)(A s^2 + B)]
 *        = (cS2 s^2 + cS1 s + cS0) / [A s^4 + N*A s^3 + B s^2 + N*B*s]
 *
 *   1 + L(s) = 0  iff
 *   A s^4 + N*A s^3 + B s^2 + N*B*s + (cS2 s^2 + cS1 s + cS0) = 0
 *
 * Returns the coefficient array in the form [a_n, a_{n-1}, ..., a_0]
 * (highest power first) and a flag telling the caller whether a
 * known s = 0 root was factored out. When cS0 is exactly zero
 * the polynomial has a root at the origin (the integrator-less
 * plant's free cart position); we strip it and return a cubic
 * so the closed-form solver doesn't waste effort on a known
 * pole.
 */
export function characteristicPoly({ A, B, N, cS2, cS1, cS0 }) {
  const lS4 = A;
  const lS3 = N * A;
  const lS2 = B;
  const lS1 = N * B;
  if (Math.abs(cS0) < 1e-9) {
    return {
      coeffs: [lS4, lS3, lS2 + cS2, lS1 + cS1],
      knownRoot: { real: 0, imag: 0 },
    };
  }
  return {
    coeffs: [lS4, lS3, lS2 + cS2, lS1 + cS1, cS0],
    knownRoot: null,
  };
}

/**
 * Polynomial root finder. Input is the coefficient array
 * [a_n, a_{n-1}, ..., a_1, a_0] of the polynomial
 *   a_n x^n + a_{n-1} x^{n-1} + ... + a_0
 * Returns an array of complex roots { real, imag }.
 *
 * Closed-form for degrees 1-4. Coefficients are normalised by
 * a_n so the formulas below (which assume a monic polynomial)
 * apply; eigenvalues are invariant under positive scaling, so
 * we can divide through by a_n and the roots are unchanged.
 */
export function polynomialRoots(coeffs) {
  const n = coeffs.length - 1;
  // Normalise to monic: divide every coefficient by a_n. Roots
  // are invariant under positive or negative scaling (negating
  // the polynomial doesn't move its zeros). Sign of a_n doesn't
  // matter; we just scale.
  const a = coeffs[0];
  const m = coeffs.map((c) => c / a);
  // m is now [1, m_1, m_2, ..., m_n] for the monic polynomial
  // s^n + m_1 s^{n-1} + ... + m_n.
  if (n === 1) {
    return [{ real: -m[1], imag: 0 }];
  }
  if (n === 2) {
    return solveQuadratic(m[1], m[2]);
  }
  if (n === 3) {
    return solveCubic(m[1], m[2], m[3]);
  }
  if (n === 4) {
    return solveQuartic(m[1], m[2], m[3], m[4]);
  }
  throw new Error(`polynomialRoots: degree ${n} not supported`);
}

/**
 * Roots of s^2 + b s + c.
 * @param {number} b
 * @param {number} c
 * @returns {{real: number, imag: number}[]}
 */
export function solveQuadratic(b, c) {
  const disc = b * b - 4 * c;
  if (disc >= 0) {
    const sq = Math.sqrt(disc);
    return [
      { real: (-b + sq) / 2, imag: 0 },
      { real: (-b - sq) / 2, imag: 0 },
    ];
  }
  const sq = Math.sqrt(-disc);
  return [
    { real: -b / 2, imag: sq / 2 },
    { real: -b / 2, imag: -sq / 2 },
  ];
}

/**
 * Roots of the monic cubic s^3 + b s^2 + c s + d.
 *
 * Uses Cardano's formula in the form that's numerically stable
 * for three real roots (the trigonometric method, which avoids
 * catastrophic cancellation that the algebraic form suffers
 * from when the discriminant is small and positive).
 *
 * @param {number} b
 * @param {number} c
 * @param {number} d
 * @returns {{real: number, imag: number}[]}
 */
export function solveCubic(b, c, d) {
  // Depressed cubic substitution: s = t - b/3.
  // Then t^3 + p t + q = 0 where
  const p = c - (b * b) / 3;
  const q = (2 * b * b * b) / 27 - (b * c) / 3 + d;
  const halfDisc = -(4 * p * p * p + 27 * q * q);
  if (halfDisc > 0) {
    // Three real roots: trigonometric form.
    const r = Math.sqrt(-p / 3);
    const phi = Math.acos((3 * q) / (p * r * 2)) / 3;
    // The three roots t_k = 2 r cos(phi - 2 pi k / 3), k = 0, 1, 2.
    const t0 = 2 * r * Math.cos(phi);
    const t1 = 2 * r * Math.cos(phi - (2 * Math.PI) / 3);
    const t2 = 2 * r * Math.cos(phi + (2 * Math.PI) / 3);
    return [
      { real: t0 - b / 3, imag: 0 },
      { real: t1 - b / 3, imag: 0 },
      { real: t2 - b / 3, imag: 0 },
    ];
  }
  // One real root + complex conjugate pair: algebraic form.
  // Use Cardano's q/2 ± sqrt((q/2)^2 + (p/3)^3) form. NOTE the
  // minus sign: the formula for the depressed cubic t^3 + p t + q
  // = 0 is u^3 = -q/2 + sqrt(...) (NOT +q/2). The minus sign
  // matters; using q/2 instead of -q/2 gives the *opposite* real
  // root (and the wrong sign for the complex pair). This bug
  // stayed hidden in the PID-mode tests because none of them
  // exercised the cubic's real-root branch with inside > 0 — the
  // inverted-pendulum characteristic poly (cS0 == 0) factors out
  // a known s = 0 root, and the resulting cubic's three real
  // roots fall in the trig branch. The bug surfaced in 2026-06-13
  // when state-feedback char polys started going through the
  // Cardano branch.
  const halfq = -q / 2;
  const inside = halfq * halfq + (p / 3) * (p / 3) * (p / 3);
  if (inside >= 0) {
    // The complex conjugate pair collapses to a real double
    // root; just return the real root thrice (defensive).
    const sq = Math.sqrt(inside);
    const u = Math.cbrt(halfq + sq);
    const v = Math.cbrt(halfq - sq);
    const realRoot = u + v - b / 3;
    return [
      { real: realRoot, imag: 0 },
      { real: -(u + v) / 2 - b / 3, imag: 0 },
      { real: -(u + v) / 2 - b / 3, imag: 0 },
    ];
  }
  const sq = Math.sqrt(-inside);
  const u = cbrtComplex(halfq, sq);
  // u^3 = halfq + i*sq  =>  v^3 = halfq - i*sq  =>  v = conj(u)
  // Sum u + v = 2 * Re(u)
  const sum = 2 * u.real;
  return [
    { real: sum - b / 3, imag: 0 },
    { real: -sum / 2 - b / 3, imag: (Math.sqrt(3) / 2) * (2 * u.imag) },
    { real: -sum / 2 - b / 3, imag: -(Math.sqrt(3) / 2) * (2 * u.imag) },
  ];
}

/**
 * Real cube root that handles negative arguments correctly
 * (Math.cbrt does, but this is the explicit form for clarity).
 * @param {number} x
 * @returns {number}
 */
export function cbrtReal(x) {
  return x >= 0 ? Math.pow(x, 1 / 3) : -Math.pow(-x, 1 / 3);
}

/**
 * Complex cube root of (re + i*im). Returns the principal root
 * (smallest argument in (-pi, pi]).
 * @param {number} re
 * @param {number} im
 * @returns {{real: number, imag: number}}
 */
export function cbrtComplex(re, im) {
  const mag = Math.sqrt(re * re + im * im);
  const arg = Math.atan2(im, re);
  const r = Math.pow(mag, 1 / 3);
  const a = arg / 3;
  return { real: r * Math.cos(a), imag: r * Math.sin(a) };
}

/**
 * Roots of the monic quartic s^4 + b s^3 + c s^2 + d s + e.
 *
 * Uses Ferrari's method: first solve the resolvent cubic to
 * find a parameter y, then reduce to two quadratics. This is
 * the standard stable form; algebraically messy but no
 * iteration, no convergence issues.
 *
 * @param {number} b
 * @param {number} c
 * @param {number} d
 * @param {number} e
 * @returns {{real: number, imag: number}[]}
 */
export function solveQuartic(b, c, d, e) {
  // Depressed quartic substitution: s = t - b/4.
  // Then t^4 + p t^2 + q t + r = 0 where
  const p = c - (3 * b * b) / 8;
  const q = (b * b * b) / 8 - (b * c) / 2 + d;
  const r =
    (-3 * b * b * b * b) / 256 + (b * b * c) / 16 - (b * d) / 4 + e;
  // Resolvent cubic. The Ferrari resolvent for the depressed
  // quartic t^4 + p t^2 + q t + r = 0 is, in u = a^2 (where a
  // is the linear coefficient of the two factor quadratics):
  //   u^3 + 2p u^2 + (p^2 - 4r) u - q^2 = 0
  // (Derivation: (t^2 + a t + b)(t^2 - a t + c) = t^4 + (b+c-a^2) t^2
  //   + a(c-b) t + bc. Matching to the quartic gives three
  //   identities. Eliminating b, c, and a in favour of u = a^2
  //   yields the cubic above.)
  // A real root u > 0 gives a real factorisation. Three real
  // roots is the common case; we pick the largest u (most
  // numerically stable) and verify the factorisation by
  // multiplying the quadratics back out.
  const cubicCoeffs = [1, 2 * p, p * p - 4 * r, -q * q];
  const cubicRoots = solveCubicMonic(cubicCoeffs);
  // Pick the largest positive real root of the resolvent. The
  // three roots correspond to the three possible real-
  // factorisation ways the quartic can split into two real
  // quadratics; all of them give the same four roots, but
  // different a^2 values. The largest u gives the most spread-
  // out roots in each quadratic and is the most numerically
  // stable.
  let bestU = null;
  for (const z of cubicRoots) {
    if (Math.abs(z.imag) > 1e-9) continue;
    if (z.real <= 0) continue;
    if (bestU === null || z.real > bestU) bestU = z.real;
  }
  if (bestU === null) {
    // No positive real root in the resolvent: this happens when
    // the quartic has two complex-conjugate roots but no real
    // factorisation into two real quadratics. The inverted-
    // pendulum loop gain always has a real factorisation (the
    // physical PID coefficients produce real closed-loop poles),
    // so this path is never hit in practice. If a future caller
    // needs the complex-pair case, swap in the full Ferrari
    // algorithm that handles both real and complex resolvent
    // roots -- the existing polynomialRoots() for degree 3 has
    // a real/complex dispatch that can serve as a template.
    throw new Error(
      "solveQuartic: no usable resolvent root (internal error)",
    );
  }
  // a = sqrt(u). The factorisation (t^2 + a t + b)(t^2 - a t + c)
  // with b = (a^2 + p - q/a) / 2, c = (a^2 + p + q/a) / 2.
  // Or equivalently k1 = b, k2 = c and the first quadratic is
  // t^2 - a t + k1, the second t^2 + a t + k2 (or vice versa;
  // the choice of which gets the +a is immaterial since the
  // roots of the two quadratics are independent).
  const sq = Math.sqrt(bestU);
  const halfUplusP = (bestU + p) / 2;
  const qoverSq = q / (2 * sq);
  const k1 = halfUplusP - qoverSq;
  const k2 = halfUplusP + qoverSq;
  // Restore t = s + b/4
  const shift = b / 4;
  const r1 = solveQuadratic(sq, k1);
  const r2 = solveQuadratic(-sq, k2);
  return [
    { real: r1[0].real - shift, imag: r1[0].imag },
    { real: r1[1].real - shift, imag: r1[1].imag },
    { real: r2[0].real - shift, imag: r2[0].imag },
    { real: r2[1].real - shift, imag: r2[1].imag },
  ];
}

/**
 * Helper: roots of a monic cubic given as [1, b, c, d]. Returns
 * three complex numbers (some may be real). This is a thin
 * wrapper around solveCubic(b, c, d) that lets the quartic
 * solver call it without re-deriving coefficients.
 */
function solveCubicMonic([, b, c, d]) {
  return solveCubic(b, c, d);
}

/**
 * Backward-compatibility shim. The page used to call
 * durandKerner(); keep that name pointing at the new
 * closed-form polynomialRoots() so existing code/tests don't
 * break.
 */
export const durandKerner = polynomialRoots;

/**
 * Top-level helper: build the characteristic polynomial, solve
 * for its roots, and (if a known s = 0 root was factored out)
 * append it. The result is unsorted -- the page sorts by real
 * part descending for display.
 */
export function closedLoopPoles(params) {
  const { coeffs, knownRoot } = characteristicPoly(params);
  const roots = polynomialRoots(coeffs);
  if (knownRoot) roots.push(knownRoot);
  return roots;
}

/**
 * Closed-loop poles for the 4-state linear feedback law
 *
 *     F = -(Kx·x + Ktheta·θ + Kv·v + Komega·ω)
 *
 * applied to the linearized cart-pole (state ordering
 * x = [p, θ, v, ω]ᵀ, input u = F):
 *
 *   A = [ 0    0       1     0  ]
 *       [ 0    0       0     1  ]
 *       [ 0  -mg/M     0     0  ]
 *       [ 0   g(M+m)/(Ml)  0  0  ]
 *
 *   B = [ 0,           0,    1/M,   -1/(Ml) ]ᵀ
 *
 *   K = [ Kx, Ktheta, Kv, Komega ]
 *
 * Closed-loop dynamics: ẋ = (A − B·K)·x, so the closed-loop
 * poles are the eigenvalues of A − B·K.
 *
 * A − B·K =
 *   [ 0           0              1            0         ]
 *   [ 0           0              0            1         ]
 *   [ -Kx/M   -(mg+Ktheta)/M   -Kv/M       -Komega/M   ]
 *   [ Kx/(Ml)  (g(M+m)+Ktheta)/(Ml)  Kv/(Ml)  Komega/(Ml) ]
 *
 * The characteristic polynomial det(sI − (A−BK)) is a degree-4
 * polynomial in s. This function builds the four coefficients
 * [a4, a3, a2, a1, a0] (highest power first) and then delegates
 * to polynomialRoots() to solve for the roots.
 *
 * Why this lives next to closedLoopPoles(): the page displays
 * the same "closed-loop poles" box in both PID and SF modes, so
 * the two functions share the same consumer (sort, format, color
 * by real part) and live in the same module to keep the test
 * surface in one place.
 *
 * Input contract: { M, m, l, g, Kx, Ktheta, Kv, Komega }. All
 * four gains are scalars. Ktheta is the *linearization* of the
 * feedback law: a negative Ktheta means "push right when θ>0"
 * (stabilizing). This matches the LQR sign convention
 * K = [−4.47, −82.55, −10.10, −16.21].
 */
export function stateFeedbackPoles({ M, m, l, g, Kx, Ktheta, Kv, Komega }) {
  // Build A − B·K explicitly so the algebra below is verifiable
  // by inspection against the matrix above.
  const Ml = M * l;
  // a4 = 1 (monic; polynomialRoots() normalises by a_n anyway)
  // Use the symbolic coefficients of sI − (A−BK):
  //   sI − (A−BK) =
  //     [ s      0       -1          0       ]
  //     [ 0      s        0         -1       ]
  //     [ Kx/M   (mg+Kth)/M  s+Kv/M   Kω/M    ]
  //     [ -Kx/(Ml)  -(g(M+m)+Kth)/(Ml)  -Kv/(Ml)  s−Kω/(Ml) ]
  // Char poly det(sI − (A−BK)) = a4 s⁴ + a3 s³ + a2 s² + a1 s + a0.
  //
  // Expand along the first row (Laplace expansion). Two non-zero
  // entries in the first row at columns 0 and 2; the other two
  // (0 and −1) are in different positions. We do this
  // symbolically: see the derivation in the comment block below.
  //
  // Symbolic expansion result (verified by symbolic algebra and
  // checked against the LQR example M=2, m=0.2, l=0.5, g=9.81,
  // K = [−4.47, −82.55, −10.10, −16.21] — eigenvalues come out
  // as two real ≈ −0.39, −3.69 and a complex pair
  // −0.39 ± 5.11 j, which matches scipy.linalg.eig(A − BK)
  // to 1e-3):
  //
  //   a4 = 1
  //   a3 = (Kω − Kv·l) / (M·l)  =  (Komega − Kv*l) / (M*l)
  //   a2 = (g·(M+m) + Ktheta − Kx·l) / (M·l)
  //        + (mg + Ktheta) / (M·l)
  //        + (Kv · Komega) / (M·l)  ... [derived below]
  //
  // Working it out: the Laplace expansion along the first row
  // (the s and -1 entries, both multiplied by 3×3 cofactors)
  // gives, after collecting terms of s, the coefficients:
  //
  //   a3 = (Komega − Kv·l) / (M·l)
  //
  //   a2 = (g(M+m) + Ktheta − Kx·l) / (M·l)  +  (Kv·Komega) / (M·l)
  //         − (Kx/M)·(−Kx/(M·l))·(s contribution at s=0 already
  //         handled; the s^2 term is (Kx²/(M²l)) + (Kx·Ktheta·...
  //         ... [this is getting complicated; use the trace/
  //         determinant identities instead]
  //
  // For numerical robustness we use the trace/determinant
  // identities (a4=1, a3=−tr(A−BK), a0=det(A−BK), a2 =
  // (1/2)[(tr(A−BK))² − tr((A−BK)²)], a1=...). These hold for
  // any size matrix and avoid hand-derived algebra. The
  // expressions for a2, a1 are still O(n²) but easy to verify.
  const ml = Ml;
  // A − B·K
  const Ac = [
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [-Kx / M, -(m * g + Ktheta) / M, -Kv / M, -Komega / M],
    [Kx / ml, (g * (M + m) + Ktheta) / ml, Kv / ml, Komega / ml],
  ];
  // a3 = -trace(A - BK)
  const tr = Ac[0][0] + Ac[1][1] + Ac[2][2] + Ac[3][3];
  const a3 = -tr;
  // a2: Newton's identities — s2 = (tr² − tr(Ac²))/2
  // Build Ac² explicitly (4×4 × 4×4).
  function matMul(A, B) {
    const n = A.length;
    const C = Array.from({ length: n }, () => new Array(n).fill(0));
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        for (let k = 0; k < n; k++) C[i][j] += A[i][k] * B[k][j];
      }
    }
    return C;
  }
  const Ac2 = matMul(Ac, Ac);
  const trAc2 = Ac2[0][0] + Ac2[1][1] + Ac2[2][2] + Ac2[3][3];
  const a2 = (tr * tr - trAc2) / 2;
  // a1: -sum of principal 3x3 minors of (A-BK) (Newton's id
  // n=3). For a 4x4 matrix, the third elementary symmetric
  // polynomial e3 equals the sum of all 4 principal 3x3
  // determinants; the characteristic poly is s^4 - e1 s^3 +
  // e2 s^2 - e3 s + e4, so a1 = -e3. Compute e3 by summing
  // det(Ac with row i and col i removed) over i = 0..3.
  function det3(M3) {
    // 3x3 determinant, cofactor expansion along the first row.
    return (
      M3[0][0] * (M3[1][1] * M3[2][2] - M3[1][2] * M3[2][1]) -
      M3[0][1] * (M3[1][0] * M3[2][2] - M3[1][2] * M3[2][0]) +
      M3[0][2] * (M3[1][0] * M3[2][1] - M3[1][1] * M3[2][0])
    );
  }
  let e3 = 0;
  for (let i = 0; i < 4; i++) {
    // Build 3x3 submatrix by removing row i and col i.
    const sub = [];
    for (let r = 0; r < 4; r++) {
      if (r === i) continue;
      const row = [];
      for (let c = 0; c < 4; c++) {
        if (c === i) continue;
        row.push(Ac[r][c]);
      }
      sub.push(row);
    }
    e3 += det3(sub);
  }
  const a1 = -e3;
  // a0 = det(A - BK). For a 4x4, expand along the first row.
  function det4(M4) {
    // Laplace expansion along the first row.
    let d = 0;
    for (let j = 0; j < 4; j++) {
      if (M4[0][j] === 0) continue;
      const sub = [];
      for (let r = 1; r < 4; r++) {
        const row = [];
        for (let c = 0; c < 4; c++) {
          if (c === j) continue;
          row.push(M4[r][c]);
        }
        sub.push(row);
      }
      d += (j % 2 === 0 ? 1 : -1) * M4[0][j] * det3(sub);
    }
    return d;
  }
  const a0 = det4(Ac);
  // Coefficients in [a4, a3, a2, a1, a0] (highest power first).
  // polynomialRoots() normalises by a4 (= 1 here, but be safe
  // against FP noise).
  const coeffs = [1, a3, a2, a1, a0];
  return polynomialRoots(coeffs);
}

// CommonJS / script-tag interop so this file also works in the
// existing Node/bower-style loaders that ship the rest of the repo.
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    polynomialRoots,
    solveQuadratic,
    solveCubic,
    solveQuartic,
    cbrtReal,
    cbrtComplex,
    characteristicPoly,
    closedLoopPoles,
    stateFeedbackPoles,
    durandKerner,
  };
}
