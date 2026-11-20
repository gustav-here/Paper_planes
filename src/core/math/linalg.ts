/** Dense square matrices as row-major number arrays (small sizes: 3×3, 6×6). */

export function zeros(n: number): number[] {
  return new Array<number>(n * n).fill(0);
}

/** Inverse by Gauss–Jordan elimination with partial pivoting. Throws if singular. */
export function invert(a: number[], n: number): number[] {
  const m = a.slice();
  const inv = zeros(n);
  for (let i = 0; i < n; i++) inv[i * n + i] = 1;
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(m[r * n + col]) > Math.abs(m[piv * n + col])) piv = r;
    const p = m[piv * n + col];
    if (Math.abs(p) < 1e-300) throw new Error('singular matrix');
    if (piv !== col) {
      for (let k = 0; k < n; k++) {
        [m[col * n + k], m[piv * n + k]] = [m[piv * n + k], m[col * n + k]];
        [inv[col * n + k], inv[piv * n + k]] = [inv[piv * n + k], inv[col * n + k]];
      }
    }
    for (let k = 0; k < n; k++) {
      m[col * n + k] /= p;
      inv[col * n + k] /= p;
    }
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = m[r * n + col];
      if (f === 0) continue;
      for (let k = 0; k < n; k++) {
        m[r * n + k] -= f * m[col * n + k];
        inv[r * n + k] -= f * inv[col * n + k];
      }
    }
  }
  return inv;
}

export function mulVec(a: number[], x: number[], n: number): number[] {
  const out = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let k = 0; k < n; k++) s += a[i * n + k] * x[k];
    out[i] = s;
  }
  return out;
}

export function mul(a: number[], b: number[], n: number): number[] {
  const out = zeros(n);
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += a[i * n + k] * b[k * n + j];
      out[i * n + j] = s;
    }
  return out;
}
