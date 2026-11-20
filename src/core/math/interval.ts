/** Closed 1D interval [lo, hi]. */
export type Interval = [number, number];

/** Merge overlapping intervals; drops pieces shorter than eps. */
export function union(list: Interval[], eps = 0): Interval[] {
  const sorted = list
    .map(([a, b]) => [Math.min(a, b), Math.max(a, b)] as Interval)
    .filter(([a, b]) => b - a > eps)
    .sort((x, y) => x[0] - y[0]);
  const out: Interval[] = [];
  for (const iv of sorted) {
    const last = out[out.length - 1];
    if (last && iv[0] <= last[1] + eps) last[1] = Math.max(last[1], iv[1]);
    else out.push([iv[0], iv[1]]);
  }
  return out;
}

/** base minus the union of `remove`; pieces shorter than eps are dropped. */
export function subtract(base: Interval, remove: Interval[], eps = 0): Interval[] {
  const rem = union(remove, 0);
  const out: Interval[] = [];
  let cur = base[0];
  for (const [a, b] of rem) {
    if (b <= cur) continue;
    if (a >= base[1]) break;
    if (a - cur > eps) out.push([cur, Math.min(a, base[1])]);
    cur = Math.max(cur, b);
  }
  if (base[1] - cur > eps) out.push([cur, base[1]]);
  return out;
}

/** Intersection of base with the union of `others`. */
export function intersect(base: Interval, others: Interval[], eps = 0): Interval[] {
  return union(others, 0)
    .map(([a, b]) => [Math.max(a, base[0]), Math.min(b, base[1])] as Interval)
    .filter(([a, b]) => b - a > eps);
}

export const totalLength = (list: Interval[]): number => list.reduce((s, [a, b]) => s + (b - a), 0);
