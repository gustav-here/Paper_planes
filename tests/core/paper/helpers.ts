import type { Vec3 } from '../../../src/core/math/vec3';
import type { FoldOp, CutOp, FoldKind, FoldScope } from '../../../src/core/paper/ops';

let n = 0;
export function fold(
  p0: Vec3,
  p1: Vec3,
  movingSide: Vec3,
  opts: Partial<{ kind: FoldKind; angleDeg: number; scope: FoldScope; N: Vec3; mirror: boolean; pick: Vec3 }> = {},
): FoldOp {
  return {
    type: 'fold',
    id: `t${n++}`,
    line: { p0, p1 },
    planeNormal: opts.N ?? [0, 1, 0],
    movingSide,
    kind: opts.kind ?? 'valley',
    angleDeg: opts.angleDeg ?? 180,
    scope: opts.scope ?? 'all',
    pick: opts.pick,
    mirror: opts.mirror,
  };
}

export function cut(path: Vec3[], opts: Partial<{ N: Vec3; depth: 'all' | 'top'; mirror: boolean }> = {}): CutOp {
  return {
    type: 'cut',
    id: `c${n++}`,
    path,
    planeNormal: opts.N ?? [0, 1, 0],
    depth: opts.depth ?? 'all',
    mirror: opts.mirror,
  };
}

/** Long centre line of the A4 sheet (along X). */
export const CENTER_LINE: [Vec3, Vec3] = [
  [-0.2, 0, 0],
  [0.2, 0, 0],
];
