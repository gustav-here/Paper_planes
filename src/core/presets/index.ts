import type { Vec3 } from '../math/vec3';
import type { DesignFile, FoldOp } from '../paper/ops';
import { A4_80GSM } from '../paper/paperSpec';

/*
 * Classic models expressed purely as fold operations, exactly as a player could fold them.
 * Model space: unfolded A4 in y = 0, +X = long side towards the nose, +Z = right.
 */

const a = A4_80GSM.heightM / 2; // 0.1485
const b = A4_80GSM.widthM / 2; // 0.105
const DEG = Math.PI / 180;
const Y: Vec3 = [0, 1, 0];

function fold(
  id: string,
  p0: Vec3,
  p1: Vec3,
  movingSide: Vec3,
  rest: Partial<Pick<FoldOp, 'kind' | 'angleDeg' | 'scope' | 'mirror'>> = {},
): FoldOp {
  return {
    type: 'fold',
    id,
    line: { p0, p1 },
    planeNormal: Y,
    movingSide,
    kind: rest.kind ?? 'valley',
    angleDeg: rest.angleDeg ?? 180,
    scope: rest.scope ?? 'all',
    mirror: rest.mirror ?? false,
  };
}

const LONG_CENTRE: [Vec3, Vec3] = [
  [-0.2, 0, 0],
  [0.2, 0, 0],
];

export const flatSheet: DesignFile = {
  format: 'paperplane-design',
  version: 1,
  name: 'Platt ark',
  paper: A4_80GSM,
  orientation: { nose: [1, 0, 0], up: [0, 1, 0] },
  ops: [],
};

/** Classic dart: corners to centre twice, fold in half (flaps outside), wings with 10° dihedral. */
export function makeDart(keel = 0.025, wingAngleDeg = 80): DesignFile {
  const t = Math.tan(22.5 * DEG);
  return {
    format: 'paperplane-design',
    version: 1,
    name: 'Dart',
    paper: A4_80GSM,
    orientation: { nose: [1, 0, 0], up: [0, 0, 1] },
    ops: [
      fold('d1', [a, 0, 0], [a - b, 0, b], [1, 0, 1], { mirror: true }),
      fold('d2', [a, 0, 0], [a - b / t, 0, b], [Math.sin(22.5 * DEG), 0, Math.cos(22.5 * DEG)], { mirror: true }),
      fold('d3', LONG_CENTRE[0], LONG_CENTRE[1], [0, 0, -1], { kind: 'mountain' }),
      fold('d4', [-0.2, 0, keel], [0.2, 0, keel], [0, 0, 1], { scope: 'top', angleDeg: wingAngleDeg, mirror: true }),
    ],
  };
}

/** Wide-wing glider: corners in, nose triangle folded back (heavy blunt nose), corners in again, short keel. */
export function makeGlider(keel = 0.015, wingAngleDeg = 85): DesignFile {
  const x1 = a - b; // base of the first nose triangle
  return {
    format: 'paperplane-design',
    version: 1,
    name: 'Glidplan (breda vingar)',
    paper: A4_80GSM,
    orientation: { nose: [1, 0, 0], up: [0, 0, 1] },
    ops: [
      fold('g1', [a, 0, 0], [a - b, 0, b], [1, 0, 1], { mirror: true }),
      fold('g2', [x1, 0, -0.2], [x1, 0, 0.2], [1, 0, 0]),
      fold('g3', [x1, 0, 0], [x1 - b, 0, b], [1, 0, 1], { mirror: true }),
      fold('g4', LONG_CENTRE[0], LONG_CENTRE[1], [0, 0, -1], { kind: 'mountain' }),
      fold('g5', [-0.2, 0, keel], [0.2, 0, keel], [0, 0, 1], { scope: 'top', angleDeg: wingAngleDeg, mirror: true }),
    ],
  };
}

export interface Preset {
  id: string;
  name: string;
  design: DesignFile;
}

export const PRESETS: Preset[] = [
  { id: 'flat', name: 'Platt ark', design: flatSheet },
  { id: 'dart', name: 'Dart', design: makeDart() },
  { id: 'glider', name: 'Glidplan (breda vingar)', design: makeGlider() },
];
