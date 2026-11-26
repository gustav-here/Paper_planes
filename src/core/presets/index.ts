import type { Vec3 } from '../math/vec3';
import type { CutOp, DesignFile, FoldOp, Op } from '../paper/ops';
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

export interface TrimOptions {
  /** Up-elevator deflection (deg); 0 = no elevator. */
  elevatorDeg: number;
  /** Elevator chord, measured forward from the trailing edge (m). */
  elevatorChord: number;
}

/** Distance of the elevator cut from the wing-root fold (m), so the two flaps clear each other. */
const ROOT_GAP = 0.003;

/**
 * Elevators exactly as a person would make them: on each wing, cut in from the trailing edge
 * parallel to the wing-root fold (ROOT_GAP out from it), then fold the freed trailing-edge flap
 * up (mirrored). Geometry only. Wings hinge on the line z = keel, y = 0; the right wing was
 * folded towards +y by wingAngleDeg.
 */
function elevatorOps(prefix: string, keel: number, wingAngleDeg: number, trim: TrimOptions): Op[] {
  if (trim.elevatorDeg <= 0) return [];
  const th = wingAngleDeg * DEG;
  const TE = -a;
  const xf = TE + trim.elevatorChord;
  const wingR: Vec3 = [0, Math.sin(th), Math.cos(th)];
  const wingL: Vec3 = [0, -Math.sin(th), Math.cos(th)];
  const nR: Vec3 = [0, -Math.cos(th), Math.sin(th)]; // "up" side of the right wing
  const nL: Vec3 = [0, Math.cos(th), Math.sin(th)];
  const onWing = (w: Vec3, x: number, s: number): Vec3 => [x, s * w[1], keel + s * w[2]];
  const cut = (id: string, w: Vec3, n: Vec3): CutOp => ({
    type: 'cut',
    id,
    path: [onWing(w, TE - 0.01, ROOT_GAP), onWing(w, xf, ROOT_GAP)],
    planeNormal: n,
    depth: 'all',
  });
  return [
    cut(`${prefix}c1`, wingR, nR),
    cut(`${prefix}c2`, wingL, nL),
    {
      type: 'fold',
      id: `${prefix}e`,
      line: { p0: onWing(wingR, xf, ROOT_GAP), p1: onWing(wingR, xf, 0.15) },
      planeNormal: nR,
      movingSide: [-1, 0, 0],
      kind: 'valley',
      angleDeg: trim.elevatorDeg,
      scope: 'flap',
      pick: onWing(wingR, TE + trim.elevatorChord / 2, 0.04),
      mirror: true,
    },
  ];
}

/** The presets have no cut flaps at the trailing edge (the player can add elevators). */
/**
 * Trim without cutting: the outer trailing corner of each wing folded up (reflex), mirrored. The
 * fold runs from the trailing edge `size` in from the wing tip to the tip edge `size` forward of
 * the trailing edge. 0 = none.
 */
export interface ReflexOptions {
  size: number;
  angleDeg: number;
}

export function reflexOps(prefix: string, keel: number, wingAngleDeg: number, r: ReflexOptions): Op[] {
  if (r.size <= 0 || r.angleDeg <= 0) return [];
  const th = wingAngleDeg * DEG;
  const TE = -a;
  const span = b - keel;
  const wingR: Vec3 = [0, Math.sin(th), Math.cos(th)];
  const nR: Vec3 = [0, -Math.cos(th), Math.sin(th)];
  const onWing = (x: number, s: number): Vec3 => [x, s * wingR[1], keel + s * wingR[2]];
  const p0 = onWing(TE, span - r.size);
  const p1 = onWing(TE + r.size, span);
  // the corner side of the line
  const corner = onWing(TE, span);
  const mid = V3mid(p0, p1);
  const away: Vec3 = [corner[0] - mid[0], corner[1] - mid[1], corner[2] - mid[2]];
  return [
    {
      type: 'fold',
      id: `${prefix}r`,
      line: { p0, p1 },
      planeNormal: nR,
      movingSide: away,
      kind: 'valley',
      angleDeg: r.angleDeg,
      scope: 'all',
      mirror: true,
    },
  ];
}

const V3mid = (p: Vec3, q: Vec3): Vec3 => [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2, (p[2] + q[2]) / 2];

export const DART_TRIM: TrimOptions = { elevatorDeg: 0, elevatorChord: 0.02 };
export const GLIDER_TRIM: TrimOptions = { elevatorDeg: 0, elevatorChord: 0.02 };
/** Small elevators as a person would make them (tests of flaps and controls use these). */
export const ELEVATOR_TRIM: TrimOptions = { elevatorDeg: 10, elevatorChord: 0.02 };
/**
 * Trim without cuts: the wings' outer trailing corners folded up (chosen so both glide steadily).
 * Lowered from 30° when paper got its own nose-up moment (aero.paperCm0, PLAN.md §43), so the
 * trim angle stays about where it was.
 */
export const DART_REFLEX: ReflexOptions = { size: 0.04, angleDeg: 20 };
/** A dart that glides steadily: 10° dihedral and turned-up corners (the validation tests use it). */
export const trimmedDart = () => makeDart(0.025, 80, DART_TRIM, DART_REFLEX);
export const GLIDER_REFLEX: ReflexOptions = { size: 0.03, angleDeg: 5 };
export const NO_REFLEX: ReflexOptions = { size: 0, angleDeg: 0 };

/**
 * Classic dart: corners to centre twice, fold in half (flaps outside), flat wings and nothing else
 * (the player trims it). Untrimmed it flies fast and nose-down. The arguments make variants: wing
 * angle (90° = flat, less = dihedral), elevators and turned-up corners.
 */
export function makeDart(keel = 0.025, wingAngleDeg = 90, trim: TrimOptions = DART_TRIM, reflex: ReflexOptions = NO_REFLEX): DesignFile {
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
      ...elevatorOps('d', keel, wingAngleDeg, trim),
      ...reflexOps('d', keel, wingAngleDeg, reflex),
    ],
  };
}

/**
 * Wide-wing glider: corners in, nose triangle folded back (heavy blunt nose), corners in again,
 * short keel, the outer trailing corners turned up a little to trim it.
 */
export function makeGlider(keel = 0.015, wingAngleDeg = 85, trim: TrimOptions = GLIDER_TRIM, reflex: ReflexOptions = GLIDER_REFLEX): DesignFile {
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
      ...elevatorOps('g', keel, wingAngleDeg, trim),
      ...reflexOps('g', keel, wingAngleDeg, reflex),
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
