import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import type { PaperSpec } from './paperSpec';
import { PaperError } from './errors';

export type FoldKind = 'valley' | 'mountain';
/**
 * Which paper moves with a fold:
 * - all:    every facet on the moving side (whole stack)
 * - top:    the topmost flap at the line (as seen from planeNormal) and everything attached to it
 * - bottom: the bottommost flap at the line
 * - flap:   the connected piece under `pick`
 */
export type FoldScope = 'all' | 'top' | 'bottom' | 'flap';

export interface FoldOp {
  type: 'fold';
  id: string;
  /** Fold line in model space, lying in the working plane. */
  line: { p0: Vec3; p1: Vec3 };
  /** Working-plane normal pointing at the viewer. Valley/mountain are relative to it. */
  planeNormal: Vec3;
  /** Direction (in the working plane) towards the side that moves. */
  movingSide: Vec3;
  kind: FoldKind;
  /** 5…180 for user folds (core accepts any angle for previews). */
  angleDeg: number;
  scope: FoldScope;
  pick?: Vec3;
  /** Also apply the mirror-image fold (sheet symmetry). */
  mirror?: boolean;
  /** The paper moved by this fold is a movable control flap in flight (stage 5). */
  control?: boolean;
}

export interface CutOp {
  type: 'cut';
  id: string;
  /** Polyline in model space on the working plane. */
  path: Vec3[];
  planeNormal: Vec3;
  /** all: through every coplanar layer; top: only the topmost layer under the path. */
  depth: 'all' | 'top';
  mirror?: boolean;
  /** Sheet points identifying pieces to throw away if the cut detaches pieces. Default: keep the largest. */
  discard?: { flat: Vec2 }[];
}

export interface ClipOp {
  type: 'clip';
  id: string;
  /** Point on a stack edge, sheet coordinates. */
  at: Vec2;
  /** Edge direction at that point, sheet coordinates. */
  edgeDir: Vec2;
  massKg: number;
  /** Paper clip (default) or a strip of tape folded over the edge. */
  kind?: 'clip' | 'tape';
}

export interface UnclipOp {
  type: 'unclip';
  id: string;
  target: string;
}

export type Op = FoldOp | CutOp | ClipOp | UnclipOp;

export interface DesignFile {
  format: 'paperplane-design';
  version: 1;
  name: string;
  paper: PaperSpec;
  /** Body axes in model space (used from stage 2/3 on). */
  orientation: { nose: Vec3; up: Vec3 };
  /** Simulate the paper as flexible (hinged panels) instead of one rigid body (stage 6). */
  flexible?: boolean;
  ops: Op[];
}

export const DESIGN_FORMAT = 'paperplane-design';
export const DESIGN_VERSION = 1;

// ---------- validation ----------

const isNum = (x: unknown): x is number => typeof x === 'number' && Number.isFinite(x);
const isVec = (x: unknown, n: number): boolean => Array.isArray(x) && x.length === n && x.every(isNum);

function fail(msg: string): never {
  throw new PaperError('invalidDesign', msg);
}

function checkOp(o: unknown, i: number): Op {
  if (typeof o !== 'object' || o === null) fail(`ops[${i}] is not an object`);
  const op = o as Record<string, unknown>;
  if (typeof op.id !== 'string') fail(`ops[${i}].id missing`);
  switch (op.type) {
    case 'fold': {
      const line = op.line as Record<string, unknown> | undefined;
      if (!line || !isVec(line.p0, 3) || !isVec(line.p1, 3)) fail(`ops[${i}].line invalid`);
      if (!isVec(op.planeNormal, 3)) fail(`ops[${i}].planeNormal invalid`);
      if (!isVec(op.movingSide, 3)) fail(`ops[${i}].movingSide invalid`);
      if (op.kind !== 'valley' && op.kind !== 'mountain') fail(`ops[${i}].kind invalid`);
      if (!isNum(op.angleDeg) || op.angleDeg <= 0 || op.angleDeg > 180) fail(`ops[${i}].angleDeg out of range`);
      if (!['all', 'top', 'bottom', 'flap'].includes(op.scope as string)) fail(`ops[${i}].scope invalid`);
      if (op.scope === 'flap' && !isVec(op.pick, 3)) fail(`ops[${i}].pick required for scope flap`);
      if (op.control !== undefined && typeof op.control !== 'boolean') fail(`ops[${i}].control must be boolean`);
      return op as unknown as FoldOp;
    }
    case 'cut': {
      if (!Array.isArray(op.path) || op.path.length < 2 || !op.path.every((p) => isVec(p, 3)))
        fail(`ops[${i}].path invalid`);
      if (!isVec(op.planeNormal, 3)) fail(`ops[${i}].planeNormal invalid`);
      if (op.depth !== 'all' && op.depth !== 'top') fail(`ops[${i}].depth invalid`);
      if (op.discard !== undefined) {
        if (!Array.isArray(op.discard) || !op.discard.every((d) => isVec((d as { flat?: unknown })?.flat, 2)))
          fail(`ops[${i}].discard invalid`);
      }
      return op as unknown as CutOp;
    }
    case 'clip':
      if (!isVec(op.at, 2) || !isVec(op.edgeDir, 2) || !isNum(op.massKg) || op.massKg <= 0)
        fail(`ops[${i}] clip invalid`);
      if (op.kind !== undefined && op.kind !== 'clip' && op.kind !== 'tape') fail(`ops[${i}].kind invalid`);
      return op as unknown as ClipOp;
    case 'unclip':
      if (typeof op.target !== 'string') fail(`ops[${i}].target missing`);
      return op as unknown as UnclipOp;
    default:
      fail(`ops[${i}].type unknown: ${String(op.type)}`);
  }
}

/** Parse and validate an untrusted design object (e.g. from JSON.parse). */
export function validateDesign(obj: unknown): DesignFile {
  if (typeof obj !== 'object' || obj === null) fail('design is not an object');
  const d = obj as Record<string, unknown>;
  if (d.format !== DESIGN_FORMAT) fail('wrong format tag');
  if (d.version !== DESIGN_VERSION) fail(`unsupported version ${String(d.version)}`);
  const paper = d.paper as Record<string, unknown> | undefined;
  if (!paper || !isNum(paper.widthM) || !isNum(paper.heightM) || !isNum(paper.gsm)) fail('paper invalid');
  if ((paper.widthM as number) <= 0 || (paper.heightM as number) <= 0 || (paper.gsm as number) <= 0)
    fail('paper dimensions must be positive');
  const orient = d.orientation as Record<string, unknown> | undefined;
  if (!orient || !isVec(orient.nose, 3) || !isVec(orient.up, 3)) fail('orientation invalid');
  if (!Array.isArray(d.ops)) fail('ops missing');
  const ops = d.ops.map(checkOp);
  const ids = new Set<string>();
  for (const op of ops) {
    if (ids.has(op.id)) fail(`duplicate op id ${op.id}`);
    ids.add(op.id);
  }
  return {
    format: DESIGN_FORMAT,
    version: DESIGN_VERSION,
    name: typeof d.name === 'string' ? d.name : 'Namnlös',
    paper: { widthM: paper.widthM as number, heightM: paper.heightM as number, gsm: paper.gsm as number },
    orientation: { nose: orient.nose as Vec3, up: orient.up as Vec3 },
    // absent = not stated (the game then chooses its default); otherwise keep the choice
    ...(typeof d.flexible === 'boolean' ? { flexible: d.flexible } : {}),
    ops,
  };
}
