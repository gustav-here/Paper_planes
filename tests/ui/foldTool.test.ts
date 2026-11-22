import { describe, expect, it } from 'vitest';
import { FoldTool } from '../../src/ui/editor/FoldTool';
import type { ToolContext } from '../../src/ui/editor/toolContext';
import { makeDart } from '../../src/core/presets';
import { applyOp, replayDesign } from '../../src/core/paper/replay';
import { reachEdges } from '../../src/core/paper/cut';
import type { CutOp, FoldOp } from '../../src/core/paper/ops';
import type { PaperModel } from '../../src/core/paper/model';
import { facetVertices } from '../../src/core/paper/model';
import * as V from '../../src/core/math/vec3';
import type { Vec3 } from '../../src/core/math/vec3';

/*
 * The fold tool driven like a player would, on the flat dart (keel 25 mm deep in the plane y = 0,
 * wings level at z = 25 mm, nose +x): cut the keel, draw the fold line along the wing root, take
 * hold of the tab on the keel and drag on past the stop under the wing.
 */
const KEEL = 0.025;
const camera: Vec3 = [-0.1, 0.6, -0.05]; // beside the plane, a little below

type FakeEvent = { clientX: number; clientY: number; shiftKey: boolean; hit?: { point: Vec3; normal: Vec3 } };

function harness(model: PaperModel, foldScope = 'top') {
  const committed: FoldOp[] = [];
  const noop = () => undefined;
  const ctx = new Proxy(
    {
      options: { foldScope, foldMirror: false, pointToPoint: false, cutDepth: 'all', cutMirror: false },
      model: () => model,
      commit: (op: FoldOp) => {
        committed.push(op);
        return { model, warnings: [], mirrored: false };
      },
      nextId: (p: string) => `${p}1`,
      hitPaper: (e: FakeEvent) => (e.hit ? { facetId: 0, point: e.hit.point, normal: e.hit.normal, frame: { q: [0, 0, 0, 1], t: [0, 0, 0] } } : null),
      ray: () => ({ origin: camera, dir: V.normalize(V.sub([-0.11, 0, 0.012], camera)) }),
      mouse: (e: FakeEvent) => [e.clientX, e.clientY],
      toScreen: () => [0, 0],
      towardsViewer: (n: Vec3, p: Vec3) => (V.dot(n, V.sub(camera, p)) >= 0 ? n : V.negate(n)),
      overlay: new Proxy({}, { get: () => noop }),
    } as Record<string, unknown>,
    { get: (t, k) => (k in t ? t[k as string] : noop) },
  ) as unknown as ToolContext;
  return { tool: new FoldTool(ctx), committed };
}

const cut = (x: number, id: string): CutOp => ({ type: 'cut', id, path: [[x, 0, -0.01], [x, 0, KEEL]], planeNormal: [0, 1, 0], depth: 'all' });

/**
 * Line along the wing root drawn on the wing (its plane z = KEEL), from x0 to x1, or another line
 * (`line`, drawn in plane `N`); then grab and drag.
 */
function foldUp(model: PaperModel, x0: number, x1: number, grab: Vec3, scope = 'top', line?: { p0: Vec3; p1: Vec3; N: Vec3 }) {
  const { tool, committed } = harness(model, scope);
  const l = line ?? { p0: [x0, 0, KEEL] as Vec3, p1: [x1, 0, KEEL] as Vec3, N: [0, 0, -1] as Vec3 };
  (tool as unknown as { toReady(p0: Vec3, p1: Vec3, N: Vec3): void }).toReady(l.p0, l.p1, l.N);
  const on: FakeEvent['hit'] = { point: grab, normal: [0, 1, 0] };
  tool.move({ clientX: 0, clientY: 500, shiftKey: false, hit: on } as unknown as PointerEvent);
  tool.down({ clientX: 0, clientY: 500, shiftKey: false, hit: on } as unknown as PointerEvent);
  // drag up well past the stop under the wing (90°) – it pops up between the wings
  for (let y = 500; y >= 500 - 150 * 0.6; y -= 6) tool.move({ clientX: 0, clientY: y, shiftKey: false, hit: on } as unknown as PointerEvent);
  tool.up({ clientX: 0, clientY: 500 - 150 * 0.6, shiftKey: false, hit: on } as unknown as PointerEvent);
  return committed[0];
}

const piece = (m: PaperModel, id: string) => m.facets.filter((f) => f.foldPath.at(-1) === id).flatMap(facetVertices);

describe('fold tool: a keel tab folded up between the wings', () => {
  const dart = makeDart();

  it('two cuts: the tab between them stands up, the keel behind the rear cut still hangs down', () => {
    const ops = [...dart.ops, cut(-0.06, 'k1'), cut(-0.11, 'k2')];
    const model = replayDesign({ ...dart, ops }).models.at(-1)!;
    const op = foldUp(model, -0.04, -0.16, [-0.085, 0, 0.012]);
    expect(op).toBeDefined();
    expect(op.through).toBe(true);
    expect(op.angleDeg).toBe(180);
    // folded in the keel's plane, not the wing's
    expect(Math.abs(op.planeNormal[1])).toBeCloseTo(1, 6);
    const after = applyOp(model, op).model;
    const tab = piece(after, op.id);
    expect(Math.min(...tab.map((v) => v[0]))).toBeGreaterThan(-0.11 - 1e-6);
    expect(Math.max(...tab.map((v) => v[0]))).toBeLessThan(-0.06 + 1e-6);
    expect(Math.max(...tab.map((v) => v[2]))).toBeGreaterThan(KEEL + 0.02);
    expect(Math.min(...tab.map((v) => v[2]))).toBeGreaterThan(KEEL - 1e-6);
    // the keel behind the rear cut is still below the wings
    const behind = after.facets.filter((f) => facetVertices(f).every((v) => v[0] < -0.11 + 1e-6 && Math.abs(v[1]) < 1e-6));
    expect(behind.length).toBeGreaterThan(0);
    expect(Math.min(...behind.flatMap(facetVertices).map((v) => v[2]))).toBeLessThan(0.001);
  });

  it('one cut: the whole keel behind it stands up', () => {
    const ops = [...dart.ops, cut(-0.08, 'k1')];
    const model = replayDesign({ ...dart, ops }).models.at(-1)!;
    const op = foldUp(model, -0.06, -0.17, [-0.12, 0, 0.012]);
    expect(op).toBeDefined();
    expect(op.through).toBe(true);
    const tab = piece(applyOp(model, op).model, op.id);
    expect(Math.min(...tab.map((v) => v[0]))).toBeLessThan(-0.14);
    expect(Math.max(...tab.map((v) => v[0]))).toBeLessThan(-0.08 + 1e-6);
    expect(Math.max(...tab.map((v) => v[2]))).toBeGreaterThan(KEEL + 0.02);
  });

  it('a cut drawn to just short of both edges, folded with all layers along a line on the keel: the keel in front stays', () => {
    // as drawn by a player: from the keel's fold to the wing root, each end half a millimetre short
    const path = reachEdges(replayDesign(dart).models.at(-1)!, [[-0.08, 0, 0.0005], [-0.08, 0, KEEL - 0.0005]], [0, 1, 0], 0.002);
    const k: CutOp = { type: 'cut', id: 'k1', path, planeNormal: [0, 1, 0], depth: 'all' };
    const model = replayDesign({ ...dart, ops: [...dart.ops, k] }).models.at(-1)!;
    // the fold line on the keel a few mm below the root, from the cut backwards
    const op = foldUp(model, 0, 0, [-0.12, 0, 0.01], 'all', { p0: [-0.08, 0, KEEL - 0.004], p1: [-0.15, 0, KEEL - 0.002], N: [0, 1, 0] });
    expect(op).toBeDefined();
    const tab = piece(applyOp(model, op).model, op.id);
    expect(tab.length).toBeGreaterThan(0);
    // only the tab behind the cut folds (the slanted line moves it a little forward, < 2 mm)
    expect(Math.max(...tab.map((v) => v[0]))).toBeLessThan(-0.08 + 0.002);
  });
});
