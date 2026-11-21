import { describe, expect, it } from 'vitest';
import { makeDart } from '../../../src/core/presets';
import { replayDesign } from '../../../src/core/paper/replay';
import { CLIP_SPEC, TAPE_SPEC, findClipPlacement, findTapePlacement, resolveClip } from '../../../src/core/paper/clips';
import type { Clip } from '../../../src/core/paper/model';
import { computeEdges } from '../../../src/core/paper/edges';
import { facetContains, facetVertices } from '../../../src/core/paper/model';
import * as P from '../../../src/core/math/polygon2d';
import * as V from '../../../src/core/math/vec3';
import { validateDesign } from '../../../src/core/paper/ops';
import { relaxCreases } from '../../../src/core/paper/springback';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';

/*
 * The dart (nose +x, up +z): the keel hangs below the wing roots at z = 25 mm from the tail to
 * x ≈ 88 mm; the wings rise from there in a shallow V.
 */
const dart = makeDart();
const model = replayDesign(dart).models.at(-1)!;
const graph = computeEdges(model);
const area = (polys: [number, number][][] | null) => (polys ?? []).reduce((a, r) => a + Math.abs(P.signedArea(r)), 0);

/** Lean backwards (away from the nose), as the editor does. */
function clipAt(p: [number, number, number], id = 'c'): Clip {
  const pl = findClipPlacement(model, p, 0.05, graph)!;
  const along = V.cross(pl.normal, pl.inward);
  return { id, at: pl.at, edgeDir: pl.edgeDir, massKg: CLIP_SPEC.massKg, lean: V.dot(along, dart.orientation.nose) <= 0 ? 1 : -1 };
}

describe('clips', () => {
  it('a clip too long for the keel leans back until it clears the wing', () => {
    const rc = resolveClip(model, clipAt([0, 0, -0.01]))!;
    // the keel is 25 mm high, the clip 28 mm long: it must lean
    expect(Math.abs(rc.tiltDeg)).toBeGreaterThan(10);
    // backwards: away from the nose, and no higher than the wing root
    expect(V.dot(rc.inward, dart.orientation.nose)).toBeLessThan(0);
    const tip = V.addScaled(rc.position, rc.inward, CLIP_SPEC.lengthM);
    expect(tip[2]).toBeLessThanOrEqual(0.025 + 1e-4);
    // as little as needed: a little less and its upper corner would reach above the wing root
    const a = ((Math.abs(rc.tiltDeg) - 2) * Math.PI) / 180;
    expect(CLIP_SPEC.lengthM * Math.cos(a) + (CLIP_SPEC.widthM / 2) * Math.sin(a)).toBeGreaterThan(0.025);
  });

  it('the lean stays when the paper springs open (one clip or several)', () => {
    const c1 = clipAt([0, 0, -0.01], 'g1');
    const c2 = clipAt([-0.07, 0, -0.01], 'g2');
    const ops = [...dart.ops, { type: 'clip' as const, ...c1 }, { type: 'clip' as const, ...c2 }];
    const folded = replayDesign({ ...dart, ops }).models.at(-1)!;
    const relaxed = relaxCreases(folded, ops, DEFAULT_CONFIG.paper.creaseSpringback);
    for (const c of relaxed.clips) {
      const rc = resolveClip(relaxed, c)!;
      expect(Math.abs(rc.tiltDeg)).toBeGreaterThan(10);
      expect(V.dot(rc.inward, dart.orientation.nose)).toBeLessThan(0);
    }
  });

  it('where there is no wing above (the nose), a clip stays upright', () => {
    const rc = resolveClip(model, clipAt([0.13, 0, -0.01]))!;
    expect(rc.tiltDeg).toBe(0);
  });
});

describe('tape', () => {
  it('is 2 cm long', () => {
    expect(2 * TAPE_SPEC.lengthM).toBeCloseTo(0.02);
  });

  it('held over the two wing halves it is laid across them and holds them together', () => {
    const pl = findTapePlacement(model, [-0.02, 0.01, 0.03], 0.05, graph)!;
    expect(pl.bridge).toBe(true);
    const rc = resolveClip(model, { id: 't', at: pl.at, edgeDir: pl.edgeDir, massKg: TAPE_SPEC.massKg, kind: 'tape', bridge: true })!;
    expect(rc.bridge).toBe(true);
    expect(rc.across).not.toBeNull();
    // both halves on top (away from the keel below the seam), each 10 × 19 mm
    expect(rc.side * rc.normal[2]).toBeGreaterThan(0.9);
    expect(rc.across!.side * rc.across!.normal[2]).toBeGreaterThan(0.9);
    expect(area(rc.tapeShape as [number, number][][])).toBeCloseTo(TAPE_SPEC.lengthM * TAPE_SPEC.widthM, 6);
    expect(area(rc.across!.tapeShape as [number, number][][])).toBeCloseTo(TAPE_SPEC.lengthM * TAPE_SPEC.widthM, 6);
    // it holds paper on both sides of the seam: one wing half on each side of the keel
    const byId = new Map(model.facets.map((f) => [f.id, f]));
    const ys = rc.clamped.map((id) => {
      const vs = facetVertices(byId.get(id)!);
      return vs.reduce((a, v) => a + v[1], 0) / vs.length;
    });
    expect(Math.min(...ys)).toBeLessThan(0);
    expect(Math.max(...ys)).toBeGreaterThan(0);
  });

  it('held over an edge it folds over to the other side', () => {
    const pl = findTapePlacement(model, [-0.06, 0, -0.01], 0.05, graph)!;
    expect(pl.bridge).toBeFalsy();
    const rc = resolveClip(model, { id: 't', at: pl.at, edgeDir: pl.edgeDir, massKg: TAPE_SPEC.massKg, kind: 'tape' })!;
    expect(rc.bridge).toBe(false);
    // 10 mm on each side of the spine, all of it on paper
    expect(area(rc.tapeShape as [number, number][][])).toBeCloseTo(TAPE_SPEC.lengthM * TAPE_SPEC.widthM, 6);
    expect(rc.clamped.length).toBeGreaterThan(1);
  });

  it('at the nose, what sticks out past the paper is cut off', () => {
    const pl = findTapePlacement(model, [0.147, 0, -0.005], 0.05, graph)!;
    const rc = resolveClip(model, { id: 't', at: pl.at, edgeDir: pl.edgeDir, massKg: TAPE_SPEC.massKg, kind: 'tape' })!;
    const left = area(rc.tapeShape as [number, number][][]);
    expect(left).toBeGreaterThan(0);
    expect(left).toBeLessThan(0.7 * TAPE_SPEC.lengthM * TAPE_SPEC.widthM);
    // every corner of what is left lies on the paper of the stack
    const under = model.facets.filter((g) => rc.clamped.includes(g.id));
    const across = V.normalize(V.cross(rc.inward, rc.normal));
    for (const poly of rc.tapeShape!)
      for (const [x, z] of poly) {
        const p = V.addScaled(V.addScaled(rc.position, rc.inward, x), across, z);
        expect(under.some((g) => facetContains(g, p, -1e-5, 1e-4))).toBe(true);
      }
  });

  it("bridging tape and a clip's lean are saved with the design", () => {
    const ops = [
      ...dart.ops,
      { type: 'clip', id: 'tejp-1', at: [0, 0], edgeDir: [1, 0], massKg: 2e-5, kind: 'tape', bridge: true },
      { type: 'clip', id: 'gem-1', at: [0, 0], edgeDir: [1, 0], massKg: 0.001, lean: -1 },
    ];
    const d = validateDesign(JSON.parse(JSON.stringify({ ...dart, ops })));
    expect(d.ops.at(-2)).toMatchObject({ bridge: true });
    expect(d.ops.at(-1)).toMatchObject({ lean: -1 });
    expect(() => validateDesign(JSON.parse(JSON.stringify({ ...dart, ops: [...dart.ops, { ...ops.at(-1), lean: 2 }] })))).toThrow();
  });
});
