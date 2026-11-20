import { describe, expect, it } from 'vitest';
import * as V from '../../../src/core/math/vec3';
import {
  createSheet,
  facetNormal,
  facetVertices,
  totalArea,
  canonicalNormal,
} from '../../../src/core/paper/model';
import type { PaperModel } from '../../../src/core/paper/model';
import { A4_80GSM } from '../../../src/core/paper/paperSpec';
import { applyFold } from '../../../src/core/paper/fold';
import { applyOp, foldAngleLimit } from '../../../src/core/paper/replay';
import { computeEdges } from '../../../src/core/paper/edges';
import { computeStackLevels, maxLayers } from '../../../src/core/paper/stack';
import { PaperError } from '../../../src/core/paper/errors';
import { sameRigid, rigidIdentity } from '../../../src/core/math/transform';
import { CENTER_LINE, cut, fold } from './helpers';

const A4_AREA = 0.21 * 0.297;
const sheet = () => createSheet(A4_80GSM);
const allVerts = (m: PaperModel) => m.facets.flatMap(facetVertices);
const halfFold = (kind: 'valley' | 'mountain' = 'valley') =>
  applyFold(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1], { kind })).model;

describe('sheet', () => {
  it('starts as one flat A4 facet', () => {
    const m = sheet();
    expect(m.facets).toHaveLength(1);
    expect(totalArea(m)).toBeCloseTo(A4_AREA, 12);
    const g = computeEdges(m);
    expect(g.links).toHaveLength(0);
    expect(g.borders).toHaveLength(4);
  });
});

describe('fold in half', () => {
  it('valley: moving half ends on top, area conserved, lies on the other half', () => {
    const r = applyFold(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1]));
    const m = r.model;
    expect(m.facets).toHaveLength(2);
    expect(totalArea(m)).toBeCloseTo(A4_AREA, 12);
    for (const v of allVerts(m)) {
      expect(v[2]).toBeLessThanOrEqual(1e-9);
      expect(Math.abs(v[1])).toBeLessThan(1e-9);
    }
    const moved = m.facets.find((f) => r.moved.includes(f.id))!;
    const still = m.facets.find((f) => !r.moved.includes(f.id))!;
    expect(moved.layer).toBeGreaterThan(still.layer);
    expect(facetNormal(moved)[1]).toBeCloseTo(-1, 9); // flipped over: back side up
    const g = computeEdges(m);
    expect(g.links.filter((l) => l.kind === 'crease')).toHaveLength(1);
    expect(maxLayers(m)).toBe(2);
  });

  it('mountain: moving half ends underneath', () => {
    const r = applyFold(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1], { kind: 'mountain' }));
    const moved = r.model.facets.find((f) => r.moved.includes(f.id))!;
    const still = r.model.facets.find((f) => !r.moved.includes(f.id))!;
    expect(moved.layer).toBeLessThan(still.layer);
    const lv = computeStackLevels(r.model);
    expect(lv.get(moved.id)).toBe(0);
    expect(lv.get(still.id)).toBe(1);
  });

  it('unfolding restores the flat sheet', () => {
    const m1 = halfFold();
    // flap now lies on the −z side, on top; swing it back towards +z
    const m2 = applyFold(m1, fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, -1], { scope: 'top' })).model;
    for (const f of m2.facets) expect(sameRigid(f.T, rigidIdentity(), 1e-9)).toBe(true);
    expect(totalArea(m2)).toBeCloseTo(A4_AREA, 12);
  });

  it('partial angle: 90° valley stands the flap up', () => {
    const r = applyFold(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1], { angleDeg: 90 }));
    const moved = r.model.facets.find((f) => r.moved.includes(f.id))!;
    const n = facetNormal(moved);
    expect(Math.abs(n[1])).toBeLessThan(1e-9);
    for (const v of facetVertices(moved)) expect(v[1]).toBeGreaterThanOrEqual(-1e-9);
  });

  it('layers flip consistently with the canonical normal', () => {
    const m = halfFold();
    for (const f of m.facets) expect(canonicalNormal(facetNormal(f))[1]).toBeCloseTo(1, 9);
  });
});

describe('scopes', () => {
  // after a valley half fold, the stack lies at z<0 with the flap on top
  const cornerLine: [V.Vec3, V.Vec3] = [
    [0.1485, 0, -0.05],
    [0.0985, 0, -0.105],
  ];
  const towardsCorner: V.Vec3 = [1, 0, -1];

  it('all: whole stack folds', () => {
    const r = applyFold(halfFold(), fold(cornerLine[0], cornerLine[1], towardsCorner, { scope: 'all' }));
    expect(r.moved.length).toBe(2);
  });

  it('top: only the upper layer folds', () => {
    const r = applyFold(halfFold(), fold(cornerLine[0], cornerLine[1], towardsCorner, { scope: 'top' }));
    expect(r.moved.length).toBe(1);
    expect(totalArea(r.model)).toBeCloseTo(A4_AREA, 12);
  });

  it('bottom valley would pass through the top layer → warning', () => {
    const r = applyOp(halfFold(), fold(cornerLine[0], cornerLine[1], towardsCorner, { scope: 'bottom' }), {
      penetration: 'sweep',
    });
    expect(r.warnings.map((w) => w.code)).toContain('penetration');
  });

  it('top valley does not warn', () => {
    const r = applyOp(halfFold(), fold(cornerLine[0], cornerLine[1], towardsCorner, { scope: 'top' }), {
      penetration: 'sweep',
    });
    expect(r.warnings).toHaveLength(0);
  });
});

describe('cuts and flaps', () => {
  const TE = -0.1485; // trailing edge (−x)
  const slit = (z: number) => cut([
    [TE - 0.01, 0, z],
    [TE + 0.03, 0, z],
  ]);
  const elevatorFold = fold([TE + 0.03, 0, -0.08], [TE + 0.03, 0, 0.08], [-1, 0, 0], {
    scope: 'flap',
    kind: 'mountain',
    angleDeg: 20,
    pick: [TE + 0.01, 0, 0],
  });

  it('without slits the whole trailing strip folds', () => {
    const r = applyFold(sheet(), elevatorFold);
    const zs = r.model.facets.filter((f) => r.moved.includes(f.id)).flatMap(facetVertices).map((v) => Math.abs(v[2]));
    expect(Math.max(...zs)).toBeCloseTo(0.105, 9);
  });

  it('folding paper that is attached out of plane would tear → error', () => {
    // stand one half up at 90°, then try to fold a corner of the base half
    const m = applyFold(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1], { angleDeg: 90 })).model;
    const corner = fold([0.1, 0, -0.2], [0.1, 0, 0.2], [1, 0, 0], { scope: 'flap', pick: [0.12, 0, -0.05] });
    expect(() => applyFold(m, corner)).toThrow(PaperError);
  });

  it('two slits make a foldable elevator flap', () => {
    let m = sheet();
    m = applyOp(m, slit(-0.03)).model;
    m = applyOp(m, slit(0.03)).model;
    expect(m.cuts.length).toBe(2);
    const r = applyFold(m, elevatorFold);
    const movedVerts = r.model.facets.filter((f) => r.moved.includes(f.id)).flatMap(facetVertices);
    for (const v of movedVerts) {
      expect(Math.abs(v[2])).toBeLessThanOrEqual(0.03 + 1e-9);
      expect(v[1]).toBeLessThanOrEqual(1e-9); // mountain: away from viewer
    }
    expect(totalArea(r.model)).toBeCloseTo(A4_AREA, 12);
  });

  it('a full cut detaches the smaller piece', () => {
    const r = applyOp(sheet(), cut([
      [0.1, 0, -0.2],
      [0.1, 0, 0.2],
    ]));
    expect(totalArea(r.model)).toBeCloseTo(0.21 * (0.297 / 2 + 0.1), 10);
    expect(r.pieces).toHaveLength(2);
    expect(r.warnings.map((w) => w.code)).toContain('piecesDiscarded');
  });

  it('discard list chooses which piece goes', () => {
    const r = applyOp(sheet(), {
      ...cut([
        [0.1, 0, -0.2],
        [0.1, 0, 0.2],
      ]),
      discard: [{ flat: [0, 0] }],
    });
    expect(totalArea(r.model)).toBeCloseTo(0.21 * (0.1485 - 0.1), 10);
  });

  it('cut depth top only cuts the upper layer of a stack', () => {
    const r = applyOp(halfFold(), cut([
      [0.1, 0, -0.05],
      [0.1, 0, -0.2],
    ], { depth: 'top' }));
    expect(r.model.cuts).toHaveLength(1);
  });
});

describe('determinism', () => {
  it('same ops give identical models', () => {
    const ops = [fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1]), fold([0.1485, 0, -0.05], [0.0985, 0, -0.105], [1, 0, -1], { scope: 'top' })];
    const run = () => ops.reduce((m, op) => applyOp(m, op).model, sheet());
    expect(JSON.stringify(run())).toBe(JSON.stringify(run()));
  });

  it('vertices stay finite and area is conserved through many folds', () => {
    let m = sheet();
    for (let i = 0; i < 6; i++) {
      const a = (i * Math.PI) / 7;
      const d: V.Vec3 = [Math.cos(a), 0, Math.sin(a)];
      m = applyOp(m, fold(V.scale(d, -0.3), V.scale(d, 0.3), V.cross([0, 1, 0], d))).model;
    }
    expect(totalArea(m)).toBeCloseTo(A4_AREA, 10);
    for (const v of allVerts(m)) for (const c of v) expect(Number.isFinite(c)).toBe(true);
  });
});

describe('fold angle limit (paper may not pass through paper)', () => {
  it('a free fold can go all the way to 180°', () => {
    expect(foldAngleLimit(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1]))).toBe(180);
  });

  it('a bottom flap folded towards the viewer is blocked by the layer above it', () => {
    const op = fold([0.1485, 0, -0.05], [0.0985, 0, -0.105], [1, 0, -1], { scope: 'bottom' });
    expect(foldAngleLimit(halfFold(), op)).toBe(0);
  });

  it('a strip swinging over a standing flap stops where it would hit it', () => {
    const standing = applyFold(sheet(), fold(CENTER_LINE[0], CENTER_LINE[1], [0, 0, 1], { angleDeg: 90 })).model;
    const strip = fold([-0.2, 0, -0.02], [0.2, 0, -0.02], [0, 0, -1], { scope: 'flap', pick: [0, 0, -0.08] });
    const limit = foldAngleLimit(standing, strip);
    expect(limit).toBeGreaterThan(95);
    expect(limit).toBeLessThan(110);
  });
});
