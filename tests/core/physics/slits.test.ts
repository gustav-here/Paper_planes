import { describe, expect, it } from 'vitest';
import { makeDart, makeGlider } from '../../../src/core/presets';
import { replayDesign } from '../../../src/core/paper/replay';
import type { CutOp, DesignFile, FoldOp } from '../../../src/core/paper/ops';
import type { Vec3 } from '../../../src/core/math/vec3';
import { facetArea, totalArea } from '../../../src/core/paper/model';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';
import { slitTabs } from '../../../src/core/physics/slits';
import { createSimulator, prepare } from '../../../src/core/physics/factory';
import { MultiBodySim } from '../../../src/core/physics/multibody';

const a = 0.1485;
const KEEL = 0.025;
const th = (80 * Math.PI) / 180;
const W: Vec3 = [0, Math.sin(th), Math.cos(th)];
const UP: Vec3 = [0, -Math.cos(th), Math.sin(th)];
/** A point on the right wing of the plain dart: x along the body, s out from the wing root. */
const onWing = (x: number, s: number): Vec3 => [x, s * W[1], KEEL + s * W[2]];
const TE = -a;

const plainDart = () => makeDart(KEEL, 80, { elevatorDeg: 0, elevatorChord: 0.02 });

/** Two parallel slits from the trailing edge of the right wing, 40 mm apart, 30 mm deep. */
function wingSlits(): CutOp[] {
  return [0.04, 0.08].map((s, i) => ({ type: 'cut', id: `w${i}`, path: [onWing(TE - 0.01, s), onWing(TE + 0.03, s)], planeNormal: UP, depth: 'all' }));
}

/** Two slits up into the keel from its bottom edge (the centre fold), through both layers. */
function keelSlits(): CutOp[] {
  return [-0.08, -0.05].map((x, i) => ({ type: 'cut', id: `k${i}`, path: [[x, 0, -0.01], [x, 0, 0.015]], planeNormal: [0, 1, 0], depth: 'all' }));
}

const withOps = (d: DesignFile, ops: DesignFile['ops'], flexible = true): DesignFile => ({ ...d, flexible, ops: [...d.ops, ...ops] });
const movedArea = (d: DesignFile, id: string) =>
  replayDesign(d).models.at(-1)!.facets.filter((f) => f.foldPath.at(-1) === id).reduce((s, f) => s + facetArea(f), 0);

describe('folding the tab between two slits', () => {
  const tabFold = (scope: FoldOp['scope']): FoldOp => ({
    type: 'fold', id: 'tab', line: { p0: onWing(TE + 0.03, 0.04), p1: onWing(TE + 0.03, 0.08) },
    planeNormal: UP, movingSide: [-1, 0, 0], kind: 'valley', angleDeg: 90, scope, pick: onWing(TE + 0.015, 0.06),
  });

  it('every scope folds only the tab on the wing (30 × 40 mm)', () => {
    for (const scope of ['top', 'all', 'bottom', 'flap'] as const) {
      expect(movedArea(withOps(plainDart(), [...wingSlits(), tabFold(scope)], false), 'tab'), scope).toBeCloseTo(0.03 * 0.04, 8);
    }
  });

  it('"all layers" folds only the keel tab, both layers, not the whole underside', () => {
    const fold: FoldOp = {
      type: 'fold', id: 'tab', line: { p0: [-0.08, 0, 0.015], p1: [-0.05, 0, 0.015] },
      planeNormal: [0, 1, 0], movingSide: [0, 0, -1], kind: 'valley', angleDeg: 90, scope: 'all',
    };
    expect(movedArea(withOps(plainDart(), [...keelSlits(), fold], false), 'tab')).toBeCloseTo(2 * 0.03 * 0.015, 8);
  });

  it('a line a hair past the slit ends still folds just the tab', () => {
    const f = tabFold('all');
    f.line = { p0: onWing(TE + 0.03, 0.0397), p1: onWing(TE + 0.03, 0.0803) };
    expect(movedArea(withOps(plainDart(), [...wingSlits(), f], false), 'tab')).toBeCloseTo(0.03 * 0.04, 6);
  });
});

describe('slit tabs in flexible paper', () => {
  it('presets have no slit tabs (one slit per elevator)', () => {
    for (const d of [makeDart(), makeGlider()]) expect(slitTabs(replayDesign(d).models.at(-1)!)).toHaveLength(0);
  });

  it('two slits in the wing free a tab that hinges on flat paper between the cut ends', () => {
    const d = withOps(plainDart(), wingSlits());
    const m = replayDesign(d).models.at(-1)!;
    expect(slitTabs(m)).toHaveLength(1);
    const p = prepare(d, DEFAULT_CONFIG);
    // splitting at the hinge does not change the paper
    expect(totalArea(p.model)).toBeCloseTo(totalArea(m), 12);
    const plain = prepare({ ...plainDart(), flexible: true }, DEFAULT_CONFIG);
    expect(p.flex!.set.panels.length).toBe(plain.flex!.set.panels.length + 1);
    const slit = p.flex!.set.hinges.filter((h) => h.kind === 'slit');
    expect(slit).toHaveLength(1);
    expect(slit[0].length).toBeCloseTo(0.04, 6);
    // the cut ends sit on that hinge, so they can tear further
    expect(slit[0].cutEnds).toHaveLength(2);
    // flat paper bending over the tab is softer than a crease of the same length
    expect(slit[0].stiffness).toBeLessThan(DEFAULT_CONFIG.paper.creaseStiffnessPerM * slit[0].length);
  });

  it('two slits in the keel free one tab (both layers move together)', () => {
    const d = withOps(plainDart(), keelSlits());
    const p = prepare(d, DEFAULT_CONFIG);
    const slit = p.flex!.set.hinges.filter((h) => h.kind === 'slit');
    expect(slit).toHaveLength(1);
    expect(slit[0].length).toBeCloseTo(2 * 0.03, 6);
  });

  it('the tab flutters in the airflow, more at higher speed', () => {
    const p = prepare(withOps(plainDart(), wingSlits()), DEFAULT_CONFIG);
    const j = p.flex!.set.hinges.findIndex((h) => h.kind === 'slit');
    const maxBend = (speed: number) => {
      const sim = createSimulator(p, DEFAULT_CONFIG, { speed, elevationDeg: 0, headingDeg: 0, heightM: 5, aoaDeg: 10 }) as MultiBodySim;
      let m = 0;
      while (sim.t < 0.3) {
        sim.step();
        m = Math.max(m, Math.abs(sim.hingeAnglesDeg()[j]));
      }
      return m;
    };
    const slow = maxBend(4);
    const fast = maxBend(10);
    expect(fast).toBeGreaterThan(0.3);
    expect(fast).toBeGreaterThan(slow * 2);
  });

  it('rigid paper ignores slit tabs', () => {
    const p = prepare(withOps(plainDart(), wingSlits(), false), DEFAULT_CONFIG);
    expect(p.flex).toBeNull();
  });
});
