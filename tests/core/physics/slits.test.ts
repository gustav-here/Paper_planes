import { describe, expect, it } from 'vitest';
import { makeDart, makeGlider, ELEVATOR_TRIM, NO_REFLEX } from '../../../src/core/presets';
import { canPushThrough, foldAngleLimit, replayDesign } from '../../../src/core/paper/replay';
import type { CutOp, DesignFile, FoldOp } from '../../../src/core/paper/ops';
import type { Vec3 } from '../../../src/core/math/vec3';
import { facetArea, facetVertices, totalArea } from '../../../src/core/paper/model';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';
import { airframeFromDesign } from '../../../src/core/physics/airframe';
import { tunnelPoint } from '../../../src/core/physics/windTunnel';
import { slitTabs } from '../../../src/core/physics/slits';
import { createSimulator, prepare } from '../../../src/core/physics/factory';
import { MultiBodySim } from '../../../src/core/physics/multibody';

/** The presets as they were made before (cut elevators, no turned-up corners): what these tests are about. */
const classicDart = () => makeDart(0.025, 80, ELEVATOR_TRIM, NO_REFLEX);
const classicGlider = () => makeGlider(0.015, 85, ELEVATOR_TRIM, NO_REFLEX);


const a = 0.1485;
const KEEL = 0.025;
const th = (80 * Math.PI) / 180;
const W: Vec3 = [0, Math.sin(th), Math.cos(th)];
const UP: Vec3 = [0, -Math.cos(th), Math.sin(th)];
/** A point on the right wing of the plain dart: x along the body, s out from the wing root. */
const onWing = (x: number, s: number): Vec3 => [x, s * W[1], KEEL + s * W[2]];
const TE = -a;

const plainDart = () => makeDart(KEEL, 80, { elevatorDeg: 0, elevatorChord: 0.02 }, NO_REFLEX);

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
    for (const d of [classicDart(), classicGlider()]) expect(slitTabs(replayDesign(d).models.at(-1)!)).toHaveLength(0);
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

  it('the air does not tear the cut ends, even in a hard throw', () => {
    for (const d of [withOps(plainDart(), wingSlits()), withOps(plainDart(), keelSlits())]) {
      const p = prepare(d, DEFAULT_CONFIG);
      const sim = createSimulator(p, DEFAULT_CONFIG, { speed: 15, elevationDeg: 5, headingDeg: 0, heightM: 1.6 });
      while (!sim.done && !sim.log.firstImpact) sim.step();
      expect(sim.damage().filter((e) => e.kind === 'tear')).toHaveLength(0);
    }
  });

  describe('a keel tab pushed up between the wings as a fin', () => {
    /** Two slits from the bottom of the keel up to `top`, and the fold of the tab between them. */
    const keelTab = (top: number, x0 = -0.13, x1 = -0.1) => {
      const cuts: CutOp[] = [x0, x1].map((x, i) => ({ type: 'cut', id: `f${i}`, path: [[x, 0, -0.01], [x, 0, top]], planeNormal: [0, 1, 0], depth: 'all' }));
      const fold: FoldOp = {
        type: 'fold', id: 'fin', line: { p0: [x0, 0, top], p1: [x1, 0, top] }, planeNormal: [0, 1, 0],
        movingSide: [0, 0, -1], kind: 'valley', angleDeg: 180, scope: 'flap', pick: [(x0 + x1) / 2, 0, top / 2],
      };
      return { cuts, fold };
    };

    for (const top of [KEEL, KEEL - 0.005]) {
      it(`swung sideways it stops under the wing, then pops up over it (cut ${(top * 1000).toFixed(0)} mm up)`, () => {
        const { cuts, fold } = keelTab(top);
        const d = withOps(plainDart(), cuts, false);
        const m = replayDesign(d).models.at(-1)!;
        // the wing is in the way on the swing
        expect(foldAngleLimit(m, fold)).toBeLessThan(150);
        expect(canPushThrough(m, fold)).toBe(true);
        // pushed through, it stands up above the wings' roots in the middle
        const up = replayDesign({ ...d, ops: [...d.ops, { ...fold, through: true }] }).models.at(-1)!;
        const fin = up.facets.filter((f) => f.foldPath.at(-1) === 'fin').flatMap(facetVertices);
        expect(Math.max(...fin.map((v) => v[2]))).toBeGreaterThan(KEEL + 0.01);
        expect(Math.max(...fin.map((v) => Math.abs(v[1])))).toBeLessThan(1e-6);
      });
    }

    it('a tab with nothing in its way is just folded, not pushed through', () => {
      const m = replayDesign(withOps(plainDart(), wingSlits(), false)).models.at(-1)!;
      const tab: FoldOp = {
        type: 'fold', id: 'tab', line: { p0: onWing(TE + 0.03, 0.04), p1: onWing(TE + 0.03, 0.08) },
        planeNormal: UP, movingSide: [-1, 0, 0], kind: 'valley', angleDeg: 180, scope: 'flap', pick: onWing(TE + 0.015, 0.06),
      };
      expect(canPushThrough(m, tab)).toBe(false);
    });

    it('standing up it is a new piece in the airflow and drags more; hanging down it is just more keel', () => {
      const { cuts, fold } = keelTab(KEEL);
      const zeroLiftDrag = (d: DesignFile) => tunnelPoint(airframeFromDesign(d, DEFAULT_CONFIG), DEFAULT_CONFIG, 5, 0).CD;
      const keel = zeroLiftDrag(withOps(plainDart(), [], false));
      const down = zeroLiftDrag(withOps(plainDart(), cuts, false));
      const up = zeroLiftDrag(withOps(plainDart(), [...cuts, { ...fold, through: true }], false));
      expect(down).toBeCloseTo(keel, 6);
      expect(up).toBeGreaterThan(down * 1.01);
    });

    it('the fin flies: the plane stays in one piece and lands', () => {
      const { cuts, fold } = keelTab(KEEL);
      const d = withOps(plainDart(), [...cuts, { ...fold, through: true }]);
      const sim = createSimulator(prepare(d, DEFAULT_CONFIG), DEFAULT_CONFIG, { speed: 6, elevationDeg: 0, headingDeg: 0, heightM: 1.6 });
      while (!sim.done && !sim.log.firstImpact) sim.step();
      expect(sim.log.firstImpact!.distance).toBeGreaterThan(1);
    });
  });

  it('rigid paper ignores slit tabs', () => {
    const p = prepare(withOps(plainDart(), wingSlits(), false), DEFAULT_CONFIG);
    expect(p.flex).toBeNull();
  });
});
