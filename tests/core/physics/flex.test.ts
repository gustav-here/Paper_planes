import { describe, expect, it } from 'vitest';
import { makeDart, makeGlider, ELEVATOR_TRIM, NO_REFLEX } from '../../../src/core/presets';
import { replayDesign } from '../../../src/core/paper/replay';
import { validateDesign } from '../../../src/core/paper/ops';
import type { DesignFile, Op } from '../../../src/core/paper/ops';
import { toModel, totalArea } from '../../../src/core/paper/model';
import { buildAirframe } from '../../../src/core/physics/airframe';
import { DEFAULT_CONFIG, makeConfig } from '../../../src/core/physics/config';
import type { PhysicsConfig } from '../../../src/core/physics/config';
import { FlightSim } from '../../../src/core/physics/simulation';
import { MultiBodySim, buildFlexAirframe } from '../../../src/core/physics/multibody';
import { damageOnFolded, damageToOps } from '../../../src/core/physics/damage';
import { findClipPlacement } from '../../../src/core/paper/clips';
import { createSimulator, prepare } from '../../../src/core/physics/factory';
import type { LaunchParams } from '../../../src/core/physics/launch';

/** The presets as they were made before (cut elevators, no turned-up corners): what these tests are about. */
const classicDart = () => makeDart(0.025, 80, ELEVATOR_TRIM, NO_REFLEX);
const classicGlider = () => makeGlider(0.015, 85, ELEVATOR_TRIM, NO_REFLEX);


const launch: LaunchParams = { speed: 5, elevationDeg: 0, headingDeg: 0, heightM: 1.6 };

function flex(d: DesignFile, cfg: PhysicsConfig = DEFAULT_CONFIG, l = launch) {
  const m = replayDesign(d).models.at(-1)!;
  const af = buildAirframe(m, d.orientation, cfg, d.ops);
  return { sim: new MultiBodySim(buildFlexAirframe(m, af, cfg), cfg, l), rigid: new FlightSim(af, cfg, l), model: m };
}

describe('flexible paper (hinged panels)', () => {
  it('very stiff creases reproduce the rigid plane', () => {
    for (const d of [classicDart(), classicGlider()]) {
      const cfg = makeConfig({ paper: { creaseStiffnessPerM: 0.8, bendingStiffness: 2.5e-2 } });
      const { sim, rigid } = flex(d, cfg);
      const a = sim.run(10).firstImpact!;
      const b = rigid.run(10).firstImpact!;
      expect(Math.abs(a.distance - b.distance) / b.distance).toBeLessThan(0.02);
      expect(Math.abs(a.t - b.t)).toBeLessThan(0.03);
    }
  });

  it('paper twisted open between two clips stays one piece (no chain of tiny panels)', () => {
    const d = classicDart();
    const m = replayDesign(d).models.at(-1)!;
    const clip = (x: number, id: string) => {
      const p = findClipPlacement(m, [x, 0, -0.01], 0.05)!;
      return { type: 'clip' as const, id, at: p.at, edgeDir: p.edgeDir, massKg: 0.001, lean: -1 as const };
    };
    const p = prepare({ ...d, ops: [...d.ops, clip(0, 'g1'), clip(-0.08, 'g2')], flexible: true }, DEFAULT_CONFIG);
    // the springback twist splits the paper into many strips, but they are not folds
    expect(p.model.facets.length).toBeGreaterThan(100);
    const plain = prepare({ ...d, flexible: true }, DEFAULT_CONFIG);
    expect(p.flex!.set.panels.length).toBeLessThanOrEqual(plain.flex!.set.panels.length + 2);
    const sim = createSimulator(p, DEFAULT_CONFIG, launch) as unknown as { substeps: number };
    expect(sim.substeps).toBeLessThanOrEqual(6);
  });

  it('the dart becomes keel + two wings + two elevator flaps', () => {
    const { sim } = flex(classicDart());
    expect(sim.panels).toHaveLength(5);
  });

  it('wings give way more at higher speed (air load grows with v²)', () => {
    const maxBend = (speed: number) => {
      const { sim } = flex(classicGlider(), DEFAULT_CONFIG, { ...launch, speed, heightM: 5 });
      let m = 0;
      while (sim.t < 0.4) {
        sim.step();
        m = Math.max(m, ...sim.hingeAnglesDeg().slice(0, 2).map(Math.abs));
      }
      return m;
    };
    expect(maxBend(10)).toBeGreaterThan(maxBend(4) * 1.5);
  });

  it('is deterministic and comes to rest on the floor', () => {
    const run = () => flex(classicDart()).sim.run(10);
    const a = run();
    expect(a.phase).toBe('rest');
    expect(JSON.stringify(run())).toBe(JSON.stringify(a));
  });

  it('control flaps work in the flexible model too', () => {
    const d: DesignFile = { ...classicGlider(), flexible: true };
    d.ops = d.ops.map((o): Op => (o.type === 'fold' && o.id.endsWith('e') ? { ...o, control: true } : o));
    const p = prepare(d, DEFAULT_CONFIG);
    expect(p.flex).not.toBeNull();
    expect(createSimulator(p, DEFAULT_CONFIG, launch)).toBeInstanceOf(MultiBodySim);
    const pitchAt = (pitch: number) => {
      const s = createSimulator(p, DEFAULT_CONFIG, { ...launch, heightM: 10 }, [{ step: 0, pitch, roll: 0 }]);
      while (s.t < 0.5) s.step();
      return s.log.samples.at(-1)!.pitchDeg;
    };
    expect(pitchAt(1)).toBeGreaterThan(pitchAt(0) + 2);
  });
});

describe('damage', () => {
  const crash: LaunchParams = { speed: 13, elevationDeg: -45, headingDeg: 0, heightM: 1.2 };

  it('a hard nose-first crash crumples the paper; the damage is ordinary ops that replay', () => {
    const d = classicDart();
    const { sim, model } = flex(d, DEFAULT_CONFIG, crash);
    sim.run(10);
    const events = sim.damage();
    expect(events.some((e) => e.kind === 'crumple')).toBe(true);
    const r = damageToOps(model, events, d.orientation.up, DEFAULT_CONFIG.paper);
    expect(r.ops.length).toBeGreaterThan(0);
    const damaged = validateDesign(JSON.parse(JSON.stringify({ ...d, ops: [...d.ops, ...r.ops] })));
    const replayed = replayDesign(damaged).models.at(-1)!;
    expect(JSON.stringify(replayed.facets)).toBe(JSON.stringify(r.model.facets));
    // paper is bent, not lost
    expect(totalArea(replayed)).toBeCloseTo(totalArea(model), 10);
  });

  it('a gentle throw leaves the plane undamaged', () => {
    const { sim } = flex(classicGlider());
    sim.run(10);
    expect(sim.damage()).toHaveLength(0);
  });

  it('the rigid model records crumpling too', () => {
    const { rigid } = flex(classicDart(), DEFAULT_CONFIG, crash);
    rigid.run(10);
    expect(rigid.damage().some((e) => e.kind === 'crumple')).toBe(true);
  });

  it('a crumpled plane thrown again stays in one piece: it never gains energy', () => {
    for (const d0 of [makeDart(), makeGlider()]) {
      let d: DesignFile = { ...d0, flexible: true };
      for (const [speed, el] of [[13, -45], [12, -30]] as const) {
        const p = prepare(d, DEFAULT_CONFIG);
        const c = createSimulator(p, DEFAULT_CONFIG, { speed, elevationDeg: el, headingDeg: 0, heightM: 1.2 });
        while (!c.done) c.step();
        d = { ...d, ops: [...d.ops, ...damageToOps(p.folded, damageOnFolded(p.model, p.folded, c.damage()), d.orientation.up, DEFAULT_CONFIG.paper).ops] };
      }
      expect(d.ops.some((o) => o.id.startsWith('dmg-crumple'))).toBe(true);
      const p = prepare(d, DEFAULT_CONFIG);
      for (let seed = 1; seed <= 4; seed++) {
        const s = createSimulator(p, DEFAULT_CONFIG, { speed: 6, elevationDeg: 5, headingDeg: 0, heightM: 1.6, scatterSeed: seed * 7919 });
        const E0 = 0.5 * 36 + 9.81 * 1.6;
        while (!s.done) {
          s.step();
          expect(0.5 * s.speed() ** 2 + 9.81 * s.position()[1]).toBeLessThan(E0 + 1);
        }
        expect(s.phase).toBe('rest');
      }
    }
  });

  it('a damaged plane flies differently', () => {
    const d = classicDart();
    const { sim, model } = flex(d, DEFAULT_CONFIG, crash);
    sim.run(10);
    const r = damageToOps(model, sim.damage(), d.orientation.up, DEFAULT_CONFIG.paper);
    const fly = (design: DesignFile) => {
      const s = createSimulator(prepare(design, DEFAULT_CONFIG), DEFAULT_CONFIG, launch);
      while (!s.done) s.step();
      return s.log.firstImpact!.distance;
    };
    // a few centimetres over ≈ 5.7 m (the bending wings cushion the nose, so the crash is milder)
    expect(Math.abs(fly({ ...d, ops: [...d.ops, ...r.ops] }) - fly(d))).toBeGreaterThan(0.02);
  });
});

describe('springback and paper bending in flight', () => {
  const hard: LaunchParams = { speed: 13, elevationDeg: 10, headingDeg: 0, heightM: 1.6, aoaDeg: 5 };

  it('the air counts a gaping stack once', () => {
    for (const d of [classicDart(), classicGlider()]) {
      const ideal = buildAirframe(replayDesign(d).models.at(-1)!, d.orientation, DEFAULT_CONFIG, d.ops);
      const area = (s: { area: number }[]) => s.reduce((a, x) => a + x.area, 0);
      const p = prepare({ ...d, flexible: true }, DEFAULT_CONFIG);
      expect(area(p.af.mesh.surfaces)).toBeCloseTo(area(ideal.mesh.surfaces), 4);
      expect(area(p.flex!.af.mesh.surfaces) / area(ideal.mesh.surfaces)).toBeCloseTo(1, 1);
    }
  });

  it('hard throws bend the wings much more than gentle ones, without permanent damage', () => {
    const p = prepare({ ...classicGlider(), flexible: true }, DEFAULT_CONFIG);
    const wings = p.flex!.set.hinges.map((h, j) => (h.length > 0.4 ? j : -1)).filter((j) => j >= 0);
    expect(wings).toHaveLength(2);
    // the wings swing at their creases, and their flat paper bends (bending shapes)
    const maxBend = (l: LaunchParams) => {
      const s = createSimulator(p, DEFAULT_CONFIG, l) as MultiBodySim;
      let hinge = 0;
      let bend = 0;
      while (!s.log.firstImpact && s.t < 1.5) {
        s.step();
        const a = s.hingeAnglesDeg();
        hinge = Math.max(hinge, ...wings.map((j) => Math.abs(a[j])));
        bend = Math.max(bend, s.maxBendingM());
      }
      return { hinge, bend, dmg: s.damage().length };
    };
    const gentle = maxBend(launch);
    const strong = maxBend(hard);
    expect(strong.bend).toBeGreaterThan(0.02);
    expect(strong.bend).toBeGreaterThan(1.8 * gentle.bend);
    expect(strong.hinge).toBeGreaterThan(2 * gentle.hinge);
    expect(strong.dmg).toBe(0);
  });

  it('folded layers close up to flat but never pass through each other', () => {
    const p = prepare({ ...classicDart(), flexible: true }, DEFAULT_CONFIG);
    const sim = createSimulator(p, DEFAULT_CONFIG, { ...hard, speed: 15, aoaDeg: 10 }) as MultiBodySim;
    const hs = p.flex!.set.hinges;
    let worst = -Infinity;
    while (!sim.done && sim.t < 3) {
      sim.step();
      const a = sim.hingeAnglesDeg();
      hs.forEach((h, j) => {
        if (h.control < 0) worst = Math.max(worst, (a[j] - (h.closeAt * 180) / Math.PI) * h.closeDir);
      });
    }
    expect(worst).toBeLessThan(1.5);
  });

  it('damage on the relaxed plane becomes ops on the design as folded', () => {
    const d: DesignFile = { ...classicDart(), flexible: true };
    const p = prepare(d, DEFAULT_CONFIG);
    const sim = createSimulator(p, DEFAULT_CONFIG, { speed: 13, elevationDeg: -45, headingDeg: 0, heightM: 1.2 });
    while (!sim.done) sim.step();
    const events = damageOnFolded(p.model, p.folded, sim.damage());
    expect(events.length).toBeGreaterThan(0);
    const r = damageToOps(p.folded, events, d.orientation.up, DEFAULT_CONFIG.paper);
    expect(r.ops.length).toBeGreaterThan(0);
    const replayed = replayDesign({ ...d, ops: [...d.ops, ...r.ops] }).models.at(-1)!;
    expect(JSON.stringify(replayed.facets)).toBe(JSON.stringify(r.model.facets));
  });
});

describe('crushed edges', () => {
  const noseLength = (m: import('../../../src/core/paper/model').PaperModel) =>
    Math.max(...m.facets.flatMap((f) => f.poly.map((p) => toModel(f, p)[0])));

  for (const make of [makeDart, makeGlider]) {
    it(`a nose-first crash pushes the nose of the ${make().name} in (an accordion of folds)`, () => {
      const d: DesignFile = { ...make(), flexible: true };
      const p = prepare(d, DEFAULT_CONFIG);
      const sim = createSimulator(p, DEFAULT_CONFIG, { speed: 15, elevationDeg: -60, headingDeg: 0, heightM: 1.2 });
      while (!sim.done) sim.step();
      const r = damageToOps(p.folded, damageOnFolded(p.model, p.folded, sim.damage()), d.orientation.up, DEFAULT_CONFIG.paper);
      const crumple = r.ops.filter((o) => o.id.startsWith('dmg-crumple'));
      // one accordion per crushed edge
      expect(crumple.length).toBeGreaterThanOrEqual(DEFAULT_CONFIG.paper.crumpleFolds);
      expect(crumple.length % DEFAULT_CONFIG.paper.crumpleFolds).toBe(0);
      // alternating folds, and the nose ends up shorter but still pointing forward
      expect(new Set(crumple.map((o) => (o.type === 'fold' ? o.kind : '')))).toEqual(new Set(['valley', 'mountain']));
      expect(noseLength(r.model)).toBeLessThan(noseLength(p.folded) - 0.0015);
      // only the very tip folds up
      expect(noseLength(r.model)).toBeGreaterThan(noseLength(p.folded) - DEFAULT_CONFIG.paper.crumpleMaxDepth - 0.001);
      expect(totalArea(r.model)).toBeCloseTo(totalArea(p.folded), 10);
    });
  }
});
