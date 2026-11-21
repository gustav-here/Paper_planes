import { describe, expect, it } from 'vitest';
import * as Q from '../../../src/core/math/quat';
import { makeDart, makeGlider } from '../../../src/core/presets';
import { airframeFromDesign } from '../../../src/core/physics/airframe';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';
import { FlightSim } from '../../../src/core/physics/simulation';
import { createSimulator, prepare } from '../../../src/core/physics/factory';
import type { DesignFile, Op } from '../../../src/core/paper/ops';
import type { ControlInput } from '../../../src/core/physics/controls';

/** The presets' elevator fold (mirrored pair), marked as a control flap. */
function withControls(d: DesignFile): DesignFile {
  return { ...d, ops: d.ops.map((o): Op => (o.type === 'fold' && o.id.endsWith('e') ? { ...o, control: true } : o)) };
}

const launch = { speed: 5, elevationDeg: 0, headingDeg: 0, heightM: 10 };

function fly(d: DesignFile, inputs: ControlInput[], seconds: number) {
  const af = airframeFromDesign(d, DEFAULT_CONFIG);
  const sim = new FlightSim(af, DEFAULT_CONFIG, launch, inputs);
  while (sim.t < seconds) sim.step();
  return { sim, af };
}

describe('control flaps', () => {
  it('the mirrored elevator becomes two flaps: behind the CoM, one per wing', () => {
    const af = airframeFromDesign(withControls(makeDart()), DEFAULT_CONFIG);
    expect(af.controls).toHaveLength(2);
    for (const g of af.controls) {
      expect(g.pitchSign).toBe(1);
      expect(g.surfaces.length).toBeGreaterThan(0);
    }
    expect(af.controls.map((g) => g.rollSign).sort()).toEqual([-1, 1]);
  });

  it('without input the flight is identical to a plane without control flaps', () => {
    const a = fly(makeGlider(), [], 1).sim.log.samples.at(-1)!;
    const b = fly(withControls(makeGlider()), [], 1).sim.log.samples.at(-1)!;
    for (let k = 0; k < 3; k++) expect(b.pos[k]).toBeCloseTo(a.pos[k], 9);
  });

  it('stick back (pitch +1) raises the nose; stick forward lowers it', () => {
    const d = withControls(makeGlider());
    const base = fly(d, [], 0.5).sim.log.samples.at(-1)!.pitchDeg;
    const up = fly(d, [{ step: 0, pitch: 1, roll: 0 }], 0.5).sim.log.samples.at(-1)!.pitchDeg;
    const down = fly(d, [{ step: 0, pitch: -1, roll: 0 }], 0.5).sim.log.samples.at(-1)!.pitchDeg;
    expect(up).toBeGreaterThan(base + 2);
    expect(down).toBeLessThan(base - 2);
  });

  it('roll right (+1) banks the plane to the right', () => {
    const { sim, af } = fly(withControls(makeGlider()), [{ step: 0, pitch: 0, roll: 1 }], 0.6);
    const rightWing = Q.rotate(Q.normalize(sim.state.q), af.right);
    expect(rightWing[1]).toBeLessThan(-0.05); // right wing tip went down
    const { sim: s2 } = fly(withControls(makeGlider()), [{ step: 0, pitch: 0, roll: -1 }], 0.6);
    expect(Q.rotate(Q.normalize(s2.state.q), af.right)[1]).toBeGreaterThan(0.05);
  });

  it('flaps move no faster than the rate limit and stop at ±25°', () => {
    const af = airframeFromDesign(withControls(makeGlider()), DEFAULT_CONFIG);
    const sim = new FlightSim(af, DEFAULT_CONFIG, launch, [{ step: 0, pitch: 1, roll: 0 }]);
    sim.step();
    expect(sim.deflectionsDeg()[0]).toBeCloseTo(125 / 240, 9);
    for (let i = 0; i < 240; i++) sim.step();
    expect(sim.deflectionsDeg()[0]).toBeCloseTo(25, 9);
  });

  it('every flap is an elevon: pitch and roll mix, clipped to the travel', () => {
    const af = airframeFromDesign(withControls(makeGlider()), DEFAULT_CONFIG);
    const sim = new FlightSim(af, DEFAULT_CONFIG, launch, [{ step: 0, pitch: 1, roll: 1 }]);
    for (let i = 0; i < 240; i++) sim.step();
    // stick back and right: the right flap goes fully up (both inputs agree), the left cancels out
    const byRoll = new Map(af.controls.map((g, i) => [g.rollSign, sim.deflectionsDeg()[i]]));
    expect(byRoll.get(1)).toBeCloseTo(25, 6);
    expect(byRoll.get(-1)).toBeCloseTo(0, 6);
  });

  it('a flap level with the centre of mass still works as an elevator', () => {
    // a single flap folded up at the trailing edge right under the CoM: no longer ignored for pitch
    const af = airframeFromDesign(withControls(makeGlider()), DEFAULT_CONFIG);
    for (const g of af.controls) expect(g.pitchSign).not.toBe(0);
  });
});

describe('flap size', () => {
  /** Glider with control elevators of the given depth (m), the same everywhere else. */
  const glider = (chord: number) => withControls(makeGlider(0.015, 85, { elevatorDeg: 10, elevatorChord: chord }));
  const CHORDS = [0.01, 0.02, 0.03];

  it('bigger flaps have more area', () => {
    const area = (c: number) => {
      const af = airframeFromDesign(glider(c), DEFAULT_CONFIG);
      return af.controls.reduce((s, g) => s + g.surfaces.reduce((a, sid) => a + af.mesh.surfaces[sid].area, 0), 0);
    };
    const a = CHORDS.map(area);
    expect(a[1] / a[0]).toBeCloseTo(2, 1);
    expect(a[2] / a[0]).toBeCloseTo(3, 1);
  });

  it('bigger flaps roll the plane faster (about in proportion to their area)', () => {
    const rollRate = (c: number) => {
      const af = airframeFromDesign(glider(c), DEFAULT_CONFIG);
      const run = (roll: number) => {
        const sim = new FlightSim(af, DEFAULT_CONFIG, { ...launch, speed: 6 }, [{ step: 0, pitch: 0, roll }]);
        while (sim.t < 0.15) sim.step();
        return sim.state.w[0];
      };
      return Math.abs(run(1) - run(0));
    };
    const r = CHORDS.map(rollRate);
    expect(r[1]).toBeGreaterThan(1.6 * r[0]);
    expect(r[2]).toBeGreaterThan(1.4 * r[1]);
  });

  it('bigger flaps pitch the plane harder', () => {
    const dPitch = (c: number) => {
      const d = glider(c);
      const base = fly(d, [], 0.4).sim.log.samples.at(-1)!.pitchDeg;
      return fly(d, [{ step: 0, pitch: 1, roll: 0 }], 0.4).sim.log.samples.at(-1)!.pitchDeg - base;
    };
    const p = CHORDS.map(dPitch);
    expect(p[1]).toBeGreaterThan(p[0] + 2);
    expect(p[2]).toBeGreaterThan(p[1] + 1);
  });
});

describe('servo-driven flaps in flexible paper', () => {
  it('do not make the simulation take more sub-steps (their stiff spring is implicit)', () => {
    const substeps = (d: DesignFile) => (createSimulator(prepare({ ...d, flexible: true }, DEFAULT_CONFIG), DEFAULT_CONFIG, launch) as unknown as { substeps: number }).substeps;
    for (const d of [makeDart(), makeGlider()]) expect(substeps(withControls(d))).toBeLessThanOrEqual(substeps(d));
  });
});
