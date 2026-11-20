import { describe, expect, it } from 'vitest';
import { makeDart, makeGlider } from '../../../src/core/presets';
import { replayDesign } from '../../../src/core/paper/replay';
import { validateDesign } from '../../../src/core/paper/ops';
import type { DesignFile, Op } from '../../../src/core/paper/ops';
import { totalArea } from '../../../src/core/paper/model';
import { buildAirframe } from '../../../src/core/physics/airframe';
import { DEFAULT_CONFIG, makeConfig } from '../../../src/core/physics/config';
import type { PhysicsConfig } from '../../../src/core/physics/config';
import { FlightSim } from '../../../src/core/physics/simulation';
import { MultiBodySim, buildFlexAirframe } from '../../../src/core/physics/multibody';
import { damageToOps } from '../../../src/core/physics/damage';
import { createSimulator, prepare } from '../../../src/core/physics/factory';
import type { LaunchParams } from '../../../src/core/physics/launch';

const launch: LaunchParams = { speed: 5, elevationDeg: 0, headingDeg: 0, heightM: 1.6 };

function flex(d: DesignFile, cfg: PhysicsConfig = DEFAULT_CONFIG, l = launch) {
  const m = replayDesign(d).models.at(-1)!;
  const af = buildAirframe(m, d.orientation, cfg, d.ops);
  return { sim: new MultiBodySim(buildFlexAirframe(m, af, cfg), cfg, l), rigid: new FlightSim(af, cfg, l), model: m };
}

describe('flexible paper (hinged panels)', () => {
  it('very stiff creases reproduce the rigid plane', () => {
    for (const d of [makeDart(), makeGlider()]) {
      const cfg = makeConfig({ paper: { creaseStiffnessPerM: 0.8 } });
      const { sim, rigid } = flex(d, cfg);
      const a = sim.run(10).firstImpact!;
      const b = rigid.run(10).firstImpact!;
      expect(Math.abs(a.distance - b.distance) / b.distance).toBeLessThan(0.02);
      expect(Math.abs(a.t - b.t)).toBeLessThan(0.03);
    }
  });

  it('the dart becomes keel + two wings + two elevator flaps', () => {
    const { sim } = flex(makeDart());
    expect(sim.panels).toHaveLength(5);
  });

  it('wings give way more at higher speed (air load grows with v²)', () => {
    const maxBend = (speed: number) => {
      const { sim } = flex(makeGlider(), DEFAULT_CONFIG, { ...launch, speed, heightM: 5 });
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
    const run = () => flex(makeDart()).sim.run(10);
    const a = run();
    expect(a.phase).toBe('rest');
    expect(JSON.stringify(run())).toBe(JSON.stringify(a));
  });

  it('control flaps work in the flexible model too', () => {
    const d: DesignFile = { ...makeGlider(), flexible: true };
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
    const d = makeDart();
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
    const { sim } = flex(makeGlider());
    sim.run(10);
    expect(sim.damage()).toHaveLength(0);
  });

  it('the rigid model records crumpling too', () => {
    const { rigid } = flex(makeDart(), DEFAULT_CONFIG, crash);
    rigid.run(10);
    expect(rigid.damage().some((e) => e.kind === 'crumple')).toBe(true);
  });

  it('a damaged plane flies differently', () => {
    const d = makeDart();
    const { sim, model } = flex(d, DEFAULT_CONFIG, crash);
    sim.run(10);
    const r = damageToOps(model, sim.damage(), d.orientation.up, DEFAULT_CONFIG.paper);
    const fly = (design: DesignFile) => {
      const s = createSimulator(prepare(design, DEFAULT_CONFIG), DEFAULT_CONFIG, launch);
      while (!s.done) s.step();
      return s.log.firstImpact!.distance;
    };
    expect(Math.abs(fly({ ...d, ops: [...d.ops, ...r.ops] }) - fly(d))).toBeGreaterThan(0.05);
  });
});
