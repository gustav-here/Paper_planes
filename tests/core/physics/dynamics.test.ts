import { describe, expect, it } from 'vitest';
import * as V from '../../../src/core/math/vec3';
import * as Q from '../../../src/core/math/quat';
import * as M3 from '../../../src/core/math/mat3';
import { flatSheet, makeDart, makeGlider } from '../../../src/core/presets';
import { airframeFromDesign } from '../../../src/core/physics/airframe';
import { makeConfig, DEFAULT_CONFIG } from '../../../src/core/physics/config';
import { FlightSim } from '../../../src/core/physics/simulation';
import { centerOfLift, stability, tunnelPoint } from '../../../src/core/physics/windTunnel';
import { computeAddedMass } from '../../../src/core/physics/addedMass';
import { attachedWeight } from '../../../src/core/physics/aeroModel';

const flat = airframeFromDesign(flatSheet, DEFAULT_CONFIG);

describe('chordwise loading', () => {
  it('attached weight integrates to 1 over the chord and peaks at the leading edge', () => {
    const n = 200;
    let sum = 0;
    let moment = 0;
    for (let i = 0; i < n; i++) {
      const w = attachedWeight(i / n, (i + 1) / n) / n;
      sum += w;
      moment += w * ((i + 0.5) / n);
    }
    expect(sum).toBeCloseTo(1, 10);
    expect(moment).toBeCloseTo(0.25, 2); // centre of pressure at quarter chord
    expect(attachedWeight(0, 0.1)).toBeGreaterThan(attachedWeight(0.9, 1));
  });

  it('flat plate: centre of pressure ≈ ¼ chord at small α, → ½ chord near 90°', () => {
    const c = 0.297;
    const cop = (alpha: number) => {
      const p = tunnelPoint(flat, DEFAULT_CONFIG, 5, alpha);
      // normal force along +y (up), pitching moment about the CoM (plate centre)
      const Fn = V.dot(p.force, flat.up);
      const M = V.dot(p.torque, flat.right);
      return 0.5 - M / Fn / c; // distance of CoP from the leading edge, in chords
    };
    expect(cop(3)).toBeGreaterThan(0.23);
    expect(cop(3)).toBeLessThan(0.27);
    expect(cop(60)).toBeGreaterThan(0.38);
    expect(cop(85)).toBeGreaterThan(0.46);
  });
});

describe('aerodynamic symmetry', () => {
  it('flat plate lift is antisymmetric in α', () => {
    for (const a of [3, 12, 40]) {
      expect(tunnelPoint(flat, DEFAULT_CONFIG, 5, -a).CL).toBeCloseTo(-tunnelPoint(flat, DEFAULT_CONFIG, 5, a).CL, 9);
    }
  });

  it('a symmetric dart has antisymmetric side force in sideslip', () => {
    const dart = airframeFromDesign(makeDart(), DEFAULT_CONFIG);
    const l = tunnelPoint(dart, DEFAULT_CONFIG, 5, 5, 8);
    const r = tunnelPoint(dart, DEFAULT_CONFIG, 5, 5, -8);
    expect(V.dot(l.force, dart.right)).toBeCloseTo(-V.dot(r.force, dart.right), 6);
    expect(V.dot(l.force, dart.up)).toBeCloseTo(V.dot(r.force, dart.up), 6);
  });
});

describe('added mass', () => {
  it('flat A4: about twice the paper mass for normal acceleration, nothing edgewise', () => {
    const { MA } = computeAddedMass(flat.mesh, 1.225, 1, 0.01);
    expect(MA[1 * 6 + 1] / flat.mass).toBeGreaterThan(1.5);
    expect(MA[1 * 6 + 1] / flat.mass).toBeLessThan(3);
    expect(MA[0]).toBeCloseTo(0, 12);
    expect(MA[2 * 6 + 2]).toBeCloseTo(0, 12);
  });
});

describe('rigid body dynamics', () => {
  it('free fall in vacuum follows ½gt²', () => {
    const cfg = makeConfig({ rho: 0 });
    const sim = new FlightSim(airframeFromDesign(flatSheet, cfg), cfg, { speed: 0, elevationDeg: 0, headingDeg: 0, heightM: 10 });
    while (sim.t < 0.5 - 1e-9) sim.step();
    expect(sim.state.x[1]).toBeCloseTo(10 - 0.5 * 9.81 * sim.t ** 2, 6);
  });

  it('torque-free tumbling conserves angular momentum and energy', () => {
    const cfg = makeConfig({ rho: 0, g: 0 });
    const dart = airframeFromDesign(makeDart(), cfg);
    const sim = new FlightSim(dart, cfg, { speed: 0, elevationDeg: 0, headingDeg: 0, heightM: 10 });
    sim.state.w = [4, -2, 3];
    const L = () => Q.rotate(Q.normalize(sim.state.q), M3.mulVec(dart.inertia, sim.state.w));
    const E = () => 0.5 * V.dot(sim.state.w, M3.mulVec(dart.inertia, sim.state.w));
    const L0 = L();
    const E0 = E();
    for (let i = 0; i < 480; i++) sim.step();
    expect(V.dist(L(), L0) / V.length(L0)).toBeLessThan(1e-6);
    expect(Math.abs(E() - E0) / E0).toBeLessThan(1e-6);
  });

  it('is deterministic', () => {
    const run = () => new FlightSim(airframeFromDesign(makeDart(), DEFAULT_CONFIG), DEFAULT_CONFIG, { speed: 6, elevationDeg: 5, headingDeg: 10, heightM: 1.6 }).run(4);
    expect(JSON.stringify(run())).toBe(JSON.stringify(run()));
  });

  it('the fixed 240 Hz step is converged (same flight at 960 Hz)', () => {
    const dart = airframeFromDesign(makeDart(), DEFAULT_CONFIG);
    const launch = { speed: 6, elevationDeg: 5, headingDeg: 0, heightM: 1.6 };
    const a = new FlightSim(dart, DEFAULT_CONFIG, launch).run(5).firstImpact!;
    const b = new FlightSim(dart, makeConfig({ substeps: 4 }), launch).run(5).firstImpact!;
    expect(Math.abs(a.distance - b.distance) / b.distance).toBeLessThan(0.02);
    expect(Math.abs(a.t - b.t)).toBeLessThan(0.02);
  });
});

describe('ground contact', () => {
  it('lands, loses energy, never sinks into the floor and comes to rest', () => {
    const dart = airframeFromDesign(makeDart(), DEFAULT_CONFIG);
    const sim = new FlightSim(dart, DEFAULT_CONFIG, { speed: 6, elevationDeg: -10, headingDeg: 0, heightM: 1.0 });
    let minY = Infinity;
    while (!sim.done) {
      sim.step();
      for (const p of sim.contactWorld()) minY = Math.min(minY, p[1]);
    }
    const log = sim.log;
    expect(log.phase).toBe('rest');
    expect(log.firstImpact).not.toBeNull();
    expect(log.rest!.distance).toBeGreaterThanOrEqual(log.firstImpact!.distance - 0.05); // slides on, not back
    expect(minY).toBeGreaterThan(-1.5e-3);
  });

  it('friction stops a sliding plane: higher μ, shorter slide', () => {
    const dart = airframeFromDesign(makeDart(), DEFAULT_CONFIG);
    const slide = (mu: number) => {
      const cfg = makeConfig({ ground: { friction: mu } });
      const log = new FlightSim(dart, cfg, { speed: 6, elevationDeg: -10, headingDeg: 0, heightM: 0.8 }).run(10);
      return log.rest!.distance - log.firstImpact!.distance;
    };
    expect(slide(0.8)).toBeLessThan(slide(0.2));
  });
});

describe('centre of lift', () => {
  it('lies behind the centre of mass of a plane that glides stably, ahead of it for a flat sheet', () => {
    for (const d of [makeDart(), makeGlider()]) {
      const af = airframeFromDesign(d, DEFAULT_CONFIG);
      const col = centerOfLift(af, DEFAULT_CONFIG)!;
      expect(col.behindCom).toBeGreaterThan(0);
      expect(stability(af, DEFAULT_CONFIG).cmAlpha!).toBeLessThan(0);
      // on the line through the centre of mass along the nose
      expect(V.length(V.rejectFrom(V.sub(col.point, af.com), af.nose))).toBeLessThan(1e-9);
    }
    const flat = centerOfLift(airframeFromDesign(flatSheet, DEFAULT_CONFIG), DEFAULT_CONFIG)!;
    expect(flat.behindCom).toBeLessThan(0);
  });
});
