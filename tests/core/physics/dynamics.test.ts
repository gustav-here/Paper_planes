import { describe, expect, it } from 'vitest';
import * as V from '../../../src/core/math/vec3';
import * as Q from '../../../src/core/math/quat';
import * as M3 from '../../../src/core/math/mat3';
import { flatSheet, makeDart, makeGlider, trimmedDart } from '../../../src/core/presets';
import { airframeFromDesign } from '../../../src/core/physics/airframe';
import { makeConfig, DEFAULT_CONFIG } from '../../../src/core/physics/config';
import { FlightSim } from '../../../src/core/physics/simulation';
import { centerOfLift, stability, tunnelPoint } from '../../../src/core/physics/windTunnel';
import { computeAddedMass } from '../../../src/core/physics/addedMass';
import { attachedWeight, createAeroCache, ellipticWeight, evaluateAero } from '../../../src/core/physics/aeroModel';
import { prepare } from '../../../src/core/physics/factory';
import { replayDesign } from '../../../src/core/paper/replay';
import { findClipPlacement } from '../../../src/core/paper/clips';

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

  it('depends on the shape only: a clip moves the centre of mass, not the centre of lift', () => {
    const d = makeDart();
    const m = replayDesign(d).models.at(-1)!;
    const withClip = (x: number) => {
      const p = findClipPlacement(m, [x, 0, -0.01], 0.05)!;
      return airframeFromDesign({ ...d, ops: [...d.ops, { type: 'clip', id: 'g', at: p.at, edgeDir: p.edgeDir, massKg: 0.001 }] }, DEFAULT_CONFIG);
    };
    const front = withClip(0.12);
    const back = withClip(-0.12);
    const colFront = centerOfLift(front, DEFAULT_CONFIG)!;
    const colBack = centerOfLift(back, DEFAULT_CONFIG)!;
    expect(front.com[0] - back.com[0]).toBeGreaterThan(0.03);
    expect(Math.abs(colFront.point[0] - colBack.point[0])).toBeLessThan(0.002);
    // so the clip at the front leaves a bigger margin behind the centre of mass
    expect(colFront.behindCom).toBeGreaterThan(colBack.behindCom + 0.03);
  });
});

describe('skin friction', () => {
  it('builds up from the leading edge: a sheet flying long side first has less friction per area', () => {
    const along = airframeFromDesign(flatSheet, DEFAULT_CONFIG);
    const across = airframeFromDesign({ ...flatSheet, orientation: { nose: [0, 0, 1], up: [0, 1, 0] } }, DEFAULT_CONFIG);
    const cd = (af: typeof along) => tunnelPoint(af, DEFAULT_CONFIG, 5, 0).CD;
    expect(cd(along)).toBeLessThan(cd(across));
    // and about as for a laminar plate: 2·1.328/√Re (297 mm and 210 mm chords) plus the edges
    const nu = DEFAULT_CONFIG.aero.airViscosity / DEFAULT_CONFIG.rho;
    for (const [af, c] of [[along, 0.297], [across, 0.21]] as const)
      expect(cd(af)).toBeCloseTo(DEFAULT_CONFIG.aero.CDedge + (2 * 1.328) / Math.sqrt((5 * c) / nu), 3);
  });
});

describe('swept wings', () => {
  it('in sideslip the wing turned into the wind lifts more (it is swept less against the flow)', () => {
    const af = airframeFromDesign(makeDart(), DEFAULT_CONFIG); // flat, strongly swept wings
    const b = (8 * Math.PI) / 180;
    const a = (6 * Math.PI) / 180;
    // moving right through the air: the wind comes from the right
    const v = V.add(V.add(V.scale(af.nose, 5 * Math.cos(a) * Math.cos(b)), V.scale(af.up, -5 * Math.sin(a))), V.scale(af.right, 5 * Math.cos(a) * Math.sin(b)));
    const forces = new Float64Array(af.mesh.elements.length * 3);
    evaluateAero(af, v, [0, 0, 0], DEFAULT_CONFIG.aero, DEFAULT_CONFIG.rho, createAeroCache(af), { elementForces: forces });
    let windward = 0;
    let leeward = 0;
    af.mesh.elements.forEach((e, i) => {
      const side = V.dot(e.center, af.right);
      if (Math.abs(side) < 0.03) return; // the wings, not the keel
      const up = V.dot([forces[3 * i], forces[3 * i + 1], forces[3 * i + 2]], af.up);
      if (side > 0) windward += up;
      else leeward += up;
    });
    expect(windward).toBeGreaterThan(leeward * 1.05);
  });
});

describe('a keel under flat wings', () => {
  it('leaves the flat dart a little directionally unstable; tips turned up make it weathervane', () => {
    const cnb = (af: ReturnType<typeof airframeFromDesign>) => {
      const k = 0.5 * DEFAULT_CONFIG.rho * 25 * af.refArea * af.refSpan;
      const yaw = (b: number) => -V.dot(tunnelPoint(af, DEFAULT_CONFIG, 5, 6, b).torque, af.up) / k;
      return (yaw(4) - yaw(-4)) / ((8 * Math.PI) / 180);
    };
    const flat = airframeFromDesign(makeDart(), DEFAULT_CONFIG);
    expect(flat.mesh.surfaces.some((s) => s.underWing)).toBe(true);
    // nose-right moment falls with sideslip to the right: it turns away from the wind, but only a little
    expect(cnb(flat)).toBeLessThan(0);
    expect(cnb(flat)).toBeGreaterThan(-0.02);
    // with all of the keel's side force at its leading edge it was far more unstable
    const front = { ...DEFAULT_CONFIG, aero: { ...DEFAULT_CONFIG.aero, keelSideSpread: 0 } };
    const k0 = 0.5 * front.rho * 25 * flat.refArea * flat.refSpan;
    const yaw0 = (b: number) => -V.dot(tunnelPoint(airframeFromDesign(makeDart(), front), front, 5, 6, b).torque, flat.up) / k0;
    expect((yaw0(4) - yaw0(-4)) / ((8 * Math.PI) / 180)).toBeLessThan(cnb(flat) - 0.01);
  });
});

describe('rotational lift', () => {
  it('the elliptic chord loading integrates to 1 with its centre at ½ chord', () => {
    const n = 200;
    let sum = 0;
    let moment = 0;
    for (let i = 0; i < n; i++) {
      const w = ellipticWeight(i / n, (i + 1) / n) / n;
      sum += w;
      moment += w * ((i + 0.5) / n);
    }
    expect(sum).toBeCloseTo(1, 10);
    expect(moment).toBeCloseTo(0.5, 6);
  });

  it('a plate turning nose-up lifts as if its angle of attack were taken at ¾ chord', () => {
    const c = 0.297;
    const U = 5;
    const w = 2;
    const alpha = (3 * Math.PI) / 180;
    const v = V.add(V.scale(flat.nose, U * Math.cos(alpha)), V.scale(flat.up, -U * Math.sin(alpha)));
    const force = (k: number, rate: number) => {
      const cfg = makeConfig({ aero: { rotationalLift: k } });
      return evaluateAero(flat, v, V.scale(flat.right, rate), cfg.aero, cfg.rho, createAeroCache(flat)).force;
    };
    // without turning it adds nothing
    expect(V.dist(force(1, 0), force(0, 0))).toBeLessThan(1e-12);
    const extra = V.sub(force(1, w), force(0, w));
    // ΔC_L = a·ωc/(2U), a: Helmbold slope of the sheet (span 210 mm across the flow)
    const AR = (0.21 * 0.21) / (0.21 * c);
    const a = (DEFAULT_CONFIG.aero.a0 * AR) / (2 + Math.sqrt(AR * AR + 4));
    const expected = 0.5 * DEFAULT_CONFIG.rho * U * U * 0.21 * c * a * ((w * c) / (2 * U));
    expect(V.dot(extra, flat.up) / expected).toBeGreaterThan(0.9);
    expect(V.dot(extra, flat.up) / expected).toBeLessThan(1.02);
    // perpendicular to the flow (at each element): next to no drag
    expect(Math.abs(V.dot(extra, V.normalize(v)))).toBeLessThan(0.03 * V.length(extra));
    // nose-down turning loses the same lift
    expect(-V.dot(V.sub(force(1, -w), force(0, -w)), flat.up) / V.dot(extra, flat.up)).toBeGreaterThan(0.9);
  });

  it('damps the pitch of a plane whose centre of lift is behind its centre of mass, and drives one where it is ahead', () => {
    const cmq = (af: typeof flat, k: number) => {
      const cfg = makeConfig({ aero: { rotationalLift: k } });
      const a = (5 * Math.PI) / 180;
      const v = V.add(V.scale(af.nose, 5 * Math.cos(a)), V.scale(af.up, -5 * Math.sin(a)));
      const m = (q: number) => V.dot(evaluateAero(af, v, V.scale(af.right, q), cfg.aero, cfg.rho, createAeroCache(af)).torque, af.right);
      return m(1) - m(-1);
    };
    const dart = airframeFromDesign(trimmedDart(), DEFAULT_CONFIG);
    expect(cmq(dart, 1)).toBeLessThan(cmq(dart, 0));
    // the flat sheet's centre of lift is ahead of its centre of mass
    expect(cmq(flat, 1)).toBeGreaterThan(cmq(flat, 0));
  });
});

describe('twisted paper at a clip', () => {
  it('is one surface for the air: a clip at the tail moves the centre of mass, not the centre of lift', () => {
    const d = { ...trimmedDart(), flexible: true };
    const m = replayDesign(d).models.at(-1)!;
    const p = findClipPlacement(m, [-0.13, 0, -0.01], 0.05)!;
    const plain = prepare(d, DEFAULT_CONFIG);
    const clipped = prepare({ ...d, ops: [...d.ops, { type: 'clip', id: 'g', at: p.at, edgeDir: p.edgeDir, massKg: 0.001 }] }, DEFAULT_CONFIG);
    // the wing is one piece with one leading edge, not a row of narrow strips
    const wing = (af: typeof flat) => Math.max(...af.mesh.surfaces.filter((s) => Math.abs(V.dot(s.normal, af.up)) > 0.7).map((s) => s.area));
    expect(wing(clipped.af)).toBeGreaterThan(0.95 * wing(plain.af));
    expect(wing(clipped.flex!.af)).toBeGreaterThan(0.85 * wing(plain.flex!.af));
    const area = (pr: typeof plain) => pr.af.mesh.surfaces.reduce((s, x) => s + x.area, 0);
    expect(area(clipped)).toBeCloseTo(area(plain), 3);
    const col = (pr: typeof plain) => centerOfLift(pr.af, DEFAULT_CONFIG)!.point[0];
    // (a few mm remain: the clip holds the creases shut, so the shape itself differs a little;
    // with every strip its own leading edge it moved 23 mm)
    expect(Math.abs(col(clipped) - col(plain))).toBeLessThan(0.005);
  });
});
