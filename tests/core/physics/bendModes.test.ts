import { describe, expect, it } from 'vitest';
import { flatSheet, makeDart } from '../../../src/core/presets';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';
import type { PhysicsConfig } from '../../../src/core/physics/config';
import { createSimulator, prepare } from '../../../src/core/physics/factory';
import { MultiBodySim } from '../../../src/core/physics/multibody';
import { Poly, deflectionAt, limitStiffness } from '../../../src/core/physics/bendModes';
import { facetVertices } from '../../../src/core/paper/model';
import * as Q from '../../../src/core/math/quat';
import * as V from '../../../src/core/math/vec3';

const LAUNCH = { speed: 8, elevationDeg: 5, headingDeg: 0, heightM: 1.6 };

function fly(cfg: PhysicsConfig, design = flatSheet, maxT = 0.6) {
  const p = prepare({ ...design, flexible: true }, cfg);
  const sim = createSimulator(p, cfg, LAUNCH) as MultiBodySim;
  let maxBend = 0;
  while (!sim.done && sim.t < maxT && !sim.log.firstImpact) {
    sim.step();
    maxBend = Math.max(maxBend, sim.maxBendingM());
  }
  return { p, sim, maxBend };
}

describe('Poly', () => {
  it('differentiates its terms', () => {
    const P = new Poly([
      [2, 1, 3],
      [0, 2, -1],
    ]); // 3u²v − v²
    expect(P.w(2, 3)).toBeCloseTo(36 - 9);
    expect(P.wu(2, 3)).toBeCloseTo(36);
    expect(P.wv(2, 3)).toBeCloseTo(12 - 6);
    expect(P.wuu(2, 3)).toBeCloseTo(18);
    expect(P.wvv(2, 3)).toBeCloseTo(-2);
    expect(P.wuv(2, 3)).toBeCloseTo(12);
    expect(P.plus(P, -1).w(1.3, 0.7)).toBeCloseTo(0);
  });
});

describe('bending shapes', () => {
  it('a flat sheet gets free shapes, orthogonal to rigid motion, with positive stiffness', () => {
    const p = prepare({ ...flatSheet, flexible: true }, DEFAULT_CONFIG);
    const modes = p.flex!.modes;
    expect(modes.length).toBe(1);
    const pm = modes[0];
    const m = pm.shapes.length;
    expect(m).toBeGreaterThan(0);
    for (let k = 0; k < m; k++) {
      expect(pm.K[k * m + k]).toBeGreaterThan(0);
      expect(pm.limit[k]).toBeGreaterThan(0);
      // no net lift of the sheet: the shape does not move the centre of mass
      expect(Math.abs(pm.Spaper[k])).toBeLessThan(1e-6);
    }
  });

  it('can be switched off', () => {
    const cfg = { ...DEFAULT_CONFIG, paper: { ...DEFAULT_CONFIG.paper, bendingModes: false } };
    const { p, maxBend, sim } = fly(cfg, flatSheet, 0.2);
    expect(p.flex!.modes.length).toBe(0);
    expect(maxBend).toBe(0);
    expect(sim.panelBending().every((b) => b === null)).toBe(true);
  });

  it('a thrown sheet bends in the air, moderately, and the same way every time', () => {
    const a = fly(DEFAULT_CONFIG);
    const b = fly(DEFAULT_CONFIG);
    expect(a.maxBend).toBeGreaterThan(0.002);
    // well within the linear range of an A4 sheet (half length ≈ 150 mm)
    expect(a.maxBend).toBeLessThan(0.08);
    expect(b.maxBend).toBe(a.maxBend);
    expect(b.sim.position()).toEqual(a.sim.position());
  });

  it('stiffer paper bends less', () => {
    const soft = fly(DEFAULT_CONFIG);
    const stiff = fly({ ...DEFAULT_CONFIG, paper: { ...DEFAULT_CONFIG.paper, bendingStiffness: DEFAULT_CONFIG.paper.bendingStiffness * 10 } });
    expect(stiff.maxBend).toBeLessThan(soft.maxBend);
  });

  it('the wings of a dart bend, and the render bending matches the shapes', () => {
    const { sim, maxBend } = fly(DEFAULT_CONFIG, makeDart(), 0.4);
    expect(maxBend).toBeGreaterThan(0.0005);
    expect(maxBend).toBeLessThan(0.08);
    const bends = sim.panelBending();
    expect(bends.length).toBe(sim.panels.length);
    const modes = (sim as unknown as { fx: { modes: { panel: number }[] } }).fx.modes;
    for (const pm of modes) expect(bends[pm.panel]).not.toBeNull();
  });

  it('a bent sheet lying on the floor stays on top of it, all over', () => {
    const p = prepare({ ...flatSheet, flexible: true }, DEFAULT_CONFIG);
    const sim = createSimulator(p, DEFAULT_CONFIG, { speed: 6, elevationDeg: 0, headingDeg: 0, heightM: 1.6 }) as MultiBodySim;
    while (!sim.done) sim.step();
    expect(sim.phase).toBe('rest');
    const pose = sim.panelPoses()[0];
    const bend = sim.panelBending()[0]!;
    const f = p.model.facets[0];
    const vs = facetVertices(f);
    let lowest = Infinity;
    // a grid over the sheet, bent and placed in the world
    for (let a = 0; a <= 10; a++)
      for (let b = 0; b <= 10; b++) {
        const pt = V.lerp(V.lerp(vs[0], vs[1], a / 10), V.lerp(vs[3], vs[2], a / 10), b / 10);
        const w = V.add(Q.rotate(pose.q, V.addScaled(pt, bend.normal, bend.at(pt).w)), pose.t);
        lowest = Math.min(lowest, w[1]);
      }
    expect(lowest).toBeGreaterThan(-0.001);
  });

  it('stiffens beyond the linear range', () => {
    const pm = prepare({ ...flatSheet, flexible: true }, DEFAULT_CONFIG).flex!.modes[0];
    const paper = DEFAULT_CONFIG.paper;
    const lim = pm.limit[0];
    expect(limitStiffness(pm, 0, 0, paper)).toBe(0);
    // small below the limit, large at it, symmetric
    expect(limitStiffness(pm, 0, 0.3 * lim, paper)).toBeLessThan(0.03 * pm.K[0]); // 30 · 0.3⁶ ≈ 0.022
    expect(limitStiffness(pm, 0, lim, paper)).toBeCloseTo(paper.bendingLimitStiffening * pm.K[0]);
    expect(limitStiffness(pm, 0, -lim, paper)).toBeCloseTo(limitStiffness(pm, 0, lim, paper));
    // at the limit amplitude the steepest slope is the limit slope
    const q = new Array<number>(pm.first + pm.shapes.length).fill(0);
    q[pm.first] = lim;
    let steepest = 0;
    for (const e of pm.elements) {
      const g = e.grad[0];
      steepest = Math.max(steepest, Math.hypot(g[0], g[1], g[2]) * lim);
    }
    expect(steepest).toBeLessThanOrEqual(Math.tan((paper.bendingLimitSlopeDeg * Math.PI) / 180) + 1e-9);
    expect(deflectionAt(pm, q, pm.origin).w).toBeLessThanOrEqual(lim);
  });
});
