import { describe, expect, it } from 'vitest';
import { flatSheet, makeDart, makeGlider } from '../../../src/core/presets';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';
import { createSimulator, prepare } from '../../../src/core/physics/factory';
import { GYM, VENUES, venueConfig } from '../../../src/core/game/venues';
import type { LaunchParams } from '../../../src/core/physics/launch';

const DEG = Math.PI / 180;

describe('venues', () => {
  it('the gym is the default kind of place: Earth gravity and air', () => {
    const c = venueConfig('gym', DEFAULT_CONFIG);
    expect(c.g).toBe(9.81);
    expect(c.rho).toBe(DEFAULT_CONFIG.rho);
  });

  it('on the Moon there is no air: every plane flies the same ballistic arc', () => {
    const cfg = venueConfig('moon', DEFAULT_CONFIG);
    const launch: LaunchParams = { speed: 6, elevationDeg: 30, headingDeg: 0, heightM: 1.6 };
    const fly = (d: typeof flatSheet, flexible: boolean) => {
      const s = createSimulator(prepare({ ...d, flexible }, cfg), cfg, launch);
      while (!s.log.firstImpact && s.t < cfg.maxTime) s.step();
      return s.log.firstImpact!;
    };
    // centre of mass on a parabola: x = v cosθ t, y = h + v sinθ t − g t²/2
    const v = launch.speed;
    const th = launch.elevationDeg * DEG;
    const tFlight = (v * Math.sin(th) + Math.sqrt((v * Math.sin(th)) ** 2 + 2 * cfg.g * launch.heightM)) / cfg.g;
    const range = v * Math.cos(th) * tFlight;
    for (const d of [flatSheet, makeDart(), makeGlider()]) {
      for (const flexible of [false, true]) {
        const imp = fly(d, flexible);
        // the paper touches down a little before its centre would (it is up to 15 cm long)
        expect(Math.abs(imp.distance - range)).toBeLessThan(0.4);
        expect(Math.abs(imp.t - tFlight)).toBeLessThan(0.15);
      }
    }
    // far longer than on Earth
    expect(range).toBeGreaterThan(20);
  });

  it('in the gym the walls stop the plane', () => {
    // the gym's walls without air, so the paper surely reaches the side wall
    const cfg = { ...venueConfig('gym', DEFAULT_CONFIG), rho: 0 };
    const p = prepare(makeDart(), cfg);
    const s = createSimulator(p, cfg, { speed: 15, elevationDeg: 10, headingDeg: 90, heightM: 2 });
    let maxZ = -Infinity;
    while (!s.done && s.t < 5) {
      s.step();
      maxZ = Math.max(maxZ, s.position()[2]);
    }
    expect(s.log.firstImpact).not.toBeNull();
    expect(s.log.firstImpact!.pos[2]).toBeGreaterThan(GYM.halfWidth - 0.4);
    expect(maxZ).toBeLessThan(GYM.halfWidth);
  });

  it('on the Sun there is nothing to land on, and the paper burns at once', () => {
    expect(VENUES.sun.planes).toHaveLength(0);
    expect(VENUES.sun.burnAfterS).toBe(0);
  });

  // twelve flexible landings followed until the paper lies still: slow on a loaded CI runner
  it('on the Moon a hard landing does not hop or cartwheel: it sinks into the dust and stops there', { timeout: 120000 }, () => {
    const cfg = venueConfig('moon', DEFAULT_CONFIG);
    for (const d of [makeDart(), makeGlider()])
      for (const flexible of [false, true]) {
        const p = prepare({ ...d, flexible }, cfg);
        // straight, and thrown a little off as real throws are (no air: the spin of the throw
        // is still there when it lands, and that used to turn into hops metres high)
        for (const scatterSeed of [undefined, 4 * 7919, 8 * 7919]) {
          const s = createSimulator(p, cfg, { speed: 14, elevationDeg: 45, headingDeg: 0, heightM: 1.6, scatterSeed });
          let top = 0;
          while (!s.done) {
            s.step();
            if (s.log.firstImpact) top = Math.max(top, s.position()[1]);
          }
          const a = s.log.firstImpact!;
          const r = s.log.rest!;
          // no hop: the centre of mass stays about as high as the plane is tall
          expect(top).toBeLessThan(0.35);
          expect(Math.hypot(r.pos[0] - a.pos[0], r.pos[2] - a.pos[2])).toBeLessThan(1);
          // it rests on the dust, sunk in a little at most
          expect(r.pos[1]).toBeGreaterThan(-0.03);
        }
      }
  });

  it('on the Moon the paper stops where it lands: it ploughs into the loose dust', () => {
    // a flat, fast landing: friction alone (μ·g in the weak gravity) would let it slide on
    const launch: LaunchParams = { speed: 9, elevationDeg: 0, headingDeg: 0, heightM: 1.6 };
    const slide = (plough: number | null, flexible: boolean) => {
      const c0 = venueConfig('moon', DEFAULT_CONFIG);
      const cfg = plough === null ? c0 : { ...c0, ground: { ...c0.ground, ploughKgPerM: plough } };
      const s = createSimulator(prepare({ ...makeDart(), flexible }, cfg), cfg, launch);
      while (!s.done) s.step();
      const a = s.log.firstImpact!;
      const r = s.log.rest!;
      return Math.hypot(r.pos[0] - a.pos[0], r.pos[2] - a.pos[2]);
    };
    for (const flexible of [false, true]) {
      expect(slide(null, flexible)).toBeLessThan(0.4);
      expect(slide(0, flexible)).toBeGreaterThan(2);
    }
  });
});
