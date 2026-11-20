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

  it('on the Sun there is nothing to land on', () => {
    expect(VENUES.sun.planes).toHaveLength(0);
    expect(VENUES.sun.burnAfterS).toBeCloseTo(0.2);
  });
});
