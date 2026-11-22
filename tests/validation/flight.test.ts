/**
 * Validation: behaviour that must EMERGE from geometry + mass with one global aero model.
 * Nothing here (or in the physics) knows which design is which.
 */
import { describe, expect, it } from 'vitest';
import { NO_REFLEX, flatSheet, makeDart, makeGlider, trimmedDart } from '../../src/core/presets';
import { createSimulator, prepare } from '../../src/core/physics/factory';
import type { FoldOp } from '../../src/core/paper/ops';
import * as V from '../../src/core/math/vec3';
import * as Q from '../../src/core/math/quat';
import { airframeFromDesign, buildAirframe } from '../../src/core/physics/airframe';
import type { Airframe } from '../../src/core/physics/airframe';
import { DEFAULT_CONFIG } from '../../src/core/physics/config';
import { FlightSim } from '../../src/core/physics/simulation';
import { bestStableGlide, glideWindows, isStableGlide } from '../../src/core/physics/analysis';
import type { GlideWindow } from '../../src/core/physics/analysis';
import { stability, tunnelPoint } from '../../src/core/physics/windTunnel';
import { replayDesign, applyOp } from '../../src/core/paper/replay';
import { findClipPlacement } from '../../src/core/paper/clips';
import type { Vec3 } from '../../src/core/math/vec3';

const cfg = DEFAULT_CONFIG;
const SPEEDS = [3, 5, 8];
/** High release so the plane has time to settle into its natural glide. */
const TEST_HEIGHT = 10;

function fly(af: Airframe, speed: number, heightM = TEST_HEIGHT) {
  return new FlightSim(af, cfg, { speed, elevationDeg: 0, headingDeg: 0, heightM }).run(20);
}

/** Lowest sink rate found in any stable glide window over a sweep of throw speeds. */
function minStableSink(af: Airframe): GlideWindow | null {
  let best: GlideWindow | null = null;
  for (const v of SPEEDS) {
    const w = bestStableGlide(fly(af, v).samples);
    if (w && (!best || w.sinkRate < best.sinkRate)) best = w;
  }
  return best;
}

/** The glide the plane settles into (last full window before landing). */
function settledGlide(af: Airframe): GlideWindow {
  return glideWindows(fly(af, 5).samples).at(-1)!;
}

// real flights: allow for a busy machine
describe('validation', { timeout: 240_000 }, () => {
  const flat = airframeFromDesign(flatSheet, cfg);
  // a dart trimmed to glide (the plain preset is untrimmed and dives)
  const dart = airframeFromDesign(trimmedDart(), cfg);
  const glider = airframeFromDesign(makeGlider(), cfg);

  it('a flat, unfolded sheet does not glide stably', () => {
    // short throws at several speeds, and long drops (where a stable glide would show) at two
    for (const v of [2, 4, 6, 8]) expect(bestStableGlide(fly(flat, v, 1.6).samples), `v=${v} h=1.6`).toBeNull();
    for (const v of [3, 6]) expect(bestStableGlide(fly(flat, v, TEST_HEIGHT).samples), `v=${v} h=${TEST_HEIGHT}`).toBeNull();
    // the wind tunnel agrees: no stable trim point (pitch moment grows with α)
    expect(stability(flat, cfg).trimAlphaDeg).toBeNull();
  });

  it('the folded models do glide stably (sanity check for the criterion)', () => {
    expect(isStableGlide(settledGlide(dart))).toBe(true);
    expect(isStableGlide(settledGlide(glider))).toBe(true);
  });

  it('the wide-wing glider sinks more slowly than the dart', () => {
    const g = minStableSink(glider)!;
    const d = minStableSink(dart)!;
    expect(g).not.toBeNull();
    expect(d).not.toBeNull();
    expect(g.sinkRate).toBeLessThan(d.sinkRate);
    // also in the glide each one settles into by itself
    expect(settledGlide(glider).sinkRate).toBeLessThan(settledGlide(dart).sinkRate);
  });

  it('moving the centre of mass forward with a clip: more stable, lower trim α, faster and steeper glide', () => {
    const design = trimmedDart();
    const model = replayDesign(design).models.at(-1)!;
    const withClipAt = (x: number) => {
      const p = findClipPlacement(model, [x, 0, 0] as Vec3, 0.02)!;
      return buildAirframe(applyOp(model, { type: 'clip', id: 'c', at: p.at, edgeDir: p.edgeDir, massKg: 0.001 }).model, design.orientation, cfg);
    };
    const nose = withClipAt(0.1);
    const tail = withClipAt(-0.12);
    expect(nose.com[0]).toBeGreaterThan(dart.com[0]);
    expect(tail.com[0]).toBeLessThan(dart.com[0]);

    const s0 = stability(dart, cfg);
    const sN = stability(nose, cfg);
    const sT = stability(tail, cfg);
    // static stability: dCm/dα more negative with the nose clip, less with the tail clip
    expect(sN.cmAlpha!).toBeLessThan(s0.cmAlpha!);
    expect(sT.cmAlpha!).toBeGreaterThan(s0.cmAlpha!);
    // trim angle of attack: lower with the nose clip, higher with the tail clip
    expect(sN.trimAlphaDeg!).toBeLessThan(s0.trimAlphaDeg!);
    expect(sT.trimAlphaDeg!).toBeGreaterThan(s0.trimAlphaDeg!);

    // free flight: the nose-heavy plane settles into a faster, steeper glide
    const g0 = settledGlide(dart);
    const gN = settledGlide(nose);
    expect(gN.speed).toBeGreaterThan(g0.speed);
    expect(gN.glideRatio).toBeLessThan(g0.glideRatio);
  });

  /*
   * Folding the wings out of plane trades lift for stability. Lateral stability in the wind
   * tunnel at α = 6°: Clβ (roll from sideslip, negative = rights itself) and Cnβ (yaw from
   * sideslip, positive = turns into the wind), and the lift at the same α and speed.
   */
  describe('wing shapes: stability against lift', () => {
    const keel = 0.025;
    const span = 0.105 - keel;
    const th = (deg: number) => (deg * Math.PI) / 180;
    /** A fold along the right wing (mirrored) s out from its root, turning the outer part up (+) or down. */
    const wingFold = (wingAngleDeg: number, s: number, upDeg: number): FoldOp => {
      const t = th(wingAngleDeg);
      const w: Vec3 = [0, Math.sin(t), Math.cos(t)];
      const on = (x: number): Vec3 => [x, s * w[1], keel + s * w[2]];
      return { type: 'fold', id: 'w', line: { p0: on(-0.2), p1: on(0.2) }, planeNormal: [0, -Math.cos(t), Math.sin(t)], movingSide: w, kind: upDeg > 0 ? 'valley' : 'mountain', angleDeg: Math.abs(upDeg), scope: 'all', mirror: true };
    };
    const shape = (wingAngleDeg: number, extra?: FoldOp) => {
      const d = makeDart(keel, wingAngleDeg, undefined, NO_REFLEX);
      return extra ? { ...d, ops: [...d.ops, extra] } : d;
    };
    const measure = (d: ReturnType<typeof makeDart>) => {
      const af = airframeFromDesign(d, cfg);
      const q = 0.5 * cfg.rho * 25;
      const k = q * af.refArea * af.refSpan;
      const Cl = (b: number) => V.dot(tunnelPoint(af, cfg, 5, 6, b).torque, af.nose) / k;
      const Cn = (b: number) => -V.dot(tunnelPoint(af, cfg, 5, 6, b).torque, af.up) / k;
      const dB = th(8);
      return { lift: tunnelPoint(af, cfg, 5, 6).CL * q * af.refArea, Clb: (Cl(4) - Cl(-4)) / dB, Cnb: (Cn(4) - Cn(-4)) / dB };
    };
    const flat = measure(shape(90));
    const variants = {
      dihedral: measure(shape(70)),
      anhedral: measure(shape(110)),
      littleAnhedral: measure(shape(100)),
      tipsUp: measure(shape(90, wingFold(90, span - 0.02, 80))),
      tipsDown: measure(shape(90, wingFold(90, span - 0.02, -80))),
      deepW: measure(shape(110, wingFold(110, span * 0.45, 40))),
      shallowW: measure(shape(97, wingFold(97, span * 0.45, 14))),
    };

    it('every shape out of the plane lifts less than flat wings', () => {
      for (const [name, v] of Object.entries(variants)) expect(v.lift, name).toBeLessThan(flat.lift);
    });

    it('dihedral rights itself in roll better than flat wings; dihedral and a deep W weathervane better', () => {
      expect(variants.dihedral.Clb).toBeLessThan(flat.Clb);
      for (const v of [variants.dihedral, variants.deepW]) expect(v.Cnb).toBeGreaterThan(flat.Cnb);
    });

    it('wing tips folded up or down act as fins: the plane turns into the wind more', () => {
      expect(variants.tipsUp.Cnb).toBeGreaterThan(flat.Cnb + 0.005);
      expect(variants.tipsDown.Cnb).toBeGreaterThan(flat.Cnb + 0.005);
    });

    it('a lot of anhedral rolls the wrong way (it can tip over); a little still rights itself on its keel', () => {
      expect(variants.anhedral.Clb).toBeGreaterThan(0);
      expect(variants.littleAnhedral.Clb).toBeLessThan(0);
    });

    it('a shallow W changes little, a deep W much more (in lift and stability)', () => {
      const d = (v: typeof flat) => Math.abs(v.lift - flat.lift) / flat.lift;
      expect(d(variants.shallowW)).toBeLessThan(d(variants.deepW) / 3);
      // the deep W's turned-up outer parts weathervane it; a shallow one hardly changes that (in
      // roll the swept flat wing is already steady, and the W's droop and upturn nearly cancel)
      expect(Math.abs(variants.shallowW.Cnb - flat.Cnb)).toBeLessThan(Math.abs(variants.deepW.Cnb - flat.Cnb) / 3);
    });
  });

  it('real throws are a little off: no two alike, and the same seed gives the same flight', () => {
    const p = prepare(trimmedDart(), cfg);
    const fly = (seed: number) => {
      const s = createSimulator(p, cfg, { speed: 6, elevationDeg: 0, headingDeg: 0, heightM: 1.6, scatterSeed: seed });
      while (!s.done && !s.log.firstImpact) s.step();
      return s.log.firstImpact!.pos;
    };
    expect(fly(7)).toEqual(fly(7));
    const landings = [1, 2, 3, 4, 5, 6].map((s) => fly(s));
    expect(new Set(landings.map((p) => `${p[0].toFixed(4)},${p[2].toFixed(4)}`)).size).toBe(6);
    expect(Math.max(...landings.map((p) => Math.abs(p[2])))).toBeGreaterThan(0.1);
  });

  it('the harder the throw, the likelier the plane climbs and stalls, but not every time', () => {
    const p = prepare(trimmedDart(), cfg);
    const af = p.af;
    const stalls = (speed: number) => {
      let n = 0;
      for (let seed = 1; seed <= 16; seed++) {
        const s = createSimulator(p, cfg, { speed, elevationDeg: 0, headingDeg: 0, heightM: 1.6, scatterSeed: seed * 104729 });
        let stalled = false;
        while (!s.done && !s.log.firstImpact) {
          s.step();
          const q = s.panelPoses()[0].q;
          const v = V.normalize(s.worldVelocity());
          const alpha = Math.atan2(-V.dot(v, Q.rotate(q, af.up)), V.dot(v, Q.rotate(q, af.nose)));
          if (alpha > (25 * Math.PI) / 180 && s.speed() < 0.6 * speed) stalled = true;
        }
        if (stalled) n++;
      }
      return n;
    };
    const gentle = stalls(5);
    const firm = stalls(11);
    const hard = stalls(13);
    expect(gentle).toBe(0);
    // thrown firmly it sometimes climbs into a stall, not always; harder, more often
    expect(firm).toBeGreaterThan(0);
    expect(firm).toBeLessThan(16);
    expect(hard).toBeGreaterThanOrEqual(firm);
  });
});
