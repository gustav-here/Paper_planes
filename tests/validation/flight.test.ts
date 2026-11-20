/**
 * Validation: behaviour that must EMERGE from geometry + mass with one global aero model.
 * Nothing here (or in the physics) knows which design is which.
 */
import { describe, expect, it } from 'vitest';
import { flatSheet, makeDart, makeGlider } from '../../src/core/presets';
import { airframeFromDesign, buildAirframe } from '../../src/core/physics/airframe';
import type { Airframe } from '../../src/core/physics/airframe';
import { DEFAULT_CONFIG } from '../../src/core/physics/config';
import { FlightSim } from '../../src/core/physics/simulation';
import { bestStableGlide, glideWindows, isStableGlide } from '../../src/core/physics/analysis';
import type { GlideWindow } from '../../src/core/physics/analysis';
import { stability } from '../../src/core/physics/windTunnel';
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

describe('validation', () => {
  const flat = airframeFromDesign(flatSheet, cfg);
  const dart = airframeFromDesign(makeDart(), cfg);
  const glider = airframeFromDesign(makeGlider(), cfg);

  it('a flat, unfolded sheet does not glide stably', () => {
    for (const v of [2, 4, 6, 8]) {
      for (const h of [1.6, TEST_HEIGHT]) {
        expect(bestStableGlide(fly(flat, v, h).samples), `v=${v} h=${h}`).toBeNull();
      }
    }
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
    const design = makeDart();
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
});
