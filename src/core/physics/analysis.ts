import * as V from '../math/vec3';
import type { FlightSample } from './simulation';

const DEG = Math.PI / 180;

export interface GlideWindow {
  t0: number;
  t1: number;
  /** Mean sink rate −v_y (m/s). */
  sinkRate: number;
  /** Mean airspeed (m/s). */
  speed: number;
  /** Horizontal speed / sink rate. */
  glideRatio: number;
  /** Standard deviation of the nose pitch (deg). */
  pitchStd: number;
  /** Mean |ω| (rad/s). */
  meanOmega: number;
  /** Standard deviation of the flight-path angle (deg). */
  pathStd: number;
}

/** Criteria for a "stable glide" (shared by all validation tests). */
export const STABLE_GLIDE = { windowS: 1, maxPitchStdDeg: 3, maxPathStdDeg: 3, maxMeanOmega: 0.5, minGlideRatio: 2 };

function stats(xs: number[]) {
  const m = xs.reduce((a, b) => a + b, 0) / xs.length;
  const sd = Math.sqrt(xs.reduce((a, b) => a + (b - m) ** 2, 0) / xs.length);
  return { m, sd };
}

/** Evaluate every window of `windowS` seconds in the airborne part of a flight. */
export function glideWindows(samples: FlightSample[], windowS = STABLE_GLIDE.windowS): GlideWindow[] {
  const air = samples.filter((s) => s.phase === 'flight');
  const out: GlideWindow[] = [];
  let j = 0;
  for (let i = 0; i < air.length; i++) {
    while (j < air.length && air[j].t - air[i].t < windowS) j++;
    if (j >= air.length) break;
    const w = air.slice(i, j + 1);
    const sink = stats(w.map((s) => -s.vel[1]));
    const horiz = stats(w.map((s) => Math.hypot(s.vel[0], s.vel[2])));
    const pitch = stats(w.map((s) => s.pitchDeg));
    const path = stats(w.map((s) => Math.atan2(s.vel[1], Math.hypot(s.vel[0], s.vel[2])) / DEG));
    const omega = stats(w.map((s) => V.length(s.w)));
    const speed = stats(w.map((s) => V.length(s.vel)));
    out.push({
      t0: w[0].t,
      t1: w[w.length - 1].t,
      sinkRate: sink.m,
      speed: speed.m,
      glideRatio: sink.m > 1e-6 ? horiz.m / sink.m : Infinity,
      pitchStd: pitch.sd,
      meanOmega: omega.m,
      pathStd: path.sd,
    });
  }
  return out;
}

export function isStableGlide(w: GlideWindow): boolean {
  return (
    w.pitchStd < STABLE_GLIDE.maxPitchStdDeg &&
    w.pathStd < STABLE_GLIDE.maxPathStdDeg &&
    w.meanOmega < STABLE_GLIDE.maxMeanOmega &&
    w.glideRatio > STABLE_GLIDE.minGlideRatio &&
    w.sinkRate > 0
  );
}

/** The stable window with the lowest sink rate, or null if the flight never glides stably. */
export function bestStableGlide(samples: FlightSample[]): GlideWindow | null {
  const stable = glideWindows(samples).filter(isStableGlide);
  if (stable.length === 0) return null;
  return stable.reduce((a, b) => (b.sinkRate < a.sinkRate ? b : a));
}
