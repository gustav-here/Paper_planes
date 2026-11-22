import type { DesignFile } from '../paper/ops';
import type { LaunchParams } from '../physics/launch';
import type { FlightLog } from '../physics/simulation';
import type { ControlInput } from '../physics/controls';

/*
 * Competition modes. Pure, deterministic game rules: the UI owns storage and randomness
 * (it passes a seed). A result can always be verified by replaying the stored design, launch
 * and control inputs through the deterministic simulation.
 */

export type GameMode = 'free' | 'time' | 'distance' | 'precision';

export interface ModeInfo {
  name: string;
  /** Throws per round (0 = unlimited, no round). */
  attempts: number;
  unit: string;
  /** Which way is better. */
  better: 'higher' | 'lower';
  description: string;
}

export const MODES: Record<GameMode, ModeInfo> = {
  free: { name: 'Fritt kast', attempts: 0, unit: 'm', better: 'higher', description: 'Kasta hur du vill. Distansen loggas.' },
  time: { name: 'Tid', attempts: 3, unit: 's', better: 'higher', description: 'Tre kast. Tiden till första nedslaget räknas.' },
  distance: { name: 'Längd', attempts: 3, unit: 'm', better: 'higher', description: 'Tre kast. Distansen till första nedslaget räknas.' },
  precision: {
    name: 'Precision',
    attempts: 3,
    unit: 'cm',
    better: 'lower',
    description: 'Tre kast mot den vita pricken 10 m rakt fram. Avståndet från första nedslaget räknas.',
  },
};

export interface Target {
  /** Centre on the floor (world x, z). */
  x: number;
  z: number;
  /** Ring radii (m), innermost first. */
  rings: number[];
}

export interface Round {
  mode: GameMode;
  seed: number;
  target: Target | null;
  /** Score per finished throw (null = no valid landing). */
  scores: (number | null)[];
}

/** Small deterministic PRNG (mulberry32). */
export function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** The precision target: a white dot, 10 cm in radius, on the floor 10 m straight ahead (+X). */
export const PRECISION_TARGET: Target = { x: 10, z: 0, rings: [0.1] };

/** The precision target (the same every round; the seed is kept for older rounds and replays). */
export function makeTarget(_seed: number): Target {
  return { ...PRECISION_TARGET, rings: [...PRECISION_TARGET.rings] };
}

export function newRound(mode: GameMode, seed: number): Round {
  return { mode, seed, target: mode === 'precision' ? makeTarget(seed) : null, scores: [] };
}

/** Score of one flight in a mode (null if it never touched the floor). */
export function scoreFlight(mode: GameMode, log: FlightLog, target: Target | null): number | null {
  const imp = log.firstImpact;
  if (!imp) return null;
  switch (mode) {
    case 'time':
      return imp.t;
    case 'free':
    case 'distance':
      return imp.distance;
    case 'precision':
      if (!target) return null;
      return Math.hypot(imp.pos[0] - target.x, imp.pos[2] - target.z) * 100; // cm
  }
}

export function recordScore(round: Round, score: number | null): Round {
  return { ...round, scores: [...round.scores, score] };
}

export const isRoundOver = (r: Round): boolean => MODES[r.mode].attempts > 0 && r.scores.length >= MODES[r.mode].attempts;

export function isBetter(mode: GameMode, a: number, b: number): boolean {
  return MODES[mode].better === 'higher' ? a > b : a < b;
}

/** The throws of a round, best first (throws without a landing last, in throwing order). */
export function rankThrows(r: Round): { throw: number; score: number | null }[] {
  return r.scores
    .map((score, i) => ({ throw: i + 1, score }))
    .sort((a, b) => {
      if (a.score === null || b.score === null) return a.score === null ? (b.score === null ? a.throw - b.throw : 1) : -1;
      return isBetter(r.mode, a.score, b.score) ? -1 : isBetter(r.mode, b.score, a.score) ? 1 : a.throw - b.throw;
    });
}

export function bestScore(r: Round): number | null {
  let best: number | null = null;
  for (const s of r.scores) if (s !== null && (best === null || isBetter(r.mode, s, best))) best = s;
  return best;
}

/** Everything needed to replay (and so verify) a throw exactly. */
export interface ThrowRecord {
  design: DesignFile;
  launch: LaunchParams;
  /** Control input changes per physics step (stage 5). */
  inputs: ControlInput[];
  target: Target | null;
}

export interface LeaderEntry extends ThrowRecord {
  mode: GameMode;
  /** Where it was thrown (older entries: the gym). */
  venue?: import('./venues').VenueId;
  score: number;
  designName: string;
  date: string;
}

/** Insert into a sorted top list (best first). Returns the new list and the rank (0-based) or -1. */
export function insertEntry(list: LeaderEntry[], entry: LeaderEntry, max = 10): { list: LeaderEntry[]; rank: number } {
  const all = [...list, entry].sort((a, b) => (isBetter(entry.mode, a.score, b.score) ? -1 : isBetter(entry.mode, b.score, a.score) ? 1 : 0));
  const top = all.slice(0, max);
  return { list: top, rank: top.indexOf(entry) };
}

export function formatScore(mode: GameMode, s: number | null): string {
  if (s === null) return '–';
  const d = mode === 'precision' ? 0 : 2;
  return `${s.toFixed(d).replace('.', ',')} ${MODES[mode].unit}`;
}
