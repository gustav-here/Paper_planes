import { describe, expect, it } from 'vitest';
import {
  bestScore,
  insertEntry,
  isRoundOver,
  makeTarget,
  newRound,
  recordScore,
  scoreFlight,
} from '../../../src/core/game/competition';
import type { LeaderEntry } from '../../../src/core/game/competition';
import { ControlTrack } from '../../../src/core/physics/controls';
import { airframeFromDesign } from '../../../src/core/physics/airframe';
import { DEFAULT_CONFIG } from '../../../src/core/physics/config';
import { FlightSim } from '../../../src/core/physics/simulation';
import { makeGlider } from '../../../src/core/presets';

describe('competition rules', () => {
  it('targets are deterministic per seed and 3–10 m out', () => {
    expect(makeTarget(42)).toEqual(makeTarget(42));
    for (let s = 0; s < 50; s++) {
      const t = makeTarget(s);
      const d = Math.hypot(t.x, t.z);
      expect(d).toBeGreaterThanOrEqual(2.99);
      expect(d).toBeLessThanOrEqual(10.01);
    }
  });

  it('rounds keep the best of three; precision prefers the smallest miss', () => {
    let r = newRound('distance', 1);
    r = recordScore(r, 4);
    r = recordScore(r, null);
    expect(isRoundOver(r)).toBe(false);
    r = recordScore(r, 6.5);
    expect(isRoundOver(r)).toBe(true);
    expect(bestScore(r)).toBe(6.5);

    let p = newRound('precision', 1);
    p = recordScore(recordScore(recordScore(p, 80), 12), 30);
    expect(bestScore(p)).toBe(12);
  });

  it('scores a real flight per mode', () => {
    const af = airframeFromDesign(makeGlider(), DEFAULT_CONFIG);
    const log = new FlightSim(af, DEFAULT_CONFIG, { speed: 5, elevationDeg: 0, headingDeg: 0, heightM: 1.6 }).run(10);
    expect(scoreFlight('time', log, null)).toBeCloseTo(log.firstImpact!.t, 12);
    expect(scoreFlight('distance', log, null)).toBeCloseTo(log.firstImpact!.distance, 12);
    const target = { x: log.firstImpact!.pos[0] + 0.1, z: log.firstImpact!.pos[2], rings: [0.25] };
    expect(scoreFlight('precision', log, target)).toBeCloseTo(10, 6);
  });

  it('leaderboard keeps the best ten, best first', () => {
    const e = (score: number): LeaderEntry => ({
      mode: 'time',
      score,
      designName: 'x',
      date: '',
      design: makeGlider(),
      launch: { speed: 5, elevationDeg: 0, headingDeg: 0, heightM: 1.6 },
      inputs: [],
      target: null,
    });
    let list: LeaderEntry[] = [];
    for (const s of [1, 5, 3, 2, 9, 4, 7, 8, 6, 0.5, 10]) list = insertEntry(list, e(s)).list;
    expect(list.map((x) => x.score)).toEqual([10, 9, 8, 7, 6, 5, 4, 3, 2, 1]);
    expect(insertEntry(list, e(0.1)).rank).toBe(-1);
    expect(insertEntry(list, e(9.5)).rank).toBe(1);
  });

  it('control tracks replay recorded inputs exactly', () => {
    const live = new ControlTrack();
    live.set(10, 1, 0);
    live.set(10, 1, 0); // unchanged → not recorded twice
    live.set(50, 0, -0.5);
    expect(live.inputs).toHaveLength(2);
    const replay = new ControlTrack(live.inputs);
    expect(replay.at(0)).toEqual({ pitch: 0, roll: 0 });
    expect(replay.at(10)).toEqual({ pitch: 1, roll: 0 });
    expect(replay.at(49)).toEqual({ pitch: 1, roll: 0 });
    expect(replay.at(50)).toEqual({ pitch: 0, roll: -0.5 });
  });
});
