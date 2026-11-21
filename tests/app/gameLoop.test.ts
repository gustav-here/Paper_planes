import { describe, expect, it } from 'vitest';
import { FixedStepper } from '../../src/app/GameLoop';

describe('FixedStepper', () => {
  it('runs the same number of steps regardless of frame rate', () => {
    const count = (fps: number) => {
      const s = new FixedStepper(1 / 240);
      let n = 0;
      for (let i = 0; i < fps; i++) s.advance(1 / fps, () => n++);
      return n;
    };
    expect(count(60)).toBe(240);
    expect(Math.abs(count(144) - 240)).toBeLessThanOrEqual(1);
    expect(Math.abs(count(30) - 240)).toBeLessThanOrEqual(1);
  });

  it('caps steps after a stall', () => {
    const s = new FixedStepper(1 / 240, 32);
    let n = 0;
    s.advance(5, () => n++);
    expect(n).toBe(32);
  });

  it('time scale slows simulation time', () => {
    const s = new FixedStepper(1 / 240);
    s.timeScale = 0.25;
    let n = 0;
    for (let i = 0; i < 60; i++) s.advance(1 / 60, () => n++);
    expect(n).toBe(60);
  });

  it('heavy physics: stops at the time budget and drops the rest instead of stalling', () => {
    let clock = 0;
    // every step costs 3 ms of computing
    const s = new FixedStepper(1 / 240, 32, 12, () => clock);
    let n = 0;
    const step = () => {
      clock += 3;
      n++;
    };
    s.advance(1 / 30, step); // 8 steps due
    expect(n).toBe(5); // 15 ms > 12 ms after the fifth
    n = 0;
    // the backlog is gone: the next frame does not try to catch up
    s.advance(1 / 60, step);
    expect(n).toBeLessThanOrEqual(5);
  });
});
