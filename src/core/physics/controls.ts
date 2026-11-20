/** Player control input: pitch and roll commands in [−1, 1]. ↑ = +pitch (climb), → = +roll (bank right). */
export interface ControlInput {
  /** Physics step from which this input applies. */
  step: number;
  pitch: number;
  roll: number;
}

export interface ControlConfig {
  /** Maximum flap deflection (deg). */
  maxDeflectionDeg: number;
  /** Deflection rate limit (deg/s). */
  rateDegPerS: number;
}

/** ±25°, full deflection in 0.2 s (player's choice: "±25°, snabba"). */
export const DEFAULT_CONTROLS: ControlConfig = { maxDeflectionDeg: 25, rateDegPerS: 125 };

/**
 * Replays or records a control schedule: a list of input changes keyed by physics step.
 * Live play appends changes; a replay reads the stored list — both give identical flights.
 */
export class ControlTrack {
  readonly inputs: ControlInput[];
  private cursor = 0;
  private current = { pitch: 0, roll: 0 };

  constructor(recorded?: ControlInput[]) {
    this.inputs = recorded ? [...recorded].sort((a, b) => a.step - b.step) : [];
  }

  /** Live input: record a change at `step` (no-op if unchanged). */
  set(step: number, pitch: number, roll: number) {
    const p = Math.max(-1, Math.min(1, pitch));
    const r = Math.max(-1, Math.min(1, roll));
    if (p === this.current.pitch && r === this.current.roll) return;
    this.current = { pitch: p, roll: r };
    this.inputs.push({ step, pitch: p, roll: r });
    this.cursor = this.inputs.length;
  }

  /** Command in effect at `step` (advances the replay cursor). */
  at(step: number): { pitch: number; roll: number } {
    while (this.cursor < this.inputs.length && this.inputs[this.cursor].step <= step) {
      const i = this.inputs[this.cursor++];
      this.current = { pitch: i.pitch, roll: i.roll };
    }
    return this.current;
  }
}
