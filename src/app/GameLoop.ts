/**
 * Fixed-timestep loop: physics always advances in steps of `dt` regardless of frame rate,
 * rendering interpolates with alpha = leftover / dt. Two limits keep the frame rate up when the
 * physics is heavier than real time (a complicated plane, a slow computer): at most
 * `maxStepsPerFrame` steps, and no more steps once `budgetMs` of computing has been spent in this
 * frame. The time left over is then dropped: the game runs slower instead of stalling (the
 * flight itself is the same, it is stepped with the same fixed dt).
 */
export class FixedStepper {
  private acc = 0;
  timeScale = 1;

  constructor(
    readonly dt = 1 / 240,
    readonly maxStepsPerFrame = 32,
    readonly budgetMs = 12,
    private readonly now: () => number = () => performance.now(),
  ) {}

  /** Advance by a frame of `frameDt` seconds; calls step(dt) zero or more times. Returns alpha. */
  advance(frameDt: number, step: (dt: number) => void): number {
    this.acc += Math.max(0, frameDt) * this.timeScale;
    const start = this.now();
    let n = 0;
    while (this.acc >= this.dt && n < this.maxStepsPerFrame) {
      step(this.dt);
      this.acc -= this.dt;
      n++;
      if (this.now() - start > this.budgetMs) break;
    }
    if (this.acc >= this.dt) this.acc = Math.min(this.acc, this.dt);
    return this.acc / this.dt;
  }

  reset() {
    this.acc = 0;
  }
}

/** requestAnimationFrame driver around a FixedStepper. */
export class GameLoop {
  readonly stepper: FixedStepper;
  private last = -1;
  private running = false;

  constructor(
    private onStep: (dt: number) => void,
    private onFrame: (frameDt: number, alpha: number) => void,
    dt = 1 / 240,
  ) {
    this.stepper = new FixedStepper(dt);
  }

  start() {
    if (this.running) return;
    this.running = true;
    const tick = (t: number) => {
      if (!this.running) return;
      const frameDt = this.last < 0 ? 0 : Math.min((t - this.last) / 1000, 0.25);
      this.last = t;
      const alpha = this.stepper.advance(frameDt, this.onStep);
      this.onFrame(frameDt, alpha);
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }

  stop() {
    this.running = false;
    this.last = -1;
  }
}
