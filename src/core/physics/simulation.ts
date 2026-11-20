import * as V from '../math/vec3';
import * as Q from '../math/quat';
import type { Vec3 } from '../math/vec3';
import type { Quat } from '../math/quat';
import type { Airframe } from './airframe';
import { createAeroCache } from './aeroModel';
import type { AeroCache, AeroDebug } from './aeroModel';
import type { PhysicsConfig } from './config';
import { cloneState, derivative, pointWorld, rk4Step } from './rigidBody';
import type { BodyState } from './rigidBody';
import { resolveGround } from './groundContact';
import { launchState } from './launch';
import type { LaunchParams } from './launch';

export type FlightPhase = 'flight' | 'ground' | 'rest' | 'timeout';

export interface FlightSample {
  t: number;
  pos: Vec3;
  /** World velocity. */
  vel: Vec3;
  q: Quat;
  /** Body angular velocity. */
  w: Vec3;
  phase: FlightPhase;
  /** Nose pitch above horizontal (deg). */
  pitchDeg: number;
  /** Body angle of attack in the symmetry plane (deg). */
  alphaDeg: number;
}

export interface FlightEvent {
  t: number;
  pos: Vec3;
  speed: number;
  /** Horizontal distance from the release point (m). */
  distance: number;
}

export interface FlightLog {
  launch: LaunchParams;
  samples: FlightSample[];
  /** The official result: first touch of the floor. */
  firstImpact: FlightEvent | null;
  rest: FlightEvent | null;
  maxHeight: number;
  phase: FlightPhase;
}

const RAD = 180 / Math.PI;

/** Fixed-step, deterministic flight of one airframe from a launch. */
export class FlightSim {
  state: BodyState;
  t = 0;
  phase: FlightPhase = 'flight';
  readonly log: FlightLog;
  private cache: AeroCache;
  private restTimer = 0;
  /** Smoothed |v| and |ω| on the ground (contact jitter must not keep the plane "moving"). */
  private emaV = 1;
  private emaW = 1;
  private nextLog = 0;
  private start: Vec3;

  constructor(
    readonly af: Airframe,
    readonly cfg: PhysicsConfig,
    launch: LaunchParams,
  ) {
    this.state = launchState(af, launch);
    this.cache = createAeroCache(af);
    this.start = [...this.state.x];
    this.log = { launch, samples: [], firstImpact: null, rest: null, maxHeight: this.state.x[1], phase: 'flight' };
    this.record();
  }

  get done(): boolean {
    return this.phase === 'rest' || this.phase === 'timeout';
  }

  /** Advance one fixed time step (cfg.dt) with cfg.substeps RK4 sub-steps and ground contact. */
  step() {
    if (this.done) return;
    const h = this.cfg.dt / this.cfg.substeps;
    for (let k = 0; k < this.cfg.substeps; k++) {
      this.state = rk4Step(this.state, h, (s) => derivative(s, this.af, this.cfg, this.cache));
      const c = resolveGround(this.state, this.af, this.cfg.ground, h);
      if (c.touching && !this.log.firstImpact) {
        this.log.firstImpact = this.event(c.impactSpeed || this.speed());
        this.phase = 'ground';
      }
    }
    this.t += this.cfg.dt;
    this.log.maxHeight = Math.max(this.log.maxHeight, this.state.x[1]);

    if (this.phase === 'ground') {
      const a = Math.min(1, this.cfg.dt / (this.cfg.ground.restTime / 4));
      this.emaV += a * (this.speed() - this.emaV);
      this.emaW += a * (V.length(this.state.w) - this.emaW);
      const still = this.emaV < this.cfg.ground.restSpeed && this.emaW < this.cfg.ground.restSpin;
      this.restTimer = still ? this.restTimer + this.cfg.dt : 0;
      if (this.restTimer >= this.cfg.ground.restTime) {
        this.phase = 'rest';
        this.log.rest = this.event(this.speed());
      }
    }
    if (this.t >= this.cfg.maxTime && !this.done) this.phase = 'timeout';
    this.log.phase = this.phase;
    if (this.t + 1e-12 >= this.nextLog || this.done) this.record();
  }

  run(maxTime = this.cfg.maxTime): FlightLog {
    while (!this.done && this.t < maxTime) this.step();
    return this.log;
  }

  /** Per-element forces of the current state, for the debug overlay. */
  debugForces(): AeroDebug {
    const d: AeroDebug = { lift: [], drag: [], alpha: [] };
    derivative(cloneState(this.state), this.af, this.cfg, this.cache, d);
    return d;
  }

  worldVelocity(): Vec3 {
    return Q.rotate(Q.normalize(this.state.q), this.state.v);
  }

  speed(): number {
    return V.length(this.state.v);
  }

  contactWorld(): Vec3[] {
    return this.af.contacts.map((r) => pointWorld(this.state, r));
  }

  private event(speed: number): FlightEvent {
    const p = this.state.x;
    return { t: this.t, pos: [...p], speed, distance: Math.hypot(p[0] - this.start[0], p[2] - this.start[2]) };
  }

  private record() {
    const s = this.state;
    const q = Q.normalize(s.q);
    const noseW = Q.rotate(q, this.af.nose);
    const vb = s.v;
    const alpha = Math.atan2(-V.dot(vb, this.af.up), V.dot(vb, this.af.nose)) * RAD;
    this.log.samples.push({
      t: this.t,
      pos: [...s.x],
      vel: this.worldVelocity(),
      q: [...q],
      w: [...s.w],
      phase: this.phase,
      pitchDeg: Math.asin(Math.max(-1, Math.min(1, noseW[1]))) * RAD,
      alphaDeg: alpha,
    });
    this.nextLog = this.t + this.cfg.logInterval;
  }
}
