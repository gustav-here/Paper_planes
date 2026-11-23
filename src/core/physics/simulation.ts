import * as V from '../math/vec3';
import * as Q from '../math/quat';
import * as M3 from '../math/mat3';
import type { Vec3 } from '../math/vec3';
import type { Quat } from '../math/quat';
import type { Airframe } from './airframe';
import { createAeroCache } from './aeroModel';
import type { AeroCache, AeroDebug, SurfacePose } from './aeroModel';
import type { PhysicsConfig } from './config';
import { cloneState, derivative, pointWorld, rk4Step } from './rigidBody';
import type { BodyState } from './rigidBody';
import { resolveGround } from './groundContact';
import { launchState } from './launch';
import type { LaunchParams } from './launch';
import { ControlTrack, DEFAULT_CONTROLS } from './controls';
import type { ControlConfig, ControlInput } from './controls';
import type { DamageEvent, PanelPose, Simulator } from './simulator';

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

/** Rotation of a flap about its hinge by `deg`, as a body-frame rigid motion. */
export function flapMotion(p: Vec3, a: Vec3, deg: number, rateDeg: number): SurfacePose {
  const ang = (deg * Math.PI) / 180;
  const q = Q.fromAxisAngle(a, ang);
  const R = Q.toMat3(q);
  const t = V.sub(p, M3.mulVec(R, p));
  const w = V.scale(a, (rateDeg * Math.PI) / 180);
  return { R, t, w, v0: V.negate(V.cross(w, p)) };
}

/**
 * Fixed-step, deterministic flight of one rigid airframe from a launch, with optional control
 * flaps driven by a recorded/live control track.
 */
export class FlightSim implements Simulator {
  state: BodyState;
  t = 0;
  stepIndex = 0;
  phase: FlightPhase = 'flight';
  readonly log: FlightLog;
  readonly controls: ControlTrack;
  readonly panels: number[][];
  private cache: AeroCache;
  private deflection: number[];
  private deflRate: number[];
  private poses: (SurfacePose | null)[];
  private events: DamageEvent[] = [];
  private restTimer = 0;
  /** Smoothed |v| and |ω| on the ground (contact jitter must not keep the plane "moving"). */
  /** Rest detection on the mean motion (velocity and spin averaged as vectors, world frame). */
  private emaV: Vec3 = [1, 0, 0];
  private emaW: Vec3 = [1, 0, 0];
  private nextLog = 0;
  private start: Vec3;

  constructor(
    readonly af: Airframe,
    readonly cfg: PhysicsConfig,
    launch: LaunchParams,
    inputs?: ControlInput[],
    readonly controlCfg: ControlConfig = DEFAULT_CONTROLS,
    /** All facet ids of the model (for render panels). */
    facetIds: number[] = [],
  ) {
    this.state = launchState(af, launch, cfg.scatter);
    this.cache = createAeroCache(af);
    this.controls = new ControlTrack(inputs);
    this.deflection = af.controls.map(() => 0);
    this.deflRate = af.controls.map(() => 0);
    this.poses = af.mesh.surfaces.map(() => null);
    const inFlaps = new Set(af.controls.flatMap((g) => g.facets));
    this.panels = [facetIds.filter((f) => !inFlaps.has(f)), ...af.controls.map((g) => g.facets)];
    this.start = [...this.state.x];
    this.log = { launch, samples: [], firstImpact: null, rest: null, maxHeight: this.state.x[1], phase: 'flight' };
    this.record();
  }

  get done(): boolean {
    return this.phase === 'rest' || this.phase === 'timeout';
  }

  /** Move the flaps towards the commanded deflection (rate-limited) and update their poses. */
  private updateFlaps() {
    const cmd = this.controls.at(this.stepIndex);
    const max = this.controlCfg.maxDeflectionDeg;
    const dmax = this.controlCfg.rateDegPerS * this.cfg.dt;
    this.af.controls.forEach((g, i) => {
      const target = max * Math.max(-1, Math.min(1, cmd.pitch * g.pitchSign + cmd.roll * g.rollSign));
      const d = Math.max(-dmax, Math.min(dmax, target - this.deflection[i]));
      this.deflection[i] += d;
      this.deflRate[i] = d / this.cfg.dt;
      const pose = this.deflection[i] === 0 && d === 0 ? null : flapMotion(g.hingeP, g.hingeA, this.deflection[i], this.deflRate[i]);
      for (const sid of g.surfaces) this.poses[sid] = pose;
    });
  }

  /** Advance one fixed time step (cfg.dt) with cfg.substeps RK4 sub-steps and ground contact. */
  step() {
    if (this.done) return;
    if (this.af.controls.length) this.updateFlaps();
    const h = this.cfg.dt / this.cfg.substeps;
    const poses = this.af.controls.length ? this.poses : undefined;
    for (let k = 0; k < this.cfg.substeps; k++) {
      this.state = rk4Step(this.state, h, (s) => derivative(s, this.af, this.cfg, this.cache, undefined, poses));
      const c = resolveGround(this.state, this.af, this.cfg.ground, h);
      if (c.touching && !this.log.firstImpact) {
        this.log.firstImpact = this.event(c.impactSpeed || this.speed());
        this.phase = 'ground';
      }
      if (c.hardest && c.impactSpeed >= this.cfg.paper.crumpleSpeed) this.recordCrumple(c.hardest.index, c.hardest.velWorld, c.impactSpeed);
    }
    this.stepIndex++;
    this.t += this.cfg.dt;
    this.log.maxHeight = Math.max(this.log.maxHeight, this.state.x[1]);

    if (this.phase === 'ground') {
      const a = Math.min(1, this.cfg.dt / (this.cfg.ground.restTime / 4));
      this.emaV = V.lerp(this.emaV, this.worldVelocity(), a);
      this.emaW = V.lerp(this.emaW, Q.rotate(Q.normalize(this.state.q), this.state.w), a);
      const still = V.length(this.emaV) < this.cfg.ground.restSpeed && V.length(this.emaW) < this.cfg.ground.restSpin;
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

  /** A hard impact crumples the paper where it hit (at most two places per flight). */
  private recordCrumple(index: number, velWorld: Vec3, speed: number) {
    if (!this.cfg.paper.damage) return;
    const point = V.add(this.af.contacts[index], this.af.com);
    const dir = V.normalize(Q.rotate(Q.conjugate(Q.normalize(this.state.q)), velWorld));
    const near = this.events.find((e) => e.kind === 'crumple' && V.dist(e.point, point) < 0.03);
    if (near && near.kind === 'crumple') {
      if (speed > near.speed) Object.assign(near, { point, dir, speed });
      return;
    }
    if (this.events.filter((e) => e.kind === 'crumple').length >= 2) return;
    this.events.push({ kind: 'crumple', facet: this.af.contactFacets[index], point, dir, speed });
  }

  damage(): DamageEvent[] {
    return this.events;
  }

  deflectionsDeg(): number[] {
    return [...this.deflection];
  }

  position(): Vec3 {
    return [...this.state.x];
  }

  /** Model→world placement of each render panel (body, then one per control flap). */
  panelPoses(): PanelPose[] {
    const q = Q.normalize(this.state.q);
    const com = this.af.com;
    const base: PanelPose = { q, t: V.sub(this.state.x, Q.rotate(q, com)) };
    return [
      base,
      ...this.af.controls.map((g, i) => {
        const qf = Q.fromAxisAngle(g.hingeA, (this.deflection[i] * Math.PI) / 180);
        const tf = V.sub(g.hingeP, Q.rotate(qf, g.hingeP));
        return { q: Q.normalize(Q.mul(q, qf)), t: V.add(this.state.x, Q.rotate(q, V.sub(tf, Q.rotate(qf, com)))) };
      }),
    ];
  }

  /** Per-element forces of the current state (world frame), for the debug overlay. */
  debugForces(): { pos: Vec3[]; lift: Vec3[]; drag: Vec3[] } {
    const d: AeroDebug = { pos: [], lift: [], drag: [], alpha: [] };
    derivative(cloneState(this.state), this.af, this.cfg, this.cache, d, this.af.controls.length ? this.poses : undefined);
    const q = Q.normalize(this.state.q);
    const toW = (v: Vec3 | undefined): Vec3 => (v ? Q.rotate(q, v) : [0, 0, 0]);
    return {
      pos: (d.pos ?? []).map((p) => V.add(this.state.x, Q.rotate(q, p))),
      lift: d.lift.map(toW),
      drag: d.drag.map(toW),
    };
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
