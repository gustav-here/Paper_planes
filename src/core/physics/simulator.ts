import type { Vec2 } from '../math/vec2';
import type { Vec3 } from '../math/vec3';
import type { Quat } from '../math/quat';
import type { PhysicsConfig } from './config';
import type { ControlTrack } from './controls';
import type { FlightLog, FlightPhase } from './simulation';

/** Rigid placement of a render panel: p_world = rotate(q, p_model) + t. */
export interface PanelPose {
  q: Quat;
  t: Vec3;
}

/**
 * How a render panel's paper is bent right now: the deflection w along `normal` (model space)
 * at a model point, and its in-plane gradient.
 */
export interface PanelBend {
  normal: Vec3;
  at(p: Vec3): { w: number; grad: Vec3 };
  /** Fast form for many points: writes [w, ∇w x, y, z] at (x, y, z) into `out`. */
  evalInto(x: number, y: number, z: number, out: Float64Array): void;
  /**
   * Drawing only: extra displacement (model space, written to out[0..2]) that carries the paper
   * next to the crease along with a bending parent. A stiff panel hanging on a bending one meets
   * the parent's bent crease exactly at the crease, fading out across the panel.
   */
  seamInto?(x: number, y: number, z: number, out: Float64Array): void;
}

/** Something that happened to the paper during a flight and leaves permanent damage. */
export type DamageEvent =
  /** Hard floor impact at a point: the paper there crumples. Model coordinates. */
  | { kind: 'crumple'; facet: number; point: Vec3; dir: Vec3; speed: number }
  /**
   * A crease was bent past its elastic limit (flexible model): its fold angle changed by
   * deltaDeg about `axis` through `point` (model space, right-hand rule). The child side is
   * the paper around `childFacet`.
   */
  | { kind: 'bend'; point: Vec3; axis: Vec3; deltaDeg: number; parentFacet: number; childFacet: number }
  /** A cut end tore further. Sheet coordinates. */
  | { kind: 'tear'; facet: number; at: Vec2; dir: Vec2; length: number };

/** Common interface of the rigid and the flexible flight simulation. */
export interface Simulator {
  readonly cfg: PhysicsConfig;
  readonly log: FlightLog;
  readonly t: number;
  readonly stepIndex: number;
  readonly phase: FlightPhase;
  readonly done: boolean;
  readonly controls: ControlTrack;
  /** Facet ids per render panel (each panel moves rigidly). */
  readonly panels: number[][];
  step(): void;
  /** Reference point (CoM of the plane at rest) in the world. */
  position(): Vec3;
  worldVelocity(): Vec3;
  speed(): number;
  panelPoses(): PanelPose[];
  /** Bending of each render panel's own paper (flexible model with bending shapes), or null. */
  panelBending?(): (PanelBend | null)[];
  /** Element centres and lift/drag forces in the world frame (debug overlay). */
  debugForces(): { pos: Vec3[]; lift: Vec3[]; drag: Vec3[] };
  /** Current flap deflections (deg), one per control group. */
  deflectionsDeg(): number[];
  /** Damage collected so far. */
  damage(): DamageEvent[];
}
