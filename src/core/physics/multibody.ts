import * as V from '../math/vec3';
import * as Q from '../math/quat';
import * as M3 from '../math/mat3';
import type { Vec3 } from '../math/vec3';
import type { Quat } from '../math/quat';
import type { Mat3 } from '../math/mat3';
import { invert } from '../math/linalg';
import { applyRigid, composeRigid, rigidIdentity, rotationAbout } from '../math/transform';
import type { RigidTransform } from '../math/transform';
import type { PaperModel } from '../paper/model';
import type { Airframe } from './airframe';
import { buildAeroMesh } from './aeroMesh';
import { createAeroCache, evaluateAero } from './aeroModel';
import type { AeroCache, AeroDebug, SurfacePose } from './aeroModel';
import { computeAddedMass } from './addedMass';
import type { PhysicsConfig } from './config';
import { ControlTrack, DEFAULT_CONTROLS } from './controls';
import type { ControlConfig, ControlInput } from './controls';
import { launchState } from './launch';
import type { LaunchParams } from './launch';
import { buildPanels, panelContacts } from './panels';
import type { PanelSet } from './panels';
import type { DamageEvent, PanelPose, Simulator } from './simulator';
import type { FlightEvent, FlightLog, FlightPhase } from './simulation';

/*
 * Flexible paper: hinged rigid panels in reduced coordinates.
 *
 * Generalized velocity u = (v₀, ω₀, θ̇₁…θ̇ₖ): world velocity of the body origin (the plane's CoM at
 * rest, fixed to the root panel), world angular velocity of the root panel, and the hinge rates.
 * For panel i with hinges j on its path from the root (axis aⱼ, point hⱼ, world frame):
 *   ωᵢ = ω₀ + Σ θ̇ⱼ aⱼ,     v(p) = v₀ + ω₀ × (p − x₀) + Σ θ̇ⱼ aⱼ × (p − hⱼ)
 * so the panel's spatial velocity is Jᵢ u. With Mᵢ the panel's spatial inertia (paper + added
 * mass of its surfaces, about its CoM) the equations of motion are
 *   (Σ Jᵢᵀ Mᵢ Jᵢ) u̇ = Σ Jᵢᵀ (Wᵢ − Mᵢ J̇ᵢu − Ṁᵢνᵢ) − k(θ − θ_rest) − c θ̇
 * with Wᵢ = aero + gravity. Aerodynamic loads come from the same element model as the rigid
 * plane (surfaces posed with their panel) and are updated every fixed step; the stiff crease
 * springs are integrated with semi-implicit Euler on sub-steps. Creases bent past their yield
 * angle keep a permanent bend (plasticity).
 */

export interface FlexAirframe {
  af: Airframe;
  set: PanelSet;
  panelSurfaces: number[][];
  /** Added mass of each panel's surfaces about the panel CoM (6×6, model axes). */
  panelMA: number[][];
  contacts: { panel: number; facet: number; p: Vec3 }[];
  model: PaperModel;
}

export function buildFlexAirframe(model: PaperModel, rigid: Airframe, cfg: PhysicsConfig): FlexAirframe {
  const set = buildPanels(model, rigid, cfg.paper);
  const mesh = buildAeroMesh(model, rigid.com, cfg.aero.elementSizeM, cfg.aero.liftingSystemAngleDeg, set.panelOf);
  const af: Airframe = { ...rigid, mesh };
  // control surfaces of the rigid airframe refer to its own mesh; remap by facet tags
  af.controls = rigid.controls.map((g) => ({ ...g, surfaces: mesh.surfaces.filter((s) => g.facets.includes(firstFacetOf(model, set, s.tag))).map((s) => s.id) }));
  const panelSurfaces = set.panels.map((_, i) => mesh.surfaces.filter((s) => s.tag === i).map((s) => s.id));
  const panelMA = set.panels.map((p, i) =>
    cfg.addedMass
      ? computeAddedMass(mesh, cfg.rho, cfg.addedMassScale, cfg.aero.elementSizeM, new Set(panelSurfaces[i]), V.sub(p.com, rigid.com)).MA
      : new Array<number>(36).fill(0),
  );
  return { af, set, panelSurfaces, panelMA, contacts: panelContacts(model, set), model };
}

function firstFacetOf(_m: PaperModel, set: PanelSet, panel: number): number {
  return set.panels[panel]?.facets[0] ?? -1;
}

const RAD = 180 / Math.PI;
const skew = (a: Vec3): Mat3 => [0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0];

interface Kin {
  R0: Mat3;
  M: RigidTransform[];
  /** Hinge axes and points (world). */
  a: Vec3[];
  h: Vec3[];
  /** Panel CoM (world) and orientation (model axes → world). */
  c: Vec3[];
  Rp: Mat3[];
  /** Panel angular velocity and CoM velocity (world). */
  w: Vec3[];
  v: Vec3[];
}

export class MultiBodySim implements Simulator {
  t = 0;
  stepIndex = 0;
  phase: FlightPhase = 'flight';
  readonly log: FlightLog;
  readonly controls: ControlTrack;
  readonly panels: number[][];
  private x0: Vec3;
  private q0: Quat;
  private u: number[];
  private theta: number[];
  private rest: number[];
  private rest0: number[];
  private n: number;
  private k: number;
  private cache: AeroCache;
  private substeps: number;
  private deflection: number[];
  private events: DamageEvent[] = [];
  private tornHinges = new Set<number>();
  private restTimer = 0;
  private emaV = 1;
  private emaW = 1;
  private nextLog = 0;
  private start: Vec3;

  constructor(
    readonly fx: FlexAirframe,
    readonly cfg: PhysicsConfig,
    launch: LaunchParams,
    inputs?: ControlInput[],
    readonly controlCfg: ControlConfig = DEFAULT_CONTROLS,
  ) {
    const { set } = fx;
    this.k = set.hinges.length;
    this.n = 6 + this.k;
    this.panels = set.panels.map((p) => p.facets);
    const s = launchState(fx.af, launch);
    this.x0 = s.x;
    this.q0 = s.q;
    this.u = new Array<number>(this.n).fill(0);
    const v0 = Q.rotate(s.q, s.v);
    this.u[0] = v0[0];
    this.u[1] = v0[1];
    this.u[2] = v0[2];
    this.theta = new Array<number>(this.k).fill(0);
    this.rest = new Array<number>(this.k).fill(0);
    this.rest0 = [...this.rest];
    this.deflection = fx.af.controls.map(() => 0);
    this.controls = new ControlTrack(inputs);
    this.cache = createAeroCache(fx.af);
    this.start = [...this.x0];
    this.substeps = this.chooseSubsteps();
    this.log = { launch, samples: [], firstImpact: null, rest: null, maxHeight: this.x0[1], phase: 'flight' };
    this.record();
  }

  get done(): boolean {
    return this.phase === 'rest' || this.phase === 'timeout';
  }

  // ---------------------------------------------------------------- kinematics

  private kin(x0: Vec3, q0: Quat, theta: number[], u: number[]): Kin {
    const { set, af } = this.fx;
    const com = af.com;
    const R0 = Q.toMat3(Q.normalize(q0));
    const M: RigidTransform[] = new Array(set.panels.length);
    const order = this.order();
    const a: Vec3[] = new Array(this.k);
    const h: Vec3[] = new Array(this.k);
    for (const i of order) {
      const p = set.panels[i];
      if (p.parent < 0) {
        M[i] = rigidIdentity();
        continue;
      }
      const hg = set.hinges[p.hinge];
      M[i] = composeRigid(M[p.parent], rotationAbout(hg.point, hg.axis, theta[p.hinge]));
      a[p.hinge] = M3.mulVec(R0, Q.rotate(M[p.parent].q, hg.axis));
      h[p.hinge] = V.add(x0, M3.mulVec(R0, V.sub(applyRigid(M[p.parent], hg.point), com)));
    }
    const v0: Vec3 = [u[0], u[1], u[2]];
    const w0: Vec3 = [u[3], u[4], u[5]];
    const c: Vec3[] = [];
    const Rp: Mat3[] = [];
    const w: Vec3[] = [];
    const v: Vec3[] = [];
    set.panels.forEach((p, i) => {
      c[i] = V.add(x0, M3.mulVec(R0, V.sub(applyRigid(M[i], p.com), com)));
      Rp[i] = M3.mul(R0, Q.toMat3(M[i].q));
      let wi = w0;
      let vi = V.add(v0, V.cross(w0, V.sub(c[i], x0)));
      for (const j of p.path) {
        wi = V.addScaled(wi, a[j], u[6 + j]);
        vi = V.add(vi, V.scale(V.cross(a[j], V.sub(c[i], h[j])), u[6 + j]));
      }
      w[i] = wi;
      v[i] = vi;
    });
    return { R0, M, a, h, c, Rp, w, v };
  }

  private orderCache: number[] | null = null;
  /** Panels ordered root first (parents before children). */
  private order(): number[] {
    if (this.orderCache) return this.orderCache;
    const ps = this.fx.set.panels;
    const out: number[] = [];
    const visit = (i: number) => {
      out.push(i);
      ps.forEach((p, j) => {
        if (p.parent === i) visit(j);
      });
    };
    visit(this.fx.set.root);
    return (this.orderCache = out);
  }

  /** Velocity of a world point on panel i. */
  private pointVel(K: Kin, i: number, p: Vec3, u: number[]): Vec3 {
    let vel = V.add([u[0], u[1], u[2]], V.cross([u[3], u[4], u[5]], V.sub(p, this.x0)));
    for (const j of this.fx.set.panels[i].path) vel = V.add(vel, V.scale(V.cross(K.a[j], V.sub(p, K.h[j])), u[6 + j]));
    return vel;
  }

  /** Jacobian rows (3×n) of the velocity of world point p on panel i. */
  private pointJ(K: Kin, i: number, p: Vec3): number[][] {
    const n = this.n;
    const J = [new Array<number>(n).fill(0), new Array<number>(n).fill(0), new Array<number>(n).fill(0)];
    const r = V.sub(p, this.x0);
    const S = skew(r);
    for (let row = 0; row < 3; row++) {
      J[row][row] = 1;
      for (let c = 0; c < 3; c++) J[row][3 + c] = -S[row * 3 + c];
    }
    for (const j of this.fx.set.panels[i].path) {
      const col = V.cross(K.a[j], V.sub(p, K.h[j]));
      for (let row = 0; row < 3; row++) J[row][6 + j] = col[row];
    }
    return J;
  }

  /** Spatial inertia of panel i in the world (6×6 about its CoM). */
  private panelInertia(K: Kin, i: number): number[] {
    const p = this.fx.set.panels[i];
    const R = K.Rp[i];
    const Rt = M3.transpose(R);
    const MA = this.fx.panelMA[i];
    const block = (r0: number, c0: number): Mat3 => [0, 1, 2].flatMap((r) => [0, 1, 2].map((c) => MA[(r0 + r) * 6 + c0 + c]));
    const rot = (B: Mat3) => M3.mul(M3.mul(R, B), Rt);
    const A11 = rot(block(0, 0));
    const A12 = rot(block(0, 3));
    const A21 = rot(block(3, 0));
    const A22 = rot(block(3, 3));
    const I = rot(p.inertia);
    const out = new Array<number>(36).fill(0);
    for (let r = 0; r < 3; r++)
      for (let c = 0; c < 3; c++) {
        out[r * 6 + c] = A11[r * 3 + c] + (r === c ? p.mass : 0);
        out[r * 6 + 3 + c] = A12[r * 3 + c];
        out[(3 + r) * 6 + c] = A21[r * 3 + c];
        out[(3 + r) * 6 + 3 + c] = A22[r * 3 + c] + I[r * 3 + c];
      }
    return out;
  }

  /** Panel Jacobians (6×n: CoM velocity, angular velocity). */
  private panelJ(K: Kin): number[][][] {
    const n = this.n;
    return this.fx.set.panels.map((p, i) => {
      const Jl = this.pointJ(K, i, K.c[i]);
      const Ja = [new Array<number>(n).fill(0), new Array<number>(n).fill(0), new Array<number>(n).fill(0)];
      for (let r = 0; r < 3; r++) Ja[r][3 + r] = 1;
      for (const j of p.path) for (let r = 0; r < 3; r++) Ja[r][6 + j] = K.a[j][r];
      return [...Jl, ...Ja];
    });
  }

  /** Generalized aerodynamic force Σ Jᵢᵀ Wᵢ for given velocities u (configuration from K). */
  private aeroQ(K: Kin, Js: number[][][], u: number[], debug?: AeroDebug): number[] {
    const W = this.aeroPanels(K, u, debug);
    const out = new Array<number>(this.n).fill(0);
    W.forEach((w, i) => {
      const f = [...w.F, ...w.T];
      const J = Js[i];
      for (let a = 0; a < this.n; a++) {
        let sum = 0;
        for (let r = 0; r < 6; r++) sum += J[r][a] * f[r];
        out[a] += sum;
      }
    });
    return out;
  }

  /** Generalized mass matrix and the non-aerodynamic right-hand side for the current state. */
  private dynamics(K: Kin, Js: number[][][]): { M: number[]; rhs: number[] } {
    const { set } = this.fx;
    const n = this.n;
    const M = new Array<number>(n * n).fill(0);
    const rhs = new Array<number>(n).fill(0);
    const u = this.u;
    const w0: Vec3 = [u[3], u[4], u[5]];
    const g: Vec3 = [0, -this.cfg.g, 0];
    set.panels.forEach((p, i) => {
      const J = Js[i];
      const Mi = this.panelInertia(K, i);
      // bias acceleration (J̇u)
      let ab: Vec3 = V.cross(w0, V.sub(K.v[i], [u[0], u[1], u[2]]));
      let alb: Vec3 = [0, 0, 0];
      for (const j of p.path) {
        const hg = set.hinges[j];
        const wp = K.w[hg.parent];
        const adot = V.cross(wp, K.a[j]);
        const vh = this.pointVel(K, hg.parent, K.h[j], u);
        ab = V.add(ab, V.scale(V.add(V.cross(adot, V.sub(K.c[i], K.h[j])), V.cross(K.a[j], V.sub(K.v[i], vh))), u[6 + j]));
        alb = V.addScaled(alb, adot, u[6 + j]);
      }
      const nu = [...K.v[i], ...K.w[i]];
      const Mnu = matVec6(Mi, nu);
      const Mab = matVec6(Mi, [...ab, ...alb]);
      const wi = K.w[i];
      const corr = matVec6(Mi, [...V.cross(wi, K.v[i]), 0, 0, 0]);
      const gyroL = V.cross(wi, [Mnu[0], Mnu[1], Mnu[2]]);
      const gyroA = V.cross(wi, [Mnu[3], Mnu[4], Mnu[5]]);
      const W = [p.mass * g[0], p.mass * g[1], p.mass * g[2], 0, 0, 0];
      const bias = [
        Mab[0] + gyroL[0] - corr[0],
        Mab[1] + gyroL[1] - corr[1],
        Mab[2] + gyroL[2] - corr[2],
        Mab[3] + gyroA[0] - corr[3],
        Mab[4] + gyroA[1] - corr[4],
        Mab[5] + gyroA[2] - corr[5],
      ];
      const f = W.map((x, k) => x - bias[k]);
      // M += Jᵀ Mi J,  rhs += Jᵀ f
      const MiJ = new Array<number>(6 * n).fill(0);
      for (let r = 0; r < 6; r++)
        for (let c = 0; c < n; c++) {
          let sum = 0;
          for (let k2 = 0; k2 < 6; k2++) sum += Mi[r * 6 + k2] * J[k2][c];
          MiJ[r * n + c] = sum;
        }
      for (let a = 0; a < n; a++) {
        for (let b = 0; b < n; b++) {
          let sum = 0;
          for (let r = 0; r < 6; r++) sum += J[r][a] * MiJ[r * n + b];
          M[a * n + b] += sum;
        }
        let s2 = 0;
        for (let r = 0; r < 6; r++) s2 += J[r][a] * f[r];
        rhs[a] += s2;
      }
    });
    // crease springs (and control servos); damping is handled implicitly in step()
    set.hinges.forEach((hg, j) => {
      rhs[6 + j] += -hg.stiffness * (this.theta[j] - this.rest[j]) - hg.damping * u[6 + j];
    });
    return { M, rhs };
  }

  /**
   * Sub-steps per fixed step. The creases and the air damping are integrated implicitly, so
   * stability does not need tiny steps; a few sub-steps keep the motion of light flaps accurate
   * (highest crease frequency resolved to ω·h ≲ 1).
   */
  private chooseSubsteps(): number {
    if (this.k === 0) return 1;
    const K = this.kin(this.x0, this.q0, this.theta, this.u);
    const { M } = this.dynamics(K, this.panelJ(K));
    const Minv = invert(M, this.n);
    // power iteration on Minv_θθ·diag(k) for the highest crease frequency
    const k = this.k;
    let x = new Array<number>(k).fill(1);
    let lambda = 0;
    for (let it = 0; it < 40; it++) {
      const y = new Array<number>(k).fill(0);
      for (let a = 0; a < k; a++)
        for (let b = 0; b < k; b++) y[a] += Minv[(6 + a) * this.n + 6 + b] * this.fx.set.hinges[b].stiffness * x[b];
      lambda = Math.hypot(...y);
      if (lambda === 0) break;
      x = y.map((v) => v / lambda);
    }
    const wmax = Math.sqrt(lambda);
    return Math.max(2, Math.min(24, Math.ceil(wmax * this.cfg.dt)));
  }

  // ---------------------------------------------------------------- aero

  private surfacePoses(K: Kin, u: number[] = this.u): { poses: (SurfacePose | null)[]; vb: Vec3; wb: Vec3 } {
    const { set, af, panelSurfaces } = this.fx;
    const com = af.com;
    const R0t = M3.transpose(K.R0);
    const poses: (SurfacePose | null)[] = af.mesh.surfaces.map(() => null);
    set.panels.forEach((p, i) => {
      if (p.parent < 0 && this.k >= 0 && i === set.root) return;
      const Mi = K.M[i];
      const R = Q.toMat3(Mi.q);
      const t = V.sub(V.add(Q.rotate(Mi.q, com), Mi.t), com);
      let W: Vec3 = [0, 0, 0];
      let v0: Vec3 = [0, 0, 0];
      for (const j of p.path) {
        const ab = M3.mulVec(R0t, K.a[j]);
        const hb = M3.mulVec(R0t, V.sub(K.h[j], this.x0));
        W = V.addScaled(W, ab, u[6 + j]);
        v0 = V.sub(v0, V.scale(V.cross(ab, hb), u[6 + j]));
      }
      const pose: SurfacePose = { R, t, w: W, v0 };
      for (const sid of panelSurfaces[i]) poses[sid] = pose;
    });
    const vb = M3.mulVec(R0t, [u[0], u[1], u[2]]);
    const wb = M3.mulVec(R0t, [u[3], u[4], u[5]]);
    return { poses, vb, wb };
  }

  /** Aerodynamic force and moment about each panel CoM (world) for velocities u. */
  private aeroPanels(K: Kin, u: number[], debug?: AeroDebug): { F: Vec3; T: Vec3 }[] {
    const { set, af, panelSurfaces } = this.fx;
    const { poses, vb, wb } = this.surfacePoses(K, u);
    const per: { F: Vec3; T: Vec3 }[] = [];
    evaluateAero(af, vb, wb, this.cfg.aero, this.cfg.rho, this.cache, { poses, perSurface: per, debug });
    return set.panels.map((_, i) => {
      let F: Vec3 = [0, 0, 0];
      let T: Vec3 = [0, 0, 0];
      for (const sid of panelSurfaces[i]) {
        const ps = per[sid];
        if (!ps) continue;
        F = V.add(F, ps.F);
        T = V.add(T, ps.T);
      }
      // moment about the panel CoM, then to the world
      const cb = M3.mulVec(M3.transpose(K.R0), V.sub(K.c[i], this.x0));
      const Tc = V.sub(T, V.cross(cb, F));
      return { F: M3.mulVec(K.R0, F), T: M3.mulVec(K.R0, Tc) };
    });
  }

  // ---------------------------------------------------------------- stepping

  private updateServos() {
    const { af, set } = this.fx;
    if (!af.controls.length) return;
    const cmd = this.controls.at(this.stepIndex);
    const max = this.controlCfg.maxDeflectionDeg;
    const dmax = this.controlCfg.rateDegPerS * this.cfg.dt;
    af.controls.forEach((g, i) => {
      const target = max * Math.max(-1, Math.min(1, cmd.pitch * g.pitchSign + cmd.roll * g.rollSign));
      this.deflection[i] += Math.max(-dmax, Math.min(dmax, target - this.deflection[i]));
    });
    set.hinges.forEach((hg, j) => {
      if (hg.control >= 0) this.rest[j] = this.deflection[hg.control] / RAD;
    });
  }

  /**
   * One fixed step: aerodynamics and its Jacobian D = ∂Q_aero/∂θ̇ (finite differences, hinge
   * columns) are evaluated once; each sub-step then solves the linearly implicit Euler update
   *   (M − h(D − C) + h²K) Δu = h·f(u, θ)
   * which is stable for the very light, heavily air-damped paper panels and stiff creases.
   */
  step() {
    if (this.done) return;
    this.updateServos();
    const n = this.n;
    const K0 = this.kin(this.x0, this.q0, this.theta, this.u);
    const J0 = this.panelJ(K0);
    const u0 = [...this.u];
    const Q0 = this.aeroQ(K0, J0, u0);
    // Only the hinge rates need implicit air damping (light panels); the root body's aero
    // damping is slow enough for explicit treatment, as in the rigid model.
    const D = new Array<number>(n * n).fill(0);
    for (let c = 6; c < n; c++) {
      const up = [...u0];
      const eps = 1e-3;
      up[c] += eps;
      const Qp = this.aeroQ(K0, J0, up);
      for (let r = 0; r < n; r++) D[r * n + c] = (Qp[r] - Q0[r]) / eps;
    }
    const h = this.cfg.dt / this.substeps;
    for (let s = 0; s < this.substeps; s++) {
      const K = this.kin(this.x0, this.q0, this.theta, this.u);
      const Js = this.panelJ(K);
      const { M, rhs } = this.dynamics(K, Js);
      const f = rhs.map((x, r) => {
        let qa = Q0[r];
        for (let c = 0; c < n; c++) qa += D[r * n + c] * (this.u[c] - u0[c]);
        return x + qa;
      });
      const A = M.map((x, idx) => x - h * D[idx]);
      this.fx.set.hinges.forEach((hg, j) => {
        A[(6 + j) * n + 6 + j] += h * hg.damping + h * h * hg.stiffness;
        // backward Euler for the spring: −k(θ + h·θ̇_new) → extra −h·k·θ̇ on the right
        f[6 + j] -= h * hg.stiffness * this.u[6 + j];
      });
      const du = mulVecN(invert(A, n), f, n);
      for (let i = 0; i < n; i++) this.u[i] += h * du[i];
      // floor impulses act on the new velocities, before the positions move
      this.contact(h, invert(M, n), K);
      // positions with the new velocities (semi-implicit Euler)
      this.x0 = V.add(this.x0, V.scale([this.u[0], this.u[1], this.u[2]], h));
      const wq: Quat = [this.u[3], this.u[4], this.u[5], 0];
      const dq = Q.mul(wq, this.q0);
      this.q0 = Q.normalize([this.q0[0] + 0.5 * h * dq[0], this.q0[1] + 0.5 * h * dq[1], this.q0[2] + 0.5 * h * dq[2], this.q0[3] + 0.5 * h * dq[3]]);
      for (let j = 0; j < this.k; j++) this.theta[j] += h * this.u[6 + j];
      this.plasticity();
    }
    this.stepIndex++;
    this.t += this.cfg.dt;
    this.log.maxHeight = Math.max(this.log.maxHeight, this.x0[1]);
    this.checkTears();
    if (this.phase === 'ground') {
      const a = Math.min(1, this.cfg.dt / (this.cfg.ground.restTime / 4));
      this.emaV += a * (this.speed() - this.emaV);
      this.emaW += a * (V.length([this.u[3], this.u[4], this.u[5]]) - this.emaW);
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

  // ---------------------------------------------------------------- floor

  private contact(dt: number, Minv: number[], K: Kin) {
    const { contacts, af } = this.fx;
    const gcfg = this.cfg.ground;
    const com = af.com;
    type C = { i: number; p: Vec3; J: number[][]; depth: number; target: number; jn: number; jt: Vec3; facet: number; pm: Vec3 };
    const active: C[] = [];
    let minY = Infinity;
    for (const ct of contacts) {
      const p = V.add(this.x0, M3.mulVec(K.R0, V.sub(applyRigid(K.M[ct.panel], ct.p), com)));
      minY = Math.min(minY, p[1]);
      if (p[1] > 1e-5) continue;
      active.push({ i: ct.panel, p, J: this.pointJ(K, ct.panel, p), depth: -p[1], target: 0, jn: 0, jt: [0, 0, 0], facet: ct.facet, pm: ct.p });
    }
    if (!active.length) return;
    const n = this.n;
    const vel = (c: C): Vec3 => [0, 1, 2].map((r) => c.J[r].reduce((s, x, k) => s + x * this.u[k], 0)) as Vec3;
    const gen = (c: C, d: Vec3): number[] => {
      const out = new Array<number>(n).fill(0);
      for (let k = 0; k < n; k++) out[k] = c.J[0][k] * d[0] + c.J[1][k] * d[1] + c.J[2][k] * d[2];
      return out;
    };
    const apply = (gvec: number[], lambda: number) => {
      const du = mulVecN(Minv, gvec, n);
      for (let k = 0; k < n; k++) this.u[k] += du[k] * lambda;
    };
    const invMass = (gvec: number[]) => {
      const mg = mulVecN(Minv, gvec, n);
      return gvec.reduce((s, x, k) => s + x * mg[k], 0);
    };
    const N: Vec3 = [0, 1, 0];
    let impact = 0;
    let hardest: C | null = null;
    let hardestVel: Vec3 = [0, 0, 0];
    for (const c of active) {
      const v = vel(c);
      const vn = v[1];
      const bounce = vn < 0 && -vn > gcfg.restitutionThreshold ? -gcfg.restitution * vn : 0;
      c.target = Math.max(bounce, (0.2 * Math.max(0, c.depth - 2e-4)) / dt);
      if (vn < 0 && -vn > impact) {
        impact = -vn;
        hardest = c;
        hardestVel = v;
      }
    }
    for (let it = 0; it < gcfg.iterations; it++) {
      for (const c of active) {
        const gn = gen(c, N);
        const vn = vel(c)[1];
        let dj = (c.target - vn) / invMass(gn);
        const old = c.jn;
        c.jn = Math.max(0, old + dj);
        dj = c.jn - old;
        if (dj !== 0) apply(gn, dj);
        const v = vel(c);
        const vt: Vec3 = [v[0], 0, v[2]];
        const vtl = V.length(vt);
        if (vtl < 1e-9) continue;
        const t = V.scale(vt, 1 / vtl);
        const gt = gen(c, t);
        const want = V.addScaled(c.jt, t, -vtl / invMass(gt));
        const maxF = gcfg.friction * c.jn;
        const wl = V.length(want);
        const next = wl > maxF ? V.scale(want, maxF / wl) : want;
        const delta = V.sub(next, c.jt);
        c.jt = next;
        // apply the tangential impulse along its own direction
        const dl = V.length(delta);
        if (dl > 0) apply(gen(c, V.scale(delta, 1 / dl)), dl);
      }
    }
    if (minY < -0.001) this.x0 = [this.x0[0], this.x0[1] - minY - 0.001, this.x0[2]];
    if (!this.log.firstImpact) {
      this.log.firstImpact = this.event(impact || this.speed());
      this.phase = 'ground';
    }
    if (hardest && impact >= this.cfg.paper.crumpleSpeed && this.cfg.paper.damage) {
      const dirModel = V.normalize(Q.rotate(Q.conjugate(Q.normalize(this.q0)), hardestVel));
      const prev = this.events.find((e) => e.kind === 'crumple' && V.dist(e.point, hardest!.pm) < 0.03);
      if (prev && prev.kind === 'crumple') {
        if (impact > prev.speed) Object.assign(prev, { point: hardest.pm, dir: dirModel, speed: impact });
      } else if (this.events.filter((e) => e.kind === 'crumple').length < 2) {
        this.events.push({ kind: 'crumple', facet: hardest.facet, point: hardest.pm, dir: dirModel, speed: impact });
      }
    }
  }

  // ---------------------------------------------------------------- damage

  /** Creases bent beyond their yield angle keep a permanent bend. */
  private plasticity() {
    if (!this.cfg.paper.damage) return;
    const y = this.cfg.paper.creaseYieldDeg / RAD;
    this.fx.set.hinges.forEach((hg, j) => {
      // flat paper bending over a whole tab stays elastic; only creases yield
      if (hg.control >= 0 || hg.kind === 'slit') return;
      const dev = this.theta[j] - this.rest[j];
      if (Math.abs(dev) > y) this.rest[j] += dev - Math.sign(dev) * y;
    });
  }

  /** Large crease moments tear cut ends lying on that crease further. */
  private checkTears() {
    if (!this.cfg.paper.damage) return;
    const P = this.cfg.paper;
    this.fx.set.hinges.forEach((hg, j) => {
      if (!hg.cutEnds.length || this.tornHinges.has(j)) return;
      const moment = Math.abs(hg.stiffness * (this.theta[j] - this.rest[j])) / Math.max(hg.length, 1e-3);
      if (moment <= P.tearMomentPerM) return;
      this.tornHinges.add(j);
      const length = Math.min(P.tearMaxLength, P.tearLengthGain * (moment / P.tearMomentPerM - 1) + 0.002);
      for (const e of hg.cutEnds) this.events.push({ kind: 'tear', facet: e.facet, at: e.at, dir: e.dir, length });
    });
  }

  damage(): DamageEvent[] {
    const bends: DamageEvent[] = [];
    this.fx.set.hinges.forEach((hg, j) => {
      const d = (this.rest[j] - this.rest0[j]) * RAD;
      if (hg.control >= 0 || Math.abs(d) < 1) return;
      bends.push({ kind: 'bend', point: hg.point, axis: hg.axis, deltaDeg: d, parentFacet: hg.parentFacet, childFacet: hg.childFacet });
    });
    return [...this.events, ...bends];
  }

  // ---------------------------------------------------------------- queries

  deflectionsDeg(): number[] {
    return [...this.deflection];
  }

  /** Current hinge angles relative to the folded shape (deg). */
  hingeAnglesDeg(): number[] {
    return this.theta.map((t) => t * RAD);
  }

  position(): Vec3 {
    return [...this.x0];
  }

  worldVelocity(): Vec3 {
    return [this.u[0], this.u[1], this.u[2]];
  }

  speed(): number {
    return V.length(this.worldVelocity());
  }

  panelPoses(): PanelPose[] {
    const K = this.kin(this.x0, this.q0, this.theta, this.u);
    const q0 = Q.normalize(this.q0);
    const com = this.fx.af.com;
    return K.M.map((Mi) => ({
      q: Q.normalize(Q.mul(q0, Mi.q)),
      t: V.add(this.x0, M3.mulVec(K.R0, V.sub(Mi.t, com))),
    }));
  }

  debugForces(): { pos: Vec3[]; lift: Vec3[]; drag: Vec3[] } {
    const K = this.kin(this.x0, this.q0, this.theta, this.u);
    const d: AeroDebug = { pos: [], lift: [], drag: [], alpha: [] };
    const { poses, vb, wb } = this.surfacePoses(K);
    evaluateAero(this.fx.af, vb, wb, this.cfg.aero, this.cfg.rho, createAeroCache(this.fx.af), { poses, debug: d });
    const toW = (v: Vec3 | undefined): Vec3 => (v ? M3.mulVec(K.R0, v) : [0, 0, 0]);
    return {
      pos: (d.pos ?? []).map((p) => V.add(this.x0, M3.mulVec(K.R0, p))),
      lift: d.lift.map(toW),
      drag: d.drag.map(toW),
    };
  }

  private event(speed: number): FlightEvent {
    const p = this.x0;
    return { t: this.t, pos: [...p], speed, distance: Math.hypot(p[0] - this.start[0], p[2] - this.start[2]) };
  }

  private record() {
    const q = Q.normalize(this.q0);
    const af = this.fx.af;
    const noseW = Q.rotate(q, af.nose);
    const vel = this.worldVelocity();
    const vb = Q.rotate(Q.conjugate(q), vel);
    const wb = Q.rotate(Q.conjugate(q), [this.u[3], this.u[4], this.u[5]]);
    this.log.samples.push({
      t: this.t,
      pos: [...this.x0],
      vel,
      q: [...q],
      w: wb,
      phase: this.phase,
      pitchDeg: Math.asin(Math.max(-1, Math.min(1, noseW[1]))) * RAD,
      alphaDeg: Math.atan2(-V.dot(vb, af.up), V.dot(vb, af.nose)) * RAD,
    });
    this.nextLog = this.t + this.cfg.logInterval;
  }
}

function matVec6(M: number[], x: number[]): number[] {
  const out = new Array<number>(6).fill(0);
  for (let r = 0; r < 6; r++) {
    let s = 0;
    for (let c = 0; c < 6; c++) s += M[r * 6 + c] * x[c];
    out[r] = s;
  }
  return out;
}

function mulVecN(M: number[], x: number[], n: number): number[] {
  const out = new Array<number>(n).fill(0);
  for (let r = 0; r < n; r++) {
    let s = 0;
    for (let c = 0; c < n; c++) s += M[r * n + c] * x[c];
    out[r] = s;
  }
  return out;
}
