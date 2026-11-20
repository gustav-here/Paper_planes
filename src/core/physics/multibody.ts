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
import type { AeroCache, AeroDebug, ElementDeform, SurfacePose } from './aeroModel';
import { computeAddedMass } from './addedMass';
import type { PhysicsConfig } from './config';
import { ControlTrack, DEFAULT_CONTROLS } from './controls';
import type { ControlConfig, ControlInput } from './controls';
import { launchState } from './launch';
import type { LaunchParams } from './launch';
import { buildPanels, panelContacts } from './panels';
import { buildPanelModes, deflectionAt, limitStiffness } from './bendModes';
import type { PanelModes } from './bendModes';
import type { PanelSet } from './panels';
import type { DamageEvent, PanelBend, PanelPose, Simulator } from './simulator';
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
  /** Bending shapes of the large flat panels (see bendModes.ts). */
  modes: PanelModes[];
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
  const controlPanels = new Set(set.hinges.filter((h) => h.control >= 0).map((h) => h.child));
  const modes = cfg.paper.bendingModes
    ? buildPanelModes(model, set, mesh, panelSurfaces, rigid.com, controlPanels, cfg.paper, cfg.addedMass ? cfg.rho : 0, cfg.addedMassScale, cfg.aero.elementSizeM)
    : [];
  return { af, set, panelSurfaces, panelMA, contacts: panelContacts(model, set), model, modes };
}

function firstFacetOf(_m: PaperModel, set: PanelSet, panel: number): number {
  return set.panels[panel]?.facets[0] ?? -1;
}

const RAD = 180 / Math.PI;
/** Steps between refreshes of the aerodynamic damping matrix (1/80 s). */
const D_REFRESH = 3;
const skew = (a: Vec3): Mat3 => [0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0];

/**
 * A degree of freedom that moves a panel and everything beyond it: a crease hinge (rotation
 * about axis a through point h), or a bending shape of the parent panel at the crease where the
 * child hangs (the crease point moves by t and turns about a, per unit of the shape's amplitude).
 */
interface JointSpec {
  col: number;
  /** Panel whose motion carries the joint (the hinge's parent / the bending panel). */
  carrier: number;
  hinge: number;
  /** For bending joints: the bending panel's modes, the shape, and the crease point (model). */
  modes?: PanelModes;
  shape?: number;
  at?: Vec3;
}

interface AeroRaw {
  per: { F: Vec3; T: Vec3 }[];
  elemF: Float64Array;
}

interface Kin {
  R0: Mat3;
  M: RigidTransform[];
  /** Joint axes, points and translations (world), indexed like the joint list. */
  a: Vec3[];
  h: Vec3[];
  t: Vec3[];
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
  /** Bending amplitudes (m), all panels' shapes in order. */
  private qm: number[];
  private nm: number;
  private joints: JointSpec[] = [];
  /** Joints on the way from the root to each panel. */
  private jpath: number[][] = [];
  /** Bending shapes of each panel (or null). */
  private modesOf: (PanelModes | null)[];
  /** Point on each hinge line nearest the child's centre (model): where the child hangs. */
  private hingeMid: Vec3[];
  private deform: ElementDeform;
  /** Aerodynamic damping matrix ∂Q/∂u of the last refresh. */
  private Dcache: number[] | null = null;
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
    this.nm = fx.modes.reduce((s, m) => s + m.shapes.length, 0);
    this.n = 6 + this.k + this.nm;
    this.qm = new Array<number>(this.nm).fill(0);
    this.modesOf = set.panels.map((_, i) => fx.modes.find((m) => m.panel === i) ?? null);
    this.hingeMid = set.hinges.map((hg) => V.addScaled(hg.point, hg.axis, V.dot(V.sub(set.panels[hg.child].com, hg.point), hg.axis)));
    const nel = fx.af.mesh.elements.length;
    this.deform = { disp: new Float64Array(nel), slope: new Float64Array(nel * 3), vel: new Float64Array(nel) };
    this.buildJoints();
    this.buildBendTables();
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

  /** Joint list and each panel's joints from the root (static structure). */
  private buildJoints() {
    const { set } = this.fx;
    set.hinges.forEach((hg, j) => this.joints.push({ col: 6 + j, carrier: hg.parent, hinge: j }));
    this.jpath = set.panels.map(() => []);
    for (const i of this.order()) {
      const p = set.panels[i];
      if (p.parent < 0) continue;
      const path = [...this.jpath[p.parent]];
      const pm = this.modesOf[p.parent];
      if (pm) {
        pm.shapes.forEach((_, k) => {
          path.push(this.joints.length);
          this.joints.push({ col: 6 + this.k + pm.first + k, carrier: p.parent, hinge: p.hinge, modes: pm, shape: k, at: this.hingeMid[p.hinge] });
        });
      }
      path.push(p.hinge);
      this.jpath[i] = path;
    }
  }

  /** Small rigid motion of panel p's paper at model point x due to its bending (or identity). */
  private bendAt(p: number, x: Vec3): RigidTransform {
    const pm = this.modesOf[p];
    if (!pm) return rigidIdentity();
    const { w, grad } = deflectionAt(pm, this.qmFor, x);
    const Om = V.cross(grad, pm.normal);
    const ang = V.length(Om);
    const rot = ang > 1e-12 ? rotationAbout(x, V.scale(Om, 1 / ang), ang) : rigidIdentity();
    return composeRigid({ q: [0, 0, 0, 1], t: V.scale(pm.normal, w) }, rot);
  }

  /** Bending amplitudes in use while building kinematics (positions). */
  private qmFor: number[] = [];

  private kin(x0: Vec3, q0: Quat, theta: number[], u: number[], qm: number[] = this.qm): Kin {
    const { set, af } = this.fx;
    const com = af.com;
    this.qmFor = qm;
    const R0 = Q.toMat3(Q.normalize(q0));
    const M: RigidTransform[] = new Array(set.panels.length);
    const order = this.order();
    const nj = this.joints.length;
    const a: Vec3[] = new Array(nj);
    const h: Vec3[] = new Array(nj);
    const t: Vec3[] = new Array(nj).fill([0, 0, 0]);
    const toWorld = (T: RigidTransform, pm: Vec3): Vec3 => V.add(x0, M3.mulVec(R0, V.sub(applyRigid(T, pm), com)));
    const dirWorld = (T: RigidTransform, d: Vec3): Vec3 => M3.mulVec(R0, Q.rotate(T.q, d));
    for (const i of order) {
      const p = set.panels[i];
      if (p.parent < 0) {
        M[i] = rigidIdentity();
        continue;
      }
      const hg = set.hinges[p.hinge];
      // the parent's paper at the crease may be bent: the child hangs on the bent paper
      const Mp = composeRigid(M[p.parent], this.bendAt(p.parent, this.hingeMid[p.hinge]));
      M[i] = composeRigid(Mp, rotationAbout(hg.point, hg.axis, theta[p.hinge]));
      a[p.hinge] = dirWorld(Mp, hg.axis);
      h[p.hinge] = toWorld(Mp, hg.point);
    }
    // bending joints: how the crease point moves per unit amplitude of the parent's shapes
    this.joints.forEach((js, j) => {
      if (!js.modes) return;
      const pm = js.modes;
      const x = js.at!;
      const r = V.sub(x, pm.origin);
      const uu = V.dot(r, pm.eu);
      const vv = V.dot(r, pm.ev);
      const P = pm.shapes[js.shape!];
      const grad = V.add(V.scale(pm.eu, P.wu(uu, vv)), V.scale(pm.ev, P.wv(uu, vv)));
      const T = composeRigid(M[js.carrier], this.bendAt(js.carrier, x));
      a[j] = dirWorld(T, V.cross(grad, pm.normal));
      h[j] = toWorld(T, x);
      t[j] = V.scale(dirWorld(T, pm.normal), P.w(uu, vv));
    });
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
      for (const j of this.jpath[i]) {
        const col = this.joints[j].col;
        wi = V.addScaled(wi, a[j], u[col]);
        vi = V.add(vi, V.scale(V.add(V.cross(a[j], V.sub(c[i], h[j])), t[j]), u[col]));
      }
      w[i] = wi;
      v[i] = vi;
    });
    return { R0, M, a, h, t, c, Rp, w, v };
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

  /** Velocity of a world point carried rigidly by panel i (its own bending not included). */
  private pointVel(K: Kin, i: number, p: Vec3, u: number[]): Vec3 {
    let vel = V.add([u[0], u[1], u[2]], V.cross([u[3], u[4], u[5]], V.sub(p, this.x0)));
    for (const j of this.jpath[i]) vel = V.add(vel, V.scale(V.add(V.cross(K.a[j], V.sub(p, K.h[j])), K.t[j]), u[this.joints[j].col]));
    return vel;
  }

  /**
   * Jacobian rows (3×n) of the velocity of world point p on panel i. With `own` (the point's
   * model position) the panel's own bending moves the point too.
   */
  private pointJ(K: Kin, i: number, p: Vec3, own?: Vec3): number[][] {
    const n = this.n;
    const J = [new Array<number>(n).fill(0), new Array<number>(n).fill(0), new Array<number>(n).fill(0)];
    const r = V.sub(p, this.x0);
    const S = skew(r);
    for (let row = 0; row < 3; row++) {
      J[row][row] = 1;
      for (let c = 0; c < 3; c++) J[row][3 + c] = -S[row * 3 + c];
    }
    for (const j of this.jpath[i]) {
      const col = V.add(V.cross(K.a[j], V.sub(p, K.h[j])), K.t[j]);
      const cc = this.joints[j].col;
      for (let row = 0; row < 3; row++) J[row][cc] += col[row];
    }
    const pm = this.modesOf[i];
    if (own && pm) {
      const nW = M3.mulVec(K.Rp[i], pm.normal);
      const rr = V.sub(own, pm.origin);
      const uu = V.dot(rr, pm.eu);
      const vv = V.dot(rr, pm.ev);
      pm.shapes.forEach((P, k) => {
        const phi = P.w(uu, vv);
        for (let row = 0; row < 3; row++) J[row][6 + this.k + pm.first + k] += nW[row] * phi;
      });
    }
    return J;
  }

  /** Inertia of a bending panel in the world: rigid block (6×6), coupling columns (m × 6), bending block (m × m). */
  private augmentedInertia(K: Kin, i: number, pm: PanelModes): { rigid: number[]; coupling: number[][]; bend: number[] } {
    const m = pm.shapes.length;
    const N = 6 + m;
    const R = K.Rp[i];
    const A = pm.Maug;
    const at = (r: number, c: number) => A[r * N + c];
    const block = (r0: number, c0: number): Mat3 => [0, 1, 2].flatMap((r) => [0, 1, 2].map((c) => at(r0 + r, c0 + c)));
    const rot = (B: Mat3) => M3.mul(M3.mul(R, B), M3.transpose(R));
    const rigid = new Array<number>(36).fill(0);
    for (const [r0, c0] of [
      [0, 0],
      [0, 3],
      [3, 0],
      [3, 3],
    ] as const) {
      const B = rot(block(r0, c0));
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) rigid[(r0 + r) * 6 + c0 + c] = B[r * 3 + c];
    }
    const coupling = Array.from({ length: m }, (_, k) => [
      ...M3.mulVec(R, [at(0, 6 + k), at(1, 6 + k), at(2, 6 + k)]),
      ...M3.mulVec(R, [at(3, 6 + k), at(4, 6 + k), at(5, 6 + k)]),
    ]);
    const bend = new Array<number>(m * m);
    for (let k = 0; k < m; k++) for (let l = 0; l < m; l++) bend[k * m + l] = at(6 + k, 6 + l);
    return { rigid, coupling, bend };
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

  /** Panel Jacobians (6×n: CoM velocity, angular velocity; rigid motion of the panel frame). */
  private panelJ(K: Kin): number[][][] {
    const n = this.n;
    return this.fx.set.panels.map((_, i) => {
      const Jl = this.pointJ(K, i, K.c[i]);
      const Ja = [new Array<number>(n).fill(0), new Array<number>(n).fill(0), new Array<number>(n).fill(0)];
      for (let r = 0; r < 3; r++) Ja[r][3 + r] = 1;
      for (const j of this.jpath[i]) for (let r = 0; r < 3; r++) Ja[r][this.joints[j].col] += K.a[j][r];
      return [...Jl, ...Ja];
    });
  }

  /** Raw aerodynamics: per-surface force and moment (body frame, about the body origin) and per-element forces. */
  private aeroRaw(K: Kin, u: number[], only?: Set<number>, base?: AeroRaw, debug?: AeroDebug): AeroRaw {
    const { af } = this.fx;
    const { poses, vb, wb } = this.surfacePoses(K, u);
    const per: { F: Vec3; T: Vec3 }[] = base ? [...base.per] : [];
    const elemF = base ? Float64Array.from(base.elemF) : new Float64Array(af.mesh.elements.length * 3);
    // the shape is set once per step; only the rates change between evaluations
    this.fillDeformRates(u);
    evaluateAero(af, vb, wb, this.cfg.aero, this.cfg.rho, this.cache, { poses, perSurface: per, debug, deform: this.deform, elementForces: elemF, only });
    return { per, elemF };
  }

  /**
   * Generalized aerodynamic force: Σ Jᵢᵀ Wᵢ for the panels' rigid motion, and for each bending
   * shape Qₖ = Σₑ (Fₑ · n) φₖ(e) (the virtual work of the element forces). With `only`, just
   * the contribution of those surfaces.
   */
  private genForce(K: Kin, Js: number[][][], raw: AeroRaw, only?: Set<number>): number[] {
    const { set, panelSurfaces, af } = this.fx;
    const out = new Array<number>(this.n).fill(0);
    const R0t = M3.transpose(K.R0);
    set.panels.forEach((_, i) => {
      let F: Vec3 = [0, 0, 0];
      let T: Vec3 = [0, 0, 0];
      let any = false;
      for (const sid of panelSurfaces[i]) {
        if (only && !only.has(sid)) continue;
        const ps = raw.per[sid];
        if (!ps) continue;
        F = V.add(F, ps.F);
        T = V.add(T, ps.T);
        any = true;
      }
      if (!any) return;
      // moment about the panel CoM, then to the world
      const cb = M3.mulVec(R0t, V.sub(K.c[i], this.x0));
      const f = [...M3.mulVec(K.R0, F), ...M3.mulVec(K.R0, V.sub(T, V.cross(cb, F)))];
      const J = Js[i];
      for (let a = 0; a < this.n; a++) {
        let sum = 0;
        for (let r = 0; r < 6; r++) sum += J[r][a] * f[r];
        out[a] += sum;
      }
    });
    this.fx.modes.forEach((pm, mi) => {
      const t = this.bendEls[mi];
      const nb = Q.rotate(K.M[pm.panel].q, pm.normal);
      for (let j = 0; j < t.index.length; j++) {
        const i = t.index[j];
        if (only && !only.has(af.mesh.elements[i].surface)) continue;
        const fn = raw.elemF[i * 3] * nb[0] + raw.elemF[i * 3 + 1] * nb[1] + raw.elemF[i * 3 + 2] * nb[2];
        for (let k = 0; k < t.m; k++) out[t.col + k] += fn * t.phi[j * t.m + k];
      }
    });
    return out;
  }

  /** Flat tables of the bending panels' aero elements (built once). */
  private bendEls: { index: Int32Array; sign: Float64Array; phi: Float64Array; grad: Float64Array; m: number; col: number; first: number }[] = [];

  private buildBendTables() {
    const { af } = this.fx;
    this.bendEls = this.fx.modes.map((pm) => {
      const m = pm.shapes.length;
      const ne = pm.elements.length;
      const t = { index: new Int32Array(ne), sign: new Float64Array(ne), phi: new Float64Array(ne * m), grad: new Float64Array(ne * m * 3), m, col: 6 + this.k + pm.first, first: pm.first };
      pm.elements.forEach((e, j) => {
        t.index[j] = e.index;
        t.sign[j] = Math.sign(V.dot(af.mesh.surfaces[af.mesh.elements[e.index].surface].normal, pm.normal)) || 1;
        for (let k = 0; k < m; k++) {
          t.phi[j * m + k] = e.phi[k];
          t.grad.set(e.grad[k], (j * m + k) * 3);
        }
      });
      return t;
    });
  }

  /** Displacement and slope of the aero elements for the current amplitudes (once per step). */
  private fillDeformShape() {
    const d = this.deform;
    for (const t of this.bendEls) {
      for (let j = 0; j < t.index.length; j++) {
        let w = 0;
        let gx = 0;
        let gy = 0;
        let gz = 0;
        for (let k = 0; k < t.m; k++) {
          const q = this.qm[t.first + k];
          w += q * t.phi[j * t.m + k];
          const o = (j * t.m + k) * 3;
          gx += q * t.grad[o];
          gy += q * t.grad[o + 1];
          gz += q * t.grad[o + 2];
        }
        const i = t.index[j];
        const sg = t.sign[j];
        d.disp[i] = sg * w;
        d.slope[i * 3] = sg * gx;
        d.slope[i * 3 + 1] = sg * gy;
        d.slope[i * 3 + 2] = sg * gz;
      }
    }
  }

  /** Normal speed of the bending aero elements for the rates in u. */
  private fillDeformRates(u: number[]) {
    const d = this.deform;
    for (const t of this.bendEls) {
      for (let j = 0; j < t.index.length; j++) {
        let v = 0;
        for (let k = 0; k < t.m; k++) v += u[t.col + k] * t.phi[j * t.m + k];
        d.vel[t.index[j]] = t.sign[j] * v;
      }
    }
  }

  private fillDeform(u: number[]) {
    this.fillDeformShape();
    this.fillDeformRates(u);
  }

  /** Aero surfaces each generalized velocity moves (for cheap finite differences). */
  private colSurfaces: Map<number, Set<number>> | null = null;
  private surfacesByColumn(): Map<number, Set<number>> {
    if (this.colSurfaces) return this.colSurfaces;
    const { set, panelSurfaces } = this.fx;
    const m = new Map<number, Set<number>>();
    const add = (col: number, i: number) => {
      const st = m.get(col) ?? new Set<number>();
      for (const sid of panelSurfaces[i]) st.add(sid);
      m.set(col, st);
    };
    set.panels.forEach((_, i) => {
      for (const j of this.jpath[i]) add(this.joints[j].col, i);
      const pm = this.modesOf[i];
      if (pm) pm.shapes.forEach((_, k) => add(6 + this.k + pm.first + k, i));
    });
    return (this.colSurfaces = m);
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
      const pm = this.modesOf[i];
      // bending panels carry their whole inertia (rigid + bending + air) in one matrix
      const aug = pm ? this.augmentedInertia(K, i, pm) : null;
      const Mi = aug ? aug.rigid : this.panelInertia(K, i);
      // bias acceleration (J̇u)
      let ab: Vec3 = V.cross(w0, V.sub(K.v[i], [u[0], u[1], u[2]]));
      let alb: Vec3 = [0, 0, 0];
      for (const j of this.jpath[i]) {
        const js = this.joints[j];
        const wp = K.w[js.carrier];
        const adot = V.cross(wp, K.a[j]);
        const tdot = V.cross(wp, K.t[j]);
        const vh = this.pointVel(K, js.carrier, K.h[j], u);
        const uj = u[js.col];
        ab = V.add(ab, V.scale(V.add(V.add(V.cross(adot, V.sub(K.c[i], K.h[j])), V.cross(K.a[j], V.sub(K.v[i], vh))), tdot), uj));
        alb = V.addScaled(alb, adot, uj);
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
      // the panel's own bending: its mass (and air) moving with the shapes, coupled to the panel
      if (pm && aug) {
        const m = pm.shapes.length;
        const base = 6 + this.k + pm.first;
        const nW = M3.mulVec(K.Rp[i], pm.normal);
        const C = aug.coupling;
        for (let k = 0; k < m; k++) {
          for (let a = 0; a < n; a++) {
            let sum = 0;
            for (let r = 0; r < 6; r++) sum += J[r][a] * C[k][r];
            M[a * n + base + k] += sum;
            M[(base + k) * n + a] += sum;
          }
          for (let l = 0; l < m; l++) M[(base + k) * n + base + l] += aug.bend[k * m + l];
          // gravity on the bent paper; the elastic restoring force of the plate
          rhs[base + k] += pm.Spaper[k] * V.dot(g, nW);
          for (let l = 0; l < m; l++) rhs[base + k] -= pm.K[k * m + l] * (this.qm[pm.first + l] + this.cfg.paper.creaseDampingS * u[base + l]);
          // beyond the linear range the shape stiffens (the paper cannot stretch)
          rhs[base + k] -= limitStiffness(pm, k, this.qm[pm.first + k], this.cfg.paper) * this.qm[pm.first + k];
        }
      }
    });
    // crease springs (and control servos); damping is handled implicitly in step()
    set.hinges.forEach((hg, j) => {
      rhs[6 + j] += -hg.stiffness * (this.theta[j] - this.rest[j]) - hg.damping * u[6 + j];
      // folded layers meeting: a stiff contact spring stops paper passing through paper
      const kc = this.contactK(j);
      if (kc) rhs[6 + j] += -kc * (this.theta[j] - hg.closeAt) - kc * this.cfg.paper.creaseDampingS * u[6 + j];
    });
    return { M, rhs };
  }

  /** Contact stiffness of hinge j if its two sides are pressed onto each other, else 0. */
  private contactK(j: number): number {
    const hg = this.fx.set.hinges[j];
    if (hg.control >= 0) return 0;
    return (this.theta[j] - hg.closeAt) * hg.closeDir > 0 ? hg.creaseStiffness * this.cfg.paper.contactStiffnessFactor : 0;
  }

  /**
   * Sub-steps per fixed step. The creases and the air damping are integrated implicitly, so
   * stability does not need tiny steps; a few sub-steps keep the motion of light flaps accurate
   * (highest crease frequency resolved to ω·h ≲ 1).
   */
  private chooseSubsteps(): number {
    // the creases set the sub-steps; the paper's bending shapes are stiff, light and heavily
    // air-damped, and the implicit update keeps them stable at any step (they only matter as
    // a slow, smooth deflection)
    const nd = this.k;
    if (nd === 0) return 1;
    const K = this.kin(this.x0, this.q0, this.theta, this.u);
    const { M } = this.dynamics(K, this.panelJ(K));
    // with the bending shapes held (rigid panels), as for the creases alone
    const nr = 6 + this.k;
    const Mr = new Array<number>(nr * nr);
    for (let a = 0; a < nr; a++) for (let b = 0; b < nr; b++) Mr[a * nr + b] = M[a * this.n + b];
    const MrInv = invert(Mr, nr);
    const Minv = { at: (a: number, b: number) => MrInv[a * nr + b] };
    const stiff = this.stiffness();
    // power iteration on Minv_dd·K_dd for the highest elastic frequency
    let x = new Array<number>(nd).fill(1);
    let lambda = 0;
    for (let it = 0; it < 40; it++) {
      const kx = new Array<number>(nd).fill(0);
      const ns = this.k + this.nm;
      for (let a = 0; a < nd; a++) for (let b = 0; b < nd; b++) kx[a] += stiff[a * ns + b] * x[b];
      const y = new Array<number>(nd).fill(0);
      for (let a = 0; a < nd; a++) for (let b = 0; b < nd; b++) y[a] += Minv.at(6 + a, 6 + b) * kx[b];
      lambda = Math.hypot(...y);
      if (lambda === 0) break;
      x = y.map((v) => v / lambda);
    }
    const wmax = Math.sqrt(lambda);
    return Math.max(2, Math.min(24, Math.ceil(wmax * this.cfg.dt)));
  }

  /** Elastic stiffness of the hinge and bending coordinates (nd × nd, without contact springs). */
  private stiffness(): number[] {
    const nd = this.k + this.nm;
    const out = new Array<number>(nd * nd).fill(0);
    this.fx.set.hinges.forEach((hg, j) => (out[j * nd + j] = hg.stiffness));
    for (const pm of this.fx.modes) {
      const m = pm.shapes.length;
      for (let a = 0; a < m; a++)
        for (let b = 0; b < m; b++) out[(this.k + pm.first + a) * nd + this.k + pm.first + b] = pm.K[a * m + b];
    }
    return out;
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
      for (const j of this.jpath[i]) {
        const uj = u[this.joints[j].col];
        const ab = M3.mulVec(R0t, K.a[j]);
        const hb = M3.mulVec(R0t, V.sub(K.h[j], this.x0));
        W = V.addScaled(W, ab, uj);
        v0 = V.add(v0, V.scale(V.sub(M3.mulVec(R0t, K.t[j]), V.cross(ab, hb)), uj));
      }
      const pose: SurfacePose = { R, t, w: W, v0 };
      for (const sid of panelSurfaces[i]) poses[sid] = pose;
    });
    const vb = M3.mulVec(R0t, [u[0], u[1], u[2]]);
    const wb = M3.mulVec(R0t, [u[3], u[4], u[5]]);
    return { poses, vb, wb };
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
    this.fillDeformShape();
    const raw0 = this.aeroRaw(K0, u0);
    const Q0 = this.genForce(K0, J0, raw0);
    // Only the hinge and bending rates need implicit air damping (light panels); the root
    // body's aero damping is slow enough for explicit treatment, as in the rigid model. Each
    // column only re-evaluates the surfaces that rate moves.
    // the air's damping changes slowly: refresh it every few steps
    const fresh = !this.Dcache || this.stepIndex % D_REFRESH === 0;
    const D = fresh ? new Array<number>(n * n).fill(0) : this.Dcache!;
    const cols = this.surfacesByColumn();
    for (let c = 6; c < n && fresh; c++) {
      const only = cols.get(c);
      if (!only || !only.size) continue;
      const up = [...u0];
      const eps = 1e-3;
      up[c] += eps;
      const rawP = this.aeroRaw(K0, up, only, raw0);
      const Qp = this.genForce(K0, J0, rawP, only);
      const Qb = this.genForce(K0, J0, raw0, only);
      for (let r = 0; r < n; r++) D[r * n + c] = (Qp[r] - Qb[r]) / eps;
    }
    this.Dcache = D;
    this.fillDeform(u0);
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
        const kc = this.contactK(j);
        const k = hg.stiffness + kc;
        A[(6 + j) * n + 6 + j] += h * (hg.damping + kc * this.cfg.paper.creaseDampingS) + h * h * k;
        // backward Euler for the spring: −k(θ + h·θ̇_new) → extra −h·k·θ̇ on the right
        f[6 + j] -= h * k * this.u[6 + j];
      });
      // the plate's bending stiffness and damping, implicitly in the same way
      const tau = this.cfg.paper.creaseDampingS;
      for (const pm of this.fx.modes) {
        const m = pm.shapes.length;
        const b = 6 + this.k + pm.first;
        for (let a = 0; a < m; a++)
          for (let c = 0; c < m; c++) {
            const kk = pm.K[a * m + c];
            A[(b + a) * n + b + c] += h * tau * kk + h * h * kk;
            f[b + a] -= h * kk * this.u[b + c];
          }
        for (let a = 0; a < m; a++) {
          const q = this.qm[pm.first + a];
          // the tangent stiffness of the limit, 7·k_extra (force ∝ q⁷), implicitly for stability
          const kl = 7 * limitStiffness(pm, a, q, this.cfg.paper);
          A[(b + a) * n + b + a] += h * h * kl;
          f[b + a] -= h * kl * this.u[b + a];
        }
      }
      const du = mulVecN(invert(A, n), f, n);
      for (let i = 0; i < n; i++) this.u[i] += h * du[i];
      // floor impulses act on the new velocities, before the positions move
      this.contact(h, M, K);
      // positions with the new velocities (semi-implicit Euler)
      this.x0 = V.add(this.x0, V.scale([this.u[0], this.u[1], this.u[2]], h));
      const wq: Quat = [this.u[3], this.u[4], this.u[5], 0];
      const dq = Q.mul(wq, this.q0);
      this.q0 = Q.normalize([this.q0[0] + 0.5 * h * dq[0], this.q0[1] + 0.5 * h * dq[1], this.q0[2] + 0.5 * h * dq[2], this.q0[3] + 0.5 * h * dq[3]]);
      for (let j = 0; j < this.k; j++) this.theta[j] += h * this.u[6 + j];
      for (let j = 0; j < this.nm; j++) this.qm[j] += h * this.u[6 + this.k + j];
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

  // ---------------------------------------------------------------- floor, walls, ceiling

  private contact(dt: number, Mfull: number[], K: Kin) {
    const { contacts, af } = this.fx;
    const gcfg = this.cfg.ground;
    const com = af.com;
    type C = { i: number; p: Vec3; J: number[][]; depth: number; target: number; jn: number; jt: Vec3; facet: number; pm: Vec3; n: Vec3 };
    const active: C[] = [];
    const minGap = gcfg.planes.map(() => Infinity);
    for (const ct of contacts) {
      // the paper's own bending moves its corners too
      const pm = this.modesOf[ct.panel];
      const pBent = pm ? V.addScaled(ct.p, pm.normal, deflectionAt(pm, this.qm, ct.p).w) : ct.p;
      const p = V.add(this.x0, M3.mulVec(K.R0, V.sub(applyRigid(K.M[ct.panel], pBent), com)));
      gcfg.planes.forEach((pl, k) => {
        const gap = V.dot(pl.n, p) - pl.d;
        minGap[k] = Math.min(minGap[k], gap);
        if (gap > 1e-5) return;
        active.push({ i: ct.panel, p, J: this.pointJ(K, ct.panel, p, ct.p), depth: -gap, target: 0, jn: 0, jt: [0, 0, 0], facet: ct.facet, pm: ct.p, n: pl.n });
      });
    }
    if (!active.length) return;
    const n = this.n;
    const Minv = invert(Mfull, n);
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
    let impact = 0;
    let hardest: C | null = null;
    let hardestVel: Vec3 = [0, 0, 0];
    for (const c of active) {
      const v = vel(c);
      const vn = V.dot(v, c.n);
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
        const gn = gen(c, c.n);
        const vn = V.dot(vel(c), c.n);
        let dj = (c.target - vn) / invMass(gn);
        const old = c.jn;
        c.jn = Math.max(0, old + dj);
        dj = c.jn - old;
        if (dj !== 0) apply(gn, dj);
        const v = vel(c);
        const vt = V.rejectFrom(v, c.n);
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
    gcfg.planes.forEach((pl, k) => {
      if (minGap[k] < -0.001) this.x0 = V.addScaled(this.x0, pl.n, -minGap[k] - 0.001);
    });
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
      // the hinge is the crease in series with elastic paper: only the crease's share yields
      const dev = this.theta[j] - this.rest[j];
      const share = hg.stiffness / hg.creaseStiffness;
      if (Math.abs(dev * share) > y) this.rest[j] = this.theta[j] - (Math.sign(dev) * y) / share;
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

  /** How each panel's paper is bent right now, or null for a panel without bending shapes. */
  panelBending(): (PanelBend | null)[] {
    const q = [...this.qm];
    return this.modesOf.map((pm) => (pm ? { normal: pm.normal, at: (p: Vec3) => deflectionAt(pm, q, p) } : null));
  }

  /** Largest bending deflection of any panel right now (m), for display and tests. */
  maxBendingM(): number {
    let best = 0;
    for (const pm of this.fx.modes) {
      for (const e of pm.elements) {
        let w = 0;
        e.phi.forEach((phi, k) => (w += this.qm[pm.first + k] * phi));
        best = Math.max(best, Math.abs(w));
      }
    }
    return best;
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
    this.fillDeform(this.u);
    evaluateAero(this.fx.af, vb, wb, this.cfg.aero, this.cfg.rho, createAeroCache(this.fx.af), { poses, debug: d, deform: this.deform });
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
