import { AshEffect, samplePaper } from './AshEffect';
import { DustEffect } from './DustEffect';
import * as THREE from 'three';
import * as V from '../core/math/vec3';
import type { Vec3 } from '../core/math/vec3';
import type { PaperModel } from '../core/paper/model';
import { resolveClip } from '../core/paper/clips';
import type { FlightEvent } from '../core/physics/simulation';
import type { PanelBend, PanelPose } from '../core/physics/simulator';
import type { Target } from '../core/game/competition';
import { PaperView } from './PaperMesh';
import { createClipObject } from './ClipMesh';
import { createTextSprite, disposeSprite } from './TextSprite';

const TRAIL_MAX = 20000;

class Trail {
  readonly line: THREE.Line;
  private n = 0;
  private buf = new Float32Array(TRAIL_MAX * 3);

  constructor(color: string) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.buf, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setDrawRange(0, 0);
    this.line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.9 }));
    this.line.frustumCulled = false;
  }

  push(p: Vec3) {
    if (this.n >= TRAIL_MAX) return;
    this.buf.set(p, this.n * 3);
    this.n++;
    const attr = this.line.geometry.getAttribute('position') as THREE.BufferAttribute;
    attr.needsUpdate = true;
    this.line.geometry.setDrawRange(0, this.n);
  }

  clear() {
    this.n = 0;
    this.line.geometry.setDrawRange(0, 0);
  }
}

/** Dynamic line segments for per-element force arrows (world space). */
class SegmentSet {
  readonly lines: THREE.LineSegments;
  private buf: Float32Array;

  constructor(color: string, max: number) {
    this.buf = new Float32Array(max * 6);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.buf, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setDrawRange(0, 0);
    this.lines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 }));
    this.lines.frustumCulled = false;
    this.lines.renderOrder = 40;
  }

  set(segs: [Vec3, Vec3][]) {
    const n = Math.min(segs.length, this.buf.length / 6);
    for (let i = 0; i < n; i++) {
      this.buf.set(segs[i][0], i * 6);
      this.buf.set(segs[i][1], i * 6 + 3);
    }
    (this.lines.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    this.lines.geometry.setDrawRange(0, n * 2);
  }
}

/**
 * Everything drawn in the flight world: the folded plane (posed by the simulation), flight and
 * ground trails, impact/rest markers and the debug overlay (CoM, velocity, per-element forces).
 */
/** Longest triangle edge of the paper in flight (m): fine enough to show smooth bending. */
const BEND_EDGE = 0.012;
/** Visual lift of the paper above its simulated position (m). */
const PAPER_LIFT = 0.001;

export class FlightView {
  /** One group per render panel, placed model → world by the simulation. */
  readonly plane = new THREE.Group();
  private panelGroups: THREE.Group[] = [];
  private papers: PaperView[] = [];
  private gridOn = false;
  private targetGroup = new THREE.Group();
  private debug = new THREE.Group();
  private lift = new SegmentSet('#3ddc84', 4000);
  private drag = new SegmentSet('#ff5a5f', 4000);
  private vel: THREE.ArrowHelper;
  private com: THREE.Mesh;
  private air = new Trail('#2f6fde');
  private ground = new Trail('#f08c2e');
  private markers = new THREE.Group();
  private sprites: THREE.Sprite[] = [];
  private forceScale = 1;
  private ash: AshEffect | null = null;
  private dust: DustEffect | null = null;

  constructor(readonly scene: THREE.Scene) {
    // the paper is drawn a hair above where the simulation has it (paper thickness and the
    // contact's tolerance), so paper lying on the floor is never hidden in it
    this.plane.position.y = PAPER_LIFT;
    scene.add(this.plane, this.air.line, this.ground.line, this.markers, this.debug, this.targetGroup);
    this.vel = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.5, 0xffd34d, 0.06, 0.035);
    this.vel.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.material) (m.material as THREE.Material).depthTest = false;
      o.renderOrder = 41;
    });
    this.com = new THREE.Mesh(new THREE.SphereGeometry(0.008, 16, 12), new THREE.MeshBasicMaterial({ color: '#ffd34d', depthTest: false }));
    this.com.renderOrder = 42;
    this.debug.add(this.lift.lines, this.drag.lines, this.vel, this.com);
    this.debug.visible = false;
  }

  /**
   * Build the plane from render panels (facet id lists): each panel is drawn as its own
   * sub-model so flaps and flexing wings can move independently.
   */
  setModel(model: PaperModel, panels: number[][], mass: number, elementCount: number) {
    for (const g of this.panelGroups) this.plane.remove(g);
    for (const p of this.papers) p.dispose();
    this.panelGroups = [];
    this.papers = [];
    const owner = new Map<number, number>();
    panels.forEach((ids, i) => ids.forEach((id) => owner.set(id, i)));
    panels.forEach((ids) => {
      const g = new THREE.Group();
      // small triangles, so flexible paper can be drawn bent
      const pv = new PaperView({ maxEdge: BEND_EDGE });
      const set = new Set(ids);
      pv.update({ ...model, facets: model.facets.filter((f) => set.has(f.id)), clips: [] });
      pv.setGrid(this.gridOn);
      g.add(pv.group);
      this.plane.add(g);
      this.panelGroups.push(g);
      this.papers.push(pv);
    });
    for (const clip of model.clips) {
      const rc = resolveClip(model, clip);
      if (!rc) continue;
      const panel = rc.clamped.map((id) => owner.get(id)).find((x) => x !== undefined) ?? 0;
      this.panelGroups[panel]?.add(createClipObject(model, rc));
    }
    // an element carrying its share of the weight gets a 3 cm arrow
    this.forceScale = (0.03 * Math.max(elementCount, 1)) / (mass * 9.81);
  }

  setGrid(on: boolean) {
    this.gridOn = on;
    for (const p of this.papers) p.setGrid(on);
  }

  /** Bend each panel's paper (flexible flight), or straighten it all with null. */
  setPanelBending(bends: (PanelBend | null)[] | null) {
    this.papers.forEach((pv, i) => pv.bend(bends?.[i] ?? null));
  }

  setPanelPoses(poses: PanelPose[]) {
    poses.forEach((p, i) => {
      const g = this.panelGroups[i];
      if (!g) return;
      g.position.set(p.t[0], p.t[1], p.t[2]);
      g.quaternion.set(p.q[0], p.q[1], p.q[2], p.q[3]);
    });
  }

  /** Precision target: rings on the floor and a flag. */
  setTarget(t: Target | null) {
    for (const c of [...this.targetGroup.children]) {
      this.targetGroup.remove(c);
      c.traverse((o) => (o as THREE.Mesh).geometry?.dispose());
    }
    if (!t) return;
    const colors = ['#e0645a', '#ffffff', '#e0645a'];
    t.rings.forEach((r, i) => {
      const inner = i === 0 ? 0 : t.rings[i - 1];
      const m = new THREE.Mesh(
        new THREE.RingGeometry(inner, r, 64),
        new THREE.MeshBasicMaterial({ color: colors[i % colors.length], transparent: true, opacity: 0.85, side: THREE.DoubleSide }),
      );
      m.rotation.x = -Math.PI / 2;
      m.position.set(t.x, 0.004 + i * 0.0005, t.z);
      this.targetGroup.add(m);
    });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 1.2, 8), new THREE.MeshBasicMaterial({ color: '#ffffff' }));
    pole.position.set(t.x, 0.6, t.z);
    const label = createTextSprite(`Mål ${Math.hypot(t.x, t.z).toFixed(1).replace('.', ',')} m`, {
      height: 0.05,
      color: '#ffffff',
      background: 'rgba(224,100,90,0.9)',
      screenSpace: true,
    });
    label.center.set(0.5, 0);
    label.position.set(t.x, 1.25, t.z);
    this.targetGroup.add(pole, label);
  }

  setDebugVisible(on: boolean) {
    this.debug.visible = on;
  }

  /** Update debug arrows: reference point, world velocity and per-element forces (world frame). */
  updateDebug(x: Vec3, velWorld: Vec3, forces: { pos: Vec3[]; lift: Vec3[]; drag: Vec3[] } | null) {
    if (!this.debug.visible) return;
    this.com.position.set(...x);
    const speed = V.length(velWorld);
    if (speed > 1e-3) {
      this.vel.visible = true;
      this.vel.position.set(...x);
      this.vel.setDirection(new THREE.Vector3(...velWorld).normalize());
      this.vel.setLength(Math.max(0.1 * speed, 0.08), 0.06, 0.035);
    } else this.vel.visible = false;
    if (!forces) return;
    const k = this.forceScale;
    const L: [Vec3, Vec3][] = [];
    const D: [Vec3, Vec3][] = [];
    forces.pos.forEach((p, i) => {
      if (!p) return;
      L.push([p, V.addScaled(p, forces.lift[i], k)]);
      D.push([p, V.addScaled(p, forces.drag[i], k)]);
    });
    this.lift.set(L);
    this.drag.set(D);
  }

  addTrailPoint(p: Vec3, onGround: boolean) {
    const lifted: Vec3 = [p[0], Math.max(p[1], 0.004), p[2]];
    (onGround ? this.ground : this.air).push(lifted);
  }

  /** Ring on the floor plus a label. */
  private marker(p: Vec3, color: string, text: string) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.07, 0.09, 40),
      new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.95 }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(p[0], 0.005, p[2]);
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.4, 8), new THREE.MeshBasicMaterial({ color }));
    pole.position.set(p[0], 0.2, p[2]);
    const label = createTextSprite(text, { height: 0.055, color: '#ffffff', background: 'rgba(20,24,30,0.82)', screenSpace: true });
    label.position.set(p[0], 0.45, p[2]);
    label.center.set(0.5, 0);
    this.sprites.push(label);
    this.markers.add(ring, pole, label);
  }

  markImpact(e: FlightEvent) {
    this.marker(e.pos, '#2f6fde', `Nedslag ${e.distance.toFixed(2).replace('.', ',')} m\n${e.t.toFixed(2).replace('.', ',')} s · ${e.speed.toFixed(1).replace('.', ',')} m/s`);
  }

  markRest(e: FlightEvent, impact: FlightEvent | null) {
    const slide = impact ? Math.hypot(e.pos[0] - impact.pos[0], e.pos[2] - impact.pos[2]) : 0;
    this.marker(e.pos, '#f08c2e', `Vila ${e.distance.toFixed(2).replace('.', ',')} m\n(+${slide.toFixed(2).replace('.', ',')} m glid/studs)`);
  }

  /** The paper burns up: the plane is replaced by flakes of ash. */
  burn() {
    this.endAsh();
    this.ash = new AshEffect(samplePaper(this.plane, 2500));
    this.plane.visible = false;
    this.scene.add(this.ash.points);
  }

  /** Advance effects; returns true while the ash is still in the air. */
  updateEffects(dt: number): boolean {
    if (!this.ash) return false;
    if (this.ash.update(dt)) return true;
    this.endAsh();
    return false;
  }

  /** A puff of dust where the paper lands (the Moon). */
  kickDust(at: Vec3, along: Vec3, speed: number, g: number) {
    this.endDust();
    const dir = new THREE.Vector3(along[0], 0, along[2]);
    if (dir.lengthSq() < 1e-9) dir.set(1, 0, 0);
    this.dust = new DustEffect(new THREE.Vector3(...at), dir.normalize(), speed, g);
    this.scene.add(this.dust.points);
  }

  updateDust(dt: number) {
    if (this.dust && !this.dust.update(dt)) this.endDust();
  }

  private endDust() {
    if (!this.dust) return;
    this.scene.remove(this.dust.points);
    this.dust.dispose();
    this.dust = null;
  }

  private endAsh() {
    if (!this.ash) return;
    this.scene.remove(this.ash.points);
    this.ash.dispose();
    this.ash = null;
  }

  clear() {
    this.endAsh();
    this.endDust();
    this.plane.visible = true;
    this.air.clear();
    this.ground.clear();
    for (const c of [...this.markers.children]) {
      this.markers.remove(c);
      const m = c as THREE.Mesh;
      if (m.geometry) m.geometry.dispose();
    }
    this.sprites.forEach(disposeSprite);
    this.sprites = [];
    this.lift.set([]);
    this.drag.set([]);
  }
}
