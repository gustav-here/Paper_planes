import * as THREE from 'three';
import * as V from '../core/math/vec3';
import * as Q from '../core/math/quat';
import type { Vec3 } from '../core/math/vec3';
import type { Quat } from '../core/math/quat';
import type { PaperModel } from '../core/paper/model';
import { resolveClip } from '../core/paper/clips';
import type { Airframe } from '../core/physics/airframe';
import type { AeroDebug } from '../core/physics/aeroModel';
import type { FlightEvent } from '../core/physics/simulation';
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
export class FlightView {
  /** World pose of the body frame (origin = CoM). */
  readonly plane = new THREE.Group();
  private modelGroup = new THREE.Group();
  private paper = new PaperView();
  private clips = new THREE.Group();
  private debug = new THREE.Group();
  private lift = new SegmentSet('#3ddc84', 4000);
  private drag = new SegmentSet('#ff5a5f', 4000);
  private vel: THREE.ArrowHelper;
  private com: THREE.Mesh;
  private air = new Trail('#2f6fde');
  private ground = new Trail('#f08c2e');
  private markers = new THREE.Group();
  private sprites: THREE.Sprite[] = [];
  private af: Airframe | null = null;

  constructor(readonly scene: THREE.Scene) {
    this.plane.add(this.modelGroup);
    this.modelGroup.add(this.paper.group, this.clips);
    scene.add(this.plane, this.air.line, this.ground.line, this.markers, this.debug);
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

  setAirframe(model: PaperModel, af: Airframe) {
    this.af = af;
    this.paper.update(model);
    this.modelGroup.position.set(-af.com[0], -af.com[1], -af.com[2]);
    for (const c of [...this.clips.children]) this.clips.remove(c);
    for (const clip of model.clips) {
      const rc = resolveClip(model, clip);
      if (rc) this.clips.add(createClipObject(model, rc));
    }
  }

  setPose(x: Vec3, q: Quat) {
    this.plane.position.set(x[0], x[1], x[2]);
    this.plane.quaternion.set(q[0], q[1], q[2], q[3]);
  }

  setDebugVisible(on: boolean) {
    this.debug.visible = on;
  }

  /** Update debug arrows for the given pose, world velocity and per-element forces (body frame). */
  updateDebug(x: Vec3, q: Quat, velWorld: Vec3, forces: AeroDebug | null) {
    if (!this.debug.visible || !this.af) return;
    const af = this.af;
    this.com.position.set(...x);
    const speed = V.length(velWorld);
    if (speed > 1e-3) {
      this.vel.visible = true;
      this.vel.position.set(...x);
      this.vel.setDirection(new THREE.Vector3(...velWorld).normalize());
      this.vel.setLength(Math.max(0.1 * speed, 0.08), 0.06, 0.035);
    } else this.vel.visible = false;
    if (!forces) return;
    // an element carrying its share of the weight gets a 3 cm arrow
    const k = (0.03 * af.mesh.elements.length) / (af.mass * 9.81);
    const qn = Q.normalize(q);
    const L: [Vec3, Vec3][] = [];
    const D: [Vec3, Vec3][] = [];
    af.mesh.elements.forEach((e, i) => {
      const p = V.add(x, Q.rotate(qn, e.center));
      const l = forces.lift[i];
      const d = forces.drag[i];
      if (l) L.push([p, V.add(p, V.scale(Q.rotate(qn, l), k))]);
      if (d) D.push([p, V.add(p, V.scale(Q.rotate(qn, d), k))]);
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

  clear() {
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
