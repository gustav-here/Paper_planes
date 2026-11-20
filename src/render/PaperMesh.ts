import * as THREE from 'three';
import * as V from '../core/math/vec3';
import type { Vec3 } from '../core/math/vec3';
import { canonicalNormal, facetNormal, facetVertices, toModel } from '../core/paper/model';
import type { Facet, PaperModel } from '../core/paper/model';
import { edgesOf, levelsOf } from './modelCache';

/** Visual distance between stacked layers (real paper is 0.1 mm; slightly exaggerated). */
export const VIS_LAYER = 0.00025;
const LINE_LIFT = 0.00005;
const FRONT = new THREE.Color('#fbf9f3');
const BACK = new THREE.Color('#e9e2d0');

export interface PaperHit {
  facetId: number;
  /** Hit point projected onto the facet's true (zero-thickness) plane, model space. */
  point: Vec3;
  /** Facet normal (front side), model space. */
  normal: Vec3;
}

/** Three.js view of a PaperModel, in model coordinates (parent applies view rotation/lift). */
export class PaperView {
  readonly group = new THREE.Group();
  readonly material = new THREE.MeshStandardMaterial({
    vertexColors: true,
    roughness: 0.92,
    metalness: 0,
    side: THREE.FrontSide,
  });
  private readonly lineMat = new THREE.LineBasicMaterial({ color: '#5f594e' });
  private readonly creaseMat = new THREE.LineBasicMaterial({ color: '#9a9283' });
  private mesh: THREE.Mesh | null = null;
  private lines: THREE.LineSegments | null = null;
  private creases: THREE.LineSegments | null = null;
  private triFacet: number[] = [];
  private facets = new Map<number, Facet>();
  model: PaperModel | null = null;

  update(model: PaperModel) {
    if (model === this.model) return;
    this.model = model;
    this.dispose();
    const levels = levelsOf(model);
    this.facets = new Map(model.facets.map((f) => [f.id, f]));
    const pos: number[] = [];
    const nor: number[] = [];
    const col: number[] = [];
    this.triFacet = [];

    for (const f of model.facets) {
      const n = facetNormal(f);
      const off = this.offsetVec(f, levels.get(f.id) ?? 0);
      const vs = facetVertices(f).map((v) => V.add(v, off));
      for (let i = 1; i + 1 < vs.length; i++) {
        let a = vs[0];
        let b = vs[i];
        let c = vs[i + 1];
        if (V.dot(V.cross(V.sub(b, a), V.sub(c, a)), n) < 0) [b, c] = [c, b];
        // front (visible from +n)
        pos.push(...a, ...b, ...c);
        for (let k = 0; k < 3; k++) {
          nor.push(...n);
          col.push(FRONT.r, FRONT.g, FRONT.b);
        }
        this.triFacet.push(f.id);
        // back (visible from −n)
        a = vs[0];
        pos.push(...a, ...c, ...b);
        for (let k = 0; k < 3; k++) {
          nor.push(-n[0], -n[1], -n[2]);
          col.push(BACK.r, BACK.g, BACK.b);
        }
        this.triFacet.push(f.id);
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
    geo.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    geo.computeBoundingSphere();
    this.mesh = new THREE.Mesh(geo, this.material);
    this.mesh.castShadow = true;
    this.group.add(this.mesh);

    const graph = edgesOf(model);
    const border: number[] = [];
    const crease: number[] = [];
    const pushSeg = (arr: number[], f: Facet, a: [number, number], b: [number, number]) => {
      const off = this.offsetVec(f, levels.get(f.id) ?? 0);
      const n = facetNormal(f);
      for (const s of [LINE_LIFT, -LINE_LIFT]) {
        const o = V.addScaled(off, n, s);
        arr.push(...V.add(toModel(f, a), o), ...V.add(toModel(f, b), o));
      }
    };
    for (const b of graph.borders) pushSeg(border, this.facets.get(b.facet)!, b.seg[0], b.seg[1]);
    for (const l of graph.links) {
      if (l.kind !== 'crease') continue;
      pushSeg(crease, this.facets.get(l.a)!, l.seg[0], l.seg[1]);
      pushSeg(crease, this.facets.get(l.b)!, l.seg[0], l.seg[1]);
    }
    this.lines = this.makeLines(border, this.lineMat);
    this.creases = this.makeLines(crease, this.creaseMat);
  }

  private makeLines(arr: number[], mat: THREE.LineBasicMaterial) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(arr, 3));
    const l = new THREE.LineSegments(g, mat);
    this.group.add(l);
    return l;
  }

  /** Render offset for a facet at a given stack level. */
  offsetVec(f: Facet, level: number): Vec3 {
    return V.scale(canonicalNormal(facetNormal(f)), level * VIS_LAYER);
  }

  levelOffset(facetId: number): Vec3 {
    const f = this.facets.get(facetId);
    if (!f || !this.model) return [0, 0, 0];
    return this.offsetVec(f, levelsOf(this.model).get(facetId) ?? 0);
  }

  /** X-ray: translucent paper, outline drawn through. */
  setXRay(opacity: number) {
    const transparent = opacity < 0.999;
    this.material.transparent = transparent;
    this.material.opacity = opacity;
    this.material.depthWrite = !transparent;
    this.material.needsUpdate = true;
    for (const m of [this.lineMat, this.creaseMat]) {
      m.depthTest = !transparent;
      m.transparent = transparent;
      m.opacity = transparent ? 0.7 : 1;
      m.needsUpdate = true;
    }
  }

  hit(raycaster: THREE.Raycaster): PaperHit | null {
    if (!this.mesh) return null;
    const hits = raycaster.intersectObject(this.mesh, false);
    const h = hits[0];
    if (!h || h.faceIndex == null) return null;
    const facetId = this.triFacet[h.faceIndex];
    const f = this.facets.get(facetId)!;
    const local = this.group.worldToLocal(h.point.clone());
    const p: Vec3 = [local.x, local.y, local.z];
    const n = facetNormal(f);
    const v0 = facetVertices(f)[0];
    const point = V.addScaled(p, n, -V.dot(n, V.sub(p, v0)));
    return { facetId, point, normal: n };
  }

  dispose() {
    for (const o of [this.mesh, this.lines, this.creases]) {
      if (!o) continue;
      this.group.remove(o);
      o.geometry.dispose();
    }
    this.mesh = this.lines = this.creases = null;
  }
}
