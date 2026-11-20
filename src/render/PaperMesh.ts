import * as THREE from 'three';
import * as V from '../core/math/vec3';
import type { Vec3 } from '../core/math/vec3';
import * as P from '../core/math/polygon2d';
import type { Vec2 } from '../core/math/vec2';
import { canonicalNormal, facetNormal, facetVertices, toModel } from '../core/paper/model';
import type { Facet, PaperModel } from '../core/paper/model';
import { edgesOf, levelsOf } from './modelCache';

/** Visual distance between stacked layers (real paper is 0.1 mm; slightly exaggerated). */
export const VIS_LAYER = 0.00025;
const LINE_LIFT = 0.00005;
const GRID_LIFT = 0.00004;
/** Grid paper: this many squares across the sheet's width; each square is split in four. */
const GRID_SQUARES_ACROSS = 10;
const FRONT = new THREE.Color('#fbf9f3');
const BACK = new THREE.Color('#e9e2d0');
/** Crushed paper (impact damage): duller, greyer, with a mottled look per crumpled facet. */
const CRUMPLED = [new THREE.Color('#ddd3bd'), new THREE.Color('#cbbfa4')];
/**
 * Facets meeting at less than this angle are drawn without a crease line (≈ 2°): paper that only
 * twists a little across the link. Folds are at least 5°; a 180° fold has opposite normals.
 */
const FLAT_COS = Math.cos((2 * Math.PI) / 180);
const isCrumpled = (f: Facet) => f.foldPath.some((id) => id.startsWith('dmg-crumple'));
/** Tints: control flaps (orange) and hover highlight (blue). */
export type FacetTint = 'control' | 'hover' | 'controlHover';
const TINTS: Record<FacetTint, [THREE.Color, THREE.Color]> = {
  control: [new THREE.Color('#ffd7a3'), new THREE.Color('#efc086')],
  hover: [new THREE.Color('#d3e5ff'), new THREE.Color('#bcd3f2')],
  controlHover: [new THREE.Color('#ffc07a'), new THREE.Color('#e9a75e')],
};

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
  private readonly gridMat = new THREE.LineBasicMaterial({ color: '#7fa6cf', transparent: true, opacity: 0.85 });
  private readonly gridDashMat = new THREE.LineDashedMaterial({
    color: '#86abd4',
    dashSize: 0.0015,
    gapSize: 0.0011,
    transparent: true,
    opacity: 0.9,
  });
  private gridOn = false;
  private grid: THREE.LineSegments[] = [];
  private triFacet: number[] = [];
  private tints = new Map<number, FacetTint>();
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
    this.applyTints(geo);
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
      // paper that only twists a little across the link (springback held by a clip) has no crease
      const fa = this.facets.get(l.a)!;
      const fb = this.facets.get(l.b)!;
      if (V.dot(facetNormal(fa), facetNormal(fb)) > FLAT_COS) continue;
      pushSeg(crease, this.facets.get(l.a)!, l.seg[0], l.seg[1]);
      pushSeg(crease, this.facets.get(l.b)!, l.seg[0], l.seg[1]);
    }
    this.lines = this.makeLines(border, this.lineMat);
    this.creases = this.makeLines(crease, this.creaseMat);
    this.rebuildGrid();
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

  /** Tint facets (control flaps, hover). Pass an empty map to clear. */
  setTints(tints: Map<number, FacetTint>) {
    const same = tints.size === this.tints.size && [...tints].every(([k, v]) => this.tints.get(k) === v);
    if (same) return;
    this.tints = new Map(tints);
    if (this.mesh) this.applyTints(this.mesh.geometry);
  }

  private applyTints(geo: THREE.BufferGeometry) {
    const attr = geo.getAttribute('color') as THREE.BufferAttribute;
    // triangles alternate front/back per facet fan (see update)
    for (let t = 0; t < this.triFacet.length; t++) {
      const id = this.triFacet[t];
      const tint = this.tints.get(id);
      const back = t % 2 === 1;
      let c = tint ? TINTS[tint][back ? 1 : 0] : back ? BACK : FRONT;
      const f = this.facets.get(id);
      if (!tint && f && isCrumpled(f)) {
        // deterministic per-facet shade: the crinkled zigzag catches the light unevenly
        const shade = 0.86 + 0.14 * (((id * 2654435761) >>> 0) / 4294967296);
        c = CRUMPLED[back ? 1 : 0].clone().multiplyScalar(shade);
      }
      for (let k = 0; k < 3; k++) attr.setXYZ(t * 3 + k, c.r, c.g, c.b);
    }
    attr.needsUpdate = true;
  }

  /** Show or hide the grid-paper print. */
  setGrid(on: boolean) {
    if (on === this.gridOn) return;
    this.gridOn = on;
    this.rebuildGrid();
  }

  /**
   * Grid paper: squares of width/10 (21 mm on A4), aligned with the long centre line and the top
   * edge, solid 1 px lines; each square split in four by dashed lines. The grid is printed in
   * sheet coordinates and mapped through every facet, so it follows all folds and layers.
   */
  private rebuildGrid() {
    for (const g of this.grid) {
      this.group.remove(g);
      g.geometry.dispose();
    }
    this.grid = [];
    const m = this.model;
    if (!m || !this.gridOn) return;
    const levels = levelsOf(m);
    const a = m.spec.heightM / 2;
    const b = m.spec.widthM / 2;
    const half = m.spec.widthM / GRID_SQUARES_ACROSS / 2;
    const lines: { p: Vec2; q: Vec2; dashed: boolean }[] = [];
    for (let k = 0; -b + k * half <= b + 1e-9; k++) {
      const v = -b + k * half;
      lines.push({ p: [-a - 0.01, v], q: [a + 0.01, v], dashed: k % 2 === 1 });
    }
    for (let k = 0; a - k * half >= -a - 1e-9; k++) {
      const u = a - k * half;
      lines.push({ p: [u, -b - 0.01], q: [u, b + 0.01], dashed: k % 2 === 1 });
    }
    const solid: number[] = [];
    const dashed: number[] = [];
    for (const f of m.facets) {
      const off = this.offsetVec(f, levels.get(f.id) ?? 0);
      const n = facetNormal(f);
      for (const l of lines) {
        const r = P.clipSegmentConvex(l.p, l.q, f.poly);
        if (!r || r[1] - r[0] < 1e-9) continue;
        const A: Vec2 = [l.p[0] + (l.q[0] - l.p[0]) * r[0], l.p[1] + (l.q[1] - l.p[1]) * r[0]];
        const B: Vec2 = [l.p[0] + (l.q[0] - l.p[0]) * r[1], l.p[1] + (l.q[1] - l.p[1]) * r[1]];
        const arr = l.dashed ? dashed : solid;
        for (const sgn of [1, -1]) {
          const o = V.addScaled(off, n, sgn * GRID_LIFT);
          arr.push(...V.add(toModel(f, A), o), ...V.add(toModel(f, B), o));
        }
      }
    }
    const make = (arr: number[], mat: THREE.LineBasicMaterial | THREE.LineDashedMaterial) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(arr, 3));
      const ls = new THREE.LineSegments(g, mat);
      if (mat instanceof THREE.LineDashedMaterial) ls.computeLineDistances();
      this.group.add(ls);
      this.grid.push(ls);
    };
    make(solid, this.gridMat);
    make(dashed, this.gridDashMat);
  }

  dispose() {
    for (const o of [this.mesh, this.lines, this.creases]) {
      if (!o) continue;
      this.group.remove(o);
      o.geometry.dispose();
    }
    this.mesh = this.lines = this.creases = null;
    for (const g of this.grid) {
      this.group.remove(g);
      g.geometry.dispose();
    }
    this.grid = [];
  }
}
