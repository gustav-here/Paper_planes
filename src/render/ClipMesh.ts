import * as THREE from 'three';
import * as V from '../core/math/vec3';
import { CLIP_SPEC, TAPE_SPEC } from '../core/paper/clips';
import type { ResolvedClip } from '../core/paper/clips';
import type { PaperModel } from '../core/paper/model';
import type { Vec2 } from '../core/math/vec2';
import type { Vec3 } from '../core/math/vec3';
import { levelsOf } from './modelCache';
import { VIS_LAYER } from './PaperMesh';

const WIRE_R = 0.00045;
const metal = new THREE.MeshStandardMaterial({ color: '#c9ced6', metalness: 0.9, roughness: 0.28 });
const ghost = new THREE.MeshStandardMaterial({
  color: '#ffd34d',
  metalness: 0.3,
  roughness: 0.5,
  transparent: true,
  opacity: 0.75,
  depthTest: false,
});

/** Rounded "stadium" loop in the clip's local XZ plane: x from x0 to x1, half-width w. */
function loop(x0: number, x1: number, w: number, y: number, segs = 10): THREE.Vector3[] {
  const pts: THREE.Vector3[] = [];
  const r = w;
  for (let i = 0; i <= segs; i++) {
    const a = -Math.PI / 2 + (Math.PI * i) / segs;
    pts.push(new THREE.Vector3(x1 - r + Math.cos(a) * r, y, Math.sin(a) * r));
  }
  for (let i = 0; i <= segs; i++) {
    const a = Math.PI / 2 + (Math.PI * i) / segs;
    pts.push(new THREE.Vector3(x0 + r + Math.cos(a) * r, y, Math.sin(a) * r));
  }
  return pts;
}

/**
 * Paper clip mesh. Local frame: X = inward from the edge, Y = stack normal, Z = along edge.
 * The outer loop lies on top of the stack, the inner loop underneath, joined around the edge.
 */
export function createClipObject(model: PaperModel, rc: ResolvedClip, isGhost = false): THREE.Group {
  if (rc.clip.kind === 'tape') return createTapeObject(model, rc, isGhost);
  const levels = levelsOf(model);
  let lo = Infinity;
  let hi = -Infinity;
  for (const id of rc.clamped) {
    const lv = levels.get(id) ?? 0;
    lo = Math.min(lo, lv);
    hi = Math.max(hi, lv);
  }
  if (!Number.isFinite(lo)) lo = hi = 0;
  const yTop = (hi + 0.5) * VIS_LAYER + WIRE_R * 1.2;
  const yBot = (lo - 0.5) * VIS_LAYER - WIRE_R * 1.2;
  const L = CLIP_SPEC.lengthM;
  const W = CLIP_SPEC.widthM / 2;
  const bend = -0.0015;

  const top = loop(bend, L, W, yTop);
  const bottom = loop(bend + 0.001, L * 0.68, W * 0.62, yBot);
  const path = new THREE.CatmullRomCurve3([...top, ...bottom.reverse()], false, 'catmullrom', 0.1);
  const mat = isGhost ? ghost : metal;
  const mesh = new THREE.Mesh(new THREE.TubeGeometry(path, 160, WIRE_R, 6, false), mat);
  mesh.castShadow = !isGhost;

  const g = new THREE.Group();
  g.add(mesh);
  if (!isGhost) {
    // invisible, generous pick volume: the wire itself is far too thin to hit with the mouse
    const pick = new THREE.Mesh(
      new THREE.BoxGeometry(L + 0.004, yTop - yBot + 0.004, 2 * W + 0.004),
      new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: false }),
    );
    pick.position.set(L / 2 + bend / 2, (yTop + yBot) / 2, 0);
    g.add(pick);
  }
  const x = new THREE.Vector3(...rc.inward);
  const y = new THREE.Vector3(...rc.normal);
  const z = new THREE.Vector3().crossVectors(x, y).normalize();
  const basis = new THREE.Matrix4().makeBasis(x, y, z);
  g.quaternion.setFromRotationMatrix(basis);
  const p = V.clone(rc.position);
  g.position.set(p[0], p[1], p[2]);
  g.userData.clipId = rc.clip.id;
  if (isGhost) g.renderOrder = 10;
  return g;
}

const tapeMat = new THREE.MeshStandardMaterial({
  color: '#9fd0ff',
  roughness: 0.2,
  metalness: 0,
  transparent: true,
  opacity: 0.55,
  side: THREE.DoubleSide,
  depthWrite: false,
});

const tapeEdge = new THREE.LineBasicMaterial({ color: '#5aa8e8', transparent: true, opacity: 0.9 });

/** A flat polygon (holder frame: x, z) at height y, as a mesh with its outline. */
function tapePiece(poly: [number, number][], y: number, mat: THREE.Material): THREE.Object3D {
  const contour = poly.map(([x, z]) => new THREE.Vector2(x, z));
  const tris = THREE.ShapeUtils.triangulateShape(contour, []);
  const pos: number[] = [];
  for (const [x, z] of poly) pos.push(x, y, z);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setIndex(tris.flat());
  geo.computeVertexNormals();
  const m = new THREE.Mesh(geo, mat);
  // the cut ends of the tape catch the light: draw its outline
  const line: number[] = [];
  poly.forEach(([x, z], i) => {
    const [x2, z2] = poly[(i + 1) % poly.length];
    line.push(x, y, z, x2, y, z2);
  });
  const lg = new THREE.BufferGeometry();
  lg.setAttribute('position', new THREE.Float32BufferAttribute(line, 3));
  m.add(new THREE.LineSegments(lg, tapeEdge));
  return m;
}

/**
 * Clear tape, 2 cm long. Local frame as for the clip: X = inward from the edge, Y = stack normal,
 * Z = along the edge.
 *  - Over an edge: one half on top, the other folded underneath, a short wrap at the edge; the
 *    preview hangs half in the air (the free half folds over when it is pressed on).
 *  - Across a seam: flat over both pieces, on the side away from the paper below the seam.
 * Placed tape is trimmed to the paper (what sticks out is cut off).
 */
function createTapeObject(model: PaperModel, rc: ResolvedClip, isGhost: boolean): THREE.Group {
  const levels = levelsOf(model);
  /** Heights just above and below the layers of these facets. */
  const heights = (ids: number[]): [number, number] => {
    let lo = Infinity;
    let hi = -Infinity;
    for (const id of ids) {
      const lv = levels.get(id) ?? 0;
      lo = Math.min(lo, lv);
      hi = Math.max(hi, lv);
    }
    if (!Number.isFinite(lo)) lo = hi = 0;
    const gap = 0.00015;
    return [(lo - 0.5) * VIS_LAYER - gap, (hi + 0.5) * VIS_LAYER + gap];
  };
  const L = TAPE_SPEC.lengthM;
  const W = TAPE_SPEC.widthM / 2;
  const mat = isGhost ? ghost : tapeMat;
  const g = new THREE.Group();
  const rect = (x0: number): [number, number][] => [
    [x0, -W],
    [L, -W],
    [L, W],
    [x0, W],
  ];
  const toPolys = (shape: Vec2[][] | null) => (shape ?? [rect(0)]).map((r) => r.map(([x, z]) => [x, z] as [number, number]));
  /** A group in a holder frame (X = inward, Y = normal, Z = X × Y) at the clip's point. */
  const frame = (inward: Vec3, normal: Vec3) => {
    const h = new THREE.Group();
    const x = new THREE.Vector3(...inward);
    const y = new THREE.Vector3(...normal);
    const z = new THREE.Vector3().crossVectors(x, y).normalize();
    h.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
    g.add(h);
    return h;
  };
  const pickBox = (h: THREE.Group, polys: [number, number][][], y0: number, y1: number) => {
    if (isGhost) return;
    const xs = polys.flat().map(([x]) => x);
    if (!xs.length) return;
    const x0 = Math.min(...xs);
    const x1 = Math.max(...xs);
    const pick = new THREE.Mesh(
      new THREE.BoxGeometry(x1 - x0 + 0.004, Math.abs(y1 - y0) + 0.004, 2 * W + 0.002),
      new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: false }),
    );
    pick.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0);
    h.add(pick);
  };
  const shape = toPolys(rc.tapeShape);
  if (rc.bridge) {
    // one half on each piece, on the side away from the paper below the seam
    const otherIds = new Set(rc.across?.clamped ?? []);
    const halves = [{ inward: rc.inward, normal: rc.normal, side: rc.side, ids: rc.clamped.filter((id) => !otherIds.has(id)), polys: shape }];
    if (rc.across) halves.push({ ...rc.across, ids: rc.across.clamped, polys: toPolys(rc.across.tapeShape) });
    for (const half of halves) {
      const h = frame(half.inward, half.normal);
      const [yb, yt] = heights(half.ids);
      const y = half.side > 0 ? yt : yb;
      const polys = isGhost ? [rect(0)] : half.polys;
      for (const poly of polys) h.add(tapePiece(poly, y, mat));
      pickBox(h, polys, y, y);
    }
  } else {
    const h = frame(rc.inward, rc.normal);
    const [yBot, yTop] = heights(rc.clamped);
    if (isGhost) {
      // half over the paper, half in the air
      h.add(tapePiece(rect(-L), yTop, mat));
    } else {
      for (const poly of shape) h.add(tapePiece(poly, yTop, mat), tapePiece(poly, yBot, mat));
      // the fold round the edge, as wide as the tape that is left there
      const atEdge = shape.flat().filter(([x]) => Math.abs(x) < 1e-5).map(([, z]) => z);
      if (atEdge.length >= 2) {
        const z0 = Math.min(...atEdge);
        const z1 = Math.max(...atEdge);
        const wrap = new THREE.Mesh(new THREE.PlaneGeometry(yTop - yBot, z1 - z0), mat);
        wrap.rotation.set(-Math.PI / 2, -Math.PI / 2, 0);
        wrap.position.set(0, (yTop + yBot) / 2, (z0 + z1) / 2);
        h.add(wrap);
      }
      pickBox(h, shape, yBot, yTop);
    }
  }
  g.position.set(...V.clone(rc.position));
  g.userData.clipId = rc.clip.id;
  if (isGhost) g.renderOrder = 10;
  return g;
}
