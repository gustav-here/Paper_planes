import * as THREE from 'three';
import * as V from '../core/math/vec3';
import { CLIP_SPEC, TAPE_SPEC } from '../core/paper/clips';
import type { ResolvedClip } from '../core/paper/clips';
import type { PaperModel } from '../core/paper/model';
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

/**
 * Clear tape folded over the stack edge. Local frame as for the clip: X = inward from the edge,
 * Y = stack normal, Z = along the edge; one strip on top, one underneath, a short wrap at the edge.
 */
function createTapeObject(model: PaperModel, rc: ResolvedClip, isGhost: boolean): THREE.Group {
  const levels = levelsOf(model);
  let lo = Infinity;
  let hi = -Infinity;
  for (const id of rc.clamped) {
    const lv = levels.get(id) ?? 0;
    lo = Math.min(lo, lv);
    hi = Math.max(hi, lv);
  }
  if (!Number.isFinite(lo)) lo = hi = 0;
  const gap = 0.00015;
  const yTop = (hi + 0.5) * VIS_LAYER + gap;
  const yBot = (lo - 0.5) * VIS_LAYER - gap;
  const L = TAPE_SPEC.lengthM;
  const W = TAPE_SPEC.widthM;
  const mat = isGhost ? ghost : tapeMat;
  const g = new THREE.Group();
  const strip = (y: number) => {
    const geo = new THREE.PlaneGeometry(L, W);
    const m = new THREE.Mesh(geo, mat);
    m.rotation.x = -Math.PI / 2;
    m.position.set(L / 2, y, 0);
    // the cut ends of the tape catch the light: draw its outline
    m.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), tapeEdge));
    return m;
  };
  g.add(strip(yTop), strip(yBot));
  const wrap = new THREE.Mesh(new THREE.PlaneGeometry(yTop - yBot, W), mat);
  wrap.rotation.set(-Math.PI / 2, -Math.PI / 2, 0);
  wrap.position.set(0, (yTop + yBot) / 2, 0);
  g.add(wrap);
  if (!isGhost) {
    const pick = new THREE.Mesh(
      new THREE.BoxGeometry(L + 0.004, yTop - yBot + 0.004, W + 0.002),
      new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: false }),
    );
    pick.position.set(L / 2, (yTop + yBot) / 2, 0);
    g.add(pick);
  }
  const x = new THREE.Vector3(...rc.inward);
  const y = new THREE.Vector3(...rc.normal);
  const z = new THREE.Vector3().crossVectors(x, y).normalize();
  g.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
  g.position.set(...V.clone(rc.position));
  g.userData.clipId = rc.clip.id;
  if (isGhost) g.renderOrder = 10;
  return g;
}
