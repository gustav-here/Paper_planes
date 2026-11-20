import * as V from '../math/vec3';
import type { Vec3 } from '../math/vec3';
import type { Mat3 } from '../math/mat3';
import { facetArea, facetVertices } from '../paper/model';
import type { PaperModel } from '../paper/model';
import { arealDensity } from '../paper/paperSpec';
import { resolveClip } from '../paper/clips';

export interface PointMass {
  m: number;
  p: Vec3;
  label: string;
}

export interface MassProperties {
  /** Total mass (kg). */
  mass: number;
  paperMass: number;
  clipMass: number;
  /** Centre of mass, model space. */
  com: Vec3;
  /** Inertia tensor about the centre of mass, model axes (kg·m²). Row-major 3×3. */
  inertia: Mat3;
}

/** Second moment ∫ x xᵀ dm of a uniform thin triangle about the origin: m/12·(Σ vᵢvᵢᵀ + s sᵀ), s = Σ vᵢ. */
function triangleCovariance(a: Vec3, b: Vec3, c: Vec3, m: number, out: number[]) {
  const s = V.add(V.add(a, b), c);
  const k = m / 12;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      out[i * 3 + j] += k * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + s[i] * s[j]);
    }
  }
}

function pointCovariance(p: Vec3, m: number, out: number[]) {
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) out[i * 3 + j] += m * p[i] * p[j];
}

/**
 * Mass, centre of mass and inertia from the folded geometry: every facet (every layer) is a
 * uniform thin plate of areal density gsm/1000, clips are point masses at their centre of mass.
 * Folding conserves area, so paper mass never changes; cutting pieces off removes their mass.
 */
export function computeMassProperties(model: PaperModel): MassProperties {
  const rhoA = arealDensity(model.spec);
  const C = new Array<number>(9).fill(0); // ∫ x xᵀ dm about model origin
  let mass = 0;
  let first: Vec3 = [0, 0, 0];
  let paperMass = 0;
  for (const f of model.facets) {
    const vs = facetVertices(f);
    for (let i = 1; i + 1 < vs.length; i++) {
      const a = vs[0];
      const b = vs[i];
      const c = vs[i + 1];
      const area = V.length(V.cross(V.sub(b, a), V.sub(c, a))) / 2;
      const m = rhoA * area;
      triangleCovariance(a, b, c, m, C);
      first = V.addScaled(first, V.add(V.add(a, b), c), m / 3);
      mass += m;
    }
    paperMass += rhoA * facetArea(f);
  }
  let clipMass = 0;
  for (const clip of model.clips) {
    const rc = resolveClip(model, clip);
    if (!rc) continue;
    pointCovariance(rc.centerOfMass, clip.massKg, C);
    first = V.addScaled(first, rc.centerOfMass, clip.massKg);
    mass += clip.massKg;
    clipMass += clip.massKg;
  }
  const com = V.scale(first, 1 / mass);
  // shift covariance to the centre of mass, then I = tr(C)·𝟙 − C
  const Cc = C.map((v, k) => v - mass * com[Math.floor(k / 3)] * com[k % 3]);
  const tr = Cc[0] + Cc[4] + Cc[8];
  const inertia = Cc.map((v, k) => (k % 4 === 0 ? tr - v : -v));
  return { mass, paperMass, clipMass, com, inertia };
}
