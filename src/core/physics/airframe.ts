import * as V from '../math/vec3';
import type { Vec3 } from '../math/vec3';
import type { Mat3 } from '../math/mat3';
import { invert, zeros } from '../math/linalg';
import { facetVertices } from '../paper/model';
import type { PaperModel } from '../paper/model';
import type { DesignFile } from '../paper/ops';
import { replayDesign } from '../paper/replay';
import { computeMassProperties } from './massProperties';
import type { MassProperties } from './massProperties';
import { buildAeroMesh } from './aeroMesh';
import type { AeroMesh } from './aeroMesh';
import { computeAddedMass } from './addedMass';
import type { PhysicsConfig } from './config';

/**
 * Everything the flight simulation needs, derived purely from the folded paper.
 * Body frame = model axes with the origin at the centre of mass.
 */
export interface Airframe {
  massProps: MassProperties;
  mass: number;
  /** Inertia about the CoM (body frame). */
  inertia: Mat3;
  /** Centre of mass in model space (body origin). */
  com: Vec3;
  mesh: AeroMesh;
  /** Generalized mass M = M_RB + M_A (6×6) and its inverse. */
  M: number[];
  Minv: number[];
  /** Added-mass block only (zero if disabled). */
  MA: number[];
  /** Contact points (unique facet vertices), body frame. */
  contacts: Vec3[];
  /** Body axes: nose, up, right = nose × up. */
  nose: Vec3;
  up: Vec3;
  right: Vec3;
  /** Planform reference area (projected on the plane ⟂ up) and mean chord / span. */
  refArea: number;
  refSpan: number;
  refChord: number;
}

export function buildAirframe(
  model: PaperModel,
  orientation: { nose: Vec3; up: Vec3 },
  cfg: PhysicsConfig,
): Airframe {
  const mp = computeMassProperties(model);
  const com = mp.com;
  const mesh = buildAeroMesh(model, com, cfg.aero.elementSizeM, cfg.aero.liftingSystemAngleDeg);
  const nose = V.normalize(orientation.nose);
  const up = V.normalize(V.rejectFrom(orientation.up, nose));
  const right = V.cross(nose, up);

  const MRB = zeros(6);
  for (let i = 0; i < 3; i++) MRB[i * 6 + i] = mp.mass;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) MRB[(3 + i) * 6 + 3 + j] = mp.inertia[i * 3 + j];
  const MA = cfg.addedMass
    ? computeAddedMass(mesh, cfg.rho, cfg.addedMassScale, cfg.aero.elementSizeM).MA
    : zeros(6);
  const M = MRB.map((v, k) => v + MA[k]);

  const contacts: Vec3[] = [];
  for (const f of model.facets) {
    for (const v of facetVertices(f)) {
      const p = V.sub(v, com);
      if (!contacts.some((c) => V.dist(c, p) < 1e-6)) contacts.push(p);
    }
  }

  let refArea = 0;
  let lo = Infinity;
  let hi = -Infinity;
  for (const e of mesh.elements) {
    const s = mesh.surfaces[e.surface];
    refArea += e.area * Math.abs(V.dot(s.normal, up));
    const y = V.dot(e.center, right);
    lo = Math.min(lo, y);
    hi = Math.max(hi, y);
  }
  const refSpan = Math.max(hi - lo, 1e-3);
  return {
    massProps: mp,
    mass: mp.mass,
    inertia: mp.inertia,
    com,
    mesh,
    M,
    Minv: invert(M, 6),
    MA,
    contacts,
    nose,
    up,
    right,
    refArea,
    refSpan,
    refChord: refArea / refSpan,
  };
}

export function airframeFromDesign(design: DesignFile, cfg: PhysicsConfig): Airframe {
  const model = replayDesign(design).models.at(-1)!;
  return buildAirframe(model, design.orientation, cfg);
}
