import * as V from '../math/vec3';
import * as P from '../math/polygon2d';
import type { Vec3 } from '../math/vec3';
import type { Mat3 } from '../math/mat3';
import { invert, zeros } from '../math/linalg';
import { facetVertices } from '../paper/model';
import type { PaperModel } from '../paper/model';
import type { DesignFile } from '../paper/ops';
import { replayDesign } from '../paper/replay';
import { computeMassProperties } from './massProperties';
import type { MassProperties } from './massProperties';
import { buildAeroMesh, markKeelJunctions } from './aeroMesh';
import type { AeroMesh } from './aeroMesh';
import { computeAddedMass } from './addedMass';
import { buildControlGroups, controlFacetTags } from './controlGroups';
import type { ControlGroup } from './controlGroups';
import type { Op } from '../paper/ops';
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
  /** For each contact point: the largest facet it belongs to (for damage). */
  contactFacets: number[];
  /** Body axes: nose, up, right = nose × up. */
  nose: Vec3;
  up: Vec3;
  right: Vec3;
  /** Planform reference area (projected on the plane ⟂ up) and mean chord / span. */
  refArea: number;
  refSpan: number;
  refChord: number;
  /** Movable control flaps (stage 5); empty when no fold is marked as control. */
  controls: ControlGroup[];
}

export function buildAirframe(
  model: PaperModel,
  orientation: { nose: Vec3; up: Vec3 },
  cfg: PhysicsConfig,
  /** Design ops (to find folds marked as control flaps). */
  ops: Op[] = [],
): Airframe {
  const mp = computeMassProperties(model);
  const com = mp.com;
  const nose = V.normalize(orientation.nose);
  const up = V.normalize(V.rejectFrom(orientation.up, nose));
  const right = V.cross(nose, up);
  const { tags, ids } = controlFacetTags(model, ops);
  const controls = buildControlGroups(model, com, tags, ids, { nose, up, right });
  // re-tag so that only facets of groups that got a hinge stay separate
  const valid = new Map<number, number>();
  controls.forEach((g, i) => g.facets.forEach((f) => valid.set(f, i)));
  const mesh = buildAeroMesh(model, com, cfg.aero.elementSizeM, cfg.aero.liftingSystemAngleDeg, valid);
  for (const s of mesh.surfaces) if (s.tag >= 0) controls[s.tag].surfaces.push(s.id);

  const MRB = zeros(6);
  for (let i = 0; i < 3; i++) MRB[i * 6 + i] = mp.mass;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) MRB[(3 + i) * 6 + 3 + j] = mp.inertia[i * 3 + j];
  const MA = cfg.addedMass
    ? computeAddedMass(mesh, cfg.rho, cfg.addedMassScale, cfg.aero.elementSizeM).MA
    : zeros(6);
  const M = MRB.map((v, k) => v + MA[k]);

  const contacts: Vec3[] = [];
  const contactFacets: number[] = [];
  const contactArea: number[] = [];
  for (const f of model.facets) {
    const area = P.area(f.poly);
    for (const v of facetVertices(f)) {
      const p = V.sub(v, com);
      const k = contacts.findIndex((c) => V.dist(c, p) < 1e-6);
      if (k < 0) {
        contacts.push(p);
        contactFacets.push(f.id);
        contactArea.push(area);
      } else if (area > contactArea[k]) {
        contactFacets[k] = f.id;
        contactArea[k] = area;
      }
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
  markKeelJunctions(mesh, model, com, up, right, refSpan / 2, cfg.aero.keelDihedral, valid);
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
    contactFacets,
    nose,
    up,
    right,
    refArea,
    refSpan,
    refChord: refArea / refSpan,
    controls,
  };
}

export function airframeFromDesign(design: DesignFile, cfg: PhysicsConfig): Airframe {
  const model = replayDesign(design).models.at(-1)!;
  return buildAirframe(model, design.orientation, cfg, design.ops);
}
