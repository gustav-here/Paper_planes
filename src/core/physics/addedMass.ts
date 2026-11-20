import * as V from '../math/vec3';
import type { Vec3 } from '../math/vec3';
import { zeros } from '../math/linalg';
import type { AeroMesh } from './aeroMesh';

export interface AddedMassInfo {
  /** 6×6 added-mass matrix in the body frame, generalized coordinates (v, ω). */
  MA: number[];
  /** Added mass of each surface for normal acceleration (kg), for display/debug. */
  perSurface: number[];
}

/**
 * Added mass (the air that must be accelerated with the paper) from strip theory.
 *
 * For a flat plate accelerating normal to itself the 2D added mass per unit span is ρπc²/4,
 * and for pitching about mid-chord ρπc⁴/128. Each surface is cut into strips along its major
 * (span) axis; strip i with chord cᵢ and width Δb at mid-chord position rᵢ contributes
 *   mᵢ = κ ρ π cᵢ² Δb / 4   acting along the plate normal n at rᵢ, and
 *   Jᵢ = κ ρ π cᵢ⁴ Δb / 128 about the span axis,
 * with the low-aspect-ratio correction κ(AR) = AR/√(1+AR²).
 * Only the normal direction gets added mass (a thin plate slides edgewise freely).
 */
export function computeAddedMass(mesh: AeroMesh, rho: number, scale: number, stripWidth: number): AddedMassInfo {
  const MA = zeros(6);
  const perSurface: number[] = [];
  for (const s of mesh.surfaces) {
    const els = mesh.elements.slice(s.firstElement, s.firstElement + s.elementCount);
    // principal axes of the planform (area second moments in plane coordinates)
    let A = 0;
    let mx = 0;
    let my = 0;
    for (const e of els) {
      A += e.area;
      mx += e.area * e.c2[0];
      my += e.area * e.c2[1];
    }
    mx /= A;
    my /= A;
    let sxx = 0;
    let syy = 0;
    let sxy = 0;
    for (const e of els) {
      const dx = e.c2[0] - mx;
      const dy = e.c2[1] - my;
      sxx += e.area * dx * dx;
      syy += e.area * dy * dy;
      sxy += e.area * dx * dy;
    }
    const th = 0.5 * Math.atan2(2 * sxy, sxx - syy);
    const span2: [number, number] = [Math.cos(th), Math.sin(th)];
    const spanAxis = V.normalize(V.add(V.scale(s.e1, span2[0]), V.scale(s.e2, span2[1])));
    // strips along the span axis
    const strips = new Map<number, { area: number; c: Vec3 }>();
    let lo = Infinity;
    let hi = -Infinity;
    for (const e of els) {
      const y = e.c2[0] * span2[0] + e.c2[1] * span2[1];
      lo = Math.min(lo, y);
      hi = Math.max(hi, y);
      const k = Math.floor(y / stripWidth);
      const st = strips.get(k) ?? { area: 0, c: [0, 0, 0] as Vec3 };
      st.area += e.area;
      st.c = V.addScaled(st.c, e.center, e.area);
      strips.set(k, st);
    }
    const span = Math.max(hi - lo + stripWidth, stripWidth);
    const AR = (span * span) / A;
    const kappa = (AR / Math.sqrt(1 + AR * AR)) * scale;
    const n = s.normal;
    let total = 0;
    for (const st of strips.values()) {
      const chord = st.area / stripWidth;
      const r = V.scale(st.c, 1 / st.area);
      const m = (kappa * rho * Math.PI * chord * chord * stripWidth) / 4;
      const J = (kappa * rho * Math.PI * chord ** 4 * stripWidth) / 128;
      total += m;
      const g = [...n, ...V.cross(r, n)]; // generalized direction
      for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) MA[i * 6 + j] += m * g[i] * g[j];
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) MA[(3 + i) * 6 + 3 + j] += J * spanAxis[i] * spanAxis[j];
    }
    perSurface.push(total);
  }
  return { MA, perSurface };
}
