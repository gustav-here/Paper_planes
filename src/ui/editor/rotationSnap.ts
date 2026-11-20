import * as THREE from 'three';

/** The 24 rotations that map the coordinate axes onto (signed) coordinate axes: all 90° steps. */
export const RIGHT_ANGLE_ROTATIONS: THREE.Quaternion[] = (() => {
  const axes = [
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(-1, 0, 0),
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 0, -1),
  ];
  const out: THREE.Quaternion[] = [];
  for (const x of axes) {
    for (const y of axes) {
      if (Math.abs(x.dot(y)) > 0.5) continue;
      const z = new THREE.Vector3().crossVectors(x, y);
      out.push(new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z)));
    }
  }
  return out;
})();

/** Angle (deg) between two orientations. */
export function angleBetween(a: THREE.Quaternion, b: THREE.Quaternion): number {
  const d = Math.min(1, Math.abs(a.dot(b)));
  return THREE.MathUtils.radToDeg(2 * Math.acos(d));
}

/**
 * Snap a view orientation to the nearest 90°-step orientation of `base` (the upright view) if
 * it is within `toleranceDeg`: the paper then lies exactly flat / straight. Returns the input
 * unchanged otherwise.
 */
export function snapToRightAngles(
  q: THREE.Quaternion,
  base: THREE.Quaternion,
  toleranceDeg: number,
): { q: THREE.Quaternion; snapped: boolean } {
  let best: THREE.Quaternion | null = null;
  let bestAngle = Infinity;
  for (const r of RIGHT_ANGLE_ROTATIONS) {
    const cand = r.clone().multiply(base);
    const a = angleBetween(q, cand);
    if (a < bestAngle) {
      bestAngle = a;
      best = cand;
    }
  }
  if (best && bestAngle <= toleranceDeg) {
    // keep the hemisphere of q so slerps from here take the short way
    if (best.dot(q) < 0) best.set(-best.x, -best.y, -best.z, -best.w);
    return { q: best, snapped: true };
  }
  return { q: q.clone(), snapped: false };
}
