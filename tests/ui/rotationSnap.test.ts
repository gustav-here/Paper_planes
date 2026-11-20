import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { RIGHT_ANGLE_ROTATIONS, angleBetween, snapToRightAngles } from '../../src/ui/editor/rotationSnap';

const deg = THREE.MathUtils.degToRad;
const rot = (axis: [number, number, number], d: number) =>
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...axis).normalize(), deg(d));

describe('360-ball snapping', () => {
  it('there are 24 distinct right-angle orientations', () => {
    expect(RIGHT_ANGLE_ROTATIONS).toHaveLength(24);
    for (let i = 0; i < 24; i++)
      for (let j = i + 1; j < 24; j++) expect(angleBetween(RIGHT_ANGLE_ROTATIONS[i], RIGHT_ANGLE_ROTATIONS[j])).toBeGreaterThan(89);
  });

  it('snaps within 3° to flat / 90° steps, not beyond', () => {
    const base = new THREE.Quaternion();
    const near = rot([1, 2, 3], 2.5);
    const s = snapToRightAngles(near, base, 3);
    expect(s.snapped).toBe(true);
    expect(angleBetween(s.q, base)).toBeLessThan(1e-6);

    const turned = rot([0, 1, 0], 90).multiply(rot([1, 0, 0], 2));
    const t = snapToRightAngles(turned, base, 3);
    expect(t.snapped).toBe(true);
    expect(angleBetween(t.q, rot([0, 1, 0], 90))).toBeLessThan(1e-6);

    const far = rot([1, 0, 0], 4);
    expect(snapToRightAngles(far, base, 3).snapped).toBe(false);
  });

  it('snaps relative to the upright base orientation', () => {
    const base = rot([1, 0, 0], -90); // e.g. a plane whose "up" is model +Z
    const q = rot([0, 0, 1], 180).multiply(base).premultiply(rot([0, 1, 0], 1.5));
    const s = snapToRightAngles(q, base, 3);
    expect(s.snapped).toBe(true);
    expect(angleBetween(s.q, rot([0, 0, 1], 180).multiply(base))).toBeLessThan(1e-6);
  });
});
