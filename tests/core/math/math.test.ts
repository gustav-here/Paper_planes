import { describe, expect, it } from 'vitest';
import * as V from '../../../src/core/math/vec3';
import * as Q from '../../../src/core/math/quat';
import * as M from '../../../src/core/math/mat3';
import * as P from '../../../src/core/math/polygon2d';
import * as I from '../../../src/core/math/interval';
import {
  applyRigid,
  composeRigid,
  invertRigid,
  rotationAbout,
  rigidToAffine,
  applyAffine,
} from '../../../src/core/math/transform';
import type { Vec2 } from '../../../src/core/math/vec2';

const close3 = (a: V.Vec3, b: V.Vec3, eps = 1e-12) => {
  for (let i = 0; i < 3; i++) expect(a[i]).toBeCloseTo(b[i], 10 + 0 * eps);
};

describe('quaternions', () => {
  it('rotates x to y by 90° about z', () => {
    const q = Q.fromAxisAngle([0, 0, 1], Math.PI / 2);
    close3(Q.rotate(q, [1, 0, 0]), [0, 1, 0]);
  });

  it('matrix form agrees with rotate()', () => {
    const q = Q.normalize([0.3, -0.2, 0.5, 0.7]);
    const v: V.Vec3 = [0.3, -1.2, 2.5];
    close3(M.mulVec(Q.toMat3(q), v), Q.rotate(q, v));
    expect(M.det(Q.toMat3(q))).toBeCloseTo(1, 12);
  });

  it('composition applies right operand first', () => {
    const a = Q.fromAxisAngle([0, 0, 1], Math.PI / 2);
    const b = Q.fromAxisAngle([1, 0, 0], Math.PI / 2);
    const v: V.Vec3 = [0, 1, 0];
    close3(Q.rotate(Q.mul(a, b), v), Q.rotate(a, Q.rotate(b, v)));
  });

  it('slerp hits endpoints', () => {
    const a = Q.identity();
    const b = Q.fromAxisAngle([0, 1, 0], 1);
    expect(Q.sameRotation(Q.slerp(a, b, 0), a)).toBe(true);
    expect(Q.sameRotation(Q.slerp(a, b, 1), b)).toBe(true);
  });
});

describe('rigid transforms', () => {
  it('rotationAbout keeps axis points fixed', () => {
    const T = rotationAbout([1, 2, 3], V.normalize([1, 1, 0]), 0.7);
    close3(applyRigid(T, [1, 2, 3]), [1, 2, 3]);
    close3(applyRigid(T, [2, 3, 3]), [2, 3, 3]);
  });

  it('inverse and compose', () => {
    const A = rotationAbout([0.1, 0, 0], [0, 0, 1], 1.1);
    const B = rotationAbout([0, 0.3, 0], [1, 0, 0], -0.4);
    const p: V.Vec3 = [0.5, -0.2, 0.9];
    close3(applyRigid(composeRigid(A, B), p), applyRigid(A, applyRigid(B, p)));
    close3(applyRigid(invertRigid(A), applyRigid(A, p)), p);
    close3(applyAffine(rigidToAffine(A), p), applyRigid(A, p));
  });
});

describe('polygon2d', () => {
  const square: Vec2[] = [
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1],
  ];

  it('area and centroid', () => {
    expect(P.area(square)).toBeCloseTo(1);
    expect(P.centroid(square)).toEqual([0.5, 0.5]);
  });

  it('splitting conserves area', () => {
    const vals = P.lineValues(square, [0, 0.3], [1, 0.8]);
    const { neg, pos } = P.splitConvexByValues(square, vals, 1e-12);
    expect(neg).not.toBeNull();
    expect(pos).not.toBeNull();
    expect(P.area(neg!) + P.area(pos!)).toBeCloseTo(1, 12);
  });

  it('split through a vertex', () => {
    const vals = P.lineValues(square, [0, 0], [1, 1]);
    const { neg, pos } = P.splitConvexByValues(square, vals, 1e-12);
    expect(neg!.length).toBe(3);
    expect(pos!.length).toBe(3);
  });

  it('polygon entirely on one side is not split', () => {
    const vals = P.lineValues(square, [2, 0], [2, 1]);
    const r = P.splitConvexByValues(square, vals, 1e-12);
    expect(r.neg === null || r.pos === null).toBe(true);
  });

  it('clips segments', () => {
    expect(P.clipSegmentConvex([-1, 0.5], [2, 0.5], square)).toEqual([1 / 3, 2 / 3]);
    expect(P.clipSegmentConvex([-1, 2], [2, 2], square)).toBeNull();
  });

  it('overlap test ignores touching', () => {
    const right = square.map(([x, y]) => [x + 1, y] as Vec2);
    const shifted = square.map(([x, y]) => [x + 0.5, y] as Vec2);
    expect(P.convexOverlap(square, right)).toBe(false);
    expect(P.convexOverlap(square, shifted)).toBe(true);
  });
});

describe('intervals', () => {
  it('union and subtract', () => {
    expect(I.union([[0, 1], [0.5, 2], [3, 4]])).toEqual([[0, 2], [3, 4]]);
    expect(I.subtract([0, 10], [[2, 3], [5, 6]])).toEqual([[0, 2], [3, 5], [6, 10]]);
    expect(I.intersect([0, 10], [[-1, 1], [9, 12]])).toEqual([[0, 1], [9, 10]]);
  });
});
