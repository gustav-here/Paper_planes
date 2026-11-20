import * as THREE from 'three';

/**
 * Paper burning up: the sheet turns into pixel-sized black flakes that drift apart for a moment,
 * then rise on the hot gas and are gone (each flake within half a second). Purely visual.
 */
export class AshEffect {
  readonly points: THREE.Points;
  private readonly pos: Float32Array;
  private readonly vel: Float32Array;
  private readonly life: Float32Array;
  private age = 0;
  /** Total duration (s). */
  static readonly DURATION = 0.5;

  constructor(samples: THREE.Vector3[], seed = 1) {
    const n = samples.length;
    this.pos = new Float32Array(n * 3);
    this.vel = new Float32Array(n * 3);
    this.life = new Float32Array(n);
    let s = seed >>> 0;
    const rnd = () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
    samples.forEach((p, i) => {
      this.pos.set([p.x, p.y, p.z], i * 3);
      // flakes break apart a little in every direction, sinking slightly at first
      this.vel.set([(rnd() - 0.5) * 0.6, -0.1 - rnd() * 0.3, (rnd() - 0.5) * 0.6], i * 3);
      this.life[i] = 0.25 + rnd() * 0.25;
    });
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    this.points = new THREE.Points(
      geo,
      new THREE.PointsMaterial({ color: '#0d0905', size: 2.5, sizeAttenuation: false, transparent: true, opacity: 1, depthWrite: false }),
    );
    this.points.frustumCulled = false;
    this.points.renderOrder = 30;
  }

  /** Advance by dt seconds; false once every flake is gone. */
  update(dt: number): boolean {
    this.age += dt;
    const lift = this.age > 0.12 ? 14 : 0; // the hot gas carries the flakes upwards
    let alive = 0;
    for (let i = 0; i < this.life.length; i++) {
      if (this.age > this.life[i]) {
        this.pos[i * 3 + 1] = 1e6; // gone
        continue;
      }
      alive++;
      this.vel[i * 3 + 1] += lift * dt;
      for (let k = 0; k < 3; k++) this.pos[i * 3 + k] += this.vel[i * 3 + k] * dt;
    }
    (this.points.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    (this.points.material as THREE.PointsMaterial).opacity = Math.max(0, 1 - this.age / AshEffect.DURATION) * 0.6 + 0.4;
    return alive > 0 && this.age < AshEffect.DURATION;
  }

  dispose() {
    this.points.geometry.dispose();
    (this.points.material as THREE.Material).dispose();
  }
}

/** Points spread evenly over the triangles of the meshes under `root` (world space). */
export function samplePaper(root: THREE.Object3D, count: number): THREE.Vector3[] {
  const tris: [THREE.Vector3, THREE.Vector3, THREE.Vector3, number][] = [];
  root.updateMatrixWorld(true);
  root.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh || !mesh.visible) return;
    const pos = mesh.geometry.getAttribute('position') as THREE.BufferAttribute | undefined;
    if (!pos || mesh.geometry.index) return;
    // paper is drawn as front/back triangle pairs: every other triangle is enough
    for (let t = 0; t + 2 < pos.count; t += 6) {
      const a = new THREE.Vector3().fromBufferAttribute(pos, t).applyMatrix4(mesh.matrixWorld);
      const b = new THREE.Vector3().fromBufferAttribute(pos, t + 1).applyMatrix4(mesh.matrixWorld);
      const c = new THREE.Vector3().fromBufferAttribute(pos, t + 2).applyMatrix4(mesh.matrixWorld);
      const area = new THREE.Vector3().subVectors(b, a).cross(new THREE.Vector3().subVectors(c, a)).length() / 2;
      if (area > 1e-9) tris.push([a, b, c, area]);
    }
  });
  const total = tris.reduce((s, t) => s + t[3], 0);
  const out: THREE.Vector3[] = [];
  let s = 7;
  const rnd = () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  for (const [a, b, c, area] of tris) {
    const n = Math.round((count * area) / Math.max(total, 1e-12));
    for (let i = 0; i < n; i++) {
      let u = rnd();
      let v = rnd();
      if (u + v > 1) {
        u = 1 - u;
        v = 1 - v;
      }
      out.push(a.clone().addScaledVector(new THREE.Vector3().subVectors(b, a), u).addScaledVector(new THREE.Vector3().subVectors(c, a), v));
    }
  }
  return out;
}
