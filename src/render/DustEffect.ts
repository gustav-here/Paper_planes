import * as THREE from 'three';

/**
 * A small puff of grey dust kicked up where the paper lands on the Moon. Without air the grains
 * do not billow: each flies its own clean parabola under the low gravity and lies still where it
 * comes down. The puff then fades. Purely visual.
 */
export class DustEffect {
  readonly points: THREE.Points;
  private readonly pos: Float32Array;
  private readonly vel: Float32Array;
  private age = 0;
  /** Total duration (s). */
  static readonly DURATION = 3.5;

  /**
   * `at`: landing point (world); `along`: the plane's horizontal direction of travel (unit);
   * `speed`: landing speed (m/s); `g`: gravity (m/s²).
   */
  constructor(
    at: THREE.Vector3,
    along: THREE.Vector3,
    speed: number,
    private readonly g: number,
    count = 450,
    seed = 7,
  ) {
    this.pos = new Float32Array(count * 3);
    this.vel = new Float32Array(count * 3);
    let s = seed >>> 0;
    const rnd = () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
    const side = new THREE.Vector3(-along.z, 0, along.x);
    // a harder landing throws the dust faster and higher (capped: it is only a little puff)
    const k = Math.min(1, 0.35 + speed / 8);
    for (let i = 0; i < count; i++) {
      const r = 0.03 * Math.sqrt(rnd());
      const a = rnd() * Math.PI * 2;
      this.pos.set([at.x + Math.cos(a) * r, Math.max(at.y, 0) + 0.002, at.z + Math.sin(a) * r], i * 3);
      // mostly thrown ahead, spread to the sides, steeply up
      const fwd = (0.1 + rnd() * 0.5) * k;
      const lat = (rnd() - 0.5) * 0.6 * k;
      const up = (0.15 + rnd() * rnd() * 0.75) * k;
      this.vel.set([along.x * fwd + side.x * lat, up, along.z * fwd + side.z * lat], i * 3);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    this.points = new THREE.Points(
      geo,
      new THREE.PointsMaterial({ color: '#a3a3a0', size: 0.004, sizeAttenuation: true, transparent: true, opacity: 0.9, depthWrite: false }),
    );
    this.points.frustumCulled = false;
    this.points.renderOrder = 30;
  }

  /** Advance by dt seconds; false once the dust has faded. */
  update(dt: number): boolean {
    this.age += dt;
    for (let i = 0; i < this.vel.length / 3; i++) {
      const y = i * 3 + 1;
      if (this.pos[y] <= 0.001 && this.vel[y] <= 0) continue; // settled
      this.vel[y] -= this.g * dt;
      for (let k = 0; k < 3; k++) this.pos[i * 3 + k] += this.vel[i * 3 + k] * dt;
      if (this.pos[y] < 0.001) {
        this.pos[y] = 0.001;
        this.vel[i * 3] = this.vel[y] = this.vel[i * 3 + 2] = 0;
      }
    }
    (this.points.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    const fade = Math.min(1, Math.max(0, (DustEffect.DURATION - this.age) / 1.2));
    (this.points.material as THREE.PointsMaterial).opacity = 0.9 * fade;
    return this.age < DustEffect.DURATION;
  }

  dispose() {
    this.points.geometry.dispose();
    (this.points.material as THREE.Material).dispose();
  }
}
