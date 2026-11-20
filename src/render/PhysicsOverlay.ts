import * as THREE from 'three';
import * as V from '../core/math/vec3';
import type { Airframe } from '../core/physics/airframe';

/** Debug view of an airframe in model coordinates: centre of mass, body axes and aero elements. */
export class PhysicsOverlay {
  readonly group = new THREE.Group();

  update(af: Airframe | null) {
    for (const c of [...this.group.children]) {
      this.group.remove(c);
      c.traverse((o) => {
        const m = o as THREE.Mesh;
        m.geometry?.dispose();
        const mat = m.material as THREE.Material | THREE.Material[] | undefined;
        if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
        else mat?.dispose();
      });
    }
    if (!af) return;
    const com = new THREE.Vector3(...af.com);

    // centre of mass: yellow/black ball visible through the paper
    const ball = new THREE.Mesh(
      new THREE.SphereGeometry(0.0035, 16, 12),
      new THREE.MeshBasicMaterial({ color: '#ffd34d', depthTest: false }),
    );
    ball.position.copy(com);
    ball.renderOrder = 31;
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.0045, 0.0008, 6, 24),
      new THREE.MeshBasicMaterial({ color: '#111111', depthTest: false }),
    );
    ring.position.copy(com);
    ring.renderOrder = 32;
    ring.lookAt(com.clone().add(new THREE.Vector3(...af.right)));
    this.group.add(ball, ring);

    // body axes
    const axis = (dir: number[], color: string) => {
      const a = new THREE.ArrowHelper(new THREE.Vector3(...dir).normalize(), com, 0.05, color, 0.012, 0.007);
      a.traverse((o) => {
        const m = o as THREE.Mesh;
        if (m.material) (m.material as THREE.Material).depthTest = false;
        o.renderOrder = 30;
      });
      this.group.add(a);
    };
    axis(af.nose, '#ffb020');
    axis(af.up, '#4fd1ff');

    // aero elements, coloured per surface
    const pos: number[] = [];
    const col: number[] = [];
    const c = new THREE.Color();
    for (const e of af.mesh.elements) {
      const p = V.add(e.center, af.com);
      pos.push(...p);
      c.setHSL((e.surface * 0.17) % 1, 0.75, 0.5);
      col.push(c.r, c.g, c.b);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    const pts = new THREE.Points(
      geo,
      new THREE.PointsMaterial({ size: 3, sizeAttenuation: false, vertexColors: true, depthTest: false, transparent: true, opacity: 0.85 }),
    );
    pts.renderOrder = 29;
    this.group.add(pts);
  }
}
