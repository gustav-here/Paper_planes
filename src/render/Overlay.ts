import * as THREE from 'three';
import * as V from '../core/math/vec3';
import type { Vec3 } from '../core/math/vec3';

export type LineStyle = 'valley' | 'mountain' | 'cut' | 'twin' | 'extension' | 'error';

const STYLES: Record<LineStyle, { color: string; dash: number; gap: number; opacity: number }> = {
  valley: { color: '#3d7dff', dash: 0.006, gap: 0.004, opacity: 1 },
  mountain: { color: '#e5484d', dash: 0.009, gap: 0.003, opacity: 1 },
  cut: { color: '#9b5cff', dash: 1, gap: 0, opacity: 1 },
  twin: { color: '#b48cff', dash: 0.004, gap: 0.004, opacity: 0.85 },
  extension: { color: '#ffffff', dash: 0.002, gap: 0.005, opacity: 0.35 },
  error: { color: '#ff3b30', dash: 0.004, gap: 0.003, opacity: 1 },
};

/**
 * Tool overlays drawn on top of everything (depthTest off) in model coordinates:
 * fold/cut lines, the mirrored twin line visible through the paper, and the side arrow.
 */
export class Overlay {
  readonly group = new THREE.Group();
  private items: THREE.Object3D[] = [];

  clear() {
    for (const o of this.items) {
      this.group.remove(o);
      o.traverse((c) => {
        const m = c as THREE.Mesh;
        m.geometry?.dispose();
        (m.material as THREE.Material | undefined)?.dispose();
      });
    }
    this.items = [];
  }

  line(a: Vec3, b: Vec3, style: LineStyle) {
    const s = STYLES[style];
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a), new THREE.Vector3(...b)]);
    const mat =
      s.gap > 0
        ? new THREE.LineDashedMaterial({ color: s.color, dashSize: s.dash, gapSize: s.gap, transparent: true, opacity: s.opacity, depthTest: false })
        : new THREE.LineBasicMaterial({ color: s.color, transparent: true, opacity: s.opacity, depthTest: false });
    const l = new THREE.Line(geo, mat);
    l.computeLineDistances();
    l.renderOrder = 20;
    this.add(l);
  }

  /** A fold/cut line with faint extensions showing that the line is infinite. */
  foldLine(p0: Vec3, p1: Vec3, style: LineStyle, extend = 0.12) {
    const d = V.normalize(V.sub(p1, p0));
    this.line(V.addScaled(p0, d, -extend), p0, 'extension');
    this.line(p1, V.addScaled(p1, d, extend), 'extension');
    this.line(p0, p1, style);
  }

  /** Flat arrow in the working plane pointing from `from` along `dir`. */
  arrow(from: Vec3, dir: Vec3, normal: Vec3, color = '#ffd34d', len = 0.03) {
    const d = V.normalize(dir);
    const side = V.normalize(V.cross(normal, d));
    const lift = V.scale(normal, 0.0006);
    const tip = V.add(V.addScaled(from, d, len), lift);
    const base = V.add(from, lift);
    const w = len * 0.28;
    const headBase = V.addScaled(tip, d, -len * 0.4);
    const pts = [
      ...V.addScaled(base, side, -w * 0.3), ...V.addScaled(headBase, side, -w * 0.3), ...V.addScaled(headBase, side, w * 0.3),
      ...V.addScaled(base, side, -w * 0.3), ...V.addScaled(headBase, side, w * 0.3), ...V.addScaled(base, side, w * 0.3),
      ...V.addScaled(headBase, side, -w), ...tip, ...V.addScaled(headBase, side, w),
    ];
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    const mesh = new THREE.Mesh(
      geo,
      new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.9, depthTest: false }),
    );
    mesh.renderOrder = 21;
    this.add(mesh);
  }

  private add(o: THREE.Object3D) {
    this.items.push(o);
    this.group.add(o);
  }
}
