import * as THREE from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import * as V from '../core/math/vec3';
import type { Vec3 } from '../core/math/vec3';

export type LineStyle = 'fold' | 'valley' | 'mountain' | 'cut' | 'twin' | 'extension' | 'error';

interface Style {
  color: string;
  /** Dash and gap length in metres (gap 0 = solid). */
  dash: number;
  gap: number;
  opacity: number;
  /** Line width in pixels. */
  width: number;
  /** Dark outline behind the line so it stands out on white paper. */
  halo: boolean;
}

const STYLES: Record<LineStyle, Style> = {
  /** Fold line whose direction is not chosen yet. */
  fold: { color: '#ffc21a', dash: 0.008, gap: 0.004, opacity: 1, width: 4, halo: true },
  valley: { color: '#2f7bff', dash: 0.008, gap: 0.004, opacity: 1, width: 4, halo: true },
  mountain: { color: '#ff3b47', dash: 0.012, gap: 0.004, opacity: 1, width: 4, halo: true },
  cut: { color: '#a066ff', dash: 1, gap: 0, opacity: 1, width: 4, halo: true },
  twin: { color: '#c3a2ff', dash: 0.006, gap: 0.005, opacity: 0.9, width: 3, halo: true },
  extension: { color: '#ffffff', dash: 0.003, gap: 0.006, opacity: 0.55, width: 1.5, halo: false },
  error: { color: '#ff3b30', dash: 0.006, gap: 0.004, opacity: 1, width: 4, halo: true },
};

/**
 * Tool overlays drawn on top of everything (depthTest off) in model coordinates:
 * fold/cut lines, the mirrored twin line visible through the paper, and the side arrow.
 */
export class Overlay {
  readonly group = new THREE.Group();
  /** Canvas size in CSS pixels (fat lines are specified in pixels). */
  readonly resolution = new THREE.Vector2(1, 1);
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
    const fat = (color: string, width: number, opacity: number, dashed: boolean, order: number) => {
      const geo = new LineGeometry();
      geo.setPositions([...a, ...b]);
      const mat = new LineMaterial({
        color: new THREE.Color(color).getHex(),
        linewidth: width,
        transparent: true,
        opacity,
        depthTest: false,
        dashed,
        dashSize: s.dash,
        gapSize: s.gap,
      });
      mat.resolution.copy(this.resolution);
      const l = new Line2(geo, mat);
      if (dashed) l.computeLineDistances();
      l.renderOrder = order;
      this.add(l);
    };
    if (s.halo) fat('#101418', s.width + 3, 0.55 * s.opacity, false, 20);
    fat(s.color, s.width, s.opacity, s.gap > 0, 21);
  }

  /** Small round marker at a line end (radius in screen pixels). */
  dot(p: Vec3, color: string, radiusPx = 4.5, order = 22) {
    const m = new THREE.Mesh(
      new THREE.CircleGeometry(1, 20),
      new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true }),
    );
    m.position.set(...p);
    m.userData.screenDot = radiusPx;
    m.renderOrder = order;
    this.add(m);
  }

  /** Draggable line end: a ringed knob that grows under the cursor. */
  handle(p: Vec3, color: string, hot: boolean) {
    this.dot(p, '#101418', hot ? 10 : 8, 23);
    this.dot(p, hot ? '#ffffff' : color, hot ? 7.5 : 6, 24);
  }

  /** A fold/cut line with faint extensions showing that the line is infinite. */
  foldLine(p0: Vec3, p1: Vec3, style: LineStyle, extend = 0.12) {
    const d = V.normalize(V.sub(p1, p0));
    this.line(V.addScaled(p0, d, -extend), p0, 'extension');
    this.line(p1, V.addScaled(p1, d, extend), 'extension');
    this.line(p0, p1, style);
    this.dot(p0, STYLES[style].color);
    this.dot(p1, STYLES[style].color);
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

  /** Keep end dots a constant size on screen and fat lines at the canvas resolution. */
  update(camera: THREE.Camera, viewportHeight: number) {
    const tmp = new THREE.Vector3();
    for (const o of this.items) {
      if (o.userData.screenDot) {
        o.getWorldPosition(tmp);
        const dist = tmp.distanceTo(camera.position);
        const fov = (camera as THREE.PerspectiveCamera).fov ?? 40;
        const worldPerPx = (2 * dist * Math.tan(THREE.MathUtils.degToRad(fov / 2))) / Math.max(viewportHeight, 1);
        o.scale.setScalar(worldPerPx * (o.userData.screenDot as number));
        o.quaternion.copy(camera.quaternion);
        if (o.parent) o.quaternion.premultiply(o.parent.getWorldQuaternion(new THREE.Quaternion()).invert());
      } else if ((o as Line2).isLine2) {
        ((o as Line2).material as LineMaterial).resolution.copy(this.resolution);
      }
    }
  }

  private add(o: THREE.Object3D) {
    this.items.push(o);
    this.group.add(o);
  }
}
