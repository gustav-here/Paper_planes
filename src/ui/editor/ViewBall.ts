import * as THREE from 'three';
import { facetVertices } from '../../core/paper/model';
import type { PaperModel } from '../../core/paper/model';
import type { Vec3 } from '../../core/math/vec3';
import { PaperView } from '../../render/PaperMesh';

/**
 * "360 ball": a small trackball in the corner. Dragging it rotates the folded model freely
 * (arcball rotation in screen space, so the model follows the mouse like a ball in the hand);
 * double-click puts it upright again (nose away, centre fold down). A miniature of the model sits inside the ball and
 * mirrors the current view orientation.
 */
export class ViewBall {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(30, 1, 0.1, 20);
  /** Rotates with the model view. */
  private spin = new THREE.Group();
  private mini = new PaperView();
  private miniHolder = new THREE.Group();
  private model: PaperModel | null = null;
  private noseMarker: THREE.Mesh;
  private ringMat = new THREE.LineBasicMaterial({ color: '#c9d6ea', transparent: true, opacity: 0.35 });
  private dragging = false;
  private last = new THREE.Vector3();

  constructor(
    private el: HTMLElement,
    /** Camera of the main view (the ball shows the model as seen from it). */
    private mainCamera: THREE.Camera,
    /** Apply an incremental world-space rotation to the model view. */
    private rotate: (q: THREE.Quaternion) => void,
    /** Reset to the upright view. */
    private reset: () => void,
    /** The drag ended. */
    private onEnd: () => void = () => {},
  ) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    el.appendChild(this.renderer.domElement);

    // glassy ball with latitude/longitude rings that turn with the model
    const ball = new THREE.Mesh(
      new THREE.SphereGeometry(1, 48, 32),
      new THREE.MeshStandardMaterial({ color: '#7f93b3', transparent: true, opacity: 0.18, roughness: 0.3, depthWrite: false }),
    );
    ball.renderOrder = 2;
    this.scene.add(ball);
    const rings: number[] = [];
    const ring = (axis: 'x' | 'y' | 'z', tilt: number) => {
      const n = 64;
      for (let i = 0; i < n; i++) {
        for (const k of [i, i + 1]) {
          const a = (k / n) * Math.PI * 2;
          const r = Math.cos(tilt);
          const h = Math.sin(tilt);
          const p =
            axis === 'y'
              ? [r * Math.cos(a), h, r * Math.sin(a)]
              : axis === 'x'
                ? [h, r * Math.cos(a), r * Math.sin(a)]
                : [r * Math.cos(a), r * Math.sin(a), h];
          rings.push(p[0] * 1.002, p[1] * 1.002, p[2] * 1.002);
        }
      }
    };
    for (const t of [-0.6, 0, 0.6]) ring('y', t);
    ring('x', 0);
    ring('z', 0);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(rings, 3));
    this.spin.add(new THREE.LineSegments(geo, this.ringMat));
    // nose marker (+X) on the ball
    this.noseMarker = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 12), new THREE.MeshBasicMaterial({ color: '#ffb020' }));
    this.spin.add(this.noseMarker);

    this.miniHolder.add(this.mini.group);
    this.spin.add(this.miniHolder);
    this.scene.add(this.spin);

    this.scene.add(new THREE.HemisphereLight('#ffffff', '#6b5a48', 1.4));
    const key = new THREE.DirectionalLight('#ffffff', 1.6);
    key.position.set(-1, 2, 3);
    this.camera.add(key);
    this.scene.add(this.camera);

    this.bind();
    new ResizeObserver(() => this.resize()).observe(el);
    this.resize();
  }

  private resize() {
    const s = Math.max(1, this.el.clientWidth);
    this.renderer.setSize(s, s, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
  }

  /** Point on the unit arcball (camera space) for a pointer position. */
  private ballPoint(e: PointerEvent): THREE.Vector3 {
    const r = this.el.getBoundingClientRect();
    const x = ((e.clientX - r.left) / r.width) * 2 - 1;
    const y = -(((e.clientY - r.top) / r.height) * 2 - 1);
    const d2 = x * x + y * y;
    return d2 <= 1 ? new THREE.Vector3(x, y, Math.sqrt(1 - d2)) : new THREE.Vector3(x, y, 0).normalize();
  }

  private bind() {
    const el = this.el;
    el.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      e.preventDefault();
      e.stopPropagation();
      this.dragging = true;
      try {
        el.setPointerCapture(e.pointerId);
      } catch {
        /* capture is a convenience (keeps the drag when leaving the ball) */
      }
      this.last = this.ballPoint(e);
      el.classList.add('grabbing');
    });
    el.addEventListener('pointermove', (e) => {
      if (!this.dragging) return;
      const p = this.ballPoint(e);
      const qCam = new THREE.Quaternion().setFromUnitVectors(this.last, p);
      // screen-space rotation → world space: q_world = q_cam' expressed in the main camera frame
      const camQ = this.mainCamera.quaternion;
      const qWorld = camQ.clone().multiply(qCam).multiply(camQ.clone().invert());
      this.rotate(qWorld);
      this.last = p;
    });
    const end = (e: PointerEvent) => {
      if (!this.dragging) return;
      this.dragging = false;
      if (el.hasPointerCapture(e.pointerId)) el.releasePointerCapture(e.pointerId);
      el.classList.remove('grabbing');
      this.onEnd();
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
    el.addEventListener('dblclick', (e) => {
      e.preventDefault();
      this.reset();
    });
    el.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  /** Highlight the rings while the model is snapped to a 90° orientation. */
  setSnapped(on: boolean) {
    this.ringMat.color.set(on ? '#5fe08a' : '#c9d6ea');
    this.ringMat.opacity = on ? 0.85 : 0.35;
  }

  /** Draw the ball for the current view rotation and model. */
  render(viewQ: THREE.Quaternion, model: PaperModel, nose: Vec3) {
    if (model !== this.model) {
      this.model = model;
      this.mini.update(model);
      // fit the model into the ball, centred (bounding box in model coordinates)
      const box = new THREE.Box3();
      for (const f of model.facets) for (const v of facetVertices(f)) box.expandByPoint(new THREE.Vector3(...v));
      const c = box.getCenter(new THREE.Vector3());
      const radius = Math.max(box.getSize(new THREE.Vector3()).length() / 2, 1e-3);
      this.miniHolder.scale.setScalar(0.82 / radius);
      this.mini.group.position.copy(c).multiplyScalar(-1);
    }
    this.noseMarker.position.set(...nose).normalize();
    this.spin.quaternion.copy(viewQ);
    // look at the ball from the same direction as the main camera looks at the table
    this.camera.quaternion.copy(this.mainCamera.quaternion);
    this.camera.position.set(0, 0, 4.4).applyQuaternion(this.camera.quaternion);
    this.renderer.render(this.scene, this.camera);
  }
}
