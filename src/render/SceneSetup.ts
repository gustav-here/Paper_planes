import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

/** Unfolded sheet extent in model space (A4: 297 mm along X, 210 mm along Z). */
const SHEET_X = 0.297;
const SHEET_Z = 0.21;
/** Screen space reserved for the HUD (px): left tool panels, top bar, bottom hint/status. */
const HUD = { left: 310, top: 140, bottom: 70, right: 20 };

/** A scene + camera pair that can be shown on the shared renderer (editor table, flight world). */
export interface View {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  /** Called every frame before rendering (e.g. controls.update()). */
  update?: () => void;
}

/** Renderer, camera, orbit controls and lighting for the folding table. */
export class Stage {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly controls: OrbitControls;
  private active: View | null = null;
  private extraCameras: THREE.PerspectiveCamera[] = [];

  constructor(readonly canvas: HTMLCanvasElement) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    this.scene.background = new THREE.Color('#2b2f36');
    this.scene.fog = new THREE.Fog('#2b2f36', 3, 9);

    this.camera = new THREE.PerspectiveCamera(32, 1, 0.005, 50);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.minDistance = 0.12;
    this.controls.maxDistance = 2.5;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.setLeftButtonOrbits(false);
    this.controls.update();
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    const hemi = new THREE.HemisphereLight('#fff8ee', '#5a4330', 0.9);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight('#fff4e0', 2.2);
    sun.position.set(-0.5, 1.3, 0.55);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    const s = 0.45;
    Object.assign(sun.shadow.camera, { left: -s, right: s, top: s, bottom: -s, near: 0.1, far: 3 });
    sun.shadow.bias = -0.0004;
    sun.shadow.normalBias = 0.002;
    sun.shadow.radius = 3;
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight('#cfe0ff', 0.45);
    fill.position.set(0.6, 0.5, -0.8);
    this.scene.add(fill);

    window.addEventListener('resize', () => this.resize());
    this.resize();
    this.resetCamera();
  }

  /** In the view tool the left button orbits; otherwise it belongs to the active tool. */
  setLeftButtonOrbits(on: boolean) {
    this.controls.mouseButtons = {
      LEFT: on ? THREE.MOUSE.ROTATE : (-1 as unknown as THREE.MOUSE),
      MIDDLE: THREE.MOUSE.PAN,
      RIGHT: THREE.MOUSE.ROTATE,
    };
  }

  /**
   * Look straight down at the table with the nose up on screen, so the sheet shows as an
   * undistorted A4 rectangle, at a distance where it fits in the area not covered by the HUD.
   */
  resetCamera() {
    const w = this.canvas.clientWidth || window.innerWidth;
    const h = this.canvas.clientHeight || window.innerHeight;
    const t = Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2));
    // free area in px, anchored to the bottom-right corner (shrinks gracefully on small screens)
    const fw = Math.max(w - HUD.left - HUD.right, w * 0.45);
    const fh = Math.max(h - HUD.top - HUD.bottom, h * 0.45);
    const centerX = w - HUD.right - fw / 2;
    const centerY = h - HUD.bottom - fh / 2;
    const margin = 1.12;
    // visible world height at distance d is 2·d·t; the sheet must fit in fh of h pixels (and likewise in width)
    const d = Math.max((SHEET_X * margin * h) / (fh * 2 * t), (SHEET_Z * margin * w) / (fw * 2 * t * this.camera.aspect));
    const metresPerPx = (2 * d * t) / h;
    // look at a point offset so the sheet lands in the middle of the free area (screen up = +X, right = +Z)
    const cx = (centerY - h / 2) * metresPerPx;
    const cz = -(centerX - w / 2) * metresPerPx;
    this.controls.target.set(cx, 0, cz);
    this.camera.position.set(cx - 1e-4, d, cz);
    this.controls.update();
  }

  resize() {
    const w = this.canvas.clientWidth || window.innerWidth;
    const h = this.canvas.clientHeight || window.innerHeight;
    this.renderer.setSize(w, h, false);
    for (const cam of [this.camera, ...this.extraCameras]) {
      cam.aspect = w / h;
      cam.updateProjectionMatrix();
    }
  }

  /** Register another camera so it follows canvas resizes. */
  addCamera(cam: THREE.PerspectiveCamera) {
    this.extraCameras.push(cam);
    this.resize();
  }

  /** Show another view (null = the folding table). The table's controls only work when it is shown. */
  setView(v: View | null) {
    this.active = v;
    this.controls.enabled = v === null;
  }

  render() {
    if (this.active) {
      this.active.update?.();
      this.renderer.render(this.active.scene, this.active.camera);
      return;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
