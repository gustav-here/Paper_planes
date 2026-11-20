import * as THREE from 'three';

/** Deterministic pseudo-random numbers for scenery (the same hall every time). */
export function seeded(seed: number): () => number {
  let s = seed >>> 0;
  return () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
}

export function canvas(w: number, h: number): { c: HTMLCanvasElement; g: CanvasRenderingContext2D } {
  const c = document.createElement('canvas');
  c.width = w;
  c.height = h;
  return { c, g: c.getContext('2d')! };
}

export function texture(c: HTMLCanvasElement, repeat?: [number, number]): THREE.CanvasTexture {
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  if (repeat) {
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.repeat.set(repeat[0], repeat[1]);
  }
  return t;
}

export interface DistanceMarks {
  /** Last marked distance (m). */
  length: number;
  /** A line every … m, a number every … m, and a longer line with a bigger number every … m. */
  step: number;
  labelStep: number;
  majorStep: number;
  /** Half width of the lines across the throw direction (m). */
  halfWidth: number;
  color: string;
  /** Text colour and its outline. */
  ink: string;
  outline: string;
}

/**
 * Distance lines from the throw point: a transparent overlay on the floor along +X from x = 0
 * (the release line), with lines across every `step` metres and numbers every `labelStep`.
 */
export function distanceOverlay(o: DistanceMarks): THREE.Mesh {
  const ppm = Math.min(96, 4096 / (o.length + 2));
  const W = Math.round((o.length + 2) * ppm);
  const H = Math.round(2 * o.halfWidth * ppm);
  const { c, g } = canvas(W, H);
  const X = (x: number) => (x + 1) * ppm;
  const Z = (z: number) => (z + o.halfWidth) * ppm;
  g.strokeStyle = o.color;
  g.lineCap = 'butt';
  for (let m = o.step; m <= o.length + 1e-9; m += o.step) {
    const major = Math.abs(m / o.majorStep - Math.round(m / o.majorStep)) < 1e-6;
    g.lineWidth = Math.max(1.5, (major ? 0.05 : 0.025) * ppm);
    g.globalAlpha = major ? 0.95 : 0.6;
    const half = major ? o.halfWidth * 0.92 : o.halfWidth * 0.55;
    g.beginPath();
    g.moveTo(X(m), Z(-half));
    g.lineTo(X(m), Z(half));
    g.stroke();
  }
  g.globalAlpha = 0.55;
  g.setLineDash([0.3 * ppm, 0.2 * ppm]);
  g.lineWidth = Math.max(1.5, 0.025 * ppm);
  g.beginPath();
  g.moveTo(X(0), Z(0));
  g.lineTo(X(o.length), Z(0));
  g.stroke();
  g.setLineDash([]);
  // the release line (throw point)
  g.globalAlpha = 1;
  g.strokeStyle = '#e0645a';
  g.lineWidth = Math.max(2, 0.07 * ppm);
  g.beginPath();
  g.moveTo(X(0), Z(-o.halfWidth * 0.92));
  g.lineTo(X(0), Z(o.halfWidth * 0.92));
  g.stroke();
  // numbers, on both sides, readable from the throw point (text runs across the hall)
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  for (let m = o.labelStep; m <= o.length + 1e-9; m += o.labelStep) {
    const major = Math.abs(m / o.majorStep - Math.round(m / o.majorStep)) < 1e-6;
    const fontPx = Math.max(12, Math.min(major ? 0.5 : 0.22, o.majorStep * (major ? 0.12 : 0.05)) * ppm);
    g.font = `${major ? 800 : 600} ${fontPx}px system-ui, -apple-system, sans-serif`;
    for (const side of [-1, 1]) {
      g.save();
      g.translate(X(m) - fontPx * 0.75, Z(side * o.halfWidth * 0.8));
      g.rotate(Math.PI / 2);
      g.lineWidth = fontPx * 0.18;
      g.strokeStyle = o.outline;
      g.strokeText(`${m} m`, 0, 0);
      g.fillStyle = o.ink;
      g.fillText(`${m} m`, 0, 0);
      g.restore();
    }
  }
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(o.length + 2, 2 * o.halfWidth),
    // pulled towards the camera in depth so it never sinks into a large ground plane
    new THREE.MeshBasicMaterial({ map: texture(c), transparent: true, depthWrite: false, toneMapped: false, polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4 }),
  );
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.set(o.length / 2, 0.004, 0);
  mesh.renderOrder = 1;
  return mesh;
}

/** Look of a venue: its scenery group and how the scene around it is lit. */
export interface VenueLook {
  group: THREE.Group;
  background: THREE.Color | THREE.Texture;
  fog: THREE.Fog | null;
  /** Shadow-casting key light: colour, intensity and its direction offset from the plane. */
  key: { color: string; intensity: number; offset: THREE.Vector3 };
  /** Fill light (sky colour, ground colour, intensity). */
  fill: { sky: string; ground: string; intensity: number };
}
