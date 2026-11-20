import * as THREE from 'three';

export interface TextSpriteOptions {
  /** World height of the text box (m). */
  height?: number;
  color?: string;
  background?: string;
  font?: string;
  /** Draw on top of everything. */
  overlay?: boolean;
  /** Constant size on screen: `height` is then a fraction of the viewport height. */
  screenSpace?: boolean;
}

/** Camera-facing text label rendered from a canvas. */
export function createTextSprite(text: string, opts: TextSpriteOptions = {}): THREE.Sprite {
  const lines = text.split('\n');
  const fontPx = 64;
  const font = opts.font ?? `600 ${fontPx}px system-ui, -apple-system, sans-serif`;
  const c = document.createElement('canvas');
  const g = c.getContext('2d')!;
  g.font = font;
  const pad = 22;
  const w = Math.ceil(Math.max(...lines.map((l) => g.measureText(l).width)) + pad * 2);
  const lh = fontPx * 1.2;
  const h = Math.ceil(lh * lines.length + pad * 1.2);
  c.width = w;
  c.height = h;
  g.font = font;
  if (opts.background) {
    g.fillStyle = opts.background;
    const r = 18;
    g.beginPath();
    g.roundRect(0, 0, w, h, r);
    g.fill();
  }
  g.fillStyle = opts.color ?? '#ffffff';
  g.textBaseline = 'middle';
  g.textAlign = 'center';
  lines.forEach((l, i) => g.fillText(l, w / 2, pad * 0.6 + lh * (i + 0.5)));
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  const mat = new THREE.SpriteMaterial({
    map: tex,
    transparent: true,
    depthTest: !opts.overlay,
    depthWrite: false,
    sizeAttenuation: !opts.screenSpace,
  });
  const sprite = new THREE.Sprite(mat);
  const height = opts.height ?? 0.3;
  sprite.scale.set((height * w) / h, height, 1);
  if (opts.overlay) sprite.renderOrder = 50;
  return sprite;
}

export function disposeSprite(s: THREE.Sprite) {
  s.material.map?.dispose();
  s.material.dispose();
}
