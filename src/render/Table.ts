import * as THREE from 'three';

/** Deterministic pseudo-random generator for procedural textures (render only). */
function mulberry32(seed: number) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Procedural oak-like wood: planks along X with wavy grain and a few knots. */
export function createWoodTexture(size = 1024): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const g = c.getContext('2d')!;
  const rnd = mulberry32(7);
  const planks = 5;
  const ph = size / planks;
  for (let p = 0; p < planks; p++) {
    const y0 = p * ph;
    const hue = 28 + rnd() * 6;
    const light = 38 + rnd() * 10;
    g.fillStyle = `hsl(${hue}, 45%, ${light}%)`;
    g.fillRect(0, y0, size, ph);
    // grain
    const phase = rnd() * 100;
    for (let i = 0; i < 70; i++) {
      const yy = y0 + rnd() * ph;
      const amp = 1.5 + rnd() * 5;
      const freq = 0.004 + rnd() * 0.01;
      g.strokeStyle = `hsla(${hue - 4}, 50%, ${light - 12 - rnd() * 10}%, ${0.15 + rnd() * 0.25})`;
      g.lineWidth = 0.6 + rnd() * 1.6;
      g.beginPath();
      for (let x = 0; x <= size; x += 8) {
        const y = yy + Math.sin(x * freq + phase + i) * amp + Math.sin(x * freq * 3.1 + i) * amp * 0.3;
        if (x === 0) g.moveTo(x, y);
        else g.lineTo(x, y);
      }
      g.stroke();
    }
    // knots
    for (let k = 0; k < 2; k++) {
      if (rnd() < 0.5) continue;
      const kx = rnd() * size;
      const ky = y0 + ph * (0.3 + rnd() * 0.4);
      for (let r = 14; r > 2; r -= 3) {
        g.strokeStyle = `hsla(${hue - 6}, 55%, ${light - 20}%, 0.35)`;
        g.beginPath();
        g.ellipse(kx, ky, r * 2.2, r, 0, 0, Math.PI * 2);
        g.stroke();
      }
    }
    // plank seam
    g.fillStyle = 'rgba(30,18,8,0.55)';
    g.fillRect(0, y0, size, 2);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.anisotropy = 8;
  return tex;
}

/** Wooden table whose top surface is at y = 0. */
export function createTable(): THREE.Group {
  const group = new THREE.Group();
  const tex = createWoodTexture();
  tex.repeat.set(1.2, 1);
  const topMat = new THREE.MeshStandardMaterial({ map: tex, roughness: 0.55, metalness: 0 });
  const edgeMat = new THREE.MeshStandardMaterial({ color: '#5e3c22', roughness: 0.6 });
  const W = 1.6;
  const D = 1.0;
  const T = 0.035;
  const top = new THREE.Mesh(new THREE.BoxGeometry(W, T, D), [edgeMat, edgeMat, topMat, edgeMat, edgeMat, edgeMat]);
  top.position.y = -T / 2;
  top.receiveShadow = true;
  group.add(top);
  const legGeo = new THREE.BoxGeometry(0.06, 0.72, 0.06);
  for (const [x, z] of [
    [-W / 2 + 0.08, -D / 2 + 0.08],
    [W / 2 - 0.08, -D / 2 + 0.08],
    [-W / 2 + 0.08, D / 2 - 0.08],
    [W / 2 - 0.08, D / 2 - 0.08],
  ]) {
    const leg = new THREE.Mesh(legGeo, edgeMat);
    leg.position.set(x, -T - 0.36, z);
    group.add(leg);
  }
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(12, 12),
    new THREE.MeshStandardMaterial({ color: '#3b3a38', roughness: 0.95 }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.755;
  floor.receiveShadow = true;
  group.add(floor);
  return group;
}
