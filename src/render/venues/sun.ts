import * as THREE from 'three';
import { canvas, seeded, texture } from './common';
import type { VenueLook } from './common';

/*
 * The Sun as an extreme-ultraviolet image shows it: a churning golden surface (flat here, no
 * curved horizon) below, glowing loops and rays of plasma rising into a hazy golden corona.
 * There is no floor: the photosphere is gas.
 */

/** The turbulent surface seen from above: a dense fuzz of fine fibrils with bright knots. */
function surface(): THREE.CanvasTexture {
  const { c, g } = canvas(1024, 1024);
  const rnd = seeded(5);
  g.fillStyle = '#6e4a08';
  g.fillRect(0, 0, c.width, c.height);
  // broad light and dark patches
  for (let i = 0; i < 90; i++) {
    const x = rnd() * c.width;
    const y = rnd() * c.height;
    const r = 40 + rnd() * 140;
    const grd = g.createRadialGradient(x, y, 0, x, y, r);
    const light = rnd() > 0.5;
    grd.addColorStop(0, light ? 'rgba(230,170,40,0.35)' : 'rgba(40,24,0,0.35)');
    grd.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = grd;
    g.fillRect(x - r, y - r, 2 * r, 2 * r);
  }
  // fine fibrils, mostly upright, softly blurred
  for (let i = 0; i < 14000; i++) {
    const x = rnd() * c.width;
    const y = rnd() * c.height;
    const len = 4 + rnd() * 16;
    const a = -Math.PI / 2 + (rnd() - 0.5) * 1.2;
    const bright = rnd();
    g.strokeStyle = bright > 0.85 ? `rgba(255,${200 + rnd() * 40},${90 + rnd() * 60},${0.25 + rnd() * 0.35})` : `rgba(${50 + rnd() * 60},${30 + rnd() * 40},0,${0.2 + rnd() * 0.3})`;
    g.lineWidth = 0.8 + rnd() * 1.5;
    g.beginPath();
    g.moveTo(x, y);
    g.lineTo(x + Math.cos(a) * len, y + Math.sin(a) * len);
    g.stroke();
  }
  // bright knots of loops
  for (let i = 0; i < 40; i++) {
    const x = rnd() * c.width;
    const y = rnd() * c.height;
    const r = 6 + rnd() * 22;
    const grd = g.createRadialGradient(x, y, 0, x, y, r);
    grd.addColorStop(0, 'rgba(255,245,190,0.9)');
    grd.addColorStop(1, 'rgba(255,190,40,0)');
    g.fillStyle = grd;
    g.fillRect(x - r, y - r, 2 * r, 2 * r);
  }
  return texture(c, [60, 60]);
}

/** The corona behind: haze, bright rays fanning upwards and a few looping filaments. */
function corona(): THREE.CanvasTexture {
  const { c, g } = canvas(2048, 1024);
  const rnd = seeded(23);
  const grd = g.createLinearGradient(0, 0, 0, c.height);
  grd.addColorStop(0, '#6b4a06');
  grd.addColorStop(0.55, '#b9870f');
  grd.addColorStop(1, '#f2c535');
  g.fillStyle = grd;
  g.fillRect(0, 0, c.width, c.height);
  g.globalCompositeOperation = 'lighter';
  // rays fanning up from bright spots near the horizon
  for (let s = 0; s < 6; s++) {
    const sx = rnd() * c.width;
    for (let i = 0; i < 90; i++) {
      const a = -Math.PI / 2 + (rnd() - 0.5) * 1.6;
      const len = 300 + rnd() * 700;
      g.strokeStyle = `rgba(255,${190 + rnd() * 50},${60 + rnd() * 60},${0.03 + rnd() * 0.07})`;
      g.lineWidth = 2 + rnd() * 10;
      g.beginPath();
      g.moveTo(sx, c.height);
      g.lineTo(sx + Math.cos(a) * len, c.height + Math.sin(a) * len);
      g.stroke();
    }
  }
  // arching loops
  for (let i = 0; i < 40; i++) {
    const x = rnd() * c.width;
    const w = 80 + rnd() * 400;
    const h = 60 + rnd() * 500;
    g.strokeStyle = `rgba(255,${200 + rnd() * 40},${80 + rnd() * 60},${0.05 + rnd() * 0.12})`;
    g.lineWidth = 2 + rnd() * 6;
    g.beginPath();
    g.moveTo(x, c.height);
    g.bezierCurveTo(x, c.height - h, x + w, c.height - h, x + w, c.height);
    g.stroke();
  }
  g.globalCompositeOperation = 'source-over';
  // a dark wisp of cooler plasma
  for (let i = 0; i < 12; i++) {
    g.strokeStyle = `rgba(60,35,0,${0.08 + rnd() * 0.1})`;
    g.lineWidth = 10 + rnd() * 30;
    g.beginPath();
    const x = c.width * (0.3 + rnd() * 0.4);
    g.moveTo(x, c.height * 0.2);
    g.bezierCurveTo(x + 200, c.height * 0.4, x - 200, c.height * 0.6, x + rnd() * 100, c.height);
    g.stroke();
  }
  return texture(c);
}

/**
 * Height of the glowing surface (m): at the feet of the thrower, as on the other venues. There is
 * nothing solid to land on. With the eye a couple of metres above it, the far edge of the surface
 * lies practically at eye level, so the horizon is a straight line.
 */
export const SUN_SURFACE_Y = 0;

export function buildSun(): VenueLook {
  const grp = new THREE.Group();
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(600, 600), new THREE.MeshBasicMaterial({ map: surface(), toneMapped: false }));
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = SUN_SURFACE_Y;
  grp.add(ground);
  // the corona: a big cylinder wall around the scene, open at the top
  // inside the camera's far plane (300 m)
  const sky = new THREE.Mesh(
    new THREE.CylinderGeometry(240, 240, 300, 64, 1, true),
    new THREE.MeshBasicMaterial({ map: corona(), side: THREE.BackSide, toneMapped: false, fog: false }),
  );
  sky.position.y = SUN_SURFACE_Y + 150;
  grp.add(sky);
  return {
    group: grp,
    background: new THREE.Color('#7a5408'),
    fog: new THREE.Fog('#b07a12', 40, 260),
    key: { color: '#ffd27a', intensity: 3.5, offset: new THREE.Vector3(0, 10, 2) },
    fill: { sky: '#ffcf60', ground: '#ff9a20', intensity: 2.2 },
  };
}
