import * as THREE from 'three';
import { FLOOR_DEPTH, canvas, distanceOverlay, seeded, texture } from './common';
import type { VenueLook } from './common';

/*
 * The Moon: fine grey regolith strewn with rocks, soft rounded hills on the horizon, a black
 * sky and hard, low sunlight (no air to scatter it).
 */

function regolith(): THREE.CanvasTexture {
  const { c, g } = canvas(1024, 1024);
  const rnd = seeded(3);
  g.fillStyle = '#8d8d8b';
  g.fillRect(0, 0, c.width, c.height);
  // fine dust: one-pixel grains (≈ 1 cm) in many shades
  for (let i = 0; i < 160000; i++) {
    const l = 40 + rnd() * 36;
    g.fillStyle = `hsla(40, 3%, ${l}%, ${0.12 + rnd() * 0.3})`;
    g.fillRect(rnd() * c.width, rnd() * c.height, 1, 1);
  }
  // small craters: dark rim on the sun side, bright on the other
  for (let i = 0; i < 70; i++) {
    const x = rnd() * c.width;
    const y = rnd() * c.height;
    const r = 3 + rnd() * rnd() * 30;
    const grd = g.createRadialGradient(x - r * 0.2, y - r * 0.2, r * 0.2, x, y, r);
    grd.addColorStop(0, 'rgba(60,60,60,0.35)');
    grd.addColorStop(0.75, 'rgba(90,90,90,0.15)');
    grd.addColorStop(0.95, 'rgba(200,200,200,0.35)');
    grd.addColorStop(1, 'rgba(200,200,200,0)');
    g.fillStyle = grd;
    g.beginPath();
    g.arc(x, y, r, 0, Math.PI * 2);
    g.fill();
  }
  return texture(c, [120, 120]);
}

export function buildMoon(): VenueLook {
  const grp = new THREE.Group();
  const size = 1200;
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.MeshStandardMaterial({ map: regolith(), roughness: 1, metalness: 0, ...FLOOR_DEPTH }));
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  grp.add(ground);
  grp.add(
    distanceOverlay({ length: 150, step: 5, labelStep: 10, majorStep: 50, halfWidth: 6, color: '#ffffff', ink: '#ffffff', outline: 'rgba(0,0,0,0.8)' }),
  );

  // rocks: many small, a few big; keep the strip along the throw clear
  const rnd = seeded(19);
  const rockGeo = new THREE.IcosahedronGeometry(1, 1);
  const pos = rockGeo.getAttribute('position') as THREE.BufferAttribute;
  for (let i = 0; i < pos.count; i++) {
    const k = 0.75 + rnd() * 0.5;
    pos.setXYZ(i, pos.getX(i) * k, pos.getY(i) * k * 0.7, pos.getZ(i) * k);
  }
  rockGeo.computeVertexNormals();
  const n = 900;
  const rocks = new THREE.InstancedMesh(rockGeo, new THREE.MeshStandardMaterial({ color: '#77777a', roughness: 1, flatShading: true }), n);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  for (let i = 0; i < n; i++) {
    let x: number;
    let z: number;
    do {
      x = (rnd() - 0.3) * 400;
      z = (rnd() - 0.5) * 400;
    } while (Math.abs(z) < 7 && x > -3 && x < 160);
    const big = rnd() < 0.04;
    const s = big ? 0.6 + rnd() * 1.4 : 0.03 + rnd() * rnd() * 0.35;
    q.setFromEuler(new THREE.Euler(rnd() * 0.4, rnd() * Math.PI * 2, rnd() * 0.4));
    m.compose(new THREE.Vector3(x, s * 0.25, z), q, new THREE.Vector3(s, s, s));
    rocks.setMatrixAt(i, m);
  }
  rocks.castShadow = true;
  rocks.receiveShadow = true;
  grp.add(rocks);

  // soft rounded hills on the horizon
  const hillMat = new THREE.MeshStandardMaterial({ color: '#9a9a98', roughness: 1 });
  for (let i = 0; i < 26; i++) {
    const a = (i / 26) * Math.PI * 2 + rnd() * 0.2;
    const r = 420 + rnd() * 120;
    const hill = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 16, 0, Math.PI * 2, 0, Math.PI / 2), hillMat);
    hill.scale.set(90 + rnd() * 80, 25 + rnd() * 35, 90 + rnd() * 80);
    hill.position.set(Math.cos(a) * r, -2, Math.sin(a) * r);
    grp.add(hill);
  }

  return {
    group: grp,
    background: new THREE.Color('#000000'),
    fog: null,
    key: { color: '#ffffff', intensity: 3.2, offset: new THREE.Vector3(-8, 5, 6) },
    fill: { sky: '#1a1a1d', ground: '#5a5a58', intensity: 0.35 },
  };
}
