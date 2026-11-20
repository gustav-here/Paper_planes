import * as THREE from 'three';
import { createTextSprite } from './TextSprite';

/** Procedural hall floor: light, slightly mottled with faint tile seams every metre. */
function floorTexture(): THREE.CanvasTexture {
  const size = 512;
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const g = c.getContext('2d')!;
  g.fillStyle = '#c9c4b8';
  g.fillRect(0, 0, size, size);
  // deterministic speckle
  let seed = 12345;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  for (let i = 0; i < 5000; i++) {
    const l = 66 + rnd() * 14;
    g.fillStyle = `hsla(40, 8%, ${l}%, ${0.25 + rnd() * 0.3})`;
    g.fillRect(rnd() * size, rnd() * size, 1 + rnd() * 3, 1 + rnd() * 3);
  }
  g.strokeStyle = 'rgba(90,85,75,0.35)';
  g.lineWidth = 3;
  g.strokeRect(0, 0, size, size);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}

export interface World {
  scene: THREE.Scene;
  /** Sun whose shadow camera follows the plane. */
  sun: THREE.DirectionalLight;
  /** Move the shadow-casting light so it covers `p`. */
  followLight(p: THREE.Vector3): void;
}

const LENGTH = 40; // measured distance (m)

/**
 * A large indoor hall: floor at y = 0, distance lines along +X from the release point every
 * metre (labels every 5 m), and a release line at x = 0.
 */
export function createWorld(): World {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#9fb3c8');
  scene.fog = new THREE.Fog('#9fb3c8', 25, 90);

  const tex = floorTexture();
  tex.repeat.set(200, 200);
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(200, 200),
    new THREE.MeshStandardMaterial({ map: tex, roughness: 0.85 }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);

  // distance lines (slightly above the floor to avoid z-fighting)
  const thin: number[] = [];
  const thick: number[] = [];
  const y = 0.002;
  for (let m = 0; m <= LENGTH; m++) {
    const half = m % 5 === 0 ? 4 : 1.5;
    (m % 5 === 0 ? thick : thin).push(m, y, -half, m, y, half);
  }
  thin.push(0, y, 0, LENGTH, y, 0); // centre line
  const mk = (arr: number[], color: string, opacity: number) => {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(arr, 3));
    return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
  };
  scene.add(mk(thin, '#ffffff', 0.55), mk(thick, '#ffffff', 0.95));
  // release line
  const rel = new THREE.Mesh(new THREE.PlaneGeometry(0.06, 8), new THREE.MeshBasicMaterial({ color: '#e0645a' }));
  rel.rotation.x = -Math.PI / 2;
  rel.position.set(0, 0.003, 0);
  scene.add(rel);
  for (let m = 5; m <= LENGTH; m += 5) {
    const s = createTextSprite(`${m} m`, { height: 0.32, color: '#1d2430' });
    s.position.set(m, 0.2, -4.4);
    scene.add(s);
  }

  scene.add(new THREE.HemisphereLight('#f2f6ff', '#8a7f6e', 1.1));
  const sun = new THREE.DirectionalLight('#fff3df', 2.0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  const r = 2.5;
  Object.assign(sun.shadow.camera, { left: -r, right: r, top: r, bottom: -r, near: 0.5, far: 30 });
  sun.shadow.bias = -0.0005;
  sun.shadow.normalBias = 0.01;
  scene.add(sun, sun.target);
  const offset = new THREE.Vector3(-4, 10, 3);
  const followLight = (p: THREE.Vector3) => {
    sun.target.position.copy(p);
    sun.position.copy(p).add(offset);
    sun.target.updateMatrixWorld();
  };
  followLight(new THREE.Vector3());
  return { scene, sun, followLight };
}
