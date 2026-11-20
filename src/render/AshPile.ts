import * as THREE from 'three';

/**
 * What is left of a sheet that burned up: a small, loose heap of grey-black ash flakes on the
 * table (about 4 cm across). Model-space group at the origin, y up from the table.
 */
export function createAshPile(): THREE.Group {
  const grp = new THREE.Group();
  let s = 99;
  const rnd = () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  const n = 1400;
  const pos = new Float32Array(n * 3);
  const col = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    // denser and higher in the middle (a low cone)
    const r = 0.022 * Math.sqrt(rnd()) * (0.6 + 0.4 * rnd());
    const a = rnd() * Math.PI * 2;
    const h = Math.max(0, 0.009 * (1 - r / 0.024) * (0.4 + 0.6 * rnd()));
    pos.set([Math.cos(a) * r, h, Math.sin(a) * r], i * 3);
    const g = 0.05 + rnd() * 0.22;
    col.set([g, g * 0.95, g * 0.9], i * 3);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  grp.add(new THREE.Points(geo, new THREE.PointsMaterial({ size: 0.0016, vertexColors: true, sizeAttenuation: true })));
  // a soft dark smudge under the heap
  const smudge = new THREE.Mesh(
    new THREE.CircleGeometry(0.03, 32),
    new THREE.MeshBasicMaterial({ color: '#1b1814', transparent: true, opacity: 0.35, depthWrite: false }),
  );
  smudge.rotation.x = -Math.PI / 2;
  smudge.position.y = 0.0004;
  grp.add(smudge);
  // a few larger curled flakes
  const flakeMat = new THREE.MeshStandardMaterial({ color: '#2a2622', roughness: 1, side: THREE.DoubleSide });
  for (let i = 0; i < 14; i++) {
    const f = new THREE.Mesh(new THREE.CircleGeometry(0.002 + rnd() * 0.004, 5), flakeMat);
    const r = 0.018 * Math.sqrt(rnd());
    const a = rnd() * Math.PI * 2;
    f.position.set(Math.cos(a) * r, 0.002 + rnd() * 0.005, Math.sin(a) * r);
    f.rotation.set(rnd() * Math.PI, rnd() * Math.PI, rnd() * Math.PI);
    grp.add(f);
  }
  return grp;
}
