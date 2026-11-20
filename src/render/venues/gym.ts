import * as THREE from 'three';
import { GYM } from '../../core/game/venues';
import { FLOOR_DEPTH, canvas, distanceOverlay, seeded, texture } from './common';
import type { VenueLook } from './common';

/*
 * A Swedish school sports hall (gymnastiksal): light wooden floor with court markings, walls of
 * vertical pine boards, wall bars along one side, benches, baskets and handball goals at both
 * ends, high windows and ceiling lights. The plane is thrown from x = 0 towards +X.
 */

const L = GYM.frontX - GYM.backX; // 30 m
const W = 2 * GYM.halfWidth; // 16 m
const H = GYM.height; // 7 m
const CX = (GYM.frontX + GYM.backX) / 2; // hall centre (x)

/** Floor: light beech strips and the court lines of a multi-sport hall. */
function floorTexture(): THREE.CanvasTexture {
  const ppm = 64;
  const { c, g } = canvas(L * ppm, W * ppm);
  const X = (x: number) => (x - GYM.backX) * ppm;
  const Z = (z: number) => (z + GYM.halfWidth) * ppm;
  const rnd = seeded(7);
  // wooden strips along the hall
  const strip = 0.07 * ppm;
  for (let y = 0; y < c.height; y += strip) {
    let x = -rnd() * 2 * ppm;
    while (x < c.width) {
      const len = (1.2 + rnd() * 1.8) * ppm;
      const l = 76 + rnd() * 7;
      g.fillStyle = `hsl(${33 + rnd() * 5}, ${38 + rnd() * 10}%, ${l}%)`;
      g.fillRect(x, y, len, strip);
      g.fillStyle = 'rgba(120,85,50,0.18)';
      g.fillRect(x, y, 1, strip);
      x += len;
    }
    g.fillStyle = 'rgba(120,85,50,0.12)';
    g.fillRect(0, y, c.width, 1);
  }
  for (let i = 0; i < 9000; i++) {
    g.fillStyle = `rgba(110,75,40,${0.04 + rnd() * 0.06})`;
    g.fillRect(rnd() * c.width, rnd() * c.height, 2 + rnd() * 10, 1);
  }

  const line = (color: string, width: number, draw: () => void) => {
    g.strokeStyle = color;
    g.lineWidth = width * ppm;
    g.beginPath();
    draw();
    g.stroke();
  };
  const rect = (x0: number, z0: number, x1: number, z1: number) => g.rect(X(x0), Z(z0), X(x1) - X(x0), Z(z1) - Z(z0));
  const circle = (x: number, z: number, r: number, a0 = 0, a1 = 2 * Math.PI) => {
    g.moveTo(X(x) + r * ppm * Math.cos(a0), Z(z) + r * ppm * Math.sin(a0));
    g.arc(X(x), Z(z), r * ppm, a0, a1);
  };

  // handball / floorball (black): the whole floor with 6 m goal areas
  line('#2a2a2a', 0.05, () => {
    rect(GYM.backX + 0.5, -GYM.halfWidth + 0.5, GYM.frontX - 0.5, GYM.halfWidth - 0.5);
    g.moveTo(X(CX), Z(-GYM.halfWidth + 0.5));
    g.lineTo(X(CX), Z(GYM.halfWidth - 0.5));
    for (const [gx, dir] of [
      [GYM.backX + 0.5, 1],
      [GYM.frontX - 0.5, -1],
    ] as const) {
      // goal area: quarter circles of 6 m round the posts (z = ±1.5) joined by a straight line
      const pts: [number, number][] = [];
      for (let k = 0; k <= 16; k++) {
        const a = (Math.PI / 2) * (k / 16);
        pts.push([gx + dir * 6 * Math.sin(a), -1.5 - 6 * Math.cos(a)]);
      }
      for (let k = 0; k <= 16; k++) {
        const a = (Math.PI / 2) * (k / 16);
        pts.push([gx + dir * 6 * Math.cos(a), 1.5 + 6 * Math.sin(a)]);
      }
      pts.forEach(([x, z], i) => (i ? g.lineTo(X(x), Z(z)) : g.moveTo(X(x), Z(z))));
    }
  });
  // basketball (red): 28 × 15 m centred in the hall
  const bx0 = CX - 14;
  const bx1 = CX + 14;
  line('#c8322b', 0.05, () => {
    rect(bx0, -7.5, bx1, 7.5);
    g.moveTo(X(CX), Z(-7.5));
    g.lineTo(X(CX), Z(7.5));
    circle(CX, 0, 1.8);
    for (const [base, dir] of [
      [bx0, 1],
      [bx1, -1],
    ] as const) {
      const hoop = base + dir * 1.575;
      // key (4.9 m wide, 5.8 m deep) and free-throw circle
      rect(Math.min(base, base + dir * 5.8), -2.45, Math.max(base, base + dir * 5.8), 2.45);
      circle(base + dir * 5.8, 0, 1.8);
      // three-point line: straight 0.9 m from the side lines, then an arc of 6.75 m
      const a = Math.asin(6.6 / 6.75);
      const xs = hoop + dir * 6.75 * Math.cos(a);
      g.moveTo(X(base), Z(-6.6));
      g.lineTo(X(xs), Z(-6.6));
      if (dir > 0) g.arc(X(hoop), Z(0), 6.75 * ppm, -a, a);
      else g.arc(X(hoop), Z(0), 6.75 * ppm, Math.PI + a, Math.PI - a, true);
      g.moveTo(X(xs), Z(6.6));
      g.lineTo(X(base), Z(6.6));
    }
  });
  // volleyball (yellow): 18 × 9 m with attack lines
  line('#e2b51c', 0.05, () => {
    rect(CX - 9, -4.5, CX + 9, 4.5);
    for (const d of [-3, 3]) {
      g.moveTo(X(CX + d), Z(-4.5));
      g.lineTo(X(CX + d), Z(4.5));
    }
  });
  // badminton (blue): two courts across the hall
  line('#2f6fd0', 0.04, () => {
    for (const cz of [-3.8, 3.8]) {
      rect(CX - 6.7, cz - 3.05, CX + 6.7, cz + 3.05);
      g.moveTo(X(CX - 1.98), Z(cz - 3.05));
      g.lineTo(X(CX - 1.98), Z(cz + 3.05));
      g.moveTo(X(CX + 1.98), Z(cz - 3.05));
      g.lineTo(X(CX + 1.98), Z(cz + 3.05));
    }
  });
  return texture(c);
}

/** Vertical pine boards with dark grooves. */
function boardTexture(): THREE.CanvasTexture {
  const { c, g } = canvas(512, 512);
  const rnd = seeded(11);
  const n = 6;
  const w = c.width / n;
  for (let i = 0; i < n; i++) {
    g.fillStyle = `hsl(${32 + rnd() * 6}, ${45 + rnd() * 10}%, ${60 + rnd() * 8}%)`;
    g.fillRect(i * w, 0, w, c.height);
    for (let k = 0; k < 40; k++) {
      g.fillStyle = `rgba(120,70,30,${0.05 + rnd() * 0.08})`;
      g.fillRect(i * w + rnd() * w, 0, 1 + rnd() * 2, c.height);
    }
    g.fillStyle = 'rgba(60,35,15,0.55)';
    g.fillRect(i * w, 0, 2, c.height);
  }
  return texture(c, [1, 1]);
}

function board(size: [number, number], texRepeat: [number, number], tex: THREE.CanvasTexture): THREE.Mesh {
  const t = tex.clone();
  t.needsUpdate = true;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(...texRepeat);
  return new THREE.Mesh(new THREE.PlaneGeometry(...size), new THREE.MeshStandardMaterial({ map: t, roughness: 0.8 }));
}

/** Basket on the wall: backboard, orange ring and a net, facing +normal. */
function basket(scale = 1): THREE.Group {
  const grp = new THREE.Group();
  const { c, g } = canvas(256, 150);
  g.fillStyle = '#fafafa';
  g.fillRect(0, 0, 256, 150);
  g.strokeStyle = '#1c1c1c';
  g.lineWidth = 8;
  g.strokeRect(4, 4, 248, 142);
  g.strokeStyle = '#c8322b';
  g.lineWidth = 7;
  g.strokeRect(90, 70, 76, 56);
  const boardMesh = new THREE.Mesh(new THREE.BoxGeometry(1.8, 1.05, 0.04), [
    ...Array.from({ length: 4 }, () => new THREE.MeshStandardMaterial({ color: '#dddddd' })),
    new THREE.MeshStandardMaterial({ map: texture(c), roughness: 0.4 }),
    new THREE.MeshStandardMaterial({ color: '#dddddd' }),
  ]);
  boardMesh.position.set(0, 0.35, 0);
  grp.add(boardMesh);
  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.23, 0.012, 8, 32), new THREE.MeshStandardMaterial({ color: '#e05a1c', metalness: 0.4, roughness: 0.4 }));
  ring.rotation.x = Math.PI / 2;
  ring.position.set(0, 0, 0.25);
  grp.add(ring);
  const net = new THREE.Mesh(
    new THREE.CylinderGeometry(0.23, 0.14, 0.4, 12, 3, true),
    new THREE.MeshBasicMaterial({ color: '#f4f4f4', wireframe: true, transparent: true, opacity: 0.8 }),
  );
  net.position.set(0, -0.2, 0.25);
  grp.add(net);
  const arm = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.25), new THREE.MeshStandardMaterial({ color: '#888888' }));
  arm.position.set(0, -0.05, 0.12);
  grp.add(arm);
  grp.scale.setScalar(scale);
  return grp;
}

/** Handball goal (3 × 2 m) with black and yellow striped posts and a net, opening towards +Z local. */
function goal(): THREE.Group {
  const grp = new THREE.Group();
  const { c, g } = canvas(16, 128);
  for (let i = 0; i < 8; i++) {
    g.fillStyle = i % 2 ? '#1d1d1d' : '#f2c614';
    g.fillRect(0, i * 16, 16, 16);
  }
  const striped = new THREE.MeshStandardMaterial({ map: texture(c), roughness: 0.5 });
  const post = (h: number) => new THREE.Mesh(new THREE.BoxGeometry(0.08, h, 0.08), striped);
  const l = post(2);
  l.position.set(-1.54, 1, 0);
  const r = post(2);
  r.position.set(1.54, 1, 0);
  const top = new THREE.Mesh(new THREE.BoxGeometry(3.16, 0.08, 0.08), striped);
  top.position.set(0, 2.04, 0);
  const net = new THREE.Mesh(
    new THREE.BoxGeometry(3, 2, 0.8, 12, 8, 3),
    new THREE.MeshBasicMaterial({ color: '#d8d8d8', wireframe: true, transparent: true, opacity: 0.35 }),
  );
  net.position.set(0, 1, -0.42);
  grp.add(l, r, top, net);
  return grp;
}

/** A run of wall bars (ribbstolar): posts and round rungs, `sections` × 0.9 m wide. */
function wallBars(sections: number): THREE.Group {
  const grp = new THREE.Group();
  const wood = new THREE.MeshStandardMaterial({ color: '#c99a5c', roughness: 0.6 });
  const width = 0.9;
  const height = 2.6;
  const rungs = 16;
  const post = new THREE.BoxGeometry(0.05, height, 0.08);
  const posts = new THREE.InstancedMesh(post, wood, sections + 1);
  const m = new THREE.Matrix4();
  for (let i = 0; i <= sections; i++) {
    m.makeTranslation(i * width, height / 2, 0);
    posts.setMatrixAt(i, m);
  }
  const rung = new THREE.CylinderGeometry(0.017, 0.017, width - 0.05, 8);
  rung.rotateZ(Math.PI / 2);
  const bars = new THREE.InstancedMesh(rung, wood, sections * rungs);
  let k = 0;
  for (let i = 0; i < sections; i++)
    for (let j = 0; j < rungs; j++) {
      m.makeTranslation(i * width + width / 2, 0.12 + j * ((height - 0.2) / (rungs - 1)), 0);
      bars.setMatrixAt(k++, m);
    }
  grp.add(posts, bars);
  return grp;
}

function bench(length: number): THREE.Group {
  const grp = new THREE.Group();
  const wood = new THREE.MeshStandardMaterial({ color: '#b98b52', roughness: 0.6 });
  const seat = new THREE.Mesh(new THREE.BoxGeometry(length, 0.04, 0.24), wood);
  seat.position.y = 0.32;
  grp.add(seat);
  for (const x of [-length / 2 + 0.3, length / 2 - 0.3]) {
    const leg = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.3, 0.3), wood);
    leg.position.set(x, 0.15, 0);
    grp.add(leg);
  }
  return grp;
}

export function buildGym(): VenueLook {
  const grp = new THREE.Group();

  const floor = new THREE.Mesh(new THREE.PlaneGeometry(L, W), new THREE.MeshStandardMaterial({ map: floorTexture(), roughness: 0.55, ...FLOOR_DEPTH }));
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(CX, 0, 0);
  floor.receiveShadow = true;
  grp.add(floor);
  grp.add(
    distanceOverlay({ length: GYM.frontX - 1, step: 1, labelStep: 1, majorStep: 5, halfWidth: 2.2, color: '#ffffff', ink: '#ffffff', outline: 'rgba(30,25,20,0.85)' }),
  );

  // walls of vertical boards (each faces into the hall)
  const boards = boardTexture();
  const wallTile = 0.9; // metres per texture repeat
  const side = (len: number) => board([len, H], [len / wallTile, 1], boards);
  const back = side(W);
  back.position.set(GYM.backX, H / 2, 0);
  back.rotation.y = Math.PI / 2;
  const front = side(W);
  front.position.set(GYM.frontX, H / 2, 0);
  front.rotation.y = -Math.PI / 2;
  const left = side(L);
  left.position.set(CX, H / 2, -GYM.halfWidth);
  const right = side(L);
  right.position.set(CX, H / 2, GYM.halfWidth);
  right.rotation.y = Math.PI;
  for (const w of [back, front, left, right]) w.receiveShadow = true;
  grp.add(back, front, left, right);

  // ceiling, beams and lights
  const ceiling = new THREE.Mesh(new THREE.PlaneGeometry(L, W), new THREE.MeshStandardMaterial({ color: '#d9d4c8', roughness: 0.9 }));
  ceiling.rotation.x = Math.PI / 2;
  ceiling.position.set(CX, H, 0);
  grp.add(ceiling);
  const beamMat = new THREE.MeshStandardMaterial({ color: '#c79a62', roughness: 0.7 });
  const lightMat = new THREE.MeshBasicMaterial({ color: '#fffaf0' });
  for (let x = GYM.backX + 2.5; x < GYM.frontX; x += 5) {
    const beam = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.45, W), beamMat);
    beam.position.set(x, H - 0.22, 0);
    grp.add(beam);
    for (const z of [-4.5, 0, 4.5]) {
      const lamp = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.05, 0.3), lightMat);
      lamp.position.set(x + 2.5, H - 0.5, z);
      grp.add(lamp);
    }
  }

  // high windows on the left wall
  const glass = new THREE.MeshBasicMaterial({ color: '#dfeefa' });
  const frame = new THREE.MeshStandardMaterial({ color: '#8a6a45' });
  for (let x = GYM.backX + 3; x < GYM.frontX - 2; x += 4) {
    const fr = new THREE.Mesh(new THREE.BoxGeometry(3.2, 1.7, 0.05), frame);
    fr.position.set(x + 1.5, 5.4, -GYM.halfWidth + 0.03);
    const gl = new THREE.Mesh(new THREE.PlaneGeometry(3, 1.5), glass);
    gl.position.set(x + 1.5, 5.4, -GYM.halfWidth + 0.06);
    grp.add(fr, gl);
  }

  // baskets: one at each end, folded side baskets along the right wall
  const endBasketB = basket();
  endBasketB.position.set(GYM.backX + 0.05, 3.05, 0);
  endBasketB.rotation.y = Math.PI / 2;
  const endBasketF = basket();
  endBasketF.position.set(GYM.frontX - 0.05, 3.05, 0);
  endBasketF.rotation.y = -Math.PI / 2;
  grp.add(endBasketB, endBasketF);
  for (const x of [CX - 7, CX + 7]) {
    const b = basket(0.6);
    b.position.set(x, 3.4, GYM.halfWidth - 0.05);
    b.rotation.y = Math.PI;
    grp.add(b);
  }

  // handball goals against both end walls
  const gB = goal();
  gB.position.set(GYM.backX + 0.45, 0, 0);
  gB.rotation.y = Math.PI / 2;
  const gF = goal();
  gF.position.set(GYM.frontX - 0.45, 0, 0);
  gF.rotation.y = -Math.PI / 2;
  grp.add(gB, gF);

  // wall bars and benches along the right wall
  const bars = wallBars(22);
  bars.position.set(GYM.backX + 4, 0, GYM.halfWidth - 0.12);
  grp.add(bars);
  for (const x of [CX - 6, CX + 1, CX + 8]) {
    const b = bench(4);
    b.position.set(x, 0, GYM.halfWidth - 0.6);
    grp.add(b);
  }

  // mats leaning against the back wall, a door in the front wall
  const matColors = ['#c8322b', '#2c4fa8', '#2c4fa8'];
  matColors.forEach((col, i) => {
    const mat = new THREE.Mesh(new THREE.BoxGeometry(0.15, 1.8, 1.0), new THREE.MeshStandardMaterial({ color: col, roughness: 0.7 }));
    mat.position.set(GYM.backX + 0.12, 0.9, -5 + i * 1.1);
    mat.rotation.z = -0.06;
    grp.add(mat);
  });
  const door = new THREE.Mesh(new THREE.BoxGeometry(0.06, 2.1, 1.0), new THREE.MeshStandardMaterial({ color: '#e5b53a', roughness: 0.6 }));
  door.position.set(GYM.frontX - 0.03, 1.05, -5.5);
  grp.add(door);

  return {
    group: grp,
    background: new THREE.Color('#c8a777'),
    fog: null,
    key: { color: '#fff4e2', intensity: 1.6, offset: new THREE.Vector3(-1.5, 6, 1.5) },
    fill: { sky: '#fff6e8', ground: '#b48a5a', intensity: 1.35 },
  };
}
