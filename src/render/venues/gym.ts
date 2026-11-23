import * as THREE from 'three';
import { GYM } from '../../core/game/venues';
import { FLOOR_DEPTH, canvas, distanceOverlay, seeded, texture } from './common';
import type { VenueLook } from './common';

/*
 * A Swedish sports hall (sporthall): blue sports floor with court markings for handball,
 * basketball, volleyball and badminton, wooden panels up to head height and light walls above,
 * glulam beams under the ceiling, baskets and handball goals at both ends, wall bars and benches
 * along one side and high windows along the other. The plane is thrown from x = 0 towards +X.
 */

const L = GYM.frontX - GYM.backX; // 46 m
const W = 2 * GYM.halfWidth; // 26 m
const H = GYM.height; // 9 m
const CX = (GYM.frontX + GYM.backX) / 2; // hall centre (x)
/** Height of the wooden panels on the walls (m). */
const PANEL_H = 2.6;

/** Floor: blue polyurethane sports floor and the court lines of a multi-sport hall. */
function floorTexture(): THREE.CanvasTexture {
  const ppm = 48;
  const { c, g } = canvas(L * ppm, W * ppm);
  const X = (x: number) => (x - GYM.backX) * ppm;
  const Z = (z: number) => (z + GYM.halfWidth) * ppm;
  // mulberry32: the hall's simple LCG puts pairs of numbers on a lattice, and on a plain floor
  // the mottling then shows up as a grid of lines
  let seed = 7;
  const rnd = () => {
    seed = (seed + 0x6d2b79f5) >>> 0;
    let t = Math.imul(seed ^ (seed >>> 15), seed | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  g.fillStyle = '#5f86b8';
  g.fillRect(0, 0, c.width, c.height);
  // a cast floor: no strips, only faint mottling and wear
  for (let i = 0; i < 20000; i++) {
    const l = rnd() < 0.5 ? 0 : 255;
    g.fillStyle = `rgba(${l},${l},${l},${0.01 + rnd() * 0.02})`;
    g.beginPath();
    g.arc(rnd() * c.width, rnd() * c.height, (0.03 + rnd() * 0.2) * ppm, 0, 2 * Math.PI);
    g.fill();
  }

  const line = (color: string, width: number, draw: () => void, dash: number[] = []) => {
    g.strokeStyle = color;
    g.lineWidth = width * ppm;
    g.setLineDash(dash.map((d) => d * ppm));
    g.beginPath();
    draw();
    g.stroke();
    g.setLineDash([]);
  };
  const rect = (x0: number, z0: number, x1: number, z1: number) => g.rect(X(x0), Z(z0), X(x1) - X(x0), Z(z1) - Z(z0));
  const circle = (x: number, z: number, r: number, a0 = 0, a1 = 2 * Math.PI) => {
    g.moveTo(X(x) + r * ppm * Math.cos(a0), Z(z) + r * ppm * Math.sin(a0));
    g.arc(X(x), Z(z), r * ppm, a0, a1);
  };
  /** Handball area line: quarter circles of radius r round the posts (z = ±1.5) and a straight line. */
  const area = (gx: number, dir: 1 | -1, r: number) => {
    const pts: [number, number][] = [];
    for (let k = 0; k <= 16; k++) {
      const a = (Math.PI / 2) * (k / 16);
      pts.push([gx + dir * r * Math.sin(a), -1.5 - r * Math.cos(a)]);
    }
    for (let k = 0; k <= 16; k++) {
      const a = (Math.PI / 2) * (k / 16);
      pts.push([gx + dir * r * Math.cos(a), 1.5 + r * Math.sin(a)]);
    }
    pts.forEach(([x, z], i) => (i ? g.lineTo(X(x), Z(z)) : g.moveTo(X(x), Z(z))));
  };

  // handball (white): 40 × 20 m with 6 m goal areas and dashed 9 m lines
  const hx0 = CX - 20;
  const hx1 = CX + 20;
  line('#f4f4f2', 0.05, () => {
    rect(hx0, -10, hx1, 10);
    g.moveTo(X(CX), Z(-10));
    g.lineTo(X(CX), Z(10));
    area(hx0, 1, 6);
    area(hx1, -1, 6);
  });
  line('#f4f4f2', 0.05, () => {
    area(hx0, 1, 9);
    area(hx1, -1, 9);
  }, [0.15, 0.15]);
  // basketball (red): 28 × 15 m centred in the hall
  const bx0 = CX - 14;
  const bx1 = CX + 14;
  line('#d2352c', 0.05, () => {
    rect(bx0, -7.5, bx1, 7.5);
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
  line('#f0c419', 0.05, () => {
    rect(CX - 9, -4.5, CX + 9, 4.5);
    for (const d of [-3, 3]) {
      g.moveTo(X(CX + d), Z(-4.5));
      g.lineTo(X(CX + d), Z(4.5));
    }
  });
  // badminton (green): four courts, two in each half of the hall
  line('#3f9a5a', 0.04, () => {
    for (const cx of [CX - 12, CX + 12])
      for (const cz of [-4, 4]) {
        rect(cx - 6.7, cz - 3.05, cx + 6.7, cz + 3.05);
        g.moveTo(X(cx), Z(cz - 3.05));
        g.lineTo(X(cx), Z(cz + 3.05));
        for (const d of [-1.98, 1.98]) {
          g.moveTo(X(cx + d), Z(cz - 3.05));
          g.lineTo(X(cx + d), Z(cz + 3.05));
        }
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

  const floor = new THREE.Mesh(new THREE.PlaneGeometry(L, W), new THREE.MeshStandardMaterial({ map: floorTexture(), roughness: 0.45, ...FLOOR_DEPTH }));
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(CX, 0, 0);
  floor.receiveShadow = true;
  grp.add(floor);
  grp.add(
    distanceOverlay({ length: GYM.frontX - 1, step: 1, labelStep: 1, majorStep: 5, halfWidth: 2.2, color: '#ffffff', ink: '#ffffff', outline: 'rgba(20,30,50,0.85)' }),
  );

  // walls: vertical wooden boards up to PANEL_H, light painted wall above (each faces into the hall)
  const boards = boardTexture();
  const wallTile = 0.9; // metres per texture repeat
  const upperMat = new THREE.MeshStandardMaterial({ color: '#ece8de', roughness: 0.9 });
  const wall = (len: number, x: number, z: number, rotY: number) => {
    const lower = board([len, PANEL_H], [len / wallTile, 1], boards);
    lower.position.set(x, PANEL_H / 2, z);
    lower.rotation.y = rotY;
    const upper = new THREE.Mesh(new THREE.PlaneGeometry(len, H - PANEL_H), upperMat);
    upper.position.set(x, (H + PANEL_H) / 2, z);
    upper.rotation.y = rotY;
    lower.receiveShadow = upper.receiveShadow = true;
    grp.add(lower, upper);
  };
  wall(W, GYM.backX, 0, Math.PI / 2);
  wall(W, GYM.frontX, 0, -Math.PI / 2);
  wall(L, CX, -GYM.halfWidth, 0);
  wall(L, CX, GYM.halfWidth, Math.PI);
  // a wooden rail where the panels end
  const railMat = new THREE.MeshStandardMaterial({ color: '#b9874c', roughness: 0.6 });
  for (const z of [-GYM.halfWidth + 0.03, GYM.halfWidth - 0.03]) {
    const rail = new THREE.Mesh(new THREE.BoxGeometry(L, 0.08, 0.06), railMat);
    rail.position.set(CX, PANEL_H, z);
    grp.add(rail);
  }
  for (const x of [GYM.backX + 0.03, GYM.frontX - 0.03]) {
    const rail = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.08, W), railMat);
    rail.position.set(x, PANEL_H, 0);
    grp.add(rail);
  }

  // ceiling of light acoustic panels, glulam beams across the hall and rows of lights between them
  const ceiling = new THREE.Mesh(new THREE.PlaneGeometry(L, W), new THREE.MeshStandardMaterial({ color: '#e4e2dc', roughness: 0.95 }));
  ceiling.rotation.x = Math.PI / 2;
  ceiling.position.set(CX, H, 0);
  grp.add(ceiling);
  const beamMat = new THREE.MeshStandardMaterial({ color: '#c98f4e', roughness: 0.6 });
  const lightMat = new THREE.MeshBasicMaterial({ color: '#fffaf0' });
  const span = 6;
  for (let x = GYM.backX + span / 2; x < GYM.frontX; x += span) {
    const beam = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.9, W), beamMat);
    beam.position.set(x, H - 0.45, 0);
    grp.add(beam);
    if (x + span / 2 >= GYM.frontX) continue;
    for (let z = -GYM.halfWidth + 3; z <= GYM.halfWidth - 3; z += 4) {
      const lamp = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.06, 0.25), lightMat);
      lamp.position.set(x + span / 2, H - 0.3, z);
      grp.add(lamp);
    }
  }

  // high windows along the left wall
  const glass = new THREE.MeshBasicMaterial({ color: '#dfeefa' });
  const frame = new THREE.MeshStandardMaterial({ color: '#f4f4f0' });
  for (let x = GYM.backX + span / 2 + 0.8; x + 4.4 < GYM.frontX; x += span) {
    const fr = new THREE.Mesh(new THREE.BoxGeometry(4.4, 1.9, 0.05), frame);
    fr.position.set(x + 2.2, H - 2, -GYM.halfWidth + 0.03);
    const gl = new THREE.Mesh(new THREE.PlaneGeometry(4.2, 1.7), glass);
    gl.position.set(x + 2.2, H - 2, -GYM.halfWidth + 0.06);
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
  for (const x of [CX - 12, CX, CX + 12]) {
    const b = basket(0.6);
    b.position.set(x, 3.6, GYM.halfWidth - 0.05);
    b.rotation.y = Math.PI;
    grp.add(b);
  }

  // handball goals pushed back against the end walls (the throw line is the near goal line)
  const gB = goal();
  gB.position.set(GYM.backX + 0.45, 0, 0);
  gB.rotation.y = Math.PI / 2;
  const gF = goal();
  gF.position.set(GYM.frontX - 0.45, 0, 0);
  gF.rotation.y = -Math.PI / 2;
  grp.add(gB, gF);

  // wall bars and benches along the right wall
  const bars = wallBars(40);
  bars.position.set(CX - 18, 0, GYM.halfWidth - 0.12);
  grp.add(bars);
  for (const x of [CX - 15, CX - 5, CX + 5, CX + 15]) {
    const b = bench(4);
    b.position.set(x, 0, GYM.halfWidth - 0.6);
    grp.add(b);
  }

  // mats leaning against the back wall, doors in the front and left walls
  const matColors = ['#c8322b', '#2c4fa8', '#2c4fa8'];
  matColors.forEach((col, i) => {
    const mat = new THREE.Mesh(new THREE.BoxGeometry(0.15, 1.8, 1.0), new THREE.MeshStandardMaterial({ color: col, roughness: 0.7 }));
    mat.position.set(GYM.backX + 0.12, 0.9, -7 + i * 1.1);
    mat.rotation.z = -0.06;
    grp.add(mat);
  });
  const doorMat = new THREE.MeshStandardMaterial({ color: '#3a3a3a', roughness: 0.6 });
  for (const z of [-8, 8]) {
    const door = new THREE.Mesh(new THREE.BoxGeometry(0.06, 2.1, 1.2), doorMat);
    door.position.set(GYM.frontX - 0.03, 1.05, z);
    grp.add(door);
  }
  for (const x of [CX - 10, CX + 10]) {
    const door = new THREE.Mesh(new THREE.BoxGeometry(1.2, 2.1, 0.06), doorMat);
    door.position.set(x, 1.05, -GYM.halfWidth + 0.03);
    grp.add(door);
  }

  return {
    group: grp,
    background: new THREE.Color('#d6cdbb'),
    fog: null,
    key: { color: '#fff6e8', intensity: 1.6, offset: new THREE.Vector3(-1.5, 6, 1.5) },
    fill: { sky: '#fff8ee', ground: '#7d95b4', intensity: 1.4 },
  };
}
