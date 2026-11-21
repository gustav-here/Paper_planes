import type { ContactPlane, GroundConfig, PhysicsConfig } from '../physics/config';

/*
 * Places to throw. Each one is only the world around the plane: gravity, the air and the solid
 * surfaces. The plane and the physics are the same everywhere.
 */

export type VenueId = 'gym' | 'moon' | 'sun';

export interface Venue {
  id: VenueId;
  name: string;
  /** Gravitational acceleration (m/s²). */
  g: number;
  /** Air density (kg/m³); 0 = vacuum (no lift, no drag, no added mass: purely ballistic). */
  rho: number;
  /** Solid surfaces (free space where n·x ≥ d). */
  planes: ContactPlane[];
  /** How the ground takes a landing paper, where it differs from the hall floor. */
  ground?: Partial<Pick<GroundConfig, 'friction' | 'restitution' | 'ploughKgPerM' | 'spinDampingPerS'>>;
  /** Simulated time limit (s). */
  maxTime: number;
  /** The paper burns up this long after the throw (s), or never. */
  burnAfterS: number | null;
}

/**
 * School sports hall (gymnastiksal), 30 × 16 m, 7 m to the ceiling. The plane is thrown from
 * 3 m in front of one end wall, along the middle of the hall towards the other end.
 */
export const GYM = {
  backX: -3,
  frontX: 27,
  halfWidth: 8,
  height: 7,
} as const;

export const VENUES: Record<VenueId, Venue> = {
  gym: {
    id: 'gym',
    name: 'Gymnastiksal',
    g: 9.81,
    rho: 1.225,
    planes: [
      { n: [0, 1, 0], d: 0 },
      { n: [0, -1, 0], d: -GYM.height },
      { n: [1, 0, 0], d: GYM.backX },
      { n: [-1, 0, 0], d: -GYM.frontX },
      { n: [0, 0, 1], d: -GYM.halfWidth },
      { n: [0, 0, -1], d: -GYM.halfWidth },
    ],
    maxTime: 30,
    burnAfterS: null,
  },
  moon: {
    id: 'moon',
    name: 'Månen',
    // lunar surface gravity; no atmosphere at all
    g: 1.62,
    rho: 0,
    planes: [{ n: [0, 1, 0], d: 0 }],
    // loose, fluffy regolith (≈ 1500 kg/m³ at the top): an edge of the paper digs ≈ 3 mm into it
    // over ≈ 1 cm, ploughing 1500 · 3·10⁻⁵ ≈ 0.05 kg/m ahead of it. That stops a landing paper
    // within centimetres, where friction alone (μ·g in the weak gravity) would let it slide on
    // for tens of metres. The sharp grains grip the fibres (μ ≈ 1) and the dust takes up the bounce.
    // The dust also closes round the paper and takes up its spin, so a plane landing nose
    // first does not pole-vault over its nose in the weak gravity; it slides a few centimetres.
    ground: { friction: 1, restitution: 0, ploughKgPerM: 0.05, spinDampingPerS: 150 },
    // a hard throw stays up for over ten seconds
    maxTime: 90,
    burnAfterS: null,
  },
  sun: {
    id: 'sun',
    name: 'Solen',
    // surface gravity of the Sun; the photosphere is gas (≈ 2·10⁻⁴ kg/m³), there is no floor
    g: 274,
    rho: 2e-4,
    planes: [],
    maxTime: 1,
    // at about 5800 K the paper is ash the moment it leaves the hand
    burnAfterS: 0,
  },
};

export const DEFAULT_VENUE: VenueId = 'gym';

export function isVenueId(x: unknown): x is VenueId {
  return x === 'gym' || x === 'moon' || x === 'sun';
}

/** The physics configuration for throwing at a venue (everything else stays as in `base`). */
export function venueConfig(id: VenueId, base: PhysicsConfig): PhysicsConfig {
  const v = VENUES[id];
  return { ...base, g: v.g, rho: v.rho, maxTime: v.maxTime, ground: { ...base.ground, ...v.ground, planes: v.planes } };
}
