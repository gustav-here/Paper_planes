import type { ContactPlane, PhysicsConfig } from '../physics/config';

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
    burnAfterS: 0.2,
  },
};

export const DEFAULT_VENUE: VenueId = 'gym';

export function isVenueId(x: unknown): x is VenueId {
  return x === 'gym' || x === 'moon' || x === 'sun';
}

/** The physics configuration for throwing at a venue (everything else stays as in `base`). */
export function venueConfig(id: VenueId, base: PhysicsConfig): PhysicsConfig {
  const v = VENUES[id];
  return { ...base, g: v.g, rho: v.rho, maxTime: v.maxTime, ground: { ...base.ground, planes: v.planes } };
}
