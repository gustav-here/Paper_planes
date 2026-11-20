/**
 * Every physical constant used by the flight model. All values are GLOBAL: nothing here may
 * depend on which aeroplane is flying. Behaviour must emerge from geometry and mass.
 */
export interface AeroConfig {
  /** 2D lift-curve slope of a thin flat plate (1/rad). Thin-airfoil theory gives 2π; thin plates
   *  at Re ≈ 3·10⁴ reach roughly 90 % of that (Pelletier & Mueller 2000). */
  a0: number;
  /** Stall angle of a thin flat plate at low Reynolds number (deg). Thin plates stall early,
   *  ≈ 8–12° at Re 3·10⁴–10⁵ (Pelletier & Mueller 2000; Okamoto & Azuma). */
  alphaStallDeg: number;
  /** Sharpness of the attached → separated blend σ(α) = 1/(1+e^(−M(α−α_s))) (1/rad). */
  stallBlendM: number;
  /** Normal-force coefficient of a plate broadside to the flow. Hoerner: ≈1.17 for a square
   *  plate up to 1.98 for an infinite one; paper-plane surfaces have low aspect ratio. */
  CN90: number;
  /** Zero-lift drag (skin friction on both sides, laminar, Re ≈ 3·10⁴: 2·1.328/√Re ≈ 0.015, plus edges). */
  CD0: number;
  /** Oswald span efficiency for induced drag. */
  oswaldE: number;
  /** Centre-of-pressure shift for separated flow: x_cp = 0.5 − Δ·cos α
   *  (≈0.34c just past stall → 0.5c at 90°, Fage & Johansen 1927 trend). */
  sepCopShift: number;
  /** Size of the square grid used to split surfaces into aerodynamic elements (m). */
  elementSizeM: number;
  /** Surfaces whose planes are within this angle act as one lifting system for aspect ratio (deg). */
  liftingSystemAngleDeg: number;
  /** Recompute chord coordinates when the in-plane flow direction turns more than this (deg). */
  chordCacheDeg: number;
}

export interface GroundConfig {
  /** Coefficient of restitution, paper on a hard floor (barely bounces). */
  restitution: number;
  /** Below this approach speed (m/s) contacts are treated as inelastic (avoids jitter). */
  restitutionThreshold: number;
  /** Coulomb friction, paper on wood/linoleum (≈0.3–0.5). */
  friction: number;
  /** Rest detection: |v| below this (m/s) … */
  restSpeed: number;
  /** … and |ω| below this (rad/s) … */
  restSpin: number;
  /** … for this long (s). */
  restTime: number;
  /** Sequential impulse iterations per step. */
  iterations: number;
}

export interface PhysicsConfig {
  /** Air density, ISA sea level (kg/m³). */
  rho: number;
  /** Gravitational acceleration (m/s²). */
  g: number;
  /** Fixed physics time step (s). */
  dt: number;
  /** RK4 sub-steps per fixed step. */
  substeps: number;
  /** Include added mass (fluid inertia) and the Munk moment. */
  addedMass: boolean;
  /**
   * Include the Munk moment (−v × p from the added-mass Kirchhoff terms). OFF by default: for a
   * thin plate the steady potential-flow moment about mid-chord equals the Munk moment, and the
   * quarter-chord loading in the aero model already contains it — including both double-counts a
   * destabilizing moment (observed: violent pitch-up). Added-mass inertia is still included.
   */
  munkMoment: boolean;
  /** 3D correction for added mass: κ(AR) = AR/√(1+AR²) (Jones-type low-AR correction). Can be scaled. */
  addedMassScale: number;
  aero: AeroConfig;
  ground: GroundConfig;
  /** Flight log sampling interval (s). */
  logInterval: number;
  /** Give up after this simulated time (s). */
  maxTime: number;
}

export const DEFAULT_CONFIG: PhysicsConfig = {
  rho: 1.225,
  g: 9.81,
  dt: 1 / 240,
  substeps: 1,
  addedMass: true,
  munkMoment: false,
  addedMassScale: 1,
  aero: {
    a0: 2 * Math.PI * 0.9,
    alphaStallDeg: 10,
    stallBlendM: 25,
    CN90: 1.2,
    CD0: 0.02,
    oswaldE: 0.8,
    sepCopShift: 0.16,
    elementSizeM: 0.01,
    liftingSystemAngleDeg: 45,
    chordCacheDeg: 1,
  },
  ground: {
    restitution: 0.15,
    restitutionThreshold: 0.3,
    friction: 0.4,
    restSpeed: 0.02,
    restSpin: 0.1,
    restTime: 0.5,
    iterations: 10,
  },
  logInterval: 1 / 60,
  maxTime: 30,
};

/** Deep-merge partial overrides (for tests and the debug panel). */
export function makeConfig(over: Partial<Omit<PhysicsConfig, 'aero' | 'ground'>> & {
  aero?: Partial<AeroConfig>;
  ground?: Partial<GroundConfig>;
} = {}): PhysicsConfig {
  return {
    ...DEFAULT_CONFIG,
    ...over,
    aero: { ...DEFAULT_CONFIG.aero, ...over.aero },
    ground: { ...DEFAULT_CONFIG.ground, ...over.ground },
  };
}
