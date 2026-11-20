import type { Vec3 } from '../math/vec3';
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

/** A solid surface: the free space is where n·x ≥ d (n: unit normal pointing into the room). */
export interface ContactPlane {
  n: Vec3;
  d: number;
}

export interface GroundConfig {
  /** Solid surfaces the paper can hit: the floor (y ≥ 0), and walls and a ceiling indoors. */
  planes: ContactPlane[];
  /** Coefficient of restitution, paper on a hard floor (barely bounces). */
  restitution: number;
  /** Below this approach speed (m/s) contacts are treated as inelastic (avoids jitter). */
  restitutionThreshold: number;
  /** Coulomb friction, paper on wood/linoleum (≈0.3–0.5). */
  friction: number;
  /**
   * Loose ground (kg/m): each touching point of the paper digs in and pushes the loose material
   * ahead of it, a resistance ρ·A·v_t² (density of the material times the cross-section the edge
   * ploughs through). It does not depend on gravity. 0 for a hard floor.
   */
  ploughKgPerM: number;
  /** Rest detection: |v| below this (m/s) … */
  restSpeed: number;
  /** … and |ω| below this (rad/s) … */
  restSpin: number;
  /** … for this long (s). */
  restTime: number;
  /** Sequential impulse iterations per step. */
  iterations: number;
}

/**
 * Paper strength and stiffness ("realistic" setting chosen by the player). Damage is permanent
 * until the plane is refolded.
 */
export interface PaperConfig {
  /** Rotational stiffness of a single-layer crease per metre of crease (N·m/rad per m).
   *  80 g/m² paper: bending stiffness D ≈ E t³/12 ≈ 2.5·10⁻⁴ N·m over a few-mm crease zone. */
  creaseStiffnessPerM: number;
  /** Bending stiffness of flat, uncreased 80 g/m² paper, D ≈ E t³/12 with E ≈ 3 GPa, t ≈ 0.1 mm (N·m). */
  bendingStiffness: number;
  /** Large flat panels bend smoothly (bending shapes, flexible paper; see bendModes.ts). */
  bendingModes: boolean;
  /**
   * The bending shapes are linear (the paper moves straight along its normal), which only holds
   * for moderate slopes: real paper cannot stretch, so a strongly bent wing curls in and turns
   * away from the air instead. Beyond about this slope (degrees) at the panel's steepest point
   * the shape stiffens quickly (by `bendingLimitStiffening` × its own stiffness at that slope,
   * growing with the 6th power of the amplitude).
   */
  bendingLimitSlopeDeg: number;
  bendingLimitStiffening: number;
  /** Width of the zone a crease bends over (m); creaseStiffnessPerM ≈ D / creaseZoneM. */
  creaseZoneM: number;
  /**
   * Springback: the fraction of its fold angle a released crease opens again. Copy paper creased
   * firmly (thumbnail-pressed) keeps most of a fold; a 180° fold then gapes ≈ 9° (0.05 · 180°).
   * Estimate from hand-folding; to be calibrated against real planes.
   */
  creaseSpringback: number;
  /** Folded layers that close onto each other meet a contact spring this many times stiffer than the crease. */
  contactStiffnessFactor: number;
  /** Structural damping of creases as a time constant (s): c = k·τ. */
  creaseDampingS: number;
  /** A crease bent further than this from its rest angle yields permanently (deg). */
  creaseYieldDeg: number;
  /** Control flaps are driven like a stiff servo spring: this many times the crease stiffness. */
  servoStiffnessFactor: number;
  /** Normal impact speed of a point on the floor above which the paper there crumples (m/s). */
  crumpleSpeed: number;
  /**
   * Crush depth: this much at the threshold, plus crumpleDepthPerMs per m/s above it, at most
   * crumpleMaxDepth (m). A nose-first crash folds up only the very tip of the paper.
   */
  crumpleMinDepth: number;
  crumpleDepthPerMs: number;
  crumpleMaxDepth: number;
  /**
   * The crushed edge buckles into an accordion over a zone this many times longer than the
   * crush depth (crumpled paper folds in short zigzags), …
   */
  crumpleZoneFactor: number;
  /** … of this many alternating folds. */
  crumpleFolds: number;
  /** A cut end within this distance of a crumpling impact tears further (m). */
  tearImpactRadius: number;
  /** Crease moment per metre of crease above which a cut end on it tears (N·m/m). */
  tearMomentPerM: number;
  /** Tear length per unit excess (m per relative excess), and its maximum (m). */
  tearLengthGain: number;
  tearMaxLength: number;
  /** Record damage at all (off = purely cosmetic is not offered; this switches it off entirely). */
  damage: boolean;
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
  paper: PaperConfig;
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
    planes: [{ n: [0, 1, 0], d: 0 }],
    restitution: 0.15,
    restitutionThreshold: 0.3,
    friction: 0.4,
    ploughKgPerM: 0,
    restSpeed: 0.02,
    restSpin: 0.1,
    restTime: 0.5,
    iterations: 10,
  },
  paper: {
    creaseStiffnessPerM: 0.08,
    bendingStiffness: 2.5e-4,
    bendingModes: true,
    bendingLimitSlopeDeg: 35,
    bendingLimitStiffening: 30,
    creaseZoneM: 0.003,
    creaseSpringback: 0.05,
    contactStiffnessFactor: 30,
    creaseDampingS: 0.01,
    creaseYieldDeg: 15,
    servoStiffnessFactor: 20,
    crumpleSpeed: 4.5,
    crumpleMinDepth: 0.002,
    crumpleDepthPerMs: 0.0012,
    crumpleMaxDepth: 0.008,
    crumpleZoneFactor: 1.6,
    crumpleFolds: 3,
    tearImpactRadius: 0.015,
    tearMomentPerM: 0.02,
    tearLengthGain: 0.01,
    tearMaxLength: 0.015,
    damage: true,
  },
  logInterval: 1 / 60,
  maxTime: 30,
};

/** Deep-merge partial overrides (for tests and the debug panel). */
export function makeConfig(over: Partial<Omit<PhysicsConfig, 'aero' | 'ground' | 'paper'>> & {
  aero?: Partial<AeroConfig>;
  ground?: Partial<GroundConfig>;
  paper?: Partial<PaperConfig>;
} = {}): PhysicsConfig {
  return {
    ...DEFAULT_CONFIG,
    ...over,
    aero: { ...DEFAULT_CONFIG.aero, ...over.aero },
    ground: { ...DEFAULT_CONFIG.ground, ...over.ground },
    paper: { ...DEFAULT_CONFIG.paper, ...over.paper },
  };
}
