import type { PaperModel } from '../paper/model';
import type { DesignFile } from '../paper/ops';
import { replayDesign } from '../paper/replay';
import { designSeed, replayCrooked } from '../paper/crooked';
import { buildAirframe } from './airframe';
import type { Airframe } from './airframe';
import type { PhysicsConfig } from './config';
import type { ControlInput } from './controls';
import type { LaunchParams } from './launch';
import { FlightSim } from './simulation';
import { MultiBodySim, buildFlexAirframe } from './multibody';
import type { FlexAirframe } from './multibody';
import type { Simulator } from './simulator';
import { slitTabs, splitAtTabs } from './slits';
import { relaxCreases } from '../paper/springback';

/** Everything built once per design (shared by all throws of that design). */
export interface Prepared {
  design: DesignFile;
  /** The plane as it flies: folded, then relaxed by crease springback (and split at slit tabs). */
  model: PaperModel;
  /** The plane exactly as folded (what the design ops produce; damage is expressed on it). */
  folded: PaperModel;
  af: Airframe;
  /** Hinged panels, when the design is flexible paper. */
  flex: FlexAirframe | null;
}

export interface PrepareOptions {
  /**
   * Fold it as a hand does: every fold line a little off (scatter.foldLineDeg), the same way
   * for the same design (seeded from its operations). The game uses it; tests and the wind
   * tunnel leave it off and get the exact plane. A number is the seed itself (tests).
   */
  crooked?: boolean | number;
}

export function prepare(design: DesignFile, cfg: PhysicsConfig, opts: PrepareOptions = {}): Prepared {
  const sigma = opts.crooked ? cfg.scatter.foldLineDeg : 0;
  const seed = typeof opts.crooked === 'number' ? opts.crooked : designSeed(design.ops);
  // a plane that cannot be folded crooked (paper would tear) is folded exactly
  const folded = (sigma > 0 ? replayCrooked(design, seed, sigma) : null) ?? replayDesign(design).models.at(-1)!;
  // released creases open a little (springback); clips and tape hold layers shut
  let model = relaxCreases(folded, design.ops, cfg.paper.creaseSpringback);
  // flexible paper: the strip between two facing cut ends becomes a hinge edge
  if (design.flexible) model = splitAtTabs(model, slitTabs(model));
  const af = buildAirframe(model, design.orientation, cfg, design.ops);
  return { design, model, folded, af, flex: design.flexible ? buildFlexAirframe(model, af, cfg) : null };
}

/** The right simulator for a design: rigid (default) or flexible paper (stage 6). */
export function createSimulator(p: Prepared, cfg: PhysicsConfig, launch: LaunchParams, inputs?: ControlInput[]): Simulator {
  if (p.flex) return new MultiBodySim(p.flex, cfg, launch, inputs);
  return new FlightSim(p.af, cfg, launch, inputs, undefined, p.model.facets.map((f) => f.id));
}
