import type { PaperModel } from '../paper/model';
import type { DesignFile } from '../paper/ops';
import { replayDesign } from '../paper/replay';
import { buildAirframe } from './airframe';
import type { Airframe } from './airframe';
import type { PhysicsConfig } from './config';
import type { ControlInput } from './controls';
import type { LaunchParams } from './launch';
import { FlightSim } from './simulation';
import { MultiBodySim, buildFlexAirframe } from './multibody';
import type { FlexAirframe } from './multibody';
import type { Simulator } from './simulator';

/** Everything built once per design (shared by all throws of that design). */
export interface Prepared {
  design: DesignFile;
  model: PaperModel;
  af: Airframe;
  /** Hinged panels, when the design is flexible paper. */
  flex: FlexAirframe | null;
}

export function prepare(design: DesignFile, cfg: PhysicsConfig): Prepared {
  const model = replayDesign(design).models.at(-1)!;
  const af = buildAirframe(model, design.orientation, cfg, design.ops);
  return { design, model, af, flex: design.flexible ? buildFlexAirframe(model, af, cfg) : null };
}

/** The right simulator for a design: rigid (default) or flexible paper (stage 6). */
export function createSimulator(p: Prepared, cfg: PhysicsConfig, launch: LaunchParams, inputs?: ControlInput[]): Simulator {
  if (p.flex) return new MultiBodySim(p.flex, cfg, launch, inputs);
  return new FlightSim(p.af, cfg, launch, inputs, undefined, p.model.facets.map((f) => f.id));
}
