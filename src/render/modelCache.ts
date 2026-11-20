import { computeEdges } from '../core/paper/edges';
import type { EdgeGraph } from '../core/paper/edges';
import { computeStackLevels } from '../core/paper/stack';
import { findSymmetry } from '../core/paper/symmetry';
import type { Affine } from '../core/math/transform';
import type { PaperModel } from '../core/paper/model';

/** Models are immutable, so derived data can be cached per model object. */
const edges = new WeakMap<PaperModel, EdgeGraph>();
const levels = new WeakMap<PaperModel, Map<number, number>>();
const sym = new WeakMap<PaperModel, Affine | null>();

export function edgesOf(m: PaperModel): EdgeGraph {
  let e = edges.get(m);
  if (!e) edges.set(m, (e = computeEdges(m)));
  return e;
}

export function levelsOf(m: PaperModel): Map<number, number> {
  let l = levels.get(m);
  if (!l) levels.set(m, (l = computeStackLevels(m)));
  return l;
}

export function symmetryOf(m: PaperModel): Affine | null {
  if (!sym.has(m)) sym.set(m, findSymmetry(m));
  return sym.get(m)!;
}
