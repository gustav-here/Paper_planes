import { coplanar, facetsOverlap } from './model';
import type { PaperModel } from './model';

/**
 * Compact stacking level per facet: 0 for the lowest layer of each local stack, and
 * 1 + max(level of overlapping coplanar facets below) otherwise. Rendering offsets facets by
 * level × thickness along the plane's canonical normal, so stacks look as thick as they are.
 */
export function computeStackLevels(model: PaperModel): Map<number, number> {
  const fs = [...model.facets].sort((a, b) => a.layer - b.layer);
  const level = new Map<number, number>();
  for (let i = 0; i < fs.length; i++) {
    let lv = 0;
    for (let j = 0; j < i; j++) {
      const g = fs[j];
      if (g.layer >= fs[i].layer) continue;
      if (!coplanar(fs[i], g) || !facetsOverlap(fs[i], g)) continue;
      lv = Math.max(lv, level.get(g.id)! + 1);
    }
    level.set(fs[i].id, lv);
  }
  return level;
}

/** Maximum number of layers stacked anywhere. */
export function maxLayers(model: PaperModel): number {
  let m = 0;
  for (const v of computeStackLevels(model).values()) m = Math.max(m, v + 1);
  return m;
}
