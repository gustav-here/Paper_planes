import type { Facet } from '../../core/paper/model';
import type { Tool, ToolContext } from './toolContext';

/** The fold that last moved a facet (a mirrored twin's id ends with ', which maps to the same op). */
export function foldOpOf(f: Facet): string | null {
  const last = f.foldPath.at(-1);
  return last ? last.replace(/'$/, '') : null;
}

/**
 * Control-flap tool: click a folded flap to make it a movable control surface (or click again
 * to make it plain paper). In throw mode the arrow keys move all control flaps; where each flap
 * sits decides what it does (elevator, canard, aileron, elevon).
 */
export class ControlTool implements Tool {
  constructor(private ctx: ToolContext) {}

  activate() {
    this.refresh();
  }

  deactivate() {
    this.ctx.setHoverFacets(null);
  }

  refresh() {
    this.ctx.hint('Klicka på en vikt flik för att göra den till klaff (orange). Klicka igen för att ta bort. Styrs med piltangenterna när du kastar.');
  }

  /** Facets moved by the fold under the cursor. */
  private candidate(e: PointerEvent): { op: string; facets: number[] } | null {
    const hit = this.ctx.hitPaper(e);
    if (!hit) return null;
    const m = this.ctx.model();
    const f = m.facets.find((x) => x.id === hit.facetId);
    const op = f ? foldOpOf(f) : null;
    if (!op) return null;
    return { op, facets: m.facets.filter((x) => x.foldPath.some((id) => id === op || id === `${op}'`)).map((x) => x.id) };
  }

  move(e: PointerEvent) {
    this.ctx.setHoverFacets(this.candidate(e)?.facets ?? null);
  }

  down(e: PointerEvent) {
    const c = this.candidate(e);
    if (!c) {
      this.ctx.toast('Den delen har inte vikts – bara vikta flikar kan bli klaffar.', 'info');
      return;
    }
    this.ctx.toggleControl(c.op);
  }

  up() {}

  key(): boolean {
    return false;
  }
}
