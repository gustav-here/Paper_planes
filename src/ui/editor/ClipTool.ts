import { CLIP_SPEC, findClipPlacement, resolveClip } from '../../core/paper/clips';
import type { ClipPlacement } from '../../core/paper/clips';
import { createClipObject } from '../../render/ClipMesh';
import { edgesOf } from '../../render/modelCache';
import type { Tool, ToolContext } from './toolContext';

/**
 * Paper clip tool: slide a 1 g clip along any stack edge and left-click to attach it.
 * Removing clips is a right-click on the clip, handled by the editor for every tool.
 */
export class ClipTool implements Tool {
  private placement: ClipPlacement | null = null;

  constructor(private ctx: ToolContext) {}

  activate() {
    this.refresh();
  }

  deactivate() {
    this.ctx.setGhost(null);
    this.placement = null;
  }

  refresh() {
    this.ctx.hint('För musen längs en kant och klicka för att fästa ett gem (1 g). Högerklicka på ett gem för att ta bort det.');
  }

  move(e: PointerEvent) {
    // no preview on top of an existing clip (that spot is for right-click removal)
    if (this.ctx.hitClip(e)) {
      this.placement = null;
      this.ctx.setGhost(null);
      return;
    }
    const hit = this.ctx.hitPaper(e);
    const m = this.ctx.model();
    this.placement = hit ? findClipPlacement(m, hit.point, 0.015, edgesOf(m)) : null;
    if (!this.placement) {
      this.ctx.setGhost(null);
      return;
    }
    const rc = resolveClip(m, { id: 'ghost', at: this.placement.at, edgeDir: this.placement.edgeDir, massKg: CLIP_SPEC.massKg });
    this.ctx.setGhost(rc ? createClipObject(m, rc, true) : null);
  }

  down() {
    if (!this.placement) return;
    this.ctx.commit({
      type: 'clip',
      id: this.ctx.nextId('gem'),
      at: this.placement.at,
      edgeDir: this.placement.edgeDir,
      massKg: CLIP_SPEC.massKg,
    });
    this.placement = null;
    this.ctx.setGhost(null);
  }

  up() {}

  key(): boolean {
    return false;
  }
}
