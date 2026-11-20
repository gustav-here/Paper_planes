import { CLIP_SPEC, TAPE_SPEC, findClipPlacement, resolveClip } from '../../core/paper/clips';
import type { ClipPlacement } from '../../core/paper/clips';
import { createClipObject } from '../../render/ClipMesh';
import { edgesOf } from '../../render/modelCache';
import type { Tool, ToolContext } from './toolContext';

/**
 * Paper clip / tape tool: slide a 1 g clip (or a 0.03 g strip of tape) along any stack edge and
 * left-click to attach it. Both hold the layers together. Removal is a right-click, handled by
 * the editor for every tool.
 */
export class ClipTool implements Tool {
  private placement: ClipPlacement | null = null;

  constructor(
    private ctx: ToolContext,
    private kind: 'clip' | 'tape' = 'clip',
  ) {}

  private get spec() {
    return this.kind === 'tape' ? TAPE_SPEC : CLIP_SPEC;
  }

  activate() {
    this.refresh();
  }

  deactivate() {
    this.ctx.setGhost(null);
    this.placement = null;
  }

  refresh() {
    this.ctx.hint(
      this.kind === 'tape'
        ? 'För musen längs en kant och klicka för att tejpa ihop lagren där (0,03 g). Högerklicka på tejpen för att ta bort den.'
        : 'För musen längs en kant och klicka för att fästa ett gem (1 g). Högerklicka på ett gem för att ta bort det.',
    );
  }

  move(e: PointerEvent) {
    // no preview on top of an existing clip (that spot is for right-click removal)
    if (this.ctx.hitClip(e)) {
      this.placement = null;
      this.ctx.setGhost(null);
      return;
    }
    const hit = this.ctx.hitPaper(e);
    if (hit) this.ctx.setFrame(hit.frame);
    const m = this.ctx.model();
    this.placement = hit ? findClipPlacement(m, hit.point, 0.015, edgesOf(m), this.spec) : null;
    if (!this.placement) {
      this.ctx.setGhost(null);
      return;
    }
    const rc = resolveClip(m, { id: 'ghost', at: this.placement.at, edgeDir: this.placement.edgeDir, massKg: this.spec.massKg, kind: this.kind });
    this.ctx.setGhost(rc ? createClipObject(m, rc, true) : null);
  }

  down() {
    if (!this.placement) return;
    this.ctx.commit({
      type: 'clip',
      id: this.ctx.nextId(this.kind === 'tape' ? 'tejp' : 'gem'),
      at: this.placement.at,
      edgeDir: this.placement.edgeDir,
      massKg: this.spec.massKg,
      ...(this.kind === 'tape' ? { kind: 'tape' as const } : {}),
    });
    this.placement = null;
    this.ctx.setGhost(null);
  }

  up() {}

  key(): boolean {
    return false;
  }
}
