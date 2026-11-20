import * as V from '../../core/math/vec3';
import { CLIP_SPEC, TAPE_SPEC, findClipPlacement, findTapePlacement, resolveClip } from '../../core/paper/clips';
import type { ClipPlacement } from '../../core/paper/clips';
import type { Clip } from '../../core/paper/model';
import { createClipObject } from '../../render/ClipMesh';
import { edgesOf } from '../../render/modelCache';
import type { Tool, ToolContext } from './toolContext';

/**
 * Paper clip / tape tool: slide a 1 g clip (or a 2 cm strip of tape) along any stack edge and
 * left-click to attach it. Both hold the layers together. Removal is a right-click, handled by
 * the editor for every tool.
 *  - A clip that would run into paper leaving its plane (a wing above the keel) leans back.
 *  - Tape over a seam (the two wing halves seen from above) presses the halves together and is
 *    laid flat across. Tape over an edge hangs half in the air until it is pressed on, then the
 *    free half folds over to the other side; what sticks out past the paper is cut off.
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
        ? 'Håll tejpen över skarven mellan vinghalvorna för att tejpa ihop dem, eller över en kant för att vika den över kanten. Klicka för att trycka fast. Högerklicka på tejpen för att ta bort den.'
        : 'För musen längs en kant och klicka för att fästa ett gem (1 g). Gemet lutar bakåt om en vinge är i vägen. Högerklicka på ett gem för att ta bort det.',
    );
  }

  /** The clip (or tape) to place at the current placement. */
  private holder(id: string): Clip | null {
    const pl = this.placement;
    if (!pl) return null;
    // a clip in the way of a wing leans backwards: away from the nose
    const along = V.cross(pl.normal, pl.inward);
    const lean: 1 | -1 = V.dot(along, this.ctx.nose()) <= 0 ? 1 : -1;
    return {
      id,
      at: pl.at,
      edgeDir: pl.edgeDir,
      massKg: this.spec.massKg,
      ...(this.kind === 'tape' ? { kind: 'tape' as const } : { lean }),
      ...(pl.bridge ? { bridge: true } : {}),
    };
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
    this.placement = hit
      ? this.kind === 'tape'
        ? findTapePlacement(m, hit.point, 0.015, edgesOf(m))
        : findClipPlacement(m, hit.point, 0.015, edgesOf(m), this.spec)
      : null;
    const ghost = this.holder('ghost');
    const rc = ghost && resolveClip(m, ghost);
    this.ctx.setGhost(rc ? createClipObject(m, rc, true) : null);
  }

  down() {
    if (!this.placement) return;
    const clip = this.holder(this.ctx.nextId(this.kind === 'tape' ? 'tejp' : 'gem'));
    if (!clip) return;
    this.ctx.commit({ type: 'clip', ...clip });
    this.placement = null;
    this.ctx.setGhost(null);
  }

  up() {}

  key(): boolean {
    return false;
  }
}
