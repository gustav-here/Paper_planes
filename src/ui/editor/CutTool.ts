import * as V from '../../core/math/vec3';
import { applyAffine, applyAffineLinear } from '../../core/math/transform';
import { applyOp } from '../../core/paper/replay';
import type { CutPiece } from '../../core/paper/cut';
import type { CutOp } from '../../core/paper/ops';
import { cutIsSelfMirror } from '../../core/paper/symmetry';
import { symmetryOf } from '../../render/modelCache';
import { LineDraw } from './LineTool';
import type { Tool, ToolContext } from './toolContext';

/** Scissors: drag a straight cut. Detached pieces are removed (largest kept; switchable). */
export class CutTool implements Tool {
  private drawing = false;
  private draw: LineDraw;

  constructor(private ctx: ToolContext) {
    this.draw = new LineDraw(ctx);
  }

  activate() {
    this.reset();
  }

  deactivate() {
    this.reset();
  }

  refresh() {
    if (!this.drawing) this.ctx.hint('Dra ett snitt med saxen. Börja gärna utanför kanten.');
  }

  private reset() {
    this.drawing = false;
    this.draw.reset();
    this.ctx.overlay.clear();
    this.ctx.setAutoXRay(false);
    this.refresh();
  }

  down(e: PointerEvent) {
    this.draw.begin(e);
    this.drawing = true;
  }

  move(e: PointerEvent) {
    if (!this.drawing) {
      LineDraw.hover(this.ctx, e);
      return;
    }
    this.draw.move(e);
    const ov = this.ctx.overlay;
    ov.clear();
    const { p0, p1, plane } = this.draw;
    if (!p0 || !p1 || !plane || this.draw.length < 1e-4) return;
    ov.line(p0, p1, 'cut');
    let hidden = false;
    if (this.ctx.options.cutMirror) {
      const R = symmetryOf(this.ctx.model());
      if (R && !cutIsSelfMirror(this.op(), R)) {
        const t0 = applyAffine(R, p0);
        const t1 = applyAffine(R, p1);
        ov.line(t0, t1, 'twin');
        const tN = applyAffineLinear(R, plane.normal);
        hidden = V.dot(this.ctx.towardsViewer(tN, t0), tN) < 0;
      }
    }
    this.ctx.setAutoXRay(hidden);
  }

  private op(id = 'probe'): CutOp {
    return {
      type: 'cut',
      id,
      path: [this.draw.p0!, this.draw.p1!],
      planeNormal: this.draw.plane!.normal,
      depth: this.ctx.options.cutDepth,
      mirror: this.ctx.options.cutMirror,
    };
  }

  up() {
    if (!this.drawing) return;
    if (this.draw.p0 && this.draw.p1 && this.draw.plane && this.draw.length > 0.002) {
      const op = this.op(this.ctx.nextId('k'));
      const r = this.ctx.commit(op);
      if (r?.pieces && r.pieces.length > 1) this.offerSwap(op, r.pieces);
    }
    this.reset();
  }

  /** Let the player keep another piece than the largest one. */
  private offerSwap(op: CutOp, pieces: CutPiece[]) {
    const keptIdx = pieces.findIndex((p) => p.kept);
    this.ctx.toast('Biten som lossnade togs bort.', 'info', [
      {
        label: 'Behåll den andra biten',
        run: () => {
          const next = (keptIdx + 1) % pieces.length;
          const swapped: CutOp = { ...op, discard: pieces.filter((_, i) => i !== next).map((p) => ({ flat: p.flat })) };
          try {
            const r = applyOp(this.ctx.previousModel(), swapped);
            this.ctx.replaceLast(swapped, r.model);
            if (r.pieces && r.pieces.length > 1) this.offerSwap(swapped, r.pieces);
          } catch {
            this.ctx.toast('Kunde inte byta bit.', 'error');
          }
        },
      },
    ]);
  }

  key(e: KeyboardEvent): boolean {
    if (e.key === 'Escape') {
      this.reset();
      return true;
    }
    return false;
  }
}
