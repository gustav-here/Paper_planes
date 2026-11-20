import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import { applyAffine, applyAffineLinear } from '../../core/math/transform';
import { applyOp, foldAngleLimit } from '../../core/paper/replay';
import { PaperError } from '../../core/paper/errors';
import type { FoldOp } from '../../core/paper/ops';
import { foldIsSelfMirror } from '../../core/paper/symmetry';
import { symmetryOf } from '../../render/modelCache';
import { ERROR_TEXT } from '../hud/messages';
import { LineDraw } from './LineTool';
import { Snapper } from './Snapping';
import type { Plane } from './Snapping';
import { rayPlane } from './toolContext';
import type { Tool, ToolContext } from './toolContext';

type State =
  | { k: 'idle' }
  | { k: 'drawing' }
  | { k: 'p2p'; A: Vec3; plane: Plane }
  | { k: 'ready'; p0: Vec3; p1: Vec3; N: Vec3; hoverSide: Vec3 | null }
  | {
      k: 'angling';
      p0: Vec3;
      p1: Vec3;
      N: Vec3;
      side: Vec3;
      pick: Vec3;
      startY: number;
      angle: number;
      error: string | null;
      /** Largest angle before paper would pass through paper (deg). */
      limit: number;
      stopped: boolean;
    };

const PX_PER_DEG = 1.35;
const SNAP_ANGLES = [45, 90, 135, 180];

/**
 * Fold tool: draw a line → press on the side that should fold → drag up/down to set the
 * angle (5°–180°, animated live) → release to commit.
 */
export class FoldTool implements Tool {
  private s: State = { k: 'idle' };
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
    if (this.s.k === 'ready') this.drawOverlay();
    if (this.s.k === 'idle') this.idleHint();
  }

  private idleHint() {
    this.ctx.hint(
      this.ctx.options.pointToPoint
        ? 'Punkt → punkt: klicka på punkten som ska flyttas och sedan på målet.'
        : 'Dra en linje över pappret där vecket ska vara.',
    );
  }

  private reset() {
    this.s = { k: 'idle' };
    this.draw.reset();
    this.ctx.overlay.clear();
    this.ctx.setPreview(null);
    this.ctx.angleLabel(null);
    this.ctx.setAutoXRay(false);
    this.idleHint();
  }

  down(e: PointerEvent) {
    const s = this.s;
    if (s.k === 'idle') {
      if (this.ctx.options.pointToPoint) {
        const hit = this.ctx.hitPaper(e);
        if (!hit) return;
        const plane = { point: hit.point, normal: this.ctx.towardsViewer(hit.normal, hit.point) };
        const A = new Snapper(this.ctx.model(), plane).snap(hit.point, this.ctx.mouse(e), (p) => this.ctx.toScreen(p), null, e.shiftKey).point;
        this.s = { k: 'p2p', A, plane };
        this.ctx.hint('Klicka där punkten ska hamna.');
        return;
      }
      this.draw.begin(e);
      this.s = { k: 'drawing' };
      return;
    }
    if (s.k === 'p2p') {
      const raw = rayPlane(this.ctx.ray(e), s.plane);
      if (!raw) return;
      const B = new Snapper(this.ctx.model(), s.plane).snap(raw, this.ctx.mouse(e), (p) => this.ctx.toScreen(p), s.A, e.shiftKey).point;
      if (V.dist(s.A, B) < 0.002) return;
      const mid = V.lerp(s.A, B, 0.5);
      const dir = V.normalize(V.cross(s.plane.normal, V.sub(B, s.A)));
      this.toReady(V.addScaled(mid, dir, -0.2), V.addScaled(mid, dir, 0.2), s.plane.normal);
      return;
    }
    if (s.k === 'ready') {
      const q = rayPlane(this.ctx.ray(e), { point: s.p0, normal: s.N });
      if (!q) return;
      const side = this.sideOf(s, q);
      if (!side) return;
      const hit = this.ctx.hitPaper(e);
      const pick = hit?.point ?? q;
      let limit = 180;
      let error: string | null = null;
      try {
        limit = foldAngleLimit(this.ctx.model(), this.makeOp({ ...s, side, pick }, 180, 'limit'));
        if (limit < 5) error = 'Fliken kan inte vikas åt det hållet: annat papper är i vägen.';
      } catch (err) {
        error = err instanceof PaperError ? ERROR_TEXT[err.code] : String(err);
      }
      this.s = { k: 'angling', p0: s.p0, p1: s.p1, N: s.N, side, pick, startY: e.clientY, angle: 0, error, limit, stopped: false };
      this.ctx.hint(
        error ?? 'Dra uppåt för större vinkel, nedåt för mindre. Släpp för att vika. Esc avbryter.',
      );
      this.drawOverlay();
    }
  }

  move(e: PointerEvent) {
    const s = this.s;
    if (s.k === 'idle') {
      LineDraw.hover(this.ctx, e);
      return;
    }
    if (s.k === 'drawing') {
      this.draw.move(e);
      this.ctx.overlay.clear();
      if (this.draw.p0 && this.draw.p1 && this.draw.length > 1e-4)
        this.ctx.overlay.foldLine(this.draw.p0, this.draw.p1, this.ctx.options.foldKind);
      return;
    }
    if (s.k === 'p2p') {
      const raw = rayPlane(this.ctx.ray(e), s.plane);
      if (!raw) return;
      const B = new Snapper(this.ctx.model(), s.plane).snap(raw, this.ctx.mouse(e), (p) => this.ctx.toScreen(p), s.A, e.shiftKey);
      this.ctx.snapDot(B.kind === 'point' || B.kind === 'edge' ? this.ctx.toScreen(B.point) : null, B.kind);
      this.ctx.overlay.clear();
      this.ctx.overlay.line(s.A, B.point, 'extension');
      if (V.dist(s.A, B.point) > 0.002) {
        const mid = V.lerp(s.A, B.point, 0.5);
        const dir = V.normalize(V.cross(s.plane.normal, V.sub(B.point, s.A)));
        this.ctx.overlay.foldLine(V.addScaled(mid, dir, -0.06), V.addScaled(mid, dir, 0.06), this.ctx.options.foldKind);
      }
      return;
    }
    if (s.k === 'ready') {
      const q = rayPlane(this.ctx.ray(e), { point: s.p0, normal: s.N });
      s.hoverSide = q ? this.sideOf(s, q) : null;
      this.drawOverlay();
      return;
    }
    if (s.k === 'angling') {
      let a = Math.max(0, Math.min(180, (s.startY - e.clientY) / PX_PER_DEG));
      if (!e.shiftKey) for (const t of SNAP_ANGLES) if (Math.abs(a - t) < 4) a = t;
      // paper stops where it would run into other paper
      s.stopped = a > s.limit;
      s.angle = Math.min(a, s.limit);
      this.preview(e);
    }
  }

  up(e: PointerEvent) {
    const s = this.s;
    if (s.k === 'drawing') {
      if (this.draw.p0 && this.draw.p1 && this.draw.length > 0.003 && this.draw.plane) {
        this.toReady(this.draw.p0, this.draw.p1, this.draw.plane.normal);
      } else {
        this.reset();
      }
      return;
    }
    if (s.k === 'angling') {
      this.ctx.angleLabel(null);
      this.ctx.setPreview(null);
      if (s.error) this.ctx.toast(s.error, 'error');
      if (s.angle >= 5 && !s.error) {
        const op = this.makeOp(s, Math.round(s.angle * 10) / 10, this.ctx.nextId('v'));
        if (this.ctx.commit(op)) {
          this.reset();
          return;
        }
      }
      this.toReady(s.p0, s.p1, s.N);
      void e;
    }
  }

  key(e: KeyboardEvent): boolean {
    if (e.key === 'Escape') {
      if (this.s.k === 'angling') {
        const s = this.s;
        this.ctx.setPreview(null);
        this.ctx.angleLabel(null);
        this.toReady(s.p0, s.p1, s.N);
      } else this.reset();
      return true;
    }
    return false;
  }

  private toReady(p0: Vec3, p1: Vec3, N: Vec3) {
    this.draw.reset();
    this.s = { k: 'ready', p0, p1, N, hoverSide: null };
    this.ctx.hint('Tryck på sidan som ska vikas och dra uppåt. Esc avbryter.');
    this.drawOverlay();
  }

  private sideOf(s: { p0: Vec3; p1: Vec3; N: Vec3 }, q: Vec3): Vec3 | null {
    const d = V.normalize(V.sub(s.p1, s.p0));
    const r = V.rejectFrom(V.rejectFrom(V.sub(q, s.p0), d), s.N);
    if (V.length(r) < 0.001) return null;
    return V.normalize(r);
  }

  private makeOp(s: { p0: Vec3; p1: Vec3; N: Vec3; side: Vec3; pick: Vec3 }, angleDeg: number, id: string): FoldOp {
    const o = this.ctx.options;
    return {
      type: 'fold',
      id,
      line: { p0: s.p0, p1: s.p1 },
      planeNormal: s.N,
      movingSide: s.side,
      kind: o.foldKind,
      angleDeg,
      scope: o.foldScope,
      pick: o.foldScope === 'flap' ? s.pick : undefined,
      mirror: o.foldMirror,
    };
  }

  private preview(e: PointerEvent) {
    const s = this.s;
    if (s.k !== 'angling') return;
    if (s.error) {
      // invalid or blocked from the start (checked once when the drag began)
      this.ctx.angleLabel(s.error, this.ctx.mouse(e));
      this.drawOverlay();
      return;
    }
    const label = `${this.ctx.options.foldKind === 'valley' ? 'Dal' : 'Berg'} ${Math.round(s.angle)}°${s.stopped ? ' · stopp: papper i vägen' : ''}`;
    this.ctx.angleLabel(label, this.ctx.mouse(e));
    if (s.angle < 0.5) {
      this.ctx.setPreview(null);
    } else {
      try {
        const r = applyOp(this.ctx.model(), this.makeOp(s, s.angle, 'preview'), { penetration: 'none' });
        this.ctx.setPreview(r.model);
      } catch (err) {
        this.ctx.setPreview(null);
        s.error = err instanceof PaperError ? ERROR_TEXT[err.code] : String(err);
        this.ctx.angleLabel(s.error, this.ctx.mouse(e));
      }
    }
    this.drawOverlay();
  }

  private drawOverlay() {
    const s = this.s;
    if (s.k !== 'ready' && s.k !== 'angling') return;
    const ov = this.ctx.overlay;
    ov.clear();
    const style = s.k === 'angling' && s.error ? 'error' : this.ctx.options.foldKind;
    ov.foldLine(s.p0, s.p1, style);
    const mid = V.lerp(s.p0, s.p1, 0.5);
    const side = s.k === 'ready' ? s.hoverSide : s.side;
    if (side) ov.arrow(V.addScaled(mid, side, 0.004), side, s.N);

    // mirrored twin (shown through the paper if it lies on the hidden side)
    let hidden = false;
    if (this.ctx.options.foldMirror) {
      const R = symmetryOf(this.ctx.model());
      if (R) {
        const probe: FoldOp = this.makeOp({ ...s, side: side ?? [0, 0, 1], pick: mid }, 90, 'probe');
        if (!foldIsSelfMirror(probe, R)) {
          const t0 = applyAffine(R, s.p0);
          const t1 = applyAffine(R, s.p1);
          const tN = applyAffineLinear(R, s.N);
          ov.foldLine(t0, t1, 'twin');
          if (side) ov.arrow(V.addScaled(V.lerp(t0, t1, 0.5), applyAffineLinear(R, side), 0.004), applyAffineLinear(R, side), tN, '#b48cff');
          const tm = V.lerp(t0, t1, 0.5);
          hidden = V.dot(this.ctx.towardsViewer(tN, tm), tN) < 0;
        }
      }
    }
    this.ctx.setAutoXRay(hidden);
  }
}
