import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import { applyAffine, applyAffineLinear } from '../../core/math/transform';
import { applyOp, foldAngleLimit } from '../../core/paper/replay';
import { facetContains } from '../../core/paper/model';
import { PaperError } from '../../core/paper/errors';
import type { FoldKind, FoldOp } from '../../core/paper/ops';
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
  | { k: 'ready'; p0: Vec3; p1: Vec3; N: Vec3; hoverSide: Vec3 | null; hoverPick?: Vec3 | null }
  | {
      k: 'angling';
      p0: Vec3;
      p1: Vec3;
      N: Vec3;
      side: Vec3;
      pick: Vec3;
      startY: number;
      /** Signed fold angle (deg): positive = valley (towards the viewer), negative = mountain. */
      signed: number;
      /** Invalid fold whatever the direction (e.g. it would tear the paper). */
      fatal: string | null;
      /** Per direction: largest angle before paper would pass through paper, and why it is blocked. */
      limit: Record<FoldKind, number>;
      blocked: Record<FoldKind, string | null>;
      stopped: boolean;
    };

/** Mouse travel per degree: 180° in about 110 px, so the drag stays well inside the window. */
const PX_PER_DEG = 0.6;
const SNAP_ANGLES = [45, 90, 135, 180];
const BLOCKED = 'Fliken kan inte vikas åt det hållet: annat papper är i vägen.';

const SCOPE_NAMES: Record<FoldOp['scope'], string> = { top: 'Översta flik', all: 'Alla lager', bottom: 'Understa flik', flap: 'Klickad flik' };

const kindOf = (signed: number): FoldKind => (signed >= 0 ? 'valley' : 'mountain');

/**
 * Fold tool: draw a line → press on the side that should fold → drag up for a valley fold
 * (towards you) or down for a mountain fold (away from you), passing through zero in the same
 * drag; the paper animates live (5°–180°) → release to commit.
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
    if (this.s.k === 'ready') {
      this.readyHint();
      this.hoverKey = '';
      this.updateHover(this.s);
      this.drawOverlay();
    }
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
    this.clearHover();
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
        this.ctx.setFrame(hit.frame);
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
      this.clearHover();
      const hit = this.ctx.hitPaper(e);
      const pick = hit?.point ?? q;
      const limit: Record<FoldKind, number> = { valley: 180, mountain: 180 };
      const blocked: Record<FoldKind, string | null> = { valley: null, mountain: null };
      let fatal: string | null = null;
      try {
        for (const kind of ['valley', 'mountain'] as const) {
          limit[kind] = foldAngleLimit(this.ctx.model(), this.makeOp({ ...s, side, pick }, kind, 180, 'limit'));
          if (limit[kind] < 5) blocked[kind] = BLOCKED;
        }
      } catch (err) {
        fatal = err instanceof PaperError ? ERROR_TEXT[err.code] : String(err);
      }
      this.s = { k: 'angling', p0: s.p0, p1: s.p1, N: s.N, side, pick, startY: e.clientY, signed: 0, fatal, limit, blocked, stopped: false };
      this.ctx.hint(
        fatal ?? 'Dra uppåt för dalveck (mot dig), nedåt för bergsveck (bort från dig). Släpp för att vika. Esc avbryter.',
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
        this.ctx.overlay.foldLine(this.draw.p0, this.draw.p1, 'fold');
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
        this.ctx.overlay.foldLine(V.addScaled(mid, dir, -0.06), V.addScaled(mid, dir, 0.06), 'fold');
      }
      return;
    }
    if (s.k === 'ready') {
      const q = rayPlane(this.ctx.ray(e), { point: s.p0, normal: s.N });
      s.hoverSide = q ? this.sideOf(s, q) : null;
      s.hoverPick = this.ctx.hitPaper(e)?.point ?? q;
      this.updateHover(s);
      this.drawOverlay();
      return;
    }
    if (s.k === 'angling') {
      const raw = Math.max(-180, Math.min(180, (s.startY - e.clientY) / PX_PER_DEG));
      const kind = kindOf(raw);
      let a = Math.abs(raw);
      if (!e.shiftKey) for (const t of SNAP_ANGLES) if (Math.abs(a - t) < 6) a = t;
      // paper stops where it would run into other paper
      s.stopped = a > s.limit[kind];
      a = Math.min(a, s.limit[kind]);
      s.signed = kind === 'valley' ? a : -a;
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
      const kind = kindOf(s.signed);
      const angle = Math.abs(s.signed);
      const error = s.fatal ?? (angle >= 0.5 || s.stopped ? s.blocked[kind] : null);
      if (error) this.ctx.toast(error, 'error');
      if (angle >= 5 && !error) {
        const op = this.makeOp(s, kind, Math.round(angle * 10) / 10, this.ctx.nextId('v'));
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
    if ((e.key === 'l' || e.key === 'L') && this.s.k !== 'angling') {
      this.ctx.cycleFoldScope();
      return true;
    }
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
    this.hoverKey = '';
    this.readyHint();
    this.drawOverlay();
  }

  private readyHint() {
    const name = SCOPE_NAMES[this.ctx.options.foldScope];
    this.ctx.hint(`Tryck på sidan som ska vikas och dra uppåt (dal) eller nedåt (berg). Lager: ${name} (byt med L eller i menyn). Esc avbryter.`);
  }

  private hoverKey = '';

  /** Highlight the paper that would fold if the player pressed on the hovered side. */
  private updateHover(s: Extract<State, { k: 'ready' }>) {
    const side = s.hoverSide;
    if (!side) {
      if (this.hoverKey) {
        this.clearHover();
        this.readyHint();
      }
      return;
    }
    const o = this.ctx.options;
    const pickFacet = o.foldScope === 'flap' && s.hoverPick ? (this.ctx.model().facets.find((f) => facetContains(f, s.hoverPick!, -1e-7, 1e-4))?.id ?? -1) : 0;
    const key = `${Math.sign(V.dot(side, V.cross(s.N, V.sub(s.p1, s.p0))))}|${o.foldScope}|${o.foldMirror}|${pickFacet}`;
    if (key === this.hoverKey) return;
    this.hoverKey = key;
    try {
      const r = applyOp(this.ctx.model(), this.makeOp({ ...s, side, pick: s.hoverPick ?? V.lerp(s.p0, s.p1, 0.5) }, 'valley', 0, 'hover'), { penetration: 'none' });
      const ids = r.model.facets.filter((f) => f.foldPath.some((id) => id.startsWith('hover'))).map((f) => f.id);
      this.ctx.setPreview(r.model);
      this.ctx.setHoverFacets(ids);
      this.readyHint();
    } catch (err) {
      this.ctx.setPreview(null);
      this.ctx.setHoverFacets(null);
      const why = err instanceof PaperError ? ERROR_TEXT[err.code] : String(err);
      this.ctx.hint(`Den sidan går inte att vika med lager ”${SCOPE_NAMES[o.foldScope]}”: ${why} Byt lager med L eller välj andra sidan.`);
    }
  }

  private clearHover() {
    if (!this.hoverKey) return;
    this.hoverKey = '';
    this.ctx.setPreview(null);
    this.ctx.setHoverFacets(null);
  }

  private sideOf(s: { p0: Vec3; p1: Vec3; N: Vec3 }, q: Vec3): Vec3 | null {
    const d = V.normalize(V.sub(s.p1, s.p0));
    const r = V.rejectFrom(V.rejectFrom(V.sub(q, s.p0), d), s.N);
    if (V.length(r) < 0.001) return null;
    return V.normalize(r);
  }

  private makeOp(
    s: { p0: Vec3; p1: Vec3; N: Vec3; side: Vec3; pick: Vec3 },
    kind: FoldKind,
    angleDeg: number,
    id: string,
  ): FoldOp {
    const o = this.ctx.options;
    return {
      type: 'fold',
      id,
      line: { p0: s.p0, p1: s.p1 },
      planeNormal: s.N,
      movingSide: s.side,
      kind,
      angleDeg,
      scope: o.foldScope,
      pick: o.foldScope === 'flap' ? s.pick : undefined,
      mirror: o.foldMirror,
    };
  }

  private preview(e: PointerEvent) {
    const s = this.s;
    if (s.k !== 'angling') return;
    if (s.fatal) {
      // invalid in both directions (checked once when the drag began)
      this.ctx.angleLabel(s.fatal, this.ctx.mouse(e));
      this.drawOverlay();
      return;
    }
    const kind = kindOf(s.signed);
    const angle = Math.abs(s.signed);
    if (angle < 0.5) {
      this.ctx.angleLabel(s.stopped && s.blocked[kind] ? s.blocked[kind] : '0° – dra upp för dal, ned för berg', this.ctx.mouse(e));
      this.ctx.setPreview(null);
    } else {
      const label = `${kind === 'valley' ? 'Dal' : 'Berg'} ${Math.round(angle)}°${s.stopped ? ' · stopp: papper i vägen' : ''}`;
      this.ctx.angleLabel(label, this.ctx.mouse(e));
      try {
        const r = applyOp(this.ctx.model(), this.makeOp(s, kind, angle, 'preview'), { penetration: 'none' });
        this.ctx.setPreview(r.model);
      } catch (err) {
        this.ctx.setPreview(null);
        s.fatal = err instanceof PaperError ? ERROR_TEXT[err.code] : String(err);
        this.ctx.angleLabel(s.fatal, this.ctx.mouse(e));
      }
    }
    this.drawOverlay();
  }

  private drawOverlay() {
    const s = this.s;
    if (s.k !== 'ready' && s.k !== 'angling') return;
    const ov = this.ctx.overlay;
    ov.clear();
    const style =
      s.k !== 'angling'
        ? 'fold'
        : s.fatal || (s.stopped && s.blocked[kindOf(s.signed)])
          ? 'error'
          : Math.abs(s.signed) < 0.5
            ? 'fold'
            : kindOf(s.signed);
    ov.foldLine(s.p0, s.p1, style);
    const mid = V.lerp(s.p0, s.p1, 0.5);
    const side = s.k === 'ready' ? s.hoverSide : s.side;
    if (side) ov.arrow(V.addScaled(mid, side, 0.004), side, s.N);

    // mirrored twin (shown through the paper if it lies on the hidden side)
    let hidden = false;
    if (this.ctx.options.foldMirror) {
      const R = symmetryOf(this.ctx.model());
      if (R) {
        const probe: FoldOp = this.makeOp({ ...s, side: side ?? [0, 0, 1], pick: mid }, 'valley', 90, 'probe');
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
