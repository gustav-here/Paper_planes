import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import { Snapper } from './Snapping';
import type { Plane, SnapResult } from './Snapping';
import { rayPlane } from './toolContext';
import type { ToolContext } from './toolContext';

/**
 * Shared line drawing for the fold and scissors tools: the working plane is taken from the
 * first paper facet under the cursor (the drag may start outside the paper), and both ends
 * snap to corners, edge midpoints, edges and 15° directions (Shift disables snapping).
 */
export class LineDraw {
  plane: Plane | null = null;
  p0: Vec3 | null = null;
  p1: Vec3 | null = null;
  private snapper: Snapper | null = null;
  private startEvent: PointerEvent | null = null;

  constructor(private ctx: ToolContext) {}

  begin(e: PointerEvent) {
    this.startEvent = e;
    this.plane = null;
    this.p0 = this.p1 = null;
    this.trySetPlane(e);
  }

  private trySetPlane(e: PointerEvent): boolean {
    const hit = this.ctx.hitPaper(e);
    if (!hit) return false;
    this.plane = { point: hit.point, normal: this.ctx.towardsViewer(hit.normal, hit.point) };
    this.snapper = new Snapper(this.ctx.model(), this.plane);
    const start = this.startEvent!;
    const raw = rayPlane(this.ctx.ray(start), this.plane);
    if (!raw) return false;
    this.p0 = this.snapAt(start, raw, null).point;
    return true;
  }

  move(e: PointerEvent): SnapResult | null {
    if (!this.plane && !this.trySetPlane(e)) return null;
    const raw = rayPlane(this.ctx.ray(e), this.plane!);
    if (!raw) return null;
    const s = this.snapAt(e, raw, this.p0);
    this.p1 = s.point;
    return s;
  }

  get length(): number {
    return this.p0 && this.p1 ? V.dist(this.p0, this.p1) : 0;
  }

  snapAt(e: PointerEvent | MouseEvent, raw: Vec3, anchor: Vec3 | null): SnapResult {
    const s = this.snapper!.snap(raw, this.ctx.mouse(e as PointerEvent), (p) => this.ctx.toScreen(p), anchor, e.shiftKey);
    const sc = s.kind === 'point' || s.kind === 'edge' ? this.ctx.toScreen(s.point) : null;
    this.ctx.snapDot(sc, s.kind);
    return s;
  }

  /** Hover snapping before a drag starts (shows where the line would begin). */
  static hover(ctx: ToolContext, e: PointerEvent) {
    const hit = ctx.hitPaper(e);
    if (!hit) {
      ctx.snapDot(null);
      return;
    }
    const plane = { point: hit.point, normal: ctx.towardsViewer(hit.normal, hit.point) };
    const s = new Snapper(ctx.model(), plane).snap(hit.point, ctx.mouse(e), (p) => ctx.toScreen(p), null, e.shiftKey);
    ctx.snapDot(s.kind === 'point' || s.kind === 'edge' ? ctx.toScreen(s.point) : null, s.kind);
  }

  reset() {
    this.plane = null;
    this.p0 = this.p1 = null;
    this.snapper = null;
    this.startEvent = null;
    this.ctx.snapDot(null);
  }
}
