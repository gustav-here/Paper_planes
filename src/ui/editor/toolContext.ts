import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import type { PaperModel } from '../../core/paper/model';
import type { FoldScope, Op } from '../../core/paper/ops';
import type { OpResult } from '../../core/paper/replay';
import type { PaperHit } from '../../render/PaperMesh';
import type { Overlay } from '../../render/Overlay';
import type { SnapKind, Plane } from './Snapping';

export interface EditorOptions {
  foldScope: FoldScope;
  foldMirror: boolean;
  pointToPoint: boolean;
  cutDepth: 'all' | 'top';
  cutMirror: boolean;
}

export interface Ray {
  origin: Vec3;
  dir: Vec3;
}

export type ToastKind = 'info' | 'warn' | 'error' | 'ok';
export interface ToastAction {
  label: string;
  run: () => void;
}

/** Everything a tool needs from the editor. All geometry is in model space. */
export interface ToolContext {
  readonly options: EditorOptions;
  readonly overlay: Overlay;
  model(): PaperModel;
  /** Show a temporary model (e.g. while dragging the fold angle); null = show the committed model. */
  setPreview(m: PaperModel | null): void;
  /** Apply and record an operation. Returns null (and shows the error) on failure. */
  commit(op: Op): OpResult | null;
  replaceLast(op: Op, model: PaperModel): void;
  previousModel(): PaperModel;
  nextId(prefix: string): string;
  toast(text: string, kind?: ToastKind, actions?: ToastAction[]): void;
  hint(text: string): void;
  mouse(e: PointerEvent): [number, number];
  ray(e: PointerEvent): Ray;
  hitPaper(e: PointerEvent): PaperHit | null;
  hitClip(e: PointerEvent): string | null;
  toScreen(p: Vec3): [number, number] | null;
  /** Orient a plane normal towards the camera. */
  towardsViewer(n: Vec3, p: Vec3): Vec3;
  snapDot(screen: [number, number] | null, kind?: SnapKind): void;
  angleLabel(text: string | null, screen?: [number, number]): void;
  /** Automatic see-through paper (e.g. when a mirrored fold is hidden behind the paper). */
  setAutoXRay(on: boolean): void;
  setGhost(obj: import('three').Object3D | null): void;
  highlightClip(id: string | null): void;
}

export interface Tool {
  activate(): void;
  deactivate(): void;
  down(e: PointerEvent): void;
  move(e: PointerEvent): void;
  up(e: PointerEvent): void;
  /** Return true if the key was handled. */
  key(e: KeyboardEvent): boolean;
  /** Called when the committed model or options change. */
  refresh(): void;
}

export function rayPlane(r: Ray, plane: Plane): Vec3 | null {
  const den = V.dot(plane.normal, r.dir);
  if (Math.abs(den) < 1e-9) return null;
  const t = V.dot(plane.normal, V.sub(plane.point, r.origin)) / den;
  if (t < 0) return null;
  return V.addScaled(r.origin, r.dir, t);
}
