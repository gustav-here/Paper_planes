import { describe, expect, it } from 'vitest';
import { FoldTool, lineEndAt, MIN_FOLD_DEG } from '../../src/ui/editor/FoldTool';
import type { ToolContext } from '../../src/ui/editor/toolContext';
import { createSheet } from '../../src/core/paper/model';
import type { PaperModel } from '../../src/core/paper/model';
import { A4_80GSM } from '../../src/core/paper/paperSpec';
import type { FoldOp } from '../../src/core/paper/ops';
import type { Vec3 } from '../../src/core/math/vec3';

/*
 * Putting a fold line right: the fold tool on a flat A4 seen straight from above, one screen
 * pixel per millimetre (screen x = model x, screen y = model z).
 */
type Ev = { clientX: number; clientY: number; shiftKey: boolean };
const ev = (x: number, y: number, shiftKey = false) => ({ clientX: x, clientY: y, shiftKey }) as unknown as PointerEvent;

function harness() {
  const model: PaperModel = createSheet(A4_80GSM);
  const committed: FoldOp[] = [];
  const ui = { editButton: null as (() => void) | null, cursor: '' };
  const noop = () => undefined;
  const onSheet = (x: number, z: number) => Math.abs(x) <= 0.1485 && Math.abs(z) <= 0.105;
  const ctx = new Proxy(
    {
      options: { foldScope: 'top', foldMirror: false, pointToPoint: false, cutDepth: 'all', cutMirror: false },
      model: () => model,
      commit: (op: FoldOp) => {
        committed.push(op);
        return { model, warnings: [], mirrored: false };
      },
      nextId: (p: string) => `${p}1`,
      mouse: (e: Ev) => [e.clientX, e.clientY],
      ray: (e: Ev) => ({ origin: [e.clientX / 1000, 1, e.clientY / 1000], dir: [0, -1, 0] }),
      hitPaper: (e: Ev) => {
        const x = e.clientX / 1000;
        const z = e.clientY / 1000;
        return onSheet(x, z) ? { facetId: 0, point: [x, 0, z], normal: [0, 1, 0], frame: { q: [0, 0, 0, 1], t: [0, 0, 0] } } : null;
      },
      toScreen: (p: Vec3) => [p[0] * 1000, p[2] * 1000],
      towardsViewer: (n: Vec3) => (n[1] >= 0 ? n : [-n[0], -n[1], -n[2]]),
      foldEditButton: (cancel: (() => void) | null) => (ui.editButton = cancel),
      setCursor: (c: string) => (ui.cursor = c),
      overlay: new Proxy({}, { get: () => noop }),
    } as Record<string, unknown>,
    { get: (t, k) => (k in t ? t[k as string] : noop) },
  ) as unknown as ToolContext;
  const tool = new FoldTool(ctx);
  const state = () => (tool as unknown as { s: { k: string; p0?: Vec3; p1?: Vec3 } }).s;
  /** A line across the sheet at x = 0 (z from -80 to +80 mm), drawn like a player would. */
  const drawLine = () => {
    tool.down(ev(0, -80, true));
    for (let z = -70; z <= 80; z += 10) tool.move(ev(0, z, true));
    tool.up(ev(0, 80, true));
  };
  /** Take hold of the +x side and drag to `deg` (valley up, mountain down). */
  const startFold = (deg: number) => {
    tool.move(ev(50, 0));
    tool.down(ev(50, 0));
    tool.move(ev(50, -deg * 0.6, true));
  };
  return { tool, committed, ui, state, drawLine, startFold };
}

describe('lineEndAt', () => {
  it('takes hold of the nearest end within the radius', () => {
    expect(lineEndAt([[0, 0], [100, 0]], [3, 4], 10)).toBe(0);
    expect(lineEndAt([[0, 0], [100, 0]], [95, 1], 10)).toBe(1);
    expect(lineEndAt([[0, 0], [8, 0]], [5, 0], 10)).toBe(1);
  });

  it('misses outside the radius and ends that are off screen', () => {
    expect(lineEndAt([[0, 0], [100, 0]], [50, 0], 10)).toBeNull();
    expect(lineEndAt([null, [100, 0]], [0, 0], 10)).toBeNull();
  });
});

describe('fold tool: putting the fold line right', () => {
  it('released back at 0°: no crease, and the line ends can be moved before folding again', () => {
    const h = harness();
    h.drawLine();
    expect(h.state().k).toBe('ready');
    h.startFold(90);
    expect(h.state().k).toBe('angling');
    expect(h.ui.editButton).toBeTypeOf('function');
    h.tool.move(ev(50, 0, true)); // back to 0°
    h.tool.up(ev(50, 0, true));
    expect(h.committed).toHaveLength(0);
    expect(h.state().k).toBe('ready');
    expect(h.ui.editButton).toBeNull();

    // drag the end at z = +80 mm over to x = 60 mm (Shift: no snapping)
    h.tool.move(ev(4, 78, true));
    expect(h.ui.cursor).toBe('grab');
    h.tool.down(ev(4, 78, true));
    expect(h.state().k).toBe('moving');
    h.tool.move(ev(60, 80, true));
    h.tool.up(ev(60, 80, true));
    expect(h.state().k).toBe('ready');
    expect(h.state().p0![0]).toBeCloseTo(0, 6);
    expect(h.state().p1![0]).toBeCloseTo(0.06, 6);
    expect(h.state().p1![2]).toBeCloseTo(0.08, 6);

    // fold along the moved line
    h.startFold(90);
    h.tool.up(ev(50, -54, true));
    expect(h.committed).toHaveLength(1);
    expect(h.committed[0].line.p1[0]).toBeCloseTo(0.06, 6);
    expect(h.committed[0].angleDeg).toBeCloseTo(90, 6);
  });

  it(`a fold under ${MIN_FOLD_DEG}° makes no crease either`, () => {
    const h = harness();
    h.drawLine();
    h.startFold(MIN_FOLD_DEG - 2);
    h.tool.up(ev(50, -(MIN_FOLD_DEG - 2) * 0.6, true));
    expect(h.committed).toHaveLength(0);
    expect(h.state().k).toBe('ready');
  });

  it('"Ändra vikning" mid-fold: no crease, the line stays and its ends can be moved', () => {
    const h = harness();
    h.drawLine();
    h.startFold(120);
    const before = { p0: h.state().p0, p1: h.state().p1 };
    h.ui.editButton!();
    expect(h.state()).toMatchObject({ k: 'ready', ...before });
    expect(h.ui.editButton).toBeNull();
    // the finger that was folding lifts later: still nothing is folded
    h.tool.up(ev(50, -72, true));
    expect(h.committed).toHaveLength(0);
    h.tool.down(ev(0, -80, true));
    expect(h.state().k).toBe('moving');
  });

  it('Esc while moving an end puts the line back; a too short line is not kept', () => {
    const h = harness();
    h.drawLine();
    const before = { p0: h.state().p0, p1: h.state().p1 };
    h.tool.down(ev(0, 80, true));
    h.tool.move(ev(40, 40, true));
    h.tool.key({ key: 'Escape' } as KeyboardEvent);
    expect(h.state()).toMatchObject({ k: 'ready', ...before });

    h.tool.down(ev(0, 80, true));
    h.tool.move(ev(1, -79, true)); // onto the other end
    h.tool.up(ev(1, -79, true));
    expect(h.state()).toMatchObject({ k: 'ready', ...before });
  });

  it('a press away from the ends still takes hold of a side to fold', () => {
    const h = harness();
    h.drawLine();
    h.tool.down(ev(-50, 0));
    expect(h.state().k).toBe('angling');
  });
});
