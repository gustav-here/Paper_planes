import { createSheet } from '../../core/paper/model';
import type { PaperModel } from '../../core/paper/model';
import { replayDesign } from '../../core/paper/replay';
import type { DesignFile, Op } from '../../core/paper/ops';
import { A4_80GSM } from '../../core/paper/paperSpec';

/** Operation history with cached models: models[i] is the state after i operations. */
export class History {
  private ops: Op[] = [];
  private models: PaperModel[] = [createSheet(A4_80GSM)];
  private cursor = 0;
  /** Damage from flights, applied after the design's ops; cleared as soon as the plane is refolded. */
  damage: Op[] = [];
  meta: Omit<DesignFile, 'ops'> = {
    format: 'paperplane-design',
    version: 1,
    name: 'Min modell',
    paper: A4_80GSM,
    orientation: { nose: [1, 0, 0], up: [0, 1, 0] },
  };

  get current(): PaperModel {
    return this.models[this.cursor];
  }

  get appliedOps(): Op[] {
    return this.ops.slice(0, this.cursor);
  }

  get canUndo() {
    return this.cursor > 0;
  }

  get canRedo() {
    return this.cursor < this.ops.length;
  }

  push(op: Op, model: PaperModel) {
    this.damage = [];
    this.ops = this.ops.slice(0, this.cursor);
    this.models = this.models.slice(0, this.cursor + 1);
    this.ops.push(op);
    this.models.push(model);
    this.cursor++;
  }

  /** Replace the most recent operation (e.g. choosing which cut piece to keep). */
  replaceLast(op: Op, model: PaperModel) {
    if (this.cursor === 0) return;
    this.damage = [];
    this.ops[this.cursor - 1] = op;
    this.models[this.cursor] = model;
    this.ops = this.ops.slice(0, this.cursor);
    this.models = this.models.slice(0, this.cursor + 1);
  }

  /** Mark a fold as a control flap (metadata only: the geometry and cached models are unchanged). */
  setControl(opId: string, on: boolean): boolean {
    const i = this.ops.findIndex((o) => o.id === opId && o.type === 'fold');
    if (i < 0) return false;
    const op = this.ops[i];
    if (op.type !== 'fold') return false;
    this.ops[i] = { ...op, control: on };
    return true;
  }

  get previous(): PaperModel {
    return this.models[Math.max(0, this.cursor - 1)];
  }

  undo() {
    if (this.canUndo) {
      this.cursor--;
      this.damage = [];
    }
  }

  redo() {
    if (this.canRedo) {
      this.cursor++;
      this.damage = [];
    }
  }

  /** Load a design; throws PaperError if it cannot be replayed. */
  load(design: DesignFile) {
    const { models } = replayDesign(design);
    const { ops, ...meta } = design;
    this.meta = meta;
    this.ops = [...ops];
    this.models = models;
    this.cursor = ops.length;
    this.damage = [];
  }

  reset() {
    this.damage = [];
    this.ops = [];
    this.models = [createSheet(this.meta.paper)];
    this.cursor = 0;
    this.meta = { ...this.meta, orientation: { nose: [1, 0, 0], up: [0, 1, 0] } };
  }

  /** The design as the player folded it (what is saved). */
  toDesign(): DesignFile {
    return { ...this.meta, ops: this.appliedOps };
  }

  /** The plane as it flies: the design plus any damage it has taken. */
  toFlightDesign(): DesignFile {
    return { ...this.meta, ops: [...this.appliedOps, ...this.damage] };
  }

  nextId(prefix: string): string {
    const used = new Set(this.ops.map((o) => o.id));
    let i = this.ops.length + 1;
    while (used.has(`${prefix}${i}`)) i++;
    return `${prefix}${i}`;
  }
}
