export interface Series {
  label: string;
  unit: string;
  color: string;
  /** Which y-axis: left or right. */
  axis: 'left' | 'right';
}

/** Minimal two-axis time-series plot on a canvas (height and speed over time). */
export class TimeSeriesPlot {
  private ctx: CanvasRenderingContext2D;
  private t: number[] = [];
  private ys: number[][];

  constructor(
    private canvas: HTMLCanvasElement,
    private series: Series[],
  ) {
    this.ctx = canvas.getContext('2d')!;
    this.ys = series.map(() => []);
  }

  clear() {
    this.t = [];
    this.ys = this.series.map(() => []);
    this.draw();
  }

  push(t: number, values: number[]) {
    this.t.push(t);
    values.forEach((v, i) => this.ys[i].push(v));
  }

  /** Round axis: a tick step from 1/2/5·10ⁿ with at most 5 intervals, and the max as a whole number of steps. */
  private niceAxis(v: number): { max: number; step: number } {
    const target = Math.max(v, 1e-6) / 5;
    const p = 10 ** Math.floor(Math.log10(target));
    const step = [1, 2, 5, 10].map((m) => m * p).find((s) => s >= target)!;
    return { step, max: Math.max(step, Math.ceil(v / step - 1e-9) * step) };
  }

  draw() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const W = this.canvas.clientWidth;
    const H = this.canvas.clientHeight;
    if (this.canvas.width !== Math.round(W * dpr) || this.canvas.height !== Math.round(H * dpr)) {
      this.canvas.width = Math.round(W * dpr);
      this.canvas.height = Math.round(H * dpr);
    }
    const g = this.ctx;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    const L = 34;
    const R = 34;
    const T = 18;
    const B = 20;
    const pw = W - L - R;
    const ph = H - T - B;
    const tAx = this.niceAxis(Math.max(1, this.t.at(-1) ?? 1));
    const tMax = tAx.max;
    const raw = { left: 0.1, right: 0.1 };
    this.series.forEach((s, i) => {
      for (const y of this.ys[i]) raw[s.axis] = Math.max(raw[s.axis], y);
    });
    // both y-axes share the same number of grid lines so the grid is common
    const lAx = this.niceAxis(raw.left);
    const rAx = this.niceAxis(raw.right);
    const n = Math.max(Math.round(lAx.max / lAx.step), Math.round(rAx.max / rAx.step));
    const axisMax = { left: lAx.step * n, right: rAx.step * n };

    g.font = '11px system-ui, sans-serif';
    g.strokeStyle = 'rgba(255,255,255,0.12)';
    g.fillStyle = 'rgba(255,255,255,0.6)';
    g.lineWidth = 1;
    for (let i = 0; i <= n; i++) {
      const y = T + (ph * i) / n;
      g.beginPath();
      g.moveTo(L, y);
      g.lineTo(L + pw, y);
      g.stroke();
      g.textAlign = 'right';
      g.fillText(fmt(axisMax.left * (1 - i / n)), L - 4, y + 4);
      g.textAlign = 'left';
      g.fillText(fmt(axisMax.right * (1 - i / n)), L + pw + 4, y + 4);
    }
    g.textAlign = 'center';
    for (let t = 0; t <= tMax + 1e-9; t += tAx.step) g.fillText(`${fmt(t)} s`, L + (t / tMax) * pw, H - 5);

    this.series.forEach((s, i) => {
      const ys = this.ys[i];
      const m = axisMax[s.axis];
      g.strokeStyle = s.color;
      g.lineWidth = 1.6;
      g.beginPath();
      for (let k = 0; k < ys.length; k++) {
        const x = L + (this.t[k] / tMax) * pw;
        const y = T + ph * (1 - ys[k] / m);
        if (k === 0) g.moveTo(x, y);
        else g.lineTo(x, y);
      }
      g.stroke();
      g.fillStyle = s.color;
      g.textAlign = s.axis === 'left' ? 'left' : 'right';
      g.fillText(`${s.label} (${s.unit})`, s.axis === 'left' ? L : L + pw, 12);
    });
  }
}

function fmt(v: number): string {
  // up to two decimals, no trailing zeros, Swedish decimal comma
  return String(Math.round(v * 100) / 100).replace('.', ',');
}
