import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import { facetVertices } from '../../core/paper/model';
import type { PaperModel } from '../../core/paper/model';
import { buildAirframe } from '../../core/physics/airframe';
import type { Airframe } from '../../core/physics/airframe';
import { DEFAULT_CONFIG } from '../../core/physics/config';
import { centerOfLift, stability } from '../../core/physics/windTunnel';
import { computeAddedMass } from '../../core/physics/addedMass';

const AXES: Record<string, Vec3> = {
  '+X': [1, 0, 0],
  '−X': [-1, 0, 0],
  '+Y': [0, 1, 0],
  '−Y': [0, -1, 0],
  '+Z': [0, 0, 1],
  '−Z': [0, 0, -1],
};

const axisName = (v: Vec3) =>
  Object.entries(AXES).find(([, a]) => V.dot(a, V.normalize(v)) > 0.999)?.[0] ?? '+X';

/**
 * Stage-2 physics readout for the folded model: mass, centre of mass, aero elements and a
 * virtual wind-tunnel stability check. Also lets the player say which way is nose / up.
 */
export class PhysicsPanel {
  private el: HTMLElement;
  private body: HTMLElement;
  private noseSel: HTMLSelectElement;
  private upSel: HTMLSelectElement;
  visible = false;

  constructor(
    private orientation: () => { nose: Vec3; up: Vec3 },
    private setOrientation: (o: { nose: Vec3; up: Vec3 }) => void,
  ) {
    this.el = document.getElementById('physics-panel')!;
    this.body = this.el.querySelector('.body') as HTMLElement;
    this.noseSel = this.el.querySelector('#phys-nose') as HTMLSelectElement;
    this.upSel = this.el.querySelector('#phys-up') as HTMLSelectElement;
    for (const sel of [this.noseSel, this.upSel]) {
      for (const k of Object.keys(AXES)) {
        const o = document.createElement('option');
        o.value = k;
        o.textContent = k;
        sel.appendChild(o);
      }
      sel.onchange = () => {
        const nose = AXES[this.noseSel.value];
        const up = AXES[this.upSel.value];
        if (Math.abs(V.dot(nose, up)) > 0.5) {
          this.body.insertAdjacentHTML('afterbegin', '<p class="warn">Nos och upp måste vara vinkelräta.</p>');
          this.syncSelects();
          return;
        }
        sel.blur();
        this.setOrientation({ nose, up });
      };
    }
  }

  setVisible(on: boolean) {
    this.visible = on;
    this.el.classList.toggle('visible', on);
  }

  private syncSelects() {
    const o = this.orientation();
    this.noseSel.value = axisName(o.nose);
    this.upSel.value = axisName(o.up);
  }

  /** Where the lift acts (neutral point, model space), from the last update. */
  centerOfLift: Vec3 | null = null;

  /** Rebuild the airframe and readout for a model. Returns the airframe for the 3D overlay. */
  update(model: PaperModel): Airframe | null {
    this.centerOfLift = null;
    this.syncSelects();
    if (!this.visible) return null;
    const cfg = DEFAULT_CONFIG;
    let af: Airframe;
    try {
      af = buildAirframe(model, this.orientation(), cfg);
    } catch (e) {
      this.body.innerHTML = `<p class="warn">Kunde inte räkna fram fysiken: ${String(e)}</p>`;
      return null;
    }
    const mp = af.massProps;
    const vs = model.facets.flatMap(facetVertices);
    const noseTip = Math.max(...vs.map((v) => V.dot(v, af.nose)));
    const comBehindNose = (noseTip - V.dot(af.com, af.nose)) * 1000;
    const length = (noseTip - Math.min(...vs.map((v) => V.dot(v, af.nose)))) * 1000;
    const st = stability(af, cfg);
    const col = centerOfLift(af, cfg);
    this.centerOfLift = col?.point ?? null;
    const am = computeAddedMass(af.mesh, cfg.rho, cfg.addedMassScale, cfg.aero.elementSizeM);
    const addedTotal = am.perSurface.reduce((a, b) => a + b, 0);
    const fmt = (x: number, d = 1) => x.toFixed(d).replace('.', ',');
    const rows: [string, string][] = [
      ['Massa', `${fmt(mp.mass * 1000, 2)} g${mp.clipMass > 0 ? ` (varav gem ${fmt(mp.clipMass * 1000, 0)} g)` : ''}`],
      ['Tyngdpunkt', `${fmt(comBehindNose, 0)} mm bakom nosen (${fmt((100 * comBehindNose) / Math.max(length, 1), 0)} % av längden)`],
      ['Ytor / ytelement', `${af.mesh.surfaces.length} / ${af.mesh.elements.length}`],
      ['Vingyta (projicerad)', `${fmt(af.refArea * 1e4, 0)} cm², spännvidd ${fmt(af.refSpan * 1000, 0)} mm`],
      ['Medflyttad luft', `${fmt(addedTotal * 1000, 1)} g (normalriktning, summa ytor)`],
    ];
    if (col) rows.push(['Lyftcentrum', `${fmt(Math.abs(col.behindCom) * 1000, 0)} mm ${col.behindCom >= 0 ? 'bakom' : 'framför'} tyngdpunkten (${col.behindCom > 0 ? 'stabil' : 'instabil'} i tippled)`]);
    if (st.trimAlphaDeg !== null) {
      rows.push(
        ['Trimvinkel α', `${fmt(st.trimAlphaDeg)}°`],
        ['Statisk stabilitet', `Cmα = ${fmt(st.cmAlpha!, 2)} /rad (${st.cmAlpha! < 0 ? 'stabil' : 'instabil'})`],
        ['Glidtal vid trim', fmt(st.LDtrim!, 1)],
      );
    } else {
      rows.push(['Trim', 'ingen stabil trimvinkel – planet kommer inte att glida jämnt']);
    }
    this.body.innerHTML =
      `<table>${rows.map(([k, v]) => `<tr><th>${k}</th><td>${v}</td></tr>`).join('')}</table>` +
      `<p class="note">Vindtunnel vid 5 m/s. Gul kula = tyngdpunkt, orange pil = nos, blå pil = lyftcentrum (där lyftkraften angriper när anfallsvinkeln ändras), prickar = ytelement.</p>`;
    return af;
  }
}
