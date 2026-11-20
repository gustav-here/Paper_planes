import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import * as V from '../../core/math/vec3';
import * as Q from '../../core/math/quat';
import type { Vec3 } from '../../core/math/vec3';
import type { PaperModel } from '../../core/paper/model';
import { buildAirframe } from '../../core/physics/airframe';
import type { Airframe } from '../../core/physics/airframe';
import { DEFAULT_CONFIG } from '../../core/physics/config';
import { DEFAULT_LAUNCH, launchState } from '../../core/physics/launch';
import type { LaunchParams } from '../../core/physics/launch';
import { FlightSim } from '../../core/physics/simulation';
import { cloneState } from '../../core/physics/rigidBody';
import type { BodyState } from '../../core/physics/rigidBody';
import type { Stage, View } from '../../render/SceneSetup';
import { createWorld } from '../../render/World';
import type { World } from '../../render/World';
import { FlightView } from '../../render/FlightView';
import type { FixedStepper } from '../../app/GameLoop';
import { TimeSeriesPlot } from '../hud/TimeSeriesPlot';
import { loadFlights, saveFlight } from '../hud/flightLogStore';
import type { StoredFlight } from '../hud/flightLogStore';

type Phase = 'aim' | 'flying' | 'done';
type CamMode = 'follow' | 'side' | 'free';

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const DEG = Math.PI / 180;
const fmt = (x: number, d = 2) => {
  const s = x.toFixed(d);
  return (Number(s) === 0 ? (0).toFixed(d) : s).replace('.', ',');
};
const AIM_KEY = 'paperplanes.aim.v1';

interface Slider {
  key: keyof LaunchParams;
  id: string;
  unit: string;
  digits: number;
}

const SLIDERS: Slider[] = [
  { key: 'speed', id: 'aim-speed', unit: 'm/s', digits: 1 },
  { key: 'elevationDeg', id: 'aim-elev', unit: '°', digits: 0 },
  { key: 'headingDeg', id: 'aim-head', unit: '°', digits: 0 },
  { key: 'aoaDeg', id: 'aim-aoa', unit: '°', digits: 0 },
  { key: 'heightM', id: 'aim-height', unit: 'm', digits: 1 },
];

/**
 * Throw mode: aim (speed, climb angle, heading, nose angle, release height), then free flight
 * stepped at the fixed physics rate, a following camera, debug overlays, graphs and results.
 */
export class FlightMode {
  readonly camera = new THREE.PerspectiveCamera(50, 1, 0.01, 300);
  readonly view: View;
  private world: World;
  private controls: OrbitControls;
  private fv: FlightView;
  private af: Airframe | null = null;
  private designName = '';
  private sim: FlightSim | null = null;
  private prev: BodyState | null = null;
  private launch: LaunchParams = { ...DEFAULT_LAUNCH };
  private phase: Phase = 'aim';
  private cam: CamMode = 'follow';
  private debug = false;
  private paused = false;
  private scale = 1;
  private plot: TimeSeriesPlot;
  private nextSample = 0;
  private aimArrow: THREE.ArrowHelper;
  private lastTarget = new THREE.Vector3();
  private shownImpact = false;
  active = false;

  constructor(
    stage: Stage,
    private stepper: FixedStepper,
    private onExit: () => void,
  ) {
    this.world = createWorld();
    this.fv = new FlightView(this.world.scene);
    stage.addCamera(this.camera);
    this.controls = new OrbitControls(this.camera, stage.canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.minDistance = 0.3;
    this.controls.maxDistance = 40;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.enabled = false;
    this.view = { scene: this.world.scene, camera: this.camera, update: () => this.controls.update() };
    this.aimArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0xe0645a, 0.12, 0.07);
    this.world.scene.add(this.aimArrow);
    this.plot = new TimeSeriesPlot($('fl-plot'), [
      { label: 'Höjd', unit: 'm', color: '#4fa3ff', axis: 'left' },
      { label: 'Fart', unit: 'm/s', color: '#ffb020', axis: 'right' },
    ]);
    this.restoreAim();
    this.bindDom();
  }

  // ---------------------------------------------------------------- lifecycle

  enter(model: PaperModel, design: { name: string; orientation: { nose: Vec3; up: Vec3 } }) {
    this.active = true;
    this.controls.enabled = true;
    this.designName = design.name;
    $('fl-title').textContent = design.name;
    try {
      this.af = buildAirframe(model, design.orientation, DEFAULT_CONFIG);
    } catch (e) {
      this.af = null;
      $('aim-info').textContent = `Kunde inte bygga flygplanet: ${String(e)}`;
      return;
    }
    this.fv.setAirframe(model, this.af);
    this.toAim(true);
  }

  exit() {
    this.active = false;
    this.controls.enabled = false;
    this.sim = null;
    this.setPaused(false);
  }

  private toAim(resetCamera: boolean) {
    this.phase = 'aim';
    this.sim = null;
    this.prev = null;
    this.fv.clear();
    this.plot.clear();
    this.shownImpact = false;
    $('result-card').classList.remove('visible');
    $('aim-panel').classList.add('visible');
    this.aimArrow.visible = true;
    this.updateAimPose();
    if (resetCamera) this.aimCamera();
    this.updateStats();
  }

  private throwNow() {
    if (!this.af) return;
    this.sim = new FlightSim(this.af, DEFAULT_CONFIG, { ...this.launch });
    this.prev = cloneState(this.sim.state);
    this.fv.clear();
    this.plot.clear();
    this.nextSample = 0;
    this.shownImpact = false;
    this.phase = 'flying';
    this.aimArrow.visible = false;
    $('aim-panel').classList.remove('visible');
    $('result-card').classList.remove('visible');
    this.stepper.reset();
    this.setPaused(false);
    this.lastTarget.set(...this.sim.state.x);
  }

  // ---------------------------------------------------------------- simulation

  /** One fixed physics step (called by the game loop at 240 Hz simulated time). */
  step() {
    if (!this.active || this.phase !== 'flying' || !this.sim) return;
    this.prev = cloneState(this.sim.state);
    this.sim.step();
    const log = this.sim.log;
    if (log.firstImpact && !this.shownImpact) {
      this.shownImpact = true;
      this.fv.markImpact(log.firstImpact);
    }
    if (this.sim.done) this.finish();
  }

  private finish() {
    const sim = this.sim!;
    const log = sim.log;
    this.phase = 'done';
    if (log.rest) this.fv.markRest(log.rest, log.firstImpact);
    const flights = log.firstImpact
      ? saveFlight({
          date: new Date().toISOString(),
          design: this.designName,
          launch: { ...this.launch },
          distance: log.firstImpact.distance,
          time: log.firstImpact.t,
          impactSpeed: log.firstImpact.speed,
          restDistance: log.rest?.distance ?? null,
          maxHeight: log.maxHeight,
        })
      : loadFlights();
    this.showResult(flights);
  }

  // ---------------------------------------------------------------- per frame

  frame(_frameDt: number, alpha: number) {
    if (!this.active) return;
    let x: Vec3;
    let q: Q.Quat;
    if (this.sim && this.prev) {
      const a = this.phase === 'flying' ? alpha : 1;
      x = V.lerp(this.prev.x, this.sim.state.x, a);
      q = Q.slerp(Q.normalize(this.prev.q), Q.normalize(this.sim.state.q), a);
      this.fv.setPose(x, q);
      this.consumeSamples();
      if (this.debug) this.fv.updateDebug(x, q, this.sim.worldVelocity(), this.sim.debugForces());
    } else {
      const p = this.fv.plane.position;
      x = [p.x, p.y, p.z];
    }
    this.world.followLight(new THREE.Vector3(...x));
    this.updateCamera(x);
    this.updateStats();
    if (this.debug) this.plot.draw();
  }

  private consumeSamples() {
    const s = this.sim!.log.samples;
    for (; this.nextSample < s.length; this.nextSample++) {
      const p = s[this.nextSample];
      this.fv.addTrailPoint(p.pos, p.phase !== 'flight');
      this.plot.push(p.t, [Math.max(0, p.pos[1]), V.length(p.vel)]);
    }
  }

  private headingDir(): THREE.Vector3 {
    const h = (this.launch.headingDeg ?? 0) * DEG;
    return new THREE.Vector3(Math.cos(h), 0, Math.sin(h));
  }

  private aimCamera() {
    const p = new THREE.Vector3(0, this.launch.heightM, 0);
    const f = this.headingDir();
    const side = new THREE.Vector3(-f.z, 0, f.x);
    this.camera.position.copy(p).addScaledVector(f, -1.4).addScaledVector(side, 0.55).add(new THREE.Vector3(0, 0.45, 0));
    this.controls.target.copy(p).addScaledVector(f, 1.2);
    this.lastTarget.copy(p);
    this.controls.update();
  }

  private updateCamera(x: Vec3) {
    const p = new THREE.Vector3(...x);
    if (this.phase === 'aim') return;
    if (this.cam === 'follow') {
      // keep the user's orbit offset, but carry it along with the plane
      const d = p.clone().sub(this.lastTarget);
      this.camera.position.add(d);
      this.controls.target.add(d);
    } else if (this.cam === 'side') {
      const f = this.headingDir();
      const side = new THREE.Vector3(-f.z, 0, f.x);
      this.camera.position.copy(p).addScaledVector(side, -3.5).add(new THREE.Vector3(0, 0.4, 0));
      this.controls.target.copy(p);
    }
    this.lastTarget.copy(p);
  }

  private updateAimPose() {
    if (!this.af) return;
    const s = launchState(this.af, this.launch);
    this.fv.setPose(s.x, s.q);
    const el = this.launch.elevationDeg * DEG;
    const f = this.headingDir();
    const dir = f.clone().multiplyScalar(Math.cos(el)).add(new THREE.Vector3(0, Math.sin(el), 0)).normalize();
    this.aimArrow.position.set(...s.x);
    this.aimArrow.setDirection(dir);
    this.aimArrow.setLength(0.25 + 0.12 * this.launch.speed, 0.12, 0.07);
  }

  private updateStats() {
    const el = $('fl-stats');
    if (!this.sim) {
      el.innerHTML = `<b>Sikta</b> och tryck <kbd>Mellanslag</kbd> för att kasta.`;
      return;
    }
    const s = this.sim.log.samples.at(-1)!;
    const dist = Math.hypot(s.pos[0], s.pos[2]);
    const phase = { flight: 'I luften', ground: 'På golvet', rest: 'Stilla', timeout: 'Tidsgräns' }[s.phase];
    el.innerHTML = [
      `<b>${phase}</b>${this.paused ? ' · pausad' : ''}${this.scale !== 1 ? ` · ${String(this.scale).replace('.', ',')}×` : ''}`,
      `Tid ${fmt(s.t)} s`,
      `Höjd ${fmt(Math.max(0, s.pos[1]))} m`,
      `Fart ${fmt(V.length(s.vel), 1)} m/s`,
      `Anfallsvinkel ${V.length(s.vel) > 0.3 ? `${fmt(s.alphaDeg, 0)}°` : '–'}`,
      `Stigning ${fmt(s.pitchDeg, 0)}°`,
      `Avstånd ${fmt(dist)} m`,
    ].join('<br>');
  }

  private showResult(flights: StoredFlight[]) {
    const log = this.sim!.log;
    const card = $('result-card');
    const imp = log.firstImpact;
    const rest = log.rest;
    const best = flights.filter((f) => f.design === this.designName).reduce<StoredFlight | null>(
      (b, f) => (!b || f.distance > b.distance ? f : b),
      null,
    );
    const rows: string[] = imp
      ? [
          `<tr><th>Distans (nedslag)</th><td><b>${fmt(imp.distance)} m</b></td></tr>`,
          `<tr><th>Flygtid</th><td><b>${fmt(imp.t)} s</b></td></tr>`,
          `<tr><th>Nedslagsfart</th><td>${fmt(imp.speed, 1)} m/s</td></tr>`,
          `<tr><th>Maxhöjd</th><td>${fmt(log.maxHeight)} m</td></tr>`,
          rest
            ? `<tr><th>Vilopunkt</th><td>${fmt(rest.distance)} m (+${fmt(Math.hypot(rest.pos[0] - imp.pos[0], rest.pos[2] - imp.pos[2]))} m glid/studs)</td></tr>`
            : `<tr><th>Vilopunkt</th><td>– (tidsgräns)</td></tr>`,
        ]
      : [`<tr><td colspan="2">Planet landade aldrig inom tidsgränsen.</td></tr>`];
    const recent = flights
      .slice(0, 6)
      .map(
        (f) =>
          `<li><span>${escapeHtml(f.design)}</span><span>${fmt(f.distance)} m · ${fmt(f.time)} s · ${fmt(f.launch.speed, 1)} m/s</span></li>`,
      )
      .join('');
    card.innerHTML = `
      <h3>Resultat</h3>
      <table>${rows.join('')}</table>
      ${best ? `<p class="best">Bästa för ”${escapeHtml(this.designName)}”: ${fmt(best.distance)} m</p>` : ''}
      <div class="actions">
        <button id="res-again" class="primary">Kasta igen (R)</button>
        <button id="res-aim">Ändra kast</button>
        <button id="res-back">Tillbaka till vikning</button>
      </div>
      <h4>Senaste kast</h4><ol class="recent">${recent}</ol>`;
    card.classList.add('visible');
    $('res-again').onclick = () => this.throwNow();
    $('res-aim').onclick = () => this.toAim(true);
    $('res-back').onclick = () => this.onExit();
  }

  // ---------------------------------------------------------------- controls

  private setPaused(p: boolean) {
    this.paused = p;
    this.stepper.timeScale = p ? 0 : this.scale;
    $('fl-pause').textContent = p ? '▶ Fortsätt' : '⏸ Paus';
  }

  private setScale(s: number) {
    this.scale = s;
    ($('fl-speed') as HTMLSelectElement).value = String(s);
    if (!this.paused) this.stepper.timeScale = s;
  }

  private setDebug(on: boolean) {
    this.debug = on;
    this.fv.setDebugVisible(on);
    $('fl-debug').classList.toggle('on', on);
    $('plot-wrap').classList.toggle('visible', on);
    if (on) this.plot.draw();
  }

  private setCam(c: CamMode) {
    this.cam = c;
    ($('fl-cam') as HTMLSelectElement).value = c;
  }

  private restoreAim() {
    try {
      const raw = localStorage.getItem(AIM_KEY);
      if (raw) this.launch = { ...this.launch, ...(JSON.parse(raw) as Partial<LaunchParams>) };
    } catch {
      /* ignore */
    }
  }

  private storeAim() {
    try {
      localStorage.setItem(AIM_KEY, JSON.stringify(this.launch));
    } catch {
      /* ignore */
    }
  }

  private bindDom() {
    for (const s of SLIDERS) {
      const input = $<HTMLInputElement>(s.id);
      const out = $(`${s.id}-v`);
      const show = () => (out.textContent = `${fmt(Number(input.value), s.digits)} ${s.unit}`);
      input.value = String(this.launch[s.key] ?? 0);
      show();
      input.oninput = () => {
        (this.launch as unknown as Record<string, number>)[s.key] = Number(input.value);
        show();
        this.storeAim();
        if (this.phase === 'aim') this.updateAimPose();
      };
    }
    $('btn-throw').onclick = () => this.throwNow();
    $('fl-back').onclick = () => this.onExit();
    $('fl-pause').onclick = () => this.setPaused(!this.paused);
    $('fl-step').onclick = () => {
      if (!this.paused) this.setPaused(true);
      this.step();
    };
    const speed = $<HTMLSelectElement>('fl-speed');
    speed.onchange = () => {
      this.setScale(Number(speed.value));
      speed.blur();
    };
    const cam = $<HTMLSelectElement>('fl-cam');
    cam.onchange = () => {
      this.setCam(cam.value as CamMode);
      cam.blur();
    };
    $('fl-debug').onclick = () => this.setDebug(!this.debug);

    window.addEventListener('keydown', (e) => {
      if (!this.active) return;
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'INPUT' && (e.target as HTMLInputElement).type !== 'range') return;
      switch (e.key) {
        case ' ':
        case 'Enter':
          e.preventDefault();
          if (this.phase === 'flying') this.setPaused(!this.paused);
          else this.throwNow();
          break;
        case 'r':
        case 'R':
          this.throwNow();
          break;
        case 'p':
        case 'P':
          this.setPaused(!this.paused);
          break;
        case '.':
          if (!this.paused) this.setPaused(true);
          this.step();
          break;
        case 'd':
        case 'D':
          this.setDebug(!this.debug);
          break;
        case 'c':
        case 'C': {
          const order: CamMode[] = ['follow', 'side', 'free'];
          this.setCam(order[(order.indexOf(this.cam) + 1) % order.length]);
          break;
        }
        case '1':
          this.setScale(1);
          break;
        case '2':
          this.setScale(0.5);
          break;
        case '3':
          this.setScale(0.25);
          break;
        case '4':
          this.setScale(0.1);
          break;
        case 'a':
        case 'A':
          this.toAim(true);
          break;
        case 'Escape':
          this.onExit();
          break;
      }
    });
  }
}

function escapeHtml(s: string): string {
  return s.replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]!);
}
