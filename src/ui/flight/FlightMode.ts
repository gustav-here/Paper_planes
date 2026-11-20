import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import * as V from '../../core/math/vec3';
import * as Q from '../../core/math/quat';
import type { Vec3 } from '../../core/math/vec3';
import type { DesignFile, Op } from '../../core/paper/ops';
import { damageOnFolded, damageToOps } from '../../core/physics/damage';
import { relaxCreases } from '../../core/paper/springback';
import type { PaperModel } from '../../core/paper/model';
import * as P from '../../core/math/polygon2d';
import { DEFAULT_CONFIG } from '../../core/physics/config';
import { DEFAULT_LAUNCH, launchState } from '../../core/physics/launch';
import type { LaunchParams } from '../../core/physics/launch';
import { createSimulator, prepare } from '../../core/physics/factory';
import type { Prepared } from '../../core/physics/factory';
import type { PanelPose, Simulator } from '../../core/physics/simulator';
import {
  MODES,
  bestScore,
  formatScore,
  isRoundOver,
  newRound,
  recordScore,
  scoreFlight,
} from '../../core/game/competition';
import type { GameMode, LeaderEntry, Round } from '../../core/game/competition';
import type { Stage, View } from '../../render/SceneSetup';
import { createWorld } from '../../render/World';
import type { World } from '../../render/World';
import { FlightView } from '../../render/FlightView';
import type { FixedStepper } from '../../app/GameLoop';
import { TimeSeriesPlot } from '../hud/TimeSeriesPlot';
import { loadFlights, saveFlight } from '../hud/flightLogStore';
import type { StoredFlight } from '../hud/flightLogStore';
import { leaderboard, submit } from '../hud/leaderboardStore';

type Phase = 'aim' | 'flying' | 'done';
type CamMode = 'follow' | 'side' | 'free';

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const DEG = Math.PI / 180;
const fmt = (x: number, d = 2) => {
  const s = x.toFixed(d);
  return (Number(s) === 0 ? (0).toFixed(d) : s).replace('.', ',');
};
const AIM_KEY = 'paperplanes.aim.v1';
const cfg = DEFAULT_CONFIG;

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
 * Throw mode: aim, free flight at the fixed physics rate (rigid or flexible paper), control
 * flaps on the arrow keys, competition rounds with a leaderboard, replays, debug overlays.
 */
export class FlightMode {
  readonly camera = new THREE.PerspectiveCamera(50, 1, 0.01, 300);
  readonly view: View;
  /** Called after a live flight with the damage it caused, as design ops (stage 6). */
  onDamage: ((ops: Op[]) => void) | null = null;
  /** Called when the player repairs the plane in throw mode. */
  onRepair: (() => void) | null = null;
  /** The damaged plane, used from the next throw on. */
  private pending: Prepared | null = null;
  private world: World;
  private controls: OrbitControls;
  private fv: FlightView;
  /** The player's plane (incl. damage), and what is currently flying (may be a replay). */
  private mine: Prepared | null = null;
  private current: Prepared | null = null;
  private sim: Simulator | null = null;
  private prevPoses: PanelPose[] = [];
  private curPoses: PanelPose[] = [];
  private prevPos: Vec3 = [0, 0, 0];
  private launch: LaunchParams = { ...DEFAULT_LAUNCH };
  private phase: Phase = 'aim';
  private cam: CamMode = 'follow';
  private debug = true;
  private paused = false;
  private scale = 1;
  private plot: TimeSeriesPlot;
  private nextSample = 0;
  private aimArrow: THREE.ArrowHelper;
  private lastTarget = new THREE.Vector3();
  private shownImpact = false;
  private mode: GameMode = 'free';
  private round: Round | null = null;
  private replaying: LeaderEntry | null = null;
  private keys = { up: false, down: false, left: false, right: false };
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
    // close enough to inspect a crushed nose or a fold
    this.controls.minDistance = 0.08;
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
    // flight data (forces, centre of mass, graphs) is shown from the start
    this.setDebug(true);
    // the info panels sit below the top bar, however many rows it wraps into
    const bar = document.querySelector('.flight-bar') as HTMLElement;
    new ResizeObserver(() => {
      if (bar.offsetHeight) document.documentElement.style.setProperty('--flight-top-h', `${bar.offsetTop + bar.offsetHeight + 10}px`);
    }).observe(bar);
  }

  // ---------------------------------------------------------------- lifecycle

  /** Enter throw mode with a design (its ops already include any damage). */
  enter(design: DesignFile, gridPaper = false) {
    this.active = true;
    this.controls.enabled = true;
    this.fv.setGrid(gridPaper);
    this.replaying = null;
    try {
      this.mine = prepare(design, cfg);
    } catch (e) {
      this.mine = null;
      $('aim-info').textContent = `Kunde inte bygga flygplanet: ${String(e)}`;
      return;
    }
    this.use(this.mine);
    this.startRound();
    this.toAim(true);
  }

  exit() {
    this.active = false;
    this.controls.enabled = false;
    this.sim = null;
    this.setPaused(false);
  }

  /** Show a prepared plane (the player's own or a replayed one). */
  private use(p: Prepared) {
    this.current = p;
    const damaged = p.design.ops.filter((o) => o.id.startsWith('dmg-')).length;
    $('fl-title').textContent = p.design.name + (p.design.flexible ? ' · flexibelt' : ' · stelt') + (damaged ? ` · skadat (${damaged})` : '');
    const probe = createSimulator(p, cfg, this.launch);
    this.fv.setModel(p.model, probe.panels, p.af.mass, p.af.mesh.elements.length);
    this.updateRepair();
    $('aim-controls').innerHTML = p.af.controls.length
      ? `Styr klaffarna med <kbd>↓</kbd> nosen upp, <kbd>↑</kbd> nosen ned och <kbd>←</kbd><kbd>→</kbd> (skevning).`
      : '';
  }

  private startRound() {
    this.round = this.mode === 'free' ? null : newRound(this.mode, (Date.now() ^ (Math.random() * 1e9)) >>> 0);
    this.fv.setTarget(this.round?.target ?? null);
    this.updateModeInfo();
  }

  /** Switch to the damaged plane once the flight that damaged it is over. */
  private applyPending() {
    if (!this.pending) return;
    this.mine = this.pending;
    this.pending = null;
    this.use(this.mine);
  }

  private toAim(resetCamera: boolean) {
    this.applyPending();
    if (this.replaying && this.mine) {
      this.replaying = null;
      this.use(this.mine);
      this.fv.setTarget(this.round?.target ?? null);
    }
    this.phase = 'aim';
    this.sim = null;
    this.fv.clear();
    this.plot.clear();
    this.shownImpact = false;
    $('result-card').classList.remove('visible');
    $('aim-panel').classList.add('visible');
    this.aimArrow.visible = true;
    this.updateAimPose();
    this.updateModeInfo();
    if (resetCamera) this.aimCamera();
    this.updateStats();
    this.updateRepair();
  }

  private throwNow(replay: LeaderEntry | null = null) {
    if (!replay) {
      if (this.replaying && this.mine) {
        this.replaying = null;
        this.use(this.mine);
        this.fv.setTarget(this.round?.target ?? null);
      }
      this.applyPending();
    }
    if (!this.current) return;
    if (!replay && this.round && isRoundOver(this.round)) this.startRound();
    const launch = replay ? replay.launch : { ...this.launch };
    this.sim = createSimulator(this.current, cfg, launch, replay ? replay.inputs : undefined);
    this.curPoses = this.sim.panelPoses();
    this.prevPoses = this.curPoses;
    this.prevPos = this.sim.position();
    this.fv.clear();
    this.plot.clear();
    this.nextSample = 0;
    this.shownImpact = false;
    this.shownCrumples = 0;
    this.phase = 'flying';
    this.aimArrow.visible = false;
    $('aim-panel').classList.remove('visible');
    $('result-card').classList.remove('visible');
    this.stepper.reset();
    this.setPaused(false);
    // carry the camera (and the player's orbit offset) along to the new release point
    const start = new THREE.Vector3(...this.sim.position());
    if (this.cam === 'follow') {
      const d = start.clone().sub(this.lastTarget);
      this.camera.position.add(d);
      this.controls.target.add(d);
    }
    this.lastTarget.copy(start);
    this.updateRepair();
  }

  /** Replay a stored throw exactly (verifies its score). */
  private replay(entry: LeaderEntry) {
    try {
      const p = prepare(entry.design, cfg);
      this.replaying = entry;
      this.use(p);
      this.fv.setTarget(entry.target);
      this.throwNow(entry);
    } catch (e) {
      alert(`Kunde inte spela upp kastet: ${String(e)}`);
    }
  }

  // ---------------------------------------------------------------- simulation

  /** One fixed physics step (called by the game loop at 240 Hz simulated time). */
  step() {
    if (!this.active || this.phase !== 'flying' || !this.sim) return;
    const sim = this.sim;
    if (!this.replaying) {
      const k = this.keys;
      // like a control stick: ↓ (pull) = nose up, ↑ (push) = nose down
      sim.controls.set(sim.stepIndex, (k.down ? 1 : 0) - (k.up ? 1 : 0), (k.right ? 1 : 0) - (k.left ? 1 : 0));
    }
    this.prevPoses = this.curPoses;
    this.prevPos = sim.position();
    sim.step();
    this.curPoses = sim.panelPoses();
    const log = sim.log;
    if (log.firstImpact && !this.shownImpact) {
      this.shownImpact = true;
      this.fv.markImpact(log.firstImpact);
    }
    if (sim.done) this.finish();
  }

  private finish() {
    const sim = this.sim!;
    const log = sim.log;
    this.phase = 'done';
    if (log.rest) this.fv.markRest(log.rest, log.firstImpact);
    if (this.replaying) {
      this.showReplayResult(this.replaying);
      return;
    }
    const events = sim.damage();
    if (events.length && this.mine && this.current === this.mine) {
      // damage stays with the plane (until it is refolded): fly the damaged plane from now on
      // damage is expressed on the plane as folded (the design's own geometry)
      const r = damageToOps(this.mine.folded, damageOnFolded(this.mine.model, this.mine.folded, events), this.mine.design.orientation.up, cfg.paper);
      if (r.ops.length) {
        const design: DesignFile = { ...this.mine.design, ops: [...this.mine.design.ops, ...r.ops] };
        try {
          this.pending = prepare(design, cfg);
          this.onDamage?.(r.ops);
          // show all of the damage on the plane where it came to rest
          this.showDamaged(this.pending.model);
        } catch {
          this.pending = null;
        }
      }
    }
    this.updateRepair();
    const flights = log.firstImpact
      ? saveFlight({
          date: new Date().toISOString(),
          design: this.current?.design.name ?? '',
          launch: { ...log.launch },
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
    if (this.sim) {
      const a = this.phase === 'flying' ? alpha : 1;
      const poses = this.curPoses.map((c, i) => {
        const p = this.prevPoses[i] ?? c;
        return { q: Q.slerp(Q.normalize(p.q), Q.normalize(c.q), a), t: V.lerp(p.t, c.t, a) };
      });
      this.fv.setPanelPoses(poses);
      x = V.lerp(this.prevPos, this.sim.position(), a);
      if (this.phase === 'flying') this.showCrumples();
      this.consumeSamples();
      if (this.debug) this.fv.updateDebug(x, this.sim.worldVelocity(), this.sim.debugForces());
    } else {
      x = [0, this.launch.heightM, 0];
    }
    this.world.followLight(new THREE.Vector3(...x));
    this.updateCamera(x);
    this.updateStats();
    if (this.debug) this.plot.draw();
  }

  private shownCrumples = 0;

  /** A crushed edge shows the moment it happens (the flight itself goes on as simulated). */
  private showCrumples() {
    const sim = this.sim;
    const p = this.current;
    if (!sim || !p || this.replaying) return;
    const events = sim.damage().filter((e) => e.kind === 'crumple');
    if (events.length <= this.shownCrumples) return;
    this.shownCrumples = events.length;
    const r = damageToOps(p.folded, damageOnFolded(p.model, p.folded, events), p.design.orientation.up, cfg.paper);
    if (!r.ops.length) return;
    this.showDamaged(relaxCreases(r.model, [...p.design.ops, ...r.ops], cfg.paper.creaseSpringback));
  }

  /**
   * Draw a damaged version of the flying plane: its paper is matched to the simulated panels
   * through sheet coordinates, so it keeps moving with them.
   */
  private showDamaged(model: PaperModel) {
    const sim = this.sim;
    const p = this.current;
    if (!sim || !p) return;
    const owner = new Map<number, number>();
    sim.panels.forEach((ids, i) => ids.forEach((id) => owner.set(id, i)));
    const panels: number[][] = sim.panels.map(() => []);
    for (const f of model.facets) {
      const c = P.centroid(f.poly);
      const src = p.model.facets.find((g) => P.pointInConvex(c, g.poly, -1e-9));
      panels[(src && owner.get(src.id)) ?? 0].push(f.id);
    }
    this.fv.setModel(model, panels, p.af.mass, p.af.mesh.elements.length);
    this.fv.setPanelPoses(this.curPoses);
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
    if (!this.current) return;
    const s = launchState(this.current.af, this.launch);
    const probe = createSimulator(this.current, cfg, this.launch);
    this.fv.setPanelPoses(probe.panelPoses());
    const el = this.launch.elevationDeg * DEG;
    const f = this.headingDir();
    const dir = f.clone().multiplyScalar(Math.cos(el)).add(new THREE.Vector3(0, Math.sin(el), 0)).normalize();
    this.aimArrow.position.set(...s.x);
    this.aimArrow.setDirection(dir);
    this.aimArrow.setLength(0.25 + 0.12 * this.launch.speed, 0.12, 0.07);
  }

  private updateModeInfo() {
    const el = $('aim-mode-info');
    if (!this.round) {
      el.innerHTML = '';
      return;
    }
    const m = MODES[this.round.mode];
    const n = this.round.scores.length;
    const best = bestScore(this.round);
    const target = this.round.target ? ` · mål ${fmt(Math.hypot(this.round.target.x, this.round.target.z), 1)} m bort` : '';
    el.innerHTML = `<b>${m.name}</b>${target}<br>Kast ${Math.min(n + 1, m.attempts)} av ${m.attempts}${best !== null ? ` · bästa ${formatScore(this.round.mode, best)}` : ''}<br><span class="muted">${m.description}</span>`;
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
    const defl = this.sim.deflectionsDeg();
    el.innerHTML = [
      `<b>${this.replaying ? 'Uppspelning · ' : ''}${phase}</b>${this.paused ? ' · pausad' : ''}${this.scale !== 1 ? ` · ${String(this.scale).replace('.', ',')}×` : ''}`,
      `Tid ${fmt(s.t)} s`,
      `Höjd ${fmt(Math.max(0, s.pos[1]))} m`,
      `Fart ${fmt(V.length(s.vel), 1)} m/s`,
      `Anfallsvinkel ${V.length(s.vel) > 0.3 ? `${fmt(s.alphaDeg, 0)}°` : '–'}`,
      `Stigning ${fmt(s.pitchDeg, 0)}°`,
      `Avstånd ${fmt(dist)} m`,
      ...(defl.length ? [`Klaffar ${defl.map((d) => `${fmt(d, 0)}°`).join(' / ')}`] : []),
    ].join('<br>');
  }

  // ---------------------------------------------------------------- results

  private showResult(flights: StoredFlight[]) {
    const sim = this.sim!;
    const log = sim.log;
    const imp = log.firstImpact;
    const rest = log.rest;
    const name = this.current?.design.name ?? '';
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
    const dmg = sim.damage();
    const damageNote = dmg.length
      ? `<p class="verify bad">Planet skadades: ${dmg.map((d) => ({ crumple: 'skrynklat', bend: 'böjt veck', tear: 'rivet' })[d.kind]).join(', ')}. Skadan finns kvar tills du reparerar eller viker om. <button id="res-repair">🔧 Reparera</button></p>`
      : '';

    let modeHtml = '';
    let again = 'Kasta igen (R)';
    if (this.round) {
      const score = scoreFlight(this.round.mode, log, this.round.target);
      this.round = recordScore(this.round, score);
      const m = MODES[this.round.mode];
      const best = bestScore(this.round);
      let rankHtml = '';
      if (isRoundOver(this.round)) {
        again = 'Ny runda (R)';
        if (best !== null && this.mine) {
          const bestIdx = this.round.scores.indexOf(best);
          const entry: LeaderEntry = {
            mode: this.round.mode,
            score: best,
            designName: name,
            date: new Date().toISOString(),
            design: this.mine.design,
            // only the best throw of the round is stored for replay
            launch: bestIdx === this.round.scores.length - 1 ? { ...log.launch } : this.bestLaunch ?? { ...log.launch },
            inputs: bestIdx === this.round.scores.length - 1 ? [...sim.controls.inputs] : this.bestInputs ?? [],
            target: this.round.target,
          };
          const rank = submit(entry);
          rankHtml = rank >= 0 ? `<br><b>Plats ${rank + 1} på topplistan!</b>` : '';
        }
      } else {
        again = 'Nästa kast (R)';
      }
      // remember the best throw so far in the round (for the leaderboard replay)
      if (score !== null && score === best) {
        this.bestLaunch = { ...log.launch };
        this.bestInputs = [...sim.controls.inputs];
      }
      modeHtml = `<div class="mode-info"><b>${m.name}</b> · kast ${this.round.scores.length} av ${m.attempts}<br>Detta kast: <b>${formatScore(this.round.mode, score)}</b> · bästa: <b>${formatScore(this.round.mode, best)}</b>${isRoundOver(this.round) ? `<br>Rundan klar!${rankHtml}` : ''}</div>`;
      if (isRoundOver(this.round)) {
        this.bestLaunch = null;
        this.bestInputs = null;
      }
    }
    const boardMode: GameMode | null = this.round?.mode ?? null;
    const board = boardMode ? this.boardHtml(boardMode) : this.recentHtml(flights);
    this.renderCard(`
      <h3>Resultat</h3>
      ${modeHtml}
      <table>${rows.join('')}</table>
      ${damageNote}
      <div class="actions">
        <button id="res-again" class="primary">${again}</button>
        <button id="res-aim">Ändra kast</button>
        <button id="res-back">Tillbaka till vikning</button>
      </div>
      ${board}`);
  }

  private bestLaunch: LaunchParams | null = null;
  private bestInputs: LeaderEntry['inputs'] | null = null;

  private recentHtml(flights: StoredFlight[]): string {
    const recent = flights
      .slice(0, 6)
      .map((f) => `<li><span>${escapeHtml(f.design)}</span> ${fmt(f.distance)} m · ${fmt(f.time)} s · ${fmt(f.launch.speed, 1)} m/s</li>`)
      .join('');
    return `<h4>Senaste kast</h4><ol class="board">${recent}</ol>`;
  }

  private boardHtml(mode: GameMode): string {
    const list = leaderboard(mode).slice(0, 5);
    if (!list.length) return `<h4>Topplista · ${MODES[mode].name}</h4><p class="note">Inga resultat än. Gör klart en runda!</p>`;
    const items = list
      .map(
        (e, i) =>
          `<li>${escapeHtml(e.designName)} · <b>${formatScore(mode, e.score)}</b><button data-replay="${i}" title="Spela upp och verifiera">▶</button></li>`,
      )
      .join('');
    return `<h4>Topplista · ${MODES[mode].name}</h4><ol class="board">${items}</ol>`;
  }

  private renderCard(html: string) {
    const card = $('result-card');
    card.innerHTML = html;
    card.classList.add('visible');
    const again = card.querySelector('#res-again') as HTMLButtonElement | null;
    if (again) again.onclick = () => this.throwNow();
    const aim = card.querySelector('#res-aim') as HTMLButtonElement | null;
    if (aim) aim.onclick = () => this.toAim(true);
    const repair = card.querySelector('#res-repair') as HTMLButtonElement | null;
    if (repair) repair.onclick = () => this.repair();
    const back = card.querySelector('#res-back') as HTMLButtonElement | null;
    if (back) back.onclick = () => this.onExit();
    const mode = this.round?.mode ?? this.replaying?.mode ?? null;
    card.querySelectorAll<HTMLButtonElement>('[data-replay]').forEach((b) => {
      b.onclick = () => {
        if (!mode) return;
        const e = leaderboard(mode)[Number(b.dataset.replay)];
        if (e) this.replay(e);
      };
    });
  }

  private showReplayResult(entry: LeaderEntry) {
    const log = this.sim!.log;
    const score = scoreFlight(entry.mode, log, entry.target);
    const ok = score !== null && Math.abs(score - entry.score) < 1e-9 * Math.max(1, Math.abs(entry.score));
    this.renderCard(`
      <h3>Uppspelning</h3>
      <div class="mode-info"><b>${escapeHtml(entry.designName)}</b> · ${MODES[entry.mode].name}<br>Sparat: <b>${formatScore(entry.mode, entry.score)}</b> · uppspelat: <b>${formatScore(entry.mode, score)}</b></div>
      <p class="verify ${ok ? 'ok' : 'bad'}">${ok ? '✓ Verifierat – exakt samma flygning' : '✗ Avviker från det sparade resultatet'}</p>
      <div class="actions">
        <button id="res-aim" class="primary">Tillbaka till mitt plan</button>
        <button id="res-back">Tillbaka till vikning</button>
      </div>
      ${this.boardHtml(entry.mode)}`);
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

  private damagedOps(): number {
    const p = this.pending ?? this.mine;
    return p ? p.design.ops.filter((o) => o.id.startsWith('dmg-')).length : 0;
  }

  private updateRepair() {
    const b = $<HTMLButtonElement>('fl-repair');
    const n = this.damagedOps();
    b.hidden = n === 0 || !!this.replaying;
    b.disabled = this.phase === 'flying';
    b.textContent = `🔧 Reparera (${n})`;
  }

  /** Remove all damage: the plane is as the player folded it again. */
  private repair() {
    const p = this.pending ?? this.mine;
    if (!p || this.phase === 'flying' || this.replaying) return;
    const design: DesignFile = { ...p.design, ops: p.design.ops.filter((o) => !o.id.startsWith('dmg-')) };
    try {
      this.mine = prepare(design, cfg);
    } catch (e) {
      alert(`Kunde inte reparera: ${String(e)}`);
      return;
    }
    this.pending = null;
    this.use(this.mine);
    this.onRepair?.();
    const note = document.querySelector('#result-card .verify.bad');
    if (note) note.outerHTML = `<p class="verify ok">🔧 Planet är reparerat.</p>`;
    if (this.phase === 'aim') this.updateAimPose();
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

  private setMode(m: GameMode) {
    this.mode = m;
    ($('fl-mode') as HTMLSelectElement).value = m;
    this.startRound();
    if (this.phase !== 'flying') this.toAim(false);
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
    const mode = $<HTMLSelectElement>('fl-mode');
    mode.onchange = () => {
      this.setMode(mode.value as GameMode);
      mode.blur();
    };
    $('fl-debug').onclick = () => this.setDebug(!this.debug);
    $('fl-repair').onclick = () => this.repair();

    const arrow = (e: KeyboardEvent, down: boolean) => {
      const map: Record<string, keyof FlightMode['keys']> = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right' };
      const k = map[e.key];
      if (!k) return false;
      this.keys[k] = down;
      e.preventDefault();
      return true;
    };
    window.addEventListener('keyup', (e) => {
      if (this.active) arrow(e, false);
    });
    window.addEventListener('blur', () => (this.keys = { up: false, down: false, left: false, right: false }));
    window.addEventListener('keydown', (e) => {
      if (!this.active) return;
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'INPUT' && (e.target as HTMLInputElement).type !== 'range') return;
      if (tag === 'SELECT') return;
      if (this.phase === 'flying' && arrow(e, true)) return;
      switch (e.key) {
        case ' ':
        case 'Enter':
          e.preventDefault();
          if (this.phase === 'flying') this.setPaused(!this.paused);
          else if (this.phase === 'done' && this.replaying) this.toAim(true);
          else this.throwNow();
          break;
        case 'r':
        case 'R':
          if (this.replaying) this.toAim(true);
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
