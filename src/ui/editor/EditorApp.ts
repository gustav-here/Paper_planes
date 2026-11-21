import * as THREE from 'three';
import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import type { PaperModel } from '../../core/paper/model';
import { facetNormal, facetVertices, toModel, toSheet } from '../../core/paper/model';
import * as P from '../../core/math/polygon2d';
import { applyRigid, applyRigidLinear, composeRigid, invertRigid, rigidIdentity } from '../../core/math/transform';
import type { RigidTransform } from '../../core/math/transform';
import { relaxCreases } from '../../core/paper/springback';
import { DEFAULT_CONFIG } from '../../core/physics/config';
import { resolveClip } from '../../core/paper/clips';
import { applyOp } from '../../core/paper/replay';
import type { OpResult } from '../../core/paper/replay';
import { PaperError } from '../../core/paper/errors';
import { validateDesign } from '../../core/paper/ops';
import type { DesignFile, Op } from '../../core/paper/ops';
import { PRESETS } from '../../core/presets';
import { maxLayers } from '../../core/paper/stack';
import type { Stage } from '../../render/SceneSetup';
import { createTable } from '../../render/Table';
import { PaperView } from '../../render/PaperMesh';
import type { PaperHit } from '../../render/PaperMesh';
import { Overlay } from '../../render/Overlay';
import { createClipObject } from '../../render/ClipMesh';
import { createAshPile } from '../../render/AshPile';
import { PhysicsOverlay } from '../../render/PhysicsOverlay';
import { PhysicsPanel } from '../hud/PhysicsPanel';
import { ERROR_TEXT, warningText } from '../hud/messages';
import { History } from './History';
import { FoldTool } from './FoldTool';
import { CutTool } from './CutTool';
import { ClipTool } from './ClipTool';
import { ControlTool } from './ControlTool';
import { controlFacetTags } from '../../core/physics/controlGroups';
import type { FacetTint } from '../../render/PaperMesh';
import type { SnapKind } from './Snapping';
import { ViewBall } from './ViewBall';
import { snapToRightAngles } from './rotationSnap';

/** The 360 ball snaps to straight/flat 90° orientations within this angle. */
const BALL_SNAP_DEG = 3;
import { detectOrientation } from '../../core/paper/orientation';
import { cutIsSelfMirror, foldIsSelfMirror, mirrorCut, mirrorFold } from '../../core/paper/symmetry';
import { symmetryOf } from '../../render/modelCache';
import type { EditorOptions, Ray, Tool, ToolContext, ToastAction, ToastKind } from './toolContext';

type ToolName = 'fold' | 'cut' | 'clip' | 'tape' | 'control' | 'view';

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

/** The folding editor: owns history, the paper view on the table, tools and the DOM HUD. */
export class EditorApp implements ToolContext {
  readonly options: EditorOptions = {
    foldScope: 'top',
    foldMirror: false,
    pointToPoint: false,
    cutDepth: 'all',
    cutMirror: false,
  };
  readonly overlay = new Overlay();
  private readonly history = new History();
  private readonly root = new THREE.Group();
  private readonly paper = new PaperView();
  private readonly clipGroup = new THREE.Group();
  private readonly noseArrow: THREE.ArrowHelper;
  private readonly physicsOverlay = new PhysicsOverlay();
  private readonly physics: PhysicsPanel;
  private readonly viewBall: ViewBall;
  /** Unsnapped orientation accumulated while dragging the ball (the shown one may be snapped). */
  private ballRaw: THREE.Quaternion | null = null;
  private ghost: THREE.Object3D | null = null;
  private preview: PaperModel | null = null;
  private shownModel: PaperModel | null = null;
  private tools: Record<Exclude<ToolName, 'view'>, Tool>;
  private toolName: ToolName = 'fold';
  private xray = false;
  private autoXRay = false;
  private viewQ = new THREE.Quaternion();
  private viewTarget = new THREE.Quaternion();
  private rootPos = new THREE.Vector3();
  private rootPosInit = false;
  private raycaster = new THREE.Raycaster();
  private pointerDown = false;
  /** Model whose orientation was last auto-detected, and whether the player overrode it since. */
  private orientedModel: PaperModel | null = null;
  private manualOrientation = false;
  /** False while the throw/flight mode is shown: the editor ignores input. */
  active = true;
  /** Called when the player wants to throw the current model. */
  onFly: ((design: DesignFile, gridPaper: boolean) => void) | null = null;
  /** Grid-paper print (per-viewer preference). */
  gridPaper = false;

  constructor(private stage: Stage) {
    stage.scene.add(createTable());
    stage.scene.add(this.root);
    this.root.add(this.paper.group, this.clipGroup, this.overlay.group, this.physicsOverlay.group, this.ghostHolder);
    this.physics = new PhysicsPanel(
      () => this.history.meta.orientation,
      (o) => {
        this.history.meta.orientation = o;
        this.manualOrientation = true;
        this.placeNoseArrow(this.display());
        this.refreshPhysics();
      },
    );
    this.viewBall = new ViewBall(
      $('view-ball'),
      stage.camera,
      (q) => {
        // follow the ball directly (no easing while dragging); snap to straight/flat near 90° steps
        if (!this.ballRaw) this.ballRaw = this.viewTarget.clone();
        this.ballRaw.premultiply(q).normalize();
        const s = snapToRightAngles(this.ballRaw, this.uprightQuat(), BALL_SNAP_DEG);
        this.viewTarget.copy(s.q);
        this.viewQ.copy(s.q);
        this.viewBall.setSnapped(s.snapped);
      },
      // upright: nose away from the player, centre fold down (a flat sheet simply lies flat)
      () => this.showUpright(),
      () => {
        this.ballRaw = null;
        this.viewBall.setSnapped(false);
      },
    );
    this.noseArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.04, 0xffd34d, 0.012, 0.008);
    this.root.add(this.noseArrow);
    this.tools = { fold: new FoldTool(this), cut: new CutTool(this), clip: new ClipTool(this), tape: new ClipTool(this, 'tape'), control: new ControlTool(this) };
    this.bindDom();
    this.bindPointer();
    // keep side panels below the (possibly wrapping) top bar
    const bar = document.querySelector('.topbar') as HTMLElement;
    new ResizeObserver(() => document.documentElement.style.setProperty('--top-h', `${bar.offsetHeight + 18}px`)).observe(bar);
    this.setTool('fold');
    this.syncView();
  }

  // ---------------------------------------------------------------- ToolContext

  model(): PaperModel {
    return this.history.current;
  }

  /**
   * Save the design as a file. The player chooses its name and where it goes: the browser's own
   * save dialog where there is one (Chrome, Edge), otherwise a question for the name and a
   * download (the browser decides the folder, or asks if it is set to). The model takes the
   * file's name.
   */
  private async save() {
    const safe = (n: string) => n.replace(/[^\wåäöÅÄÖ\- ]+/g, '').trim();
    const suggested = `${safe(this.history.meta.name) || 'modell'}.json`;
    const contents = (name: string) => {
      this.history.meta = { ...this.history.meta, name };
      return JSON.stringify(this.history.toDesign(), null, 2);
    };
    type SaveHandle = { name: string; createWritable(): Promise<{ write(d: string): Promise<void>; close(): Promise<void> }> };
    const picker = (window as unknown as { showSaveFilePicker?: (o: unknown) => Promise<SaveHandle> }).showSaveFilePicker;
    if (picker) {
      try {
        const handle = await picker({
          suggestedName: suggested,
          types: [{ description: 'Pappersflygplan', accept: { 'application/json': ['.json'] } }],
        });
        const name = handle.name.replace(/\.json$/i, '');
        const w = await handle.createWritable();
        await w.write(contents(name));
        await w.close();
        this.toast(`Sparade ${handle.name}.`, 'ok');
      } catch (e) {
        // the player closed the dialog: nothing to do
        if ((e as { name?: string }).name !== 'AbortError') this.toast(`Kunde inte spara: ${String(e)}`, 'error');
      }
      return;
    }
    const answer = prompt('Spara modellen som:', suggested.replace(/\.json$/, ''));
    if (answer === null) return;
    const name = safe(answer) || 'modell';
    const blob = new Blob([contents(name)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${name}.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  nose(): Vec3 {
    return this.history.meta.orientation.nose;
  }

  previousModel(): PaperModel {
    return this.history.previous;
  }

  nextId(prefix: string): string {
    return this.history.nextId(prefix);
  }

  setPreview(m: PaperModel | null) {
    this.preview = m;
    // the physics overlay describes the committed model; hide it while previewing a fold
    this.physicsOverlay.group.visible = m === null;
    this.syncPaper();
  }

  commit(op: Op): OpResult | null {
    try {
      const r = applyOp(this.history.current, op, { penetration: 'sweep' });
      // paper may not pass through paper (designs loaded from files only get a warning)
      if (op.type === 'fold' && r.warnings.some((w) => w.code === 'penetration')) {
        this.toast('Vecket stoppades: papperet skulle gå igenom annat papper.', 'error');
        return null;
      }
      this.history.push(op, r.model);
      for (const w of r.warnings) this.toast(warningText(w), w.code === 'penetration' ? 'warn' : 'info');
      this.syncView();
      return r;
    } catch (e) {
      this.toast(e instanceof PaperError ? ERROR_TEXT[e.code] : `Fel: ${String(e)}`, 'error');
      return null;
    }
  }

  replaceLast(op: Op, model: PaperModel) {
    this.history.replaceLast(op, model);
    this.syncView();
  }

  toast(text: string, kind: ToastKind = 'info', actions: ToastAction[] = []) {
    const el = document.createElement('div');
    el.className = `toast ${kind}`;
    el.textContent = text;
    for (const a of actions) {
      const b = document.createElement('button');
      b.textContent = a.label;
      b.onclick = () => {
        el.remove();
        a.run();
      };
      el.appendChild(document.createElement('br'));
      el.appendChild(b);
    }
    $('toasts').prepend(el);
    setTimeout(() => el.remove(), kind === 'error' ? 7000 : actions.length ? 9000 : 4500);
  }

  hint(text: string) {
    $('hint').textContent = text;
  }

  mouse(e: PointerEvent): [number, number] {
    const r = this.stage.canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }

  private setRay(e: PointerEvent) {
    const r = this.stage.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.stage.camera);
  }

  ray(e: PointerEvent): Ray {
    this.setRay(e);
    this.root.updateMatrixWorld();
    const o = this.root.worldToLocal(this.raycaster.ray.origin.clone());
    const d = this.raycaster.ray.direction.clone().applyQuaternion(this.root.quaternion.clone().invert());
    const r: Ray = { origin: [o.x, o.y, o.z], dir: [d.x, d.y, d.z] };
    if (!this.workFrame) return r;
    // into the folded coordinates of the paper being worked on
    const inv = invertRigid(this.workFrame);
    return { origin: applyRigid(inv, r.origin), dir: applyRigidLinear(inv, r.dir) };
  }

  hitPaper(e: PointerEvent): (PaperHit & { frame: RigidTransform }) | null {
    this.setRay(e);
    this.root.updateMatrixWorld();
    const h = this.paper.hit(this.raycaster);
    if (!h) return null;
    // the paper is shown as it rests; find the same paper in the folded model (sheet coordinates)
    const shown = this.paper.model!;
    const ideal = this.history.current;
    const f = shown.facets.find((x) => x.id === h.facetId);
    if (!f || shown === ideal) return { ...h, frame: rigidIdentity() };
    const s = toSheet(f, h.point);
    const same = ideal.facets.find((x) => x.id === f.id);
    const g =
      same && P.insideDistance(s, same.poly) > -1e-7
        ? same
        : ideal.facets.reduce((best, x) => (P.insideDistance(s, x.poly) > P.insideDistance(s, best.poly) ? x : best));
    return { facetId: g.id, point: toModel(g, s), normal: facetNormal(g), frame: composeRigid(f.T, invertRigid(g.T)) };
  }

  private workFrame: RigidTransform | null = null;

  setFrame(frame: RigidTransform | null) {
    this.workFrame = frame;
    const f = frame ?? rigidIdentity();
    for (const g of [this.overlay.group, this.ghostHolder]) {
      g.quaternion.set(...f.q);
      g.position.set(...f.t);
    }
  }

  hitClip(e: PointerEvent): string | null {
    this.setRay(e);
    const hits = this.raycaster.intersectObject(this.clipGroup, true);
    for (const h of hits) {
      let o: THREE.Object3D | null = h.object;
      while (o && !o.userData.clipId) o = o.parent;
      if (o) return o.userData.clipId as string;
    }
    return null;
  }

  toScreen(p: Vec3): [number, number] | null {
    const q = this.workFrame ? applyRigid(this.workFrame, p) : p;
    const v = this.root.localToWorld(new THREE.Vector3(...q)).project(this.stage.camera);
    if (v.z > 1) return null;
    const r = this.stage.canvas.getBoundingClientRect();
    return [((v.x + 1) / 2) * r.width, ((1 - v.y) / 2) * r.height];
  }

  towardsViewer(n: Vec3, p: Vec3): Vec3 {
    const c = this.root.worldToLocal(this.stage.camera.position.clone());
    return V.dot(n, V.sub([c.x, c.y, c.z], p)) >= 0 ? n : V.negate(n);
  }

  snapDot(screen: [number, number] | null, kind: SnapKind = 'point') {
    const el = $('snap-dot');
    if (!screen) {
      el.style.display = 'none';
      return;
    }
    el.style.display = 'block';
    el.style.left = `${screen[0]}px`;
    el.style.top = `${screen[1]}px`;
    el.classList.toggle('edge', kind === 'edge');
  }

  angleLabel(text: string | null, screen?: [number, number]) {
    const el = $('angle-label');
    if (!text || !screen) {
      el.style.display = 'none';
      return;
    }
    el.style.display = 'block';
    el.textContent = text;
    el.style.left = `${screen[0]}px`;
    el.style.top = `${screen[1]}px`;
  }

  setAutoXRay(on: boolean) {
    if (on === this.autoXRay) return;
    this.autoXRay = on;
    this.applyXRay();
  }

  setGhost(obj: THREE.Object3D | null) {
    if (this.ghost) this.ghostHolder.remove(this.ghost);
    this.ghost = obj;
    if (obj) this.ghostHolder.add(obj);
  }

  /** Ghost objects are built in folded coordinates; this group puts them where the paper is shown. */
  private readonly ghostHolder = new THREE.Group();

  private hoverFacets: number[] | null = null;

  setHoverFacets(ids: number[] | null) {
    this.hoverFacets = ids;
    this.applyTints();
  }

  cycleFoldScope() {
    const sel = $<HTMLSelectElement>('fold-scope');
    const opts = [...sel.options].map((o) => o.value);
    sel.value = opts[(opts.indexOf(this.options.foldScope) + 1) % opts.length];
    this.options.foldScope = sel.value as EditorOptions['foldScope'];
    this.toast(`Lager: ${sel.selectedOptions[0].textContent}`, 'info');
    this.tools.fold.refresh();
  }

  toggleControl(opId: string) {
    const op = this.history.appliedOps.find((o) => o.id === opId);
    if (!op || op.type !== 'fold') return;
    const on = !op.control;
    this.history.setControl(opId, on);
    this.toast(on ? 'Fliken är nu en klaff. Styr med piltangenterna när du kastar.' : 'Fliken är inte längre en klaff.', 'ok');
    this.syncView();
  }

  /** Orange = control flap, blue = hover (in the control tool). */
  private applyTints() {
    const m = this.display();
    const tints = new Map<number, FacetTint>();
    const { tags } = controlFacetTags(m, this.history.appliedOps);
    for (const id of tags.keys()) tints.set(id, 'control');
    for (const id of this.hoverFacets ?? []) tints.set(id, tints.has(id) ? 'controlHover' : 'hover');
    this.paper.setTints(tints);
  }

  highlightClip(id: string | null) {
    for (const c of this.clipGroup.children) {
      c.traverse((o) => {
        const m = o as THREE.Mesh;
        if (m.isMesh) (m.material as THREE.MeshStandardMaterial).emissive?.set(c.userData.clipId === id ? '#7a1010' : '#000000');
      });
    }
    this.stage.canvas.style.cursor = id ? 'pointer' : '';
  }

  // ---------------------------------------------------------------- view sync

  /** The folded model (or a tool's preview of it) with the plane's damage. */
  private shown(): PaperModel {
    return this.preview ?? this.damaged();
  }

  private damagedCache: { base: PaperModel; n: number; model: PaperModel } | null = null;

  /** The committed model with the damage from flights applied (it stays until refolded). */
  private damaged(): PaperModel {
    const base = this.history.current;
    const dmg = this.history.damage;
    if (!dmg.length) return base;
    const c = this.damagedCache;
    if (c && c.base === base && c.n === dmg.length) return c.model;
    let m = base;
    for (const op of dmg) {
      try {
        m = applyOp(m, op).model;
      } catch {
        /* damage that no longer applies is skipped */
      }
    }
    this.damagedCache = { base, n: dmg.length, model: m };
    return m;
  }

  private relaxedCache = new WeakMap<PaperModel, PaperModel>();

  /** What is shown: the model as the paper rests after folding (crease springback). */
  private display(): PaperModel {
    const m = this.shown();
    let r = this.relaxedCache.get(m);
    if (!r) {
      r = relaxCreases(m, [...this.history.appliedOps, ...this.history.damage], DEFAULT_CONFIG.paper.creaseSpringback);
      this.relaxedCache.set(m, r);
    }
    return r;
  }

  private syncPaper() {
    const m = this.display();
    if (m === this.shownModel) return;
    this.shownModel = m;
    this.paper.update(m);
    this.rebuildClips(m);
    this.placeNoseArrow(m);
  }

  private rebuildClips(m: PaperModel) {
    for (const c of [...this.clipGroup.children]) {
      this.clipGroup.remove(c);
      c.traverse((o) => (o as THREE.Mesh).geometry?.dispose());
    }
    for (const clip of m.clips) {
      const rc = resolveClip(m, clip);
      if (!rc) continue;
      // clips get their own material so highlighting one doesn't affect the others
      const obj = createClipObject(m, rc);
      obj.traverse((o) => {
        const mesh = o as THREE.Mesh;
        if (mesh.isMesh) mesh.material = (mesh.material as THREE.Material).clone();
      });
      this.clipGroup.add(obj);
    }
  }

  private placeNoseArrow(m: PaperModel) {
    const nose = this.history.meta.orientation.nose;
    const vs = m.facets.flatMap(facetVertices);
    let best = vs[0];
    for (const v of vs) if (V.dot(v, nose) > V.dot(best, nose)) best = v;
    const c = V.centroid(vs);
    const base = V.add(V.rejectFrom(c, V.normalize(nose)), V.scale(V.normalize(nose), V.dot(best, V.normalize(nose)) + 0.012));
    this.noseArrow.position.set(...base);
    this.noseArrow.setDirection(new THREE.Vector3(...nose).normalize());
  }

  private refreshPhysics() {
    const af = this.physics.update(this.history.current);
    this.physicsOverlay.update(this.physics.visible ? af : null, this.physics.centerOfLift);
    $('btn-physics').classList.toggle('on', this.physics.visible);
  }

  /**
   * Find nose/up from the folded geometry (symmetry plane, centre fold down) whenever the model
   * changes, unless the player chose an orientation by hand for this model.
   */
  private autoOrient(force = false) {
    const m = this.history.current;
    if (!force && m === this.orientedModel) return;
    if (m !== this.orientedModel) this.manualOrientation = false;
    this.orientedModel = m;
    if (this.manualOrientation) return;
    const o = detectOrientation(m, this.history.meta.orientation);
    if (o.detected) this.history.meta.orientation = { nose: o.nose, up: o.up };
  }

  /** Can the last fold/cut be mirrored onto the other half? Returns the twin op or a reason. */
  private mirrorLastOp(): { op: Op } | { reason: string } {
    const last = this.history.appliedOps.at(-1);
    if (!last || (last.type !== 'fold' && last.type !== 'cut')) return { reason: 'Det finns inget veck eller snitt att spegla.' };
    if (last.mirror) return { reason: 'Det senaste vecket är redan speglat.' };
    const R = symmetryOf(this.history.previous);
    if (!R) return { reason: 'Går inte att spegla: modellen var inte symmetrisk före det senaste vecket.' };
    if (last.type === 'fold') {
      if (foldIsSelfMirror(last, R)) return { reason: 'Vecket ligger på mittlinjen – det har ingen spegelbild.' };
      return { op: { ...mirrorFold(last, R), id: this.history.nextId('s') } };
    }
    if (cutIsSelfMirror(last, R)) return { reason: 'Snittet ligger på mittlinjen – det har ingen spegelbild.' };
    return { op: { ...mirrorCut(last, R), id: this.history.nextId('s') } };
  }

  private mirrorLast() {
    this.cancelTool();
    const r = this.mirrorLastOp();
    if ('reason' in r) {
      this.toast(r.reason, 'warn');
      return;
    }
    if (this.commit(r.op)) this.toast('Speglade det senaste till andra halvan.', 'ok');
  }

  private syncView() {
    this.autoOrient();
    this.syncPaper();
    this.applyTints();
    this.placeNoseArrow(this.display());
    const ml = this.mirrorLastOp();
    for (const id of ['btn-mirror-last', 'btn-mirror-last-cut']) {
      const b = $<HTMLButtonElement>(id);
      b.disabled = 'reason' in ml;
      b.title = 'reason' in ml ? ml.reason : 'Gör det senaste en gång till, spegelvänt på andra halvan (N)';
    }
    this.refreshPhysics();
    if (this.toolName !== 'view') this.tools[this.toolName].refresh();
    ($('btn-undo') as HTMLButtonElement).disabled = !this.history.canUndo;
    ($('btn-redo') as HTMLButtonElement).disabled = !this.history.canRedo;
    const m = this.history.current;
    const ops = this.history.appliedOps;
    const folds = ops.filter((o) => o.type === 'fold').length;
    const cuts = ops.filter((o) => o.type === 'cut').length;
    $('status').innerHTML = [
      `Veck: ${folds}`,
      `Snitt: ${cuts}`,
      `Gem: ${m.clips.filter((c) => c.kind !== 'tape').length}`,
      `Tejp: ${m.clips.filter((c) => c.kind === 'tape').length}`,
      `Klaffar: ${ops.filter((o) => o.type === 'fold' && o.control).length}`,
      `Facetter: ${m.facets.length}`,
      `Max lager: ${maxLayers(m)}`,
    ]
      .map((s) => `<span>${s}</span>`)
      .join('');
    ($('design-name') as HTMLInputElement).value = this.history.meta.name;
    $<HTMLInputElement>('flexible-paper').checked = this.history.meta.flexible === true;
    const repair = $<HTMLButtonElement>('btn-repair');
    const dmg = this.history.damage.length;
    repair.hidden = dmg === 0 && !this.burned;
    repair.textContent = this.burned ? '🔥 Uppbränt · Reparera' : `⚠ Skadat (${dmg}) · Reparera`;
  }

  private applyXRay() {
    this.paper.setXRay(this.xray ? 0.35 : this.autoXRay ? 0.55 : 1);
    $('btn-xray').classList.toggle('on', this.xray);
  }

  /** Called every frame: animate view rotation and keep the paper resting on the table. */
  frame(frameDt: number) {
    const k = 1 - Math.exp(-frameDt * 9);
    this.viewQ.slerp(this.viewTarget, k);
    this.root.quaternion.copy(this.viewQ);
    this.root.position.set(0, 0, 0);
    this.root.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(this.paper.group, true);
    if (!box.isEmpty()) {
      const target = new THREE.Vector3(-(box.min.x + box.max.x) / 2, -box.min.y + 0.0003, -(box.min.z + box.max.z) / 2);
      if (!this.rootPosInit) {
        this.rootPos.copy(target);
        this.rootPosInit = true;
      }
      // horizontal re-centring is smoothed; resting height follows immediately (no sinking into the table)
      this.rootPos.x += (target.x - this.rootPos.x) * k;
      this.rootPos.z += (target.z - this.rootPos.z) * k;
      this.rootPos.y = target.y;
    }
    this.root.position.copy(this.rootPos);
    this.root.updateMatrixWorld(true);
    this.viewBall.render(this.viewQ, this.display(), this.history.meta.orientation.nose);
    const c = this.stage.canvas;
    this.overlay.resolution.set(c.clientWidth, c.clientHeight);
    this.overlay.update(this.stage.camera, c.clientHeight);
  }

  /** Rotate the view so the design's nose points away from the player and its "up" is up. */
  private showUpright() {
    this.viewTarget.copy(this.uprightQuat());
  }

  /** View rotation that shows the design upright (nose along +X away from the player, up = up). */
  private uprightQuat(): THREE.Quaternion {
    const { nose, up } = this.history.meta.orientation;
    const x = new THREE.Vector3(...nose).normalize();
    const y = new THREE.Vector3(...up).projectOnPlane(x).normalize();
    const z = new THREE.Vector3().crossVectors(x, y);
    const toModel = new THREE.Matrix4().makeBasis(x, y, z); // world axes → model axes
    return new THREE.Quaternion().setFromRotationMatrix(toModel.transpose());
  }

  /** Damage from a flight (already turned into ops by the throw mode). */
  addDamage(ops: Op[]) {
    this.history.damage.push(...ops);
    this.syncView();
  }

  /** The plane was repaired in throw mode. */
  repairDamage() {
    this.history.damage = [];
    this.setBurned(false);
    this.syncView();
  }

  /** The plane burned up: only a little heap of ash is left on the table (until repaired). */
  burned = false;
  private ashPile: THREE.Group | null = null;

  setBurned(on: boolean) {
    this.burned = on;
    if (on && !this.ashPile) {
      this.ashPile = createAshPile();
      // on the table itself (not with the paper, which the view may have turned over)
      this.stage.scene.add(this.ashPile);
    }
    if (this.ashPile) this.ashPile.visible = on;
    this.paper.group.visible = !on;
    this.clipGroup.visible = !on;
    if (on) {
      this.cancelTool();
      this.overlay.clear();
      this.hint('Planet brann upp. Tryck på Reparera för ett nytt ark, vikt likadant.');
    } else if (this.toolName !== 'view') this.tools[this.toolName].refresh();
    this.syncView();
  }

  /** Hand the committed model over to the throw mode. */
  private fly() {
    this.cancelTool();
    // the plane is thrown with its centre fold down (unless the player set nose/up by hand)
    this.autoOrient(true);
    this.syncView();
    this.onFly?.(this.history.toFlightDesign(), this.gridPaper);
  }

  private rotateView(axis: THREE.Vector3, angle: number) {
    const q = new THREE.Quaternion().setFromAxisAngle(axis, angle);
    this.viewTarget.premultiply(q).normalize();
  }

  // ---------------------------------------------------------------- DOM

  private setTool(name: ToolName) {
    const prev = this.toolName;
    if (prev !== 'view') this.tools[prev].deactivate();
    // leaving the view tool puts the camera back to the standard top-down view
    if (prev === 'view' && name !== 'view') this.stage.resetCamera();
    this.toolName = name;
    this.stage.setLeftButtonOrbits(name === 'view');
    document.querySelectorAll<HTMLButtonElement>('.tools button').forEach((b) => b.classList.toggle('active', b.dataset.tool === name));
    for (const t of ['fold', 'cut', 'clip']) $(`opts-${t}`).classList.toggle('visible', t === name);
    if (name !== 'control') this.setHoverFacets(null);
    this.snapDot(null);
    if (name === 'view') {
      this.overlay.clear();
      this.setAutoXRay(false);
      this.hint('Vy: vänster knapp roterar, mittknapp panorerar, scrollhjul zoomar.');
    } else this.tools[name].activate();
  }

  private bindPointer() {
    const c = this.stage.canvas;
    // Right-click on a clip removes it, whatever tool is active. Registered in the capture
    // phase on the container so OrbitControls (listening on the canvas) never starts orbiting.
    c.parentElement!.addEventListener(
      'pointerdown',
      (e) => {
        if (!this.active || e.button !== 2 || e.target !== c) return;
        const id = this.hitClip(e);
        if (!id) return;
        e.stopPropagation();
        e.preventDefault();
        this.highlightClip(null);
        const tape = this.history.current.clips.find((c) => c.id === id)?.kind === 'tape';
        if (this.commit({ type: 'unclip', id: this.nextId('u'), target: id })) this.toast(tape ? 'Tejpen togs bort.' : 'Gemet togs bort.', 'ok');
      },
      { capture: true },
    );
    c.addEventListener('pointerdown', (e) => {
      if (!this.active || e.button !== 0 || this.toolName === 'view') return;
      if (this.burned) {
        this.toast('Det finns bara aska kvar. Tryck på Reparera för ett nytt ark.', 'info');
        return;
      }
      this.pointerDown = true;
      c.setPointerCapture(e.pointerId);
      this.tools[this.toolName].down(e);
    });
    c.addEventListener('pointermove', (e) => {
      if (!this.active) return;
      if (e.buttons & 6 && !this.pointerDown) return; // orbiting/panning with right/middle
      this.highlightClip(this.clipGroup.children.length ? this.hitClip(e) : null);
      if (this.toolName === 'view') return;
      this.tools[this.toolName].move(e);
    });
    c.addEventListener('pointerup', (e) => {
      if (!this.active || e.button !== 0 || !this.pointerDown) return;
      this.pointerDown = false;
      if (this.toolName !== 'view') this.tools[this.toolName].up(e);
    });
    c.addEventListener('pointerleave', () => this.snapDot(null));
  }

  private undo() {
    this.cancelTool();
    this.history.undo();
    this.syncView();
  }

  private redo() {
    this.cancelTool();
    this.history.redo();
    this.syncView();
  }

  private cancelTool() {
    if (this.toolName !== 'view') this.tools[this.toolName].key(new KeyboardEvent('keydown', { key: 'Escape' }));
    this.preview = null;
  }

  private loadDesign(obj: unknown) {
    try {
      const d = validateDesign(obj);
      this.cancelTool();
      this.history.load(d);
      if (this.burned) this.setBurned(false);
      this.showUpright();
      this.syncView();
      this.toast(`Laddade ”${d.name}”.`, 'ok');
    } catch (e) {
      const msg = e instanceof PaperError ? `${ERROR_TEXT[e.code]} ${e.message}` : String(e);
      this.toast(msg, 'error');
    }
  }

  private bindDom() {
    document.querySelectorAll<HTMLButtonElement>('.tools button').forEach((b) => (b.onclick = () => this.setTool(b.dataset.tool as ToolName)));

    const scope = $<HTMLSelectElement>('fold-scope');
    scope.onchange = () => {
      this.options.foldScope = scope.value as EditorOptions['foldScope'];
      scope.blur();
      this.tools.fold.refresh();
    };
    const fm = $<HTMLInputElement>('fold-mirror');
    fm.onchange = () => {
      this.options.foldMirror = fm.checked;
      this.tools.fold.refresh();
    };
    const p2p = $<HTMLInputElement>('fold-p2p');
    p2p.onchange = () => {
      this.options.pointToPoint = p2p.checked;
      this.tools.fold.activate();
    };
    const grid = $<HTMLInputElement>('grid-paper');
    try {
      this.gridPaper = localStorage.getItem('paperplanes.grid.v1') === '1';
    } catch {
      /* storage unavailable */
    }
    grid.checked = this.gridPaper;
    this.paper.setGrid(this.gridPaper);
    grid.onchange = () => {
      this.gridPaper = grid.checked;
      this.paper.setGrid(this.gridPaper);
      grid.blur();
      try {
        localStorage.setItem('paperplanes.grid.v1', this.gridPaper ? '1' : '0');
      } catch {
        /* storage unavailable */
      }
    };
    const depth = $<HTMLSelectElement>('cut-depth');
    depth.onchange = () => (this.options.cutDepth = depth.value as EditorOptions['cutDepth']);
    const cm = $<HTMLInputElement>('cut-mirror');
    cm.onchange = () => (this.options.cutMirror = cm.checked);

    $('btn-undo').onclick = () => this.undo();
    $('btn-redo').onclick = () => this.redo();
    $('btn-flip-long').onclick = () => this.rotateView(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    $('btn-flip-short').onclick = () => this.rotateView(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    $('btn-rotate').onclick = () => this.rotateView(new THREE.Vector3(0, 1, 0), Math.PI / 2);
    $('btn-reset-view').onclick = () => this.stage.resetCamera();
    $('btn-fly').onclick = () => this.fly();
    $('btn-mirror-last').onclick = () => this.mirrorLast();
    $('btn-repair').onclick = () => {
      this.history.damage = [];
      const wasBurned = this.burned;
      this.setBurned(false);
      this.toast(wasBurned ? 'Ett nytt ark, vikt likadant.' : 'Planet är reparerat.', 'ok');
      this.syncView();
    };
    const flex = $<HTMLInputElement>('flexible-paper');
    flex.onchange = () => {
      this.history.meta = { ...this.history.meta, flexible: flex.checked };
      flex.blur();
      this.toast(flex.checked ? 'Flexibelt papper: vingar och veck ger efter och kan skadas.' : 'Stelt papper.', 'info');
    };
    $('btn-mirror-last-cut').onclick = () => this.mirrorLast();
    $('btn-physics').onclick = () => {
      this.physics.setVisible(!this.physics.visible);
      this.refreshPhysics();
    };
    $('btn-xray').onclick = () => {
      this.xray = !this.xray;
      this.applyXRay();
    };

    const name = $<HTMLInputElement>('design-name');
    name.onchange = () => (this.history.meta.name = name.value.trim() || 'Min modell');

    const sel = $<HTMLSelectElement>('preset-select');
    for (const p of PRESETS) {
      const o = document.createElement('option');
      o.value = p.id;
      o.textContent = p.name;
      sel.appendChild(o);
    }
    sel.onchange = () => {
      const p = PRESETS.find((x) => x.id === sel.value);
      sel.value = '';
      sel.blur();
      if (p) this.loadDesign(JSON.parse(JSON.stringify(p.design)));
    };

    $('btn-new').onclick = () => {
      if (this.history.appliedOps.length && !confirm('Börja om med ett nytt platt ark? Osparade veck försvinner.')) return;
      this.cancelTool();
      this.history.reset();
      if (this.burned) this.setBurned(false);
      this.viewTarget.identity();
      this.syncView();
    };
    $('btn-save').onclick = () => void this.save();
    const file = $<HTMLInputElement>('file-input');
    $('btn-load').onclick = () => file.click();
    file.onchange = async () => {
      const f = file.files?.[0];
      file.value = '';
      if (!f) return;
      try {
        this.loadDesign(JSON.parse(await f.text()));
      } catch {
        this.toast('Filen är inte giltig JSON.', 'error');
      }
    };

    window.addEventListener('keydown', (e) => {
      if (!this.active) return;
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'SELECT' || (tag === 'INPUT' && (e.target as HTMLInputElement).type === 'text')) return;
      const mod = e.metaKey || e.ctrlKey;
      if (mod && e.key.toLowerCase() === 'z') {
        e.preventDefault();
        if (e.shiftKey) this.redo();
        else this.undo();
        return;
      }
      if (mod && e.key.toLowerCase() === 'y') {
        e.preventDefault();
        this.redo();
        return;
      }
      if (mod) return;
      if (this.toolName !== 'view' && this.tools[this.toolName].key(e)) return;
      switch (e.key.toLowerCase()) {
        case 'v': this.setTool('fold'); break;
        case 's': this.setTool('cut'); break;
        case 'g': this.setTool('clip'); break;
        case 'j': this.setTool('tape'); break;
        case 'k': this.setTool('control'); break;
        case 'o': this.setTool('view'); break;
        case 'm':
          if (this.toolName === 'cut') cm.click();
          else fm.click();
          break;
        case 'p': p2p.click(); break;
        case 'x': $('btn-xray').click(); break;
        case 'i': $('btn-physics').click(); break;
        case 't': this.fly(); break;
        case 'n': this.mirrorLast(); break;
        case 'f': this.rotateView(new THREE.Vector3(e.shiftKey ? 0 : 1, 0, e.shiftKey ? 1 : 0), Math.PI / 2); break;
        case 'r': this.rotateView(new THREE.Vector3(0, 1, 0), Math.PI / 2); break;
      }
    });
  }
}
