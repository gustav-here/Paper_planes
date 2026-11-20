import * as THREE from 'three';
import * as V from '../../core/math/vec3';
import type { Vec3 } from '../../core/math/vec3';
import type { PaperModel } from '../../core/paper/model';
import { facetVertices } from '../../core/paper/model';
import { resolveClip } from '../../core/paper/clips';
import { applyOp } from '../../core/paper/replay';
import type { OpResult } from '../../core/paper/replay';
import { PaperError } from '../../core/paper/errors';
import { validateDesign } from '../../core/paper/ops';
import type { Op } from '../../core/paper/ops';
import { PRESETS } from '../../core/presets';
import { maxLayers } from '../../core/paper/stack';
import type { Stage } from '../../render/SceneSetup';
import { createTable } from '../../render/Table';
import { PaperView } from '../../render/PaperMesh';
import type { PaperHit } from '../../render/PaperMesh';
import { Overlay } from '../../render/Overlay';
import { createClipObject } from '../../render/ClipMesh';
import { ERROR_TEXT, warningText } from '../hud/messages';
import { History } from './History';
import { FoldTool } from './FoldTool';
import { CutTool } from './CutTool';
import { ClipTool } from './ClipTool';
import type { SnapKind } from './Snapping';
import type { EditorOptions, Ray, Tool, ToolContext, ToastAction, ToastKind } from './toolContext';

type ToolName = 'fold' | 'cut' | 'clip' | 'view';

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

/** The folding editor: owns history, the paper view on the table, tools and the DOM HUD. */
export class EditorApp implements ToolContext {
  readonly options: EditorOptions = {
    foldKind: 'valley',
    foldScope: 'all',
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

  constructor(private stage: Stage) {
    stage.scene.add(createTable());
    stage.scene.add(this.root);
    this.root.add(this.paper.group, this.clipGroup, this.overlay.group);
    this.noseArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.04, 0xffd34d, 0.012, 0.008);
    this.root.add(this.noseArrow);
    this.tools = { fold: new FoldTool(this), cut: new CutTool(this), clip: new ClipTool(this) };
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

  previousModel(): PaperModel {
    return this.history.previous;
  }

  nextId(prefix: string): string {
    return this.history.nextId(prefix);
  }

  setPreview(m: PaperModel | null) {
    this.preview = m;
    this.syncPaper();
  }

  commit(op: Op): OpResult | null {
    try {
      const r = applyOp(this.history.current, op, { penetration: 'sweep' });
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
    return { origin: [o.x, o.y, o.z], dir: [d.x, d.y, d.z] };
  }

  hitPaper(e: PointerEvent): PaperHit | null {
    this.setRay(e);
    this.root.updateMatrixWorld();
    return this.paper.hit(this.raycaster);
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
    const v = this.root.localToWorld(new THREE.Vector3(...p)).project(this.stage.camera);
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
    if (this.ghost) this.root.remove(this.ghost);
    this.ghost = obj;
    if (obj) this.root.add(obj);
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

  private shown(): PaperModel {
    return this.preview ?? this.history.current;
  }

  private syncPaper() {
    const m = this.shown();
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

  private syncView() {
    this.syncPaper();
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
      `Gem: ${m.clips.length}`,
      `Facetter: ${m.facets.length}`,
      `Max lager: ${maxLayers(m)}`,
    ]
      .map((s) => `<span>${s}</span>`)
      .join('');
    ($('design-name') as HTMLInputElement).value = this.history.meta.name;
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
  }

  /** Rotate the view so the design's nose points away from the player and its "up" is up. */
  private showUpright() {
    const { nose, up } = this.history.meta.orientation;
    const x = new THREE.Vector3(...nose).normalize();
    const y = new THREE.Vector3(...up).projectOnPlane(x).normalize();
    const z = new THREE.Vector3().crossVectors(x, y);
    const toModel = new THREE.Matrix4().makeBasis(x, y, z); // world axes → model axes
    this.viewTarget.setFromRotationMatrix(toModel.transpose());
  }

  private rotateView(axis: THREE.Vector3, angle: number) {
    const q = new THREE.Quaternion().setFromAxisAngle(axis, angle);
    this.viewTarget.premultiply(q).normalize();
  }

  // ---------------------------------------------------------------- DOM

  private setTool(name: ToolName) {
    const prev = this.toolName;
    if (prev !== 'view') this.tools[prev].deactivate();
    this.toolName = name;
    this.stage.setLeftButtonOrbits(name === 'view');
    document.querySelectorAll<HTMLButtonElement>('.tools button').forEach((b) => b.classList.toggle('active', b.dataset.tool === name));
    for (const t of ['fold', 'cut', 'clip']) $(`opts-${t}`).classList.toggle('visible', t === name);
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
        if (e.button !== 2 || e.target !== c) return;
        const id = this.hitClip(e);
        if (!id) return;
        e.stopPropagation();
        e.preventDefault();
        this.highlightClip(null);
        if (this.commit({ type: 'unclip', id: this.nextId('u'), target: id })) this.toast('Gemet togs bort.', 'ok');
      },
      { capture: true },
    );
    c.addEventListener('pointerdown', (e) => {
      if (e.button !== 0 || this.toolName === 'view') return;
      this.pointerDown = true;
      c.setPointerCapture(e.pointerId);
      this.tools[this.toolName].down(e);
    });
    c.addEventListener('pointermove', (e) => {
      if (e.buttons & 6 && !this.pointerDown) return; // orbiting/panning with right/middle
      this.highlightClip(this.clipGroup.children.length ? this.hitClip(e) : null);
      if (this.toolName === 'view') return;
      this.tools[this.toolName].move(e);
    });
    c.addEventListener('pointerup', (e) => {
      if (e.button !== 0 || !this.pointerDown) return;
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

  private setFoldKind(kind: EditorOptions['foldKind']) {
    this.options.foldKind = kind;
    document.querySelectorAll<HTMLButtonElement>('#fold-kind button').forEach((b) => b.classList.toggle('active', b.dataset.kind === kind));
    this.tools.fold.refresh();
  }

  private loadDesign(obj: unknown) {
    try {
      const d = validateDesign(obj);
      this.cancelTool();
      this.history.load(d);
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
    document.querySelectorAll<HTMLButtonElement>('#fold-kind button').forEach((b) => (b.onclick = () => this.setFoldKind(b.dataset.kind as EditorOptions['foldKind'])));
    this.setFoldKind('valley');

    const scope = $<HTMLSelectElement>('fold-scope');
    scope.onchange = () => (this.options.foldScope = scope.value as EditorOptions['foldScope']);
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
      this.viewTarget.identity();
      this.syncView();
    };
    $('btn-save').onclick = () => {
      const d = this.history.toDesign();
      const blob = new Blob([JSON.stringify(d, null, 2)], { type: 'application/json' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = `${d.name.replace(/[^\wåäöÅÄÖ\- ]+/g, '').trim() || 'modell'}.json`;
      a.click();
      setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    };
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
        case 'o': this.setTool('view'); break;
        case 'd': this.setFoldKind('valley'); break;
        case 'b': this.setFoldKind('mountain'); break;
        case 'm':
          if (this.toolName === 'cut') cm.click();
          else fm.click();
          break;
        case 'p': p2p.click(); break;
        case 'x': $('btn-xray').click(); break;
        case 'f': this.rotateView(new THREE.Vector3(e.shiftKey ? 0 : 1, 0, e.shiftKey ? 1 : 0), Math.PI / 2); break;
        case 'r': this.rotateView(new THREE.Vector3(0, 1, 0), Math.PI / 2); break;
      }
    });
  }
}
