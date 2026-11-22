import { Stage } from './render/SceneSetup';
import { EditorApp } from './ui/editor/EditorApp';
import { FlightMode } from './ui/flight/FlightMode';
import { GameLoop } from './app/GameLoop';
import { DEFAULT_CONFIG } from './core/physics/config';
import type { GameMode } from './core/game/competition';

const canvas = document.getElementById('viewport') as HTMLCanvasElement;

/** Without WebGL there is nothing to draw: say why instead of showing an empty page. */
function noGraphics(why: string) {
  const el = document.createElement('div');
  el.className = 'no-webgl';
  el.innerHTML = `<h2>3D-grafiken kunde inte startas</h2><p>${why}</p><p>Starta om webbläsaren (eller appen) och ladda sidan igen.</p>`;
  document.body.appendChild(el);
}
canvas.addEventListener('webglcontextlost', () => noGraphics('Grafikkortet tappade kontakten med sidan.'));
canvas.addEventListener('webglcontextrestored', () => document.querySelectorAll('.no-webgl').forEach((e) => e.remove()));

let stage: Stage;
try {
  stage = new Stage(canvas);
} catch {
  noGraphics('Webbläsaren gav inte sidan tillgång till WebGL (den kan ha spärrats efter ett grafikfel).');
  throw new Error('WebGL unavailable');
}
const editor = new EditorApp(stage);

// Physics steps at a fixed rate (DEFAULT_CONFIG.dt = 1/240 s) independent of the frame rate.
const loop = new GameLoop(
  () => flight.step(),
  (frameDt, alpha) => {
    if (flight.active) flight.frame(frameDt, alpha);
    else editor.frame(frameDt);
    stage.render();
  },
  DEFAULT_CONFIG.dt,
);

const continueBtn = document.getElementById('btn-continue') as HTMLButtonElement;
const compMode = document.getElementById('comp-mode') as HTMLSelectElement;

const toFoldingTable = () => {
  flight.exit();
  stage.setView(null);
  document.body.classList.remove('mode-flight');
  editor.active = true;
  // in the middle of a competition: "Fortsätt" takes the plane back for the next throw
  continueBtn.hidden = !flight.roundInProgress();
};

const flight: FlightMode = new FlightMode(stage, loop.stepper, toFoldingTable);

flight.onDamage = (ops) => editor.addDamage(ops);
flight.onRepair = () => editor.repairDamage();
flight.onBurn = () => editor.setBurned(true);
flight.onRefold = toFoldingTable;

editor.onFly = (design, gridPaper) => {
  editor.active = false;
  continueBtn.hidden = true;
  document.body.classList.add('mode-flight');
  stage.setView(flight.view);
  flight.enter(design, gridPaper, editor.burned);
};

// competition mode: three throws, ranked best to worst afterwards
compMode.onchange = () => {
  flight.setMode(compMode.value as GameMode);
  continueBtn.hidden = true;
  compMode.blur();
};
continueBtn.onclick = () => editor.fly();

loop.start();

// handy for debugging in the console
Object.assign(window as unknown as Record<string, unknown>, { editor, flight });
