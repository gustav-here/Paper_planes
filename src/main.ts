import { Stage } from './render/SceneSetup';
import { EditorApp } from './ui/editor/EditorApp';
import { FlightMode } from './ui/flight/FlightMode';
import { GameLoop } from './app/GameLoop';
import { DEFAULT_CONFIG } from './core/physics/config';

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

const flight: FlightMode = new FlightMode(stage, loop.stepper, () => {
  // back to the folding table
  flight.exit();
  stage.setView(null);
  document.body.classList.remove('mode-flight');
  editor.active = true;
});

flight.onDamage = (ops) => editor.addDamage(ops);
flight.onRepair = () => editor.repairDamage();
flight.onBurn = () => editor.setBurned(true);

editor.onFly = (design, gridPaper) => {
  editor.active = false;
  document.body.classList.add('mode-flight');
  stage.setView(flight.view);
  flight.enter(design, gridPaper, editor.burned);
};

loop.start();

// handy for debugging in the console
Object.assign(window as unknown as Record<string, unknown>, { editor, flight });
