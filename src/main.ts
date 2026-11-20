import { Stage } from './render/SceneSetup';
import { EditorApp } from './ui/editor/EditorApp';
import { FlightMode } from './ui/flight/FlightMode';
import { GameLoop } from './app/GameLoop';
import { DEFAULT_CONFIG } from './core/physics/config';

const canvas = document.getElementById('viewport') as HTMLCanvasElement;
const stage = new Stage(canvas);
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

editor.onFly = (design, gridPaper) => {
  editor.active = false;
  document.body.classList.add('mode-flight');
  stage.setView(flight.view);
  flight.enter(design, gridPaper);
};

loop.start();

// handy for debugging in the console
Object.assign(window as unknown as Record<string, unknown>, { editor, flight });
