import { Stage } from './render/SceneSetup';
import { EditorApp } from './ui/editor/EditorApp';
import { GameLoop } from './app/GameLoop';

const canvas = document.getElementById('viewport') as HTMLCanvasElement;
const stage = new Stage(canvas);
const editor = new EditorApp(stage);

// Physics (stage 2+) will step at a fixed 240 Hz inside onStep; the editor only renders.
const loop = new GameLoop(
  () => {},
  (frameDt) => {
    editor.frame(frameDt);
    stage.render();
  },
);
loop.start();

// handy for debugging in the console
(window as unknown as { editor: EditorApp }).editor = editor;
