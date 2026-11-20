import { defineConfig } from 'vite';

export default defineConfig({
  // relative asset paths: the built game works from any folder, e.g. GitHub Pages' /Paper_planes/
  base: './',
  // three.js alone is ≈ 600 kB minified; one chunk is fine for this game
  build: { chunkSizeWarningLimit: 1200 },
  server: { port: 5173 },
});
