import { describe, expect, it } from 'vitest';
import { readdirSync, readFileSync, statSync } from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

function files(dir: string): string[] {
  return readdirSync(dir).flatMap((n) => {
    const p = join(dir, n);
    return statSync(p).isDirectory() ? files(p) : p.endsWith('.ts') ? [p] : [];
  });
}

describe('core boundary', () => {
  const core = files(fileURLToPath(new URL('../../src/core', import.meta.url)));

  it('core never imports three.js, render, ui or app code', () => {
    for (const f of core) {
      const src = readFileSync(f, 'utf8');
      expect(src, f).not.toMatch(/from ['"]three/);
      expect(src, f).not.toMatch(/from ['"][./]*(render|ui|app)\//);
    }
  });

  it('core is deterministic (no Math.random / Date / performance)', () => {
    for (const f of core) {
      const src = readFileSync(f, 'utf8');
      expect(src, f).not.toMatch(/Math\.random|new Date|Date\.now|performance\./);
    }
  });
});
