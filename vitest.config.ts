import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    environment: 'node',
    include: ['tests/**/*.test.ts'],
    // validation tests run real flights (several seconds each on a busy machine)
    testTimeout: 60_000,
  },
});
