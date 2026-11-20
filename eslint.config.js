import js from '@eslint/js';
import tseslint from 'typescript-eslint';

export default tseslint.config(
  { ignores: ['dist', 'node_modules'] },
  js.configs.recommended,
  ...tseslint.configs.recommended,
  {
    files: ['src/core/**/*.ts'],
    rules: {
      // core must stay renderer/DOM independent so it can run headless in tests
      'no-restricted-imports': ['error', { patterns: ['three', 'three/*', '**/render/**', '**/ui/**', '**/app/**'] }],
      'no-restricted-globals': ['error', 'window', 'document', 'performance', 'localStorage'],
      'no-restricted-properties': ['error', { object: 'Math', property: 'random', message: 'core must be deterministic' }],
    },
  },
  {
    rules: {
      '@typescript-eslint/no-unused-vars': ['error', { argsIgnorePattern: '^_' }],
    },
  },
);
