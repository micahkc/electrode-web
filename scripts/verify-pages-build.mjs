import { readFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';

const buildDir = 'apps/web/build';
const expectedBase = normalizeBasePath(process.env.ELECTRODE_WEB_BASE_PATH ?? '');

function normalizeBasePath(base) {
  if (base === '') return '';
  if (!base.startsWith('/') || base.endsWith('/')) {
    throw new Error('ELECTRODE_WEB_BASE_PATH must be empty or a root-relative path without a trailing slash');
  }
  return base;
}

const files = readdirSync(buildDir, { recursive: true }).map(String);

if (!files.includes('index.html')) {
  throw new Error(`${buildDir}/index.html missing`);
}

if (!files.some((file) => /zenoh_wasm_bg.*\.wasm$/.test(file))) {
  throw new Error('zenoh wasm asset missing from static build');
}

const indexHtml = readFileSync(join(buildDir, 'index.html'), 'utf8');
const appPrefix = `${expectedBase}/_app/`;

if (!indexHtml.includes(appPrefix)) {
  throw new Error(`index.html does not reference expected app prefix ${appPrefix}`);
}

if (expectedBase !== '') {
  assertNoRootAbsoluteRuntimePaths(files);
}

console.log(`Verified static build for base path "${expectedBase || '/'}".`);

function assertNoRootAbsoluteRuntimePaths(files) {
  const offenders = files.flatMap((file) => {
    if (!/\.(html|js|css)$/.test(file)) return [];
    const text = readFileSync(join(buildDir, file), 'utf8');
    return rootAbsolutePatterns()
      .filter(({ pattern }) => pattern.test(text))
      .map(({ label }) => `${file}: ${label}`);
  });

  if (offenders.length > 0) {
    throw new Error(`static build contains root-absolute runtime paths:\n${offenders.join('\n')}`);
  }
}

function rootAbsolutePatterns() {
  return [
    { label: 'href="/_app/', pattern: /href="\/_app\// },
    { label: 'import("/_app/', pattern: /import\("\/_app\// },
    { label: 'import("/wasm/', pattern: /import\("\/wasm\// },
    { label: 'from "/_app/', pattern: /from "\/_app\// }
  ];
}
