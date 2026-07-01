import { mkdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const schemaDir = resolve(root, 'schemas/flatbuffers');
const outDir = resolve(root, 'packages/electrode-flatbuffers/src/generated');

mkdirSync(outDir, { recursive: true });

const flatc = spawnSync(
  'flatc',
  ['--ts', '-o', outDir, '-I', schemaDir, ...[
    'common.fbs',
    'state.fbs',
    'commands.fbs',
    'events.fbs',
    'gcs_log.fbs',
    'mission.fbs',
    'parameters.fbs'
  ].map((file) => resolve(schemaDir, file))],
  { stdio: 'inherit' }
);

if (flatc.error?.code === 'ENOENT') {
  console.warn('flatc was not found. Install FlatBuffers to generate TypeScript bindings.');
  process.exit(0);
}

process.exit(flatc.status ?? 1);
