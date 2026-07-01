export const ELECTRODE_SCHEMA_VERSION = 1;

export const FLATBUFFER_SCHEMA_FILES = [
  'common.fbs',
  'state.fbs',
  'commands.fbs',
  'events.fbs',
  'gcs_log.fbs',
  'mission.fbs',
  'parameters.fbs'
] as const;

export type FlatbufferSchemaFile = (typeof FLATBUFFER_SCHEMA_FILES)[number];

export * from './schema-assets';
