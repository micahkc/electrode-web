import type { GcsFrame } from './types';

export interface ReplayCursor {
  index: number;
  cursorMs: number;
}

export function normalizeReplayFrames(frames: GcsFrame[]): GcsFrame[] {
  return [...frames].sort((a, b) => a.header.sourceTimeNs - b.header.sourceTimeNs);
}

export function replayDurationMs(frames: GcsFrame[]): number {
  if (frames.length === 0) {
    return 0;
  }

  const normalized = normalizeReplayFrames(frames);
  const startMs = normalized[0].header.sourceTimeNs / 1_000_000;
  const endMs = normalized.at(-1)!.header.sourceTimeNs / 1_000_000;
  return Math.max(0, endMs - startMs);
}

export function framesThroughCursor(frames: GcsFrame[], cursorMs: number): GcsFrame[] {
  if (frames.length === 0) {
    return [];
  }

  const normalized = normalizeReplayFrames(frames);
  const startMs = normalized[0].header.sourceTimeNs / 1_000_000;
  return normalized.filter((frame) => frame.header.sourceTimeNs / 1_000_000 - startMs <= cursorMs);
}

