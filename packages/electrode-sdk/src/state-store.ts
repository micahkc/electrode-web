import { ELECTRODE_SCHEMA_VERSION } from '@electrode/flatbuffers';
import { DEFAULT_VEHICLE_ID, resolveTopicDefinition, topicKeyFromTopic, vehicleIdFromTopic } from './topics';
import type {
  Attitude,
  Battery,
  EventFrame,
  EventMessage,
  GcsFrame,
  LinkStatus,
  LocalizationState,
  ModeState,
  Pose,
  TelemetryFrame,
  TopicSnapshot,
  VehicleState,
  Velocity
} from './types';

const RATE_WINDOW_MS = 2000;
const MAX_EVENTS = 80;

export function createInitialVehicleState(vehicleId = DEFAULT_VEHICLE_ID): VehicleState {
  return {
    vehicleId,
    connected: false,
    lastUpdateMs: 0,
    pose: null,
    velocity: null,
    attitude: null,
    battery: null,
    link: null,
    mode: {
      name: 'standby',
      armed: false,
      failsafe: false
    },
    localization: {
      source: 'none',
      fresh: false,
      quality: 0,
      updatedAtMs: 0
    },
    events: [],
    topics: {},
    commandHistory: []
  };
}

export function applyGcsFrame(state: VehicleState, frame: GcsFrame, nowMs = Date.now()): VehicleState {
  if (frame.kind === 'event') {
    return applyEventFrame(state, frame, nowMs);
  }

  return applyTelemetryFrame(state, frame, nowMs);
}

export function applyTelemetryFrame(state: VehicleState, frame: TelemetryFrame, nowMs = Date.now()): VehicleState {
  if (frame.header.schemaVersion !== ELECTRODE_SCHEMA_VERSION) {
    return appendEvent(state, {
      severity: 'warning',
      code: 'schema_mismatch',
      message: `Schema ${frame.header.schemaVersion} is not supported`,
      timestampMs: nowMs
    });
  }

  if (frame.header.expireTimeNs > 0 && frame.header.expireTimeNs < nowMs * 1_000_000) {
    return state;
  }

  const vehicleId = vehicleIdFromTopic(frame.topic) ?? frame.header.vehicleId;
  if (state.vehicleId !== vehicleId) {
    state.vehicleId = vehicleId;
  }

  const snapshot = updateTopicSnapshot(state, frame, nowMs);
  const key = topicKeyFromTopic(frame.topic);

  if (key === 'pose') {
    state.pose = frame.payload as Pose;
  } else if (key === 'velocity') {
    state.velocity = frame.payload as Velocity;
  } else if (key === 'attitude') {
    state.attitude = frame.payload as Attitude;
  } else if (key === 'battery') {
    state.battery = frame.payload as Battery;
  } else if (key === 'link') {
    state.link = frame.payload as LinkStatus;
  } else if (key === 'mode') {
    state.mode = frame.payload as ModeState;
  } else if (key === 'localization') {
    state.localization = frame.payload as LocalizationState;
  }

  state.topics[frame.topic] = snapshot;
  state.lastUpdateMs = nowMs;
  return refreshStaleTopics(state, nowMs);
}

export function applyEventFrame(state: VehicleState, frame: EventFrame, nowMs = Date.now()): VehicleState {
  updateTopicSnapshot(state, frame, nowMs);
  appendEvent(state, frame.payload);
  state.lastUpdateMs = nowMs;
  return refreshStaleTopics(state, nowMs);
}

export function refreshStaleTopics(state: VehicleState, nowMs = Date.now()): VehicleState {
  let hasFreshTopic = false;

  for (const snapshot of Object.values(state.topics)) {
    if (snapshot.staleTimeoutMs <= 0) {
      continue;
    }

    snapshot.stale = nowMs - snapshot.lastReceiveMs > snapshot.staleTimeoutMs;
    if (!snapshot.stale) {
      hasFreshTopic = true;
    }
  }

  state.connected = hasFreshTopic && nowMs - state.lastUpdateMs < 3000;
  state.localization.fresh = state.localization.updatedAtMs > 0 && nowMs - state.localization.updatedAtMs < 1200;
  return state;
}

export function appendEvent(state: VehicleState, event: EventMessage): VehicleState {
  state.events = [event, ...state.events].slice(0, MAX_EVENTS);
  return state;
}

function updateTopicSnapshot(state: VehicleState, frame: GcsFrame, nowMs: number): TopicSnapshot {
  const definition = resolveTopicDefinition(frame.topic);
  const current = state.topics[frame.topic];
  const sourceMs = frame.header.sourceTimeNs / 1_000_000;
  const arrivalTimesMs = [...(current?.arrivalTimesMs ?? []), nowMs].filter((timeMs) => nowMs - timeMs <= RATE_WINDOW_MS);
  const elapsedMs = arrivalTimesMs.at(-1)! - arrivalTimesMs[0]!;
  const rateHz = elapsedMs > 0 ? ((arrivalTimesMs.length - 1) * 1000) / elapsedMs : 0;

  return {
    topic: frame.topic,
    key: definition?.key ?? 'unknown',
    label: definition?.label ?? frame.topic.split('/').at(-1) ?? frame.topic,
    schema: definition?.schema ?? frame.header.messageType,
    expectedRateHz: definition?.expectedRateHz ?? 0,
    staleTimeoutMs: definition?.staleTimeoutMs ?? 1000,
    units: definition?.units ?? '',
    loggable: definition?.loggable ?? true,
    lastSequence: frame.header.sequence,
    lastReceiveMs: nowMs,
    latencyMs: Math.max(0, nowMs - sourceMs),
    rateHz,
    stale: false,
    samples: (current?.samples ?? 0) + 1,
    arrivalTimesMs
  };
}

