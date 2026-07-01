import { resolveTopic } from './topics';
import type { CommandDefinition, CommandIntent, CommandName, CommandResult, VehicleState } from './types';

export const COMMAND_DEFINITIONS = {
  arm: {
    command: 'arm',
    topic: 'vehicle/{id}/cmd/arm',
    label: 'Arm',
    description: 'Arm selected vehicle',
    requiresConnected: true,
    requiresLocalizationFresh: true,
    requiresNotFailsafe: true,
    requiresConfirmation: true,
    timeoutMs: 1000,
    ackTopic: 'vehicle/{id}/cmd/ack'
  },
  disarm: {
    command: 'disarm',
    topic: 'vehicle/{id}/cmd/disarm',
    label: 'Disarm',
    description: 'Disarm selected vehicle',
    requiresConnected: true,
    requiresLocalizationFresh: false,
    requiresNotFailsafe: false,
    requiresConfirmation: true,
    timeoutMs: 1000,
    ackTopic: 'vehicle/{id}/cmd/ack'
  },
  setMode: {
    command: 'setMode',
    topic: 'vehicle/{id}/cmd/mode',
    label: 'Set Mode',
    description: 'Set selected vehicle mode',
    requiresConnected: true,
    requiresLocalizationFresh: false,
    requiresNotFailsafe: true,
    requiresConfirmation: false,
    timeoutMs: 1000,
    ackTopic: 'vehicle/{id}/cmd/ack'
  },
  land: {
    command: 'land',
    topic: 'vehicle/{id}/cmd/land',
    label: 'Land',
    description: 'Request controlled landing',
    requiresConnected: true,
    requiresLocalizationFresh: false,
    requiresNotFailsafe: false,
    requiresConfirmation: true,
    timeoutMs: 1500,
    ackTopic: 'vehicle/{id}/cmd/ack'
  },
  return: {
    command: 'return',
    topic: 'vehicle/{id}/cmd/return',
    label: 'Return',
    description: 'Request return mode',
    requiresConnected: true,
    requiresLocalizationFresh: true,
    requiresNotFailsafe: false,
    requiresConfirmation: true,
    timeoutMs: 1500,
    ackTopic: 'vehicle/{id}/cmd/ack'
  },
  clearMission: {
    command: 'clearMission',
    topic: 'vehicle/{id}/mission/clear',
    label: 'Clear Mission',
    description: 'Clear staged mission',
    requiresConnected: true,
    requiresLocalizationFresh: false,
    requiresNotFailsafe: true,
    requiresConfirmation: true,
    timeoutMs: 1500,
    ackTopic: 'vehicle/{id}/cmd/ack'
  },
  uploadMission: {
    command: 'uploadMission',
    topic: 'vehicle/{id}/mission/upload',
    label: 'Upload Mission',
    description: 'Upload staged mission placeholder',
    requiresConnected: true,
    requiresLocalizationFresh: false,
    requiresNotFailsafe: true,
    requiresConfirmation: true,
    timeoutMs: 2500,
    ackTopic: 'vehicle/{id}/cmd/ack'
  },
  setParameter: {
    command: 'setParameter',
    topic: 'vehicle/{id}/param/set',
    label: 'Set Parameter',
    description: 'Set staged parameter placeholder',
    requiresConnected: true,
    requiresLocalizationFresh: false,
    requiresNotFailsafe: true,
    requiresConfirmation: true,
    timeoutMs: 1500,
    ackTopic: 'vehicle/{id}/cmd/ack'
  }
} satisfies Record<CommandName, CommandDefinition>;

export function validateCommandPreconditions(state: VehicleState, command: CommandName): string[] {
  const definition = COMMAND_DEFINITIONS[command];
  const failures: string[] = [];

  if (definition.requiresConnected && !state.connected) {
    failures.push('vehicle is not connected');
  }

  if (definition.requiresLocalizationFresh && !state.localization.fresh) {
    failures.push('localization is stale');
  }

  if (definition.requiresNotFailsafe && state.mode.failsafe) {
    failures.push('vehicle is in failsafe');
  }

  return failures;
}

export function buildCommandIntent(options: {
  vehicleId: string;
  command: CommandName;
  args?: Record<string, unknown>;
  sequence: number;
  nowMs?: number;
}): CommandIntent {
  const nowMs = options.nowMs ?? Date.now();
  const definition = COMMAND_DEFINITIONS[options.command];

  return {
    kind: 'command',
    commandId: `${options.vehicleId}-${options.command}-${options.sequence}-${nowMs}`,
    command: options.command,
    vehicleId: options.vehicleId,
    topic: resolveTopic(definition.topic, options.vehicleId),
    args: options.args ?? {},
    createdAtMs: nowMs,
    expiresAtMs: nowMs + definition.timeoutMs,
    sequence: options.sequence
  };
}

export function rejectedCommandResult(intent: CommandIntent, reason: string, nowMs = Date.now()): CommandResult {
  return {
    kind: 'commandAck',
    commandId: intent.commandId,
    command: intent.command,
    status: 'rejected',
    reason,
    sequence: intent.sequence,
    receivedAtMs: nowMs
  };
}

