import type { TopicDefinition } from './types';

export const DEFAULT_VEHICLE_ID = 'electrode-01';

export const TOPIC_DEFINITIONS = {
  pose: {
    key: 'pose',
    topic: 'vehicle/{id}/state/pose',
    label: 'Pose',
    schema: 'Pose',
    expectedRateHz: 20,
    staleTimeoutMs: 250,
    loggable: true,
    display: true,
    units: 'deg, m',
    reliability: 'latest-wins',
    commandAuthority: 'none'
  },
  velocity: {
    key: 'velocity',
    topic: 'vehicle/{id}/state/velocity',
    label: 'Velocity',
    schema: 'Velocity',
    expectedRateHz: 20,
    staleTimeoutMs: 250,
    loggable: true,
    display: true,
    units: 'm/s',
    reliability: 'latest-wins',
    commandAuthority: 'none'
  },
  attitude: {
    key: 'attitude',
    topic: 'vehicle/{id}/state/attitude',
    label: 'Attitude',
    schema: 'Attitude',
    expectedRateHz: 30,
    staleTimeoutMs: 200,
    loggable: true,
    display: true,
    units: 'deg',
    reliability: 'latest-wins',
    commandAuthority: 'none'
  },
  battery: {
    key: 'battery',
    topic: 'vehicle/{id}/state/battery',
    label: 'Battery',
    schema: 'Battery',
    expectedRateHz: 2,
    staleTimeoutMs: 2000,
    loggable: true,
    display: true,
    units: 'V, A, %',
    reliability: 'latest-wins',
    commandAuthority: 'none'
  },
  link: {
    key: 'link',
    topic: 'vehicle/{id}/state/link',
    label: 'Link',
    schema: 'LinkStatus',
    expectedRateHz: 2,
    staleTimeoutMs: 2000,
    loggable: true,
    display: true,
    units: 'dBm, ms, %',
    reliability: 'latest-wins',
    commandAuthority: 'none'
  },
  mode: {
    key: 'mode',
    topic: 'vehicle/{id}/state/mode',
    label: 'Mode',
    schema: 'ModeState',
    expectedRateHz: 2,
    staleTimeoutMs: 2000,
    loggable: true,
    display: true,
    units: 'state',
    reliability: 'ordered',
    commandAuthority: 'none'
  },
  localization: {
    key: 'localization',
    topic: 'vehicle/{id}/state/localization',
    label: 'Localization',
    schema: 'LocalizationState',
    expectedRateHz: 5,
    staleTimeoutMs: 1000,
    loggable: true,
    display: true,
    units: 'quality',
    reliability: 'latest-wins',
    commandAuthority: 'none'
  },
  event: {
    key: 'event',
    topic: 'vehicle/{id}/event',
    label: 'Events',
    schema: 'Event',
    expectedRateHz: 0,
    staleTimeoutMs: 0,
    loggable: true,
    display: true,
    units: 'event',
    reliability: 'ordered',
    commandAuthority: 'none'
  }
} satisfies Record<string, TopicDefinition>;

export type TopicKey = keyof typeof TOPIC_DEFINITIONS;

export function resolveTopic(topicPattern: string, vehicleId = DEFAULT_VEHICLE_ID): string {
  return topicPattern.replace('{id}', vehicleId);
}

export function resolveTopicDefinition(topic: string): TopicDefinition | undefined {
  return Object.values(TOPIC_DEFINITIONS).find((definition) => {
    const expression = new RegExp(`^${definition.topic.replace('{id}', '[^/]+')}$`);
    return expression.test(topic);
  });
}

export function topicKeyFromTopic(topic: string): TopicKey | undefined {
  const definition = resolveTopicDefinition(topic);
  return definition?.key as TopicKey | undefined;
}

export function vehicleIdFromTopic(topic: string): string | undefined {
  const match = topic.match(/^vehicle\/([^/]+)\//);
  return match?.[1];
}

