import { ELECTRODE_SCHEMA_VERSION } from '@electrode/flatbuffers';
import { DEFAULT_VEHICLE_ID, resolveTopic, TOPIC_DEFINITIONS } from './topics';
import type { EventFrame, GcsFrame, MessageHeader, TelemetryFrame } from './types';

export interface SimulationOptions {
  vehicleId?: string;
  elapsedMs: number;
  sequenceStart: number;
  nowMs?: number;
  armed?: boolean;
  mode?: string;
}

export function makeHeader(options: {
  vehicleId: string;
  sequence: number;
  nowMs: number;
  ttlMs: number;
  messageType: string;
  streamId: string;
}): MessageHeader {
  return {
    sequence: options.sequence,
    sourceTimeNs: options.nowMs * 1_000_000,
    receiveTimeNs: options.nowMs * 1_000_000,
    expireTimeNs: (options.nowMs + options.ttlMs) * 1_000_000,
    vehicleId: options.vehicleId,
    schemaVersion: ELECTRODE_SCHEMA_VERSION,
    messageType: options.messageType,
    priority: 'normal',
    streamId: options.streamId
  };
}

export function makeSimulatedTelemetryBundle(options: SimulationOptions): TelemetryFrame[] {
  const vehicleId = options.vehicleId ?? DEFAULT_VEHICLE_ID;
  const t = options.elapsedMs / 1000;
  const nowMs = options.nowMs ?? Date.now();
  const radiusM = 42;
  const xM = Math.cos(t / 8) * radiusM;
  const yM = Math.sin(t / 8) * radiusM;
  const altM = 18 + Math.sin(t / 5) * 3;
  const lat = 40.4237 + yM / 111_111;
  const lon = -86.9212 + xM / (111_111 * Math.cos((40.4237 * Math.PI) / 180));
  const northMps = Math.cos(t / 4) * 1.8;
  const eastMps = -Math.sin(t / 4) * 1.8;
  const downMps = Math.cos(t / 3) * 0.2;
  const rollDeg = Math.sin(t * 1.3) * 12;
  const pitchDeg = Math.cos(t * 0.9) * 8;
  const yawDeg = ((t * 18) % 360 + 360) % 360;
  const voltageV = 15.9 - t * 0.002;
  const currentA = 4.5 + Math.sin(t) * 1.2;
  const localizationQuality = 0.91 + Math.sin(t / 6) * 0.05;
  const armed = options.armed ?? true;
  let sequence = options.sequenceStart;

  const frame = <TPayload>(key: keyof typeof TOPIC_DEFINITIONS, payload: TPayload, ttlMs = 1000): TelemetryFrame<TPayload> => {
    const definition = TOPIC_DEFINITIONS[key];
    return {
      kind: 'telemetry',
      topic: resolveTopic(definition.topic, vehicleId),
      header: makeHeader({
        vehicleId,
        sequence: sequence++,
        nowMs,
        ttlMs,
        messageType: definition.schema,
        streamId: definition.key
      }),
      payload
    };
  };

  const synapseFrame = <TPayload>(topic: string, messageType: string, streamId: string, payload: TPayload, ttlMs = 1000): TelemetryFrame<TPayload> => ({
    kind: 'telemetry',
    topic,
    header: makeHeader({
      vehicleId,
      sequence: sequence++,
      nowMs,
      ttlMs,
      messageType,
      streamId
    }),
    payload
  });

  const rollRad = degToRad(rollDeg);
  const pitchRad = degToRad(pitchDeg);
  const yawRad = degToRad(yawDeg);
  const rollInput = Math.sin(t * 0.9) * 0.32;
  const pitchInput = Math.cos(t * 0.72) * 0.26;
  const yawInput = Math.sin(t * 0.43) * 0.22;
  const throttleInput = clamp01(0.58 + Math.sin(t * 0.37) * 0.12);
  const rc = makeRcChannels(rollInput, pitchInput, yawInput, throttleInput, armed);
  const motorMix = [
    throttleInput + rollInput * 0.13 - pitchInput * 0.11 + yawInput * 0.07,
    throttleInput - rollInput * 0.13 - pitchInput * 0.11 - yawInput * 0.07,
    throttleInput - rollInput * 0.13 + pitchInput * 0.11 + yawInput * 0.07,
    throttleInput + rollInput * 0.13 + pitchInput * 0.11 - yawInput * 0.07
  ].map(clamp01);
  const timestampUs = Math.round(nowMs * 1000);

  return [
    frame('pose', { lat, lon, altM, xM, yM, zM: -altM }, 250),
    frame(
      'velocity',
      {
        northMps,
        eastMps,
        downMps,
        groundSpeedMps: 1.8
      },
      250
    ),
    frame(
      'attitude',
      {
        rollDeg,
        pitchDeg,
        yawDeg
      },
      220
    ),
    frame('battery', { voltageV, currentA, remainingPct: Math.max(22, 94 - t * 0.05) }, 2500),
    frame('link', { rssiDbm: -49 - Math.sin(t / 4) * 5, latencyMs: 23 + Math.sin(t / 2) * 5, packetLossPct: Math.max(0, Math.sin(t / 5) * 1.8) }, 2500),
    frame('mode', { name: options.mode ?? 'hold', armed, failsafe: false }, 2500),
    frame('localization', { source: 'mocap+ekf', fresh: true, quality: localizationQuality, updatedAtMs: nowMs }, 1200),
    synapseFrame(
      'synapse/manual_control',
      'synapse.topic.ManualControl',
      'synapse_manual_control',
      {
        data: {
          timestamp_us: timestampUs,
          axes: {
            roll: rollInput,
            pitch: pitchInput,
            yaw: yawInput,
            throttle: throttleInput
          },
          aux: makeAux(t),
          flight_mode: modeToSynapseFlightMode(options.mode ?? 'hold'),
          arm_switch: armed,
          kill_switch: false,
          active: true,
          valid: true
        }
      },
      180
    ),
    synapseFrame(
      'synapse/flight_snapshot',
      'synapse.topic.FlightSnapshot',
      'synapse_flight_snapshot',
      {
        gyro: {
          x: degToRad(Math.cos(t * 1.3) * 15.6),
          y: degToRad(-Math.sin(t * 0.9) * 7.2),
          z: degToRad(18)
        },
        accel: {
          x: Math.sin(pitchRad) * 9.81,
          y: -Math.sin(rollRad) * 9.81,
          z: -Math.cos(rollRad) * Math.cos(pitchRad) * 9.81
        },
        rc,
        status: {
          rc_stamp_ms: nowMs,
          throttle_us: rc.ch2,
          rc_link_quality: Math.round(localizationQuality * 100),
          flight_mode: modeToSynapseFlightMode(options.mode ?? 'hold'),
          armed,
          rc_valid: true,
          rc_stale: false,
          imu_ok: true,
          arm_switch: armed
        },
        attitude: { roll: rollRad, pitch: pitchRad, yaw: yawRad },
        attitude_desired: { roll: rollInput * 0.55, pitch: pitchInput * 0.45, yaw: yawRad },
        rate_desired: { roll: rollInput * 1.8, pitch: pitchInput * 1.6, yaw: yawInput * 1.2 },
        rate_cmd: { roll: rollInput * 1.5, pitch: pitchInput * 1.35, yaw: yawInput },
        main_loop_latency_us: Math.round(900 + Math.sin(t * 2.1) * 140)
      },
      180
    ),
    synapseFrame(
      'synapse/motor_output',
      'synapse.topic.MotorOutput',
      'synapse_motor_output',
      {
        motors: motorMixObject(motorMix),
        raw: motorMixObject(motorMix.map((value) => Math.round(1000 + value * 1000))),
        armed,
        test_mode: false
      },
      180
    ),
    synapseFrame('synapse/control_output', 'synapse.topic.RcChannels16', 'synapse_control_output', rc, 180),
    synapseFrame(
      'synapse/mocap/frame',
      'synapse.topic.MocapFrame',
      'synapse_mocap_frame',
      {
        timestamp_us: timestampUs,
        frame_number: Math.floor(options.elapsedMs / 10),
        labeled_markers: [
          {
            id: 1,
            position: { x: xM + 0.28, y: yM, z: altM + 0.08 },
            residual: 0.0009 + Math.abs(Math.sin(t)) * 0.0006
          }
        ],
        unlabeled_markers: [],
        rigid_bodies: [
          {
            id: 1,
            position: { x: xM, y: yM, z: altM },
            attitude: eulerToQuaternion(rollRad, pitchRad, yawRad),
            residual: 0.0014 + Math.abs(Math.cos(t / 2)) * 0.0008,
            tracking_valid: true
          }
        ],
        skeleton_segments: []
      },
      180
    ),
    synapseFrame(
      'synapse/optical_flow',
      'synapse.topic.VehicleOpticalFlow',
      'synapse_optical_flow',
      {
        data: {
          timestamp_us: timestampUs,
          pixel_flow: { x: eastMps * 0.016, y: northMps * 0.016 },
          delta_angle: { x: rollRad * 0.004, y: pitchRad * 0.004, z: yawInput * 0.006 },
          distance_m: Math.max(0.35, altM),
          integration_timespan_us: 10_000,
          quality: Math.round(localizationQuality * 100),
          max_flow_rate: 8,
          min_ground_distance: 0.25,
          max_ground_distance: 60
        }
      },
      180
    ),
    synapseFrame(
      'synapse/optical_flow_vel',
      'synapse.topic.VehicleOpticalFlowVel',
      'synapse_optical_flow_vel',
      {
        data: {
          timestamp_us: timestampUs,
          vel_body: { x: northMps * Math.cos(yawRad) + eastMps * Math.sin(yawRad), y: -northMps * Math.sin(yawRad) + eastMps * Math.cos(yawRad) },
          vel_ne: { x: northMps, y: eastMps },
          flow_rate_uncompensated: { x: eastMps * 0.11, y: northMps * 0.11 },
          flow_rate_compensated: { x: eastMps * 0.09, y: northMps * 0.09 },
          gyro_rate: { x: rollInput * 0.12, y: pitchInput * 0.12, z: yawInput * 0.1 }
        }
      },
      180
    ),
    synapseFrame(
      'synapse/sim/input',
      'synapse.sil.SimInput',
      'synapse_sim_input',
      {
        gyro: {
          x: degToRad(Math.cos(t * 1.3) * 15.6),
          y: degToRad(-Math.sin(t * 0.9) * 7.2),
          z: degToRad(18)
        },
        accel: {
          x: Math.sin(pitchRad) * 9.81,
          y: -Math.sin(rollRad) * 9.81,
          z: -Math.cos(rollRad) * Math.cos(pitchRad) * 9.81
        },
        rc,
        rc_link_quality: Math.round(localizationQuality * 100),
        rc_valid: true,
        imu_valid: true,
        target_boot_time_ns: Math.round(options.elapsedMs * 1_000_000)
      },
      180
    )
  ];
}

function makeRcChannels(roll: number, pitch: number, yaw: number, throttle: number, armed: boolean): Record<string, number> {
  return {
    ch0: pwmFromBipolar(roll),
    ch1: pwmFromBipolar(pitch),
    ch2: Math.round(1000 + clamp01(throttle) * 1000),
    ch3: pwmFromBipolar(yaw),
    ch4: armed ? 1900 : 1100,
    ch5: 1500,
    ch6: 1500,
    ch7: 1500,
    ch8: 1500,
    ch9: 1500,
    ch10: 1500,
    ch11: 1500,
    ch12: 1500,
    ch13: 1500,
    ch14: 1500,
    ch15: 1500
  };
}

function makeAux(t: number): Record<string, number> {
  return {
    aux0: Math.sin(t * 0.31),
    aux1: Math.cos(t * 0.29),
    aux2: Math.sin(t * 0.19) * 0.5,
    aux3: Math.cos(t * 0.17) * 0.5,
    aux4: 0,
    aux5: 0,
    aux6: 0,
    aux7: 0
  };
}

function motorMixObject(values: number[]): Record<string, number> {
  return {
    m0: values[0] ?? 0,
    m1: values[1] ?? 0,
    m2: values[2] ?? 0,
    m3: values[3] ?? 0
  };
}

function eulerToQuaternion(roll: number, pitch: number, yaw: number): Record<string, number> {
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);

  return {
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
    w: cr * cp * cy + sr * sp * sy
  };
}

function pwmFromBipolar(value: number): number {
  return Math.round(1500 + clamp(value, -1, 1) * 500);
}

function modeToSynapseFlightMode(mode: string): number {
  return ['hold', 'manual', 'mission', 'return', 'land'].indexOf(mode) + 1 || 1;
}

function degToRad(degrees: number): number {
  return (degrees * Math.PI) / 180;
}

function clamp01(value: number): number {
  return clamp(value, 0, 1);
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

export function makeSimulatedEvent(vehicleId = DEFAULT_VEHICLE_ID, sequence = 1, message = 'Simulator heartbeat'): EventFrame {
  const nowMs = Date.now();
  return {
    kind: 'event',
    topic: resolveTopic(TOPIC_DEFINITIONS.event.topic, vehicleId),
    header: makeHeader({
      vehicleId,
      sequence,
      nowMs,
      ttlMs: 0,
      messageType: 'Event',
      streamId: 'event'
    }),
    payload: {
      severity: 'info',
      code: 'sim',
      message,
      timestampMs: nowMs
    }
  };
}

export function createDemoReplay(vehicleId = DEFAULT_VEHICLE_ID, durationMs = 60_000, stepMs = 100): GcsFrame[] {
  const frames: GcsFrame[] = [];
  let sequence = 1;
  const baseMs = Date.now();

  for (let elapsedMs = 0; elapsedMs <= durationMs; elapsedMs += stepMs) {
    const bundle = makeSimulatedTelemetryBundle({
      vehicleId,
      elapsedMs,
      sequenceStart: sequence,
      nowMs: baseMs + elapsedMs,
      armed: elapsedMs > 5000,
      mode: elapsedMs > 30_000 ? 'mission' : 'hold'
    });
    sequence += bundle.length;
    frames.push(...bundle);

    if (elapsedMs % 10_000 === 0) {
      frames.push(makeSimulatedEvent(vehicleId, sequence++, elapsedMs === 0 ? 'Replay log loaded' : 'Replay marker'));
    }
  }

  return frames;
}
