import { describe, expect, it } from 'vitest';
import { parseKey } from '@cognipilot/synapse-fbs/topic_catalog';

import {
  encodeCompactRigidBodyPose,
  encodeMocapFrame,
  encodeMocapPoseFrame,
  encodeRawPose
} from './mocap-encode';
import { classify, decode, expectedTopicEncoding } from './synapse-decode';

const EXTERNAL_ODOMETRY_TOPIC = parseKey('cub1/external_pose')!.topic;

describe('Synapse decoder', () => {
  it('decodes the Synapse 0.7 raw MocapPoseFrame without using an odometry estimate', () => {
    const bytes = encodeMocapPoseFrame(
      {
        position: { x: 1.25, y: -2.5, z: 3.75 },
        attitude: { x: 0, y: 0, z: 0.7071068, w: 0.7071068 }
      },
      { timestampUs: 42, frameNumber: 7, bodyId: 3, residual: 0.5 }
    );
    const topic = parseKey('qualisys/cub1/mocap')!.topic;
    const decoded = decode('qualisys/cub1/mocap', bytes, expectedTopicEncoding(topic));

    expect(decoded.schema).toBe('MocapPoseFrame');
    expect(decoded.decoded).toBe(true);
    expect(decoded.payload).toMatchObject({
      timestamp_us: 42,
      frame_number: 7,
      rigid_bodies: [{ id: 3, position: { x: 1.25, y: -2.5, z: 3.75 }, tracking_valid: true }]
    });
  });

  it('classifies known topic keys', () => {
    expect(classify('robot/manual')).toBe('ManualControl');
    // The telemetry bridge publishes multi-instance topics with the producer
    // instance in the key, so both forms must resolve.
    expect(classify('gnss')).toBe('GnssFix');
    expect(classify('gnss/0')).toBe('GnssFix');
    expect(classify('cub1/gnss/1')).toBe('GnssFix');
    expect(classify('synapse/mocap/rigid_body/cub1/pose')).toBe('MocapFrame');
    expect(classify('synapse/mocap/frame')).toBe('MocapFrame');
    expect(classify('synapse/v1/topic/mocap_frame')).toBe('MocapFrame');
    expect(classify('qualisys/cub1/pose_raw')).toBe('RawPose');
    expect(classify('qualisys/cub1/pose')).toBe('Raw');
    expect(classify('synapse/mocap/definition')).toBe('Raw');
    expect(classify('synapse/v1/topic/unknown')).toBe('Raw');
  });

  it('decodes encoded mocap FlatBuffer samples', () => {
    const bytes = encodeMocapFrame(
      {
        position: { x: 1.25, y: -2.5, z: 3.75 },
        attitude: { x: 0, y: 0, z: 0, w: 1 }
      },
      {
        frameNumber: 42,
        timestampUs: 123_456,
        bodyId: 9,
        residual: 0.01,
        trackingValid: true
      }
    );

    const decoded = decode('synapse/mocap/rigid_body/cub1/pose', bytes);

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('MocapFrame');
    expect(decoded.payload).toMatchObject({
      timestamp_us: 123_456,
      frame_number: 42,
      rigid_bodies: [
        {
          id: 9,
          position: { x: 1.25, y: -2.5, z: 3.75 },
          attitude: { x: 0, y: 0, z: 0, w: 1 },
          tracking_valid: true
        }
      ]
    });
  });

  it('falls back to a raw payload preview for unknown topics', () => {
    const decoded = decode('synapse/v1/topic/not_yet_supported', new Uint8Array([0, 1, 2, 255]));

    expect(decoded).toEqual({
      schema: 'Raw',
      decoded: false,
      payload: { bytes: 4, hexPreview: '000102ff' }
    });
  });

  it('decodes the bridge raw pose fixed-layout payload', () => {
    const bytes = encodeRawPose(
      { position: { x: 1, y: 2, z: 3 }, attitude: { w: 1, x: 0, y: 0, z: 0 } },
      123_456
    );
    const topic = parseKey('qualisys/cub1/pose_raw')!.topic;
    const decoded = decode('qualisys/cub1/pose_raw', bytes, expectedTopicEncoding(topic));

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('RawPose');
    expect(decoded.payload).toMatchObject({
      data: {
        timestamp_us: 123_456,
        position: { x: 1, y: 2, z: 3 },
        attitude: { w: 1, x: 0, y: 0, z: 0 }
      }
    });
  });
});

describe('Value contract enforcement', () => {
  const MOCAP_TOPIC = parseKey('mocap')!.topic;

  function externalOdometryBytes(): Uint8Array {
    const bytes = new Uint8Array(64);
    const view = new DataView(bytes.buffer);
    view.setBigUint64(0, 42n, true);
    view.setFloat32(20, 1, true); // attitude.w
    return bytes;
  }

  it('accepts a struct topic carrying the exact struct encoding', () => {
    const decoded = decode(
      'external_pose/1',
      externalOdometryBytes(),
      expectedTopicEncoding(EXTERNAL_ODOMETRY_TOPIC)
    );

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('ExternalOdometry');
  });

  it('accepts the zenoh-pico byte wrapper around an exact struct encoding', () => {
    const decoded = decode(
      'external_pose/1',
      externalOdometryBytes(),
      `zenoh/bytes;${expectedTopicEncoding(EXTERNAL_ODOMETRY_TOPIC)}`
    );

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('ExternalOdometry');
  });

  it('accepts a root-table topic carrying the exact flatbuffers encoding', () => {
    const bytes = encodeMocapPoseFrame(
      { position: { x: 1, y: 2, z: 3 }, attitude: { x: 0, y: 0, z: 0, w: 1 } },
      { frameNumber: 3, timestampUs: 77, bodyId: 1 }
    );

    const decoded = decode('mocap', bytes, expectedTopicEncoding(MOCAP_TOPIC));

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('MocapPoseFrame');
    expect(decoded.payload).toMatchObject({ frame_number: 3, timestamp_us: 77 });
  });

  it('rejects a catalog-keyed sample with no encoding', () => {
    const decoded = decode('external_pose/1', externalOdometryBytes());

    expect(decoded.decoded).toBe(false);
    expect(decoded.schema).toBe('ExternalOdometry');
    expect((decoded.payload as { contractError?: string }).contractError).toMatch(/missing encoding/);
  });

  it('rejects a catalog-keyed sample with a mismatched wire type', () => {
    const wrongType = expectedTopicEncoding(MOCAP_TOPIC);
    const decoded = decode('external_pose/1', externalOdometryBytes(), wrongType);

    expect(decoded.decoded).toBe(false);
    expect((decoded.payload as { contractError?: string }).contractError).toMatch(/encoding mismatch/);
  });

  it('rejects a catalog-keyed sample with a mismatched schema hash', () => {
    const staleHash = expectedTopicEncoding(EXTERNAL_ODOMETRY_TOPIC).replace(
      /schema=sha256-128:.*/,
      'schema=sha256-128:00000000000000000000000000000000'
    );
    const decoded = decode('external_pose/1', externalOdometryBytes(), staleHash);

    expect(decoded.decoded).toBe(false);
    expect((decoded.payload as { contractError?: string }).contractError).toMatch(/encoding mismatch/);
  });

  it('exempts custom non-catalog keys from the encoding contract', () => {
    const bytes = encodeCompactRigidBodyPose({
      position: { x: 1, y: 2, z: 3 },
      attitude: { x: 0, y: 0, z: 0, w: 1 }
    });

    const decoded = decode('synapse/mocap/rigid_body/cub1/pose', bytes);

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('MocapFrame');
  });
});

describe('Mocap wire contract', () => {
  it('decodes the compact 28-byte pose exactly as synapse_qualisys_bridge encodes it', () => {
    // Hand-built wire payload — 7 little-endian f32 values
    // [px, py, pz, qx, qy, qz, qw], quaternion scalar (w) LAST. This layout is
    // the synapse_qualisys_bridge contract; it must never be read w-first.
    const bytes = new Uint8Array(28);
    const view = new DataView(bytes.buffer);
    [1.5, -2.25, 0.75, 0.1, -0.2, 0.55, 0.8].forEach((value, index) =>
      view.setFloat32(index * 4, value, true)
    );

    const decoded = decode('synapse/mocap/rigid_body/cub1/pose', bytes);

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('MocapFrame');
    const body = (decoded.payload as { rigid_bodies: Array<Record<string, unknown>> })
      .rigid_bodies[0];
    expect(body.position).toMatchObject({ x: 1.5, y: -2.25, z: 0.75 });
    const attitude = body.attitude as { x: number; y: number; z: number; w: number };
    expect(attitude.x).toBeCloseTo(0.1, 6);
    expect(attitude.y).toBeCloseTo(-0.2, 6);
    expect(attitude.z).toBeCloseTo(0.55, 6);
    expect(attitude.w).toBeCloseTo(0.8, 6);
  });

  it('round-trips the compact pose encoder through the decoder', () => {
    const pose = {
      position: { x: 4.5, y: 5.5, z: 6.5 },
      attitude: { x: 0.25, y: -0.5, z: 0.125, w: 0.75 }
    };
    const bytes = encodeCompactRigidBodyPose(pose);
    expect(bytes.length).toBe(28);

    const decoded = decode('synapse/mocap/rigid_body/cub1/pose', bytes);
    const body = (decoded.payload as { rigid_bodies: Array<Record<string, unknown>> })
      .rigid_bodies[0];
    expect(body.position).toMatchObject(pose.position);
    expect(body.attitude).toMatchObject(pose.attitude);
  });

  it('decodes raw MocapFrame FlatBuffers on the Qualisys bridge topic', () => {
    const bytes = encodeMocapFrame(
      { position: { x: 1, y: 2, z: 3 }, attitude: { x: 0, y: 0, z: 0, w: 1 } },
      { frameNumber: 7, timestampUs: 99, bodyId: 1 }
    );

    const decoded = decode('synapse/v1/topic/mocap_frame', bytes);

    expect(decoded.decoded).toBe(true);
    expect(decoded.schema).toBe('MocapFrame');
    expect(decoded.payload).toMatchObject({ frame_number: 7, timestamp_us: 99 });
  });
});

/** Field offsets are the vehicle's own struct layout, padding included. */
function encodeGnssFix(fields: Record<string, number> = {}): Uint8Array {
  const bytes = new Uint8Array(64);
  const view = new DataView(bytes.buffer);
  view.setBigUint64(0, BigInt(fields.timestampUs ?? 0), true);
  view.setBigUint64(8, BigInt(fields.timeUnixUs ?? 0), true);
  view.setInt32(16, fields.latitudeDegE7 ?? 0, true);
  view.setInt32(20, fields.longitudeDegE7 ?? 0, true);
  view.setInt32(24, fields.altitudeMslMm ?? 0, true);
  view.setInt32(28, fields.altitudeEllipsoidMm ?? 0, true);
  // The onboard NMEA path saturates all three accuracies; default to that.
  view.setUint16(32, fields.horizontalAccuracyMm ?? 0xffff, true);
  view.setUint16(34, fields.verticalAccuracyMm ?? 0xffff, true);
  view.setUint16(36, fields.velocityAccuracyMmS ?? 0xffff, true);
  view.setUint16(38, fields.yawAccuracyCdeg ?? 0, true);
  view.setUint16(40, fields.hdopCenti ?? 0, true);
  view.setUint16(42, fields.vdopCenti ?? 0, true);
  view.setUint16(44, fields.groundSpeedCmS ?? 0, true);
  view.setUint16(46, fields.courseOverGroundCdeg ?? 0, true);
  view.setUint16(48, fields.yawCdeg ?? 0, true);
  view.setInt16(50, fields.velocityUpCmS ?? 0, true);
  view.setUint8(52, fields.flags ?? 0);
  view.setUint8(53, fields.fixType ?? 0);
  view.setUint8(54, fields.satellitesUsed ?? 0);
  view.setUint8(55, fields.satellitesVisible ?? 0);
  view.setUint8(56, fields.id ?? 0);
  return bytes;
}

function decodeGnss(fields: Record<string, number> = {}): Record<string, unknown> {
  const topic = parseKey('gnss')!.topic;
  const decoded = decode('gnss', encodeGnssFix(fields), expectedTopicEncoding(topic));
  expect(decoded.schema).toBe('GnssFix');
  expect(decoded.decoded).toBe(true);
  return (decoded.payload as { data: Record<string, unknown> }).data;
}

describe('GnssFix decoder', () => {
  it('decodes a 3D fix', () => {
    const data = decodeGnss({
      latitudeDegE7: 377_749_000,
      longitudeDegE7: -1_224_194_000,
      altitudeMslMm: 12_000,
      fixType: 3,
      satellitesUsed: 11,
      satellitesVisible: 14,
      groundSpeedCmS: 350,
      hdopCenti: 90,
      horizontalAccuracyMm: 1500
    });

    expect(data).toMatchObject({
      position_valid: true,
      fix_type_name: '3D',
      latitude_deg: 37.7749,
      longitude_deg: -122.4194,
      altitude_msl_m: 12,
      ground_speed_mps: 3.5,
      satellites_used: 11,
      satellites_visible: 14,
      hdop: 0.9,
      horizontal_accuracy_m: 1.5
    });
  });

  // The vehicle publishes samples while the receiver has no lock, and those
  // carry a zeroed latitude/longitude that would plot off West Africa.
  it.each([
    ['NoFix', 0],
    ['TimeOnly', 1]
  ])('reports no position for a %s sample', (_name, fixType) => {
    const data = decodeGnss({ fixType, latitudeDegE7: 0, longitudeDegE7: 0 });

    expect(data.position_valid).toBe(false);
    expect(data.latitude_deg).toBeNull();
    expect(data.longitude_deg).toBeNull();
    expect(data.altitude_msl_m).toBeNull();
  });

  it('treats a saturated accuracy as unusable rather than as 65.5 m', () => {
    const data = decodeGnss({ fixType: 3 });

    expect(data.horizontal_accuracy_m).toBeNull();
    expect(data.vertical_accuracy_m).toBeNull();
    expect(data.velocity_accuracy_mps).toBeNull();
  });

  it('reports unpopulated optional fields as missing, not as zero', () => {
    const data = decodeGnss({ fixType: 3, satellitesUsed: 11 });

    // No validity flags set, so none of these were measured.
    expect(data.course_over_ground_deg).toBeNull();
    expect(data.yaw_deg).toBeNull();
    expect(data.velocity_up_mps).toBeNull();
    expect(data.time_unix_us).toBeNull();
    // Left at 0 rather than saturated, so it must not read as a perfect fix.
    expect(data.yaw_accuracy_deg).toBeNull();
    expect(data.vdop).toBeNull();
    // Cannot see fewer satellites than it is using: the field is unpopulated.
    expect(data.satellites_visible).toBeNull();
  });

  it('honours the validity flags when the fields are populated', () => {
    const data = decodeGnss({
      fixType: 3,
      flags: 1 | 2 | 8, // TimeValid | CourseValid | VelocityUpValid
      timeUnixUs: 1_700_000_000_000_000,
      courseOverGroundCdeg: 9_000,
      velocityUpCmS: -25
    });

    expect(data.time_unix_us).toBe(1_700_000_000_000_000);
    expect(data.course_over_ground_deg).toBe(90);
    expect(data.velocity_up_mps).toBe(-0.25);
    // YawValid is still clear.
    expect(data.yaw_deg).toBeNull();
  });
});

const SENSOR_GYRO = 1;
const SENSOR_ACCEL = 2;
const SENSOR_RC = 512;
const SENSOR_MOTORS = 1024;
const SENSOR_BATTERY = 2048;
const SENSOR_ESTIMATOR = 4096;

function encodeVehicleHealth(fields: Record<string, number> = {}): Uint8Array {
  const bytes = new Uint8Array(48);
  const view = new DataView(bytes.buffer);
  view.setBigUint64(0, BigInt(fields.timestampUs ?? 0), true);
  view.setUint32(8, fields.sensorsPresent ?? 0, true);
  view.setUint32(12, fields.sensorsEnabled ?? 0, true);
  view.setUint32(16, fields.sensorsHealth ?? 0, true);
  view.setUint16(32, fields.loadDpermille ?? 0, true);
  view.setUint16(34, fields.voltageBatteryCv ?? 0, true);
  view.setInt16(36, fields.currentBatteryDa ?? 0, true);
  view.setInt8(42, fields.batteryRemainingPct ?? 0);
  view.setUint8(44, fields.flightMode ?? 0);
  view.setUint8(45, fields.systemState ?? 0);
  view.setUint8(46, fields.linkQualityPct ?? 0);
  view.setUint8(47, fields.flags ?? 0);
  return bytes;
}

function decodeHealth(fields: Record<string, number> = {}): Record<string, unknown> {
  const topic = parseKey('health')!.topic;
  const decoded = decode('health', encodeVehicleHealth(fields), expectedTopicEncoding(topic));
  expect(decoded.decoded).toBe(true);
  return (decoded.payload as { data: Record<string, unknown> }).data;
}

describe('VehicleHealth decoder', () => {
  // A vehicle with no battery monitor leaves these at zero, which would
  // otherwise render as a flat pack rather than as an absent one.
  it('reports no battery when the vehicle has no battery component', () => {
    const data = decodeHealth({
      sensorsPresent: SENSOR_GYRO | SENSOR_ACCEL | SENSOR_MOTORS,
      sensorsEnabled: SENSOR_GYRO | SENSOR_ACCEL | SENSOR_MOTORS,
      sensorsHealth: SENSOR_GYRO | SENSOR_ACCEL | SENSOR_MOTORS
    });

    expect(data.battery_present).toBe(false);
    expect(data.voltage_battery_v).toBeNull();
    expect(data.current_battery_a).toBeNull();
    expect(data.battery_remaining_pct).toBeNull();
    expect(data.load_pct).toBeNull();
  });

  it('decodes battery telemetry when the component is present', () => {
    const data = decodeHealth({
      sensorsPresent: SENSOR_BATTERY,
      voltageBatteryCv: 1_650,
      currentBatteryDa: 82,
      batteryRemainingPct: 74
    });

    expect(data).toMatchObject({
      battery_present: true,
      voltage_battery_v: 16.5,
      current_battery_a: 8.2,
      battery_remaining_pct: 74
    });
  });

  // The sensor bitmask is the authoritative failsafe indicator: RDD2 raises
  // Armed but never the Failsafe flag itself.
  it('raises failsafe when an enabled component drops out of health', () => {
    const enabled = SENSOR_GYRO | SENSOR_ACCEL | SENSOR_RC | SENSOR_MOTORS | SENSOR_ESTIMATOR;
    const data = decodeHealth({
      sensorsPresent: enabled,
      sensorsEnabled: enabled,
      sensorsHealth: enabled & ~SENSOR_RC,
      flags: 1 // Armed, and deliberately not Failsafe
    });

    expect(data.armed).toBe(true);
    expect(data.failsafe_flag).toBe(false);
    expect(data.failsafe).toBe(true);
    expect(data.unhealthy_sensors).toEqual(['rc']);
  });

  it('stays clear of failsafe while every enabled component is healthy', () => {
    const enabled = SENSOR_GYRO | SENSOR_ACCEL | SENSOR_MOTORS | SENSOR_ESTIMATOR;
    const data = decodeHealth({
      sensorsPresent: enabled,
      sensorsEnabled: enabled,
      sensorsHealth: enabled,
      flags: 1
    });

    expect(data.failsafe).toBe(false);
    expect(data.unhealthy_sensors).toEqual([]);
  });
});

describe('InertialSample decoding', () => {
  it('decodes a bare InertialSampleData struct and drops absent fields', () => {
    // Layout of synapse.topic.InertialSampleData (56 bytes): u64 timestamp_us,
    // Vec3f accel, Vec3f gyro, Vec3f mag, f32 pressure, f32 temperature,
    // u8 flags, u8 id. The vehicle only marks Accel|Gyro present.
    const bytes = new Uint8Array(56);
    const view = new DataView(bytes.buffer);
    view.setBigUint64(0, 123_456n, true);
    view.setFloat32(8, 0.1, true);
    view.setFloat32(12, -0.2, true);
    view.setFloat32(16, 9.81, true);
    view.setFloat32(20, 0.01, true);
    view.setFloat32(24, -0.02, true);
    view.setFloat32(28, 0.03, true);
    bytes[52] = 0b0000_0011; // InertialFieldFlags Accel | Gyro
    bytes[53] = 1;

    const topic = parseKey('imu')!.topic;
    const decoded = decode('imu', bytes, expectedTopicEncoding(topic));

    expect(decoded.schema).toBe('InertialSample');
    expect(decoded.decoded).toBe(true);
    const payload = decoded.payload as {
      data: {
        timestamp_us: number;
        accel: { z: number } | null;
        gyro: { x: number } | null;
        id: number;
      };
    };
    expect(payload.data.timestamp_us).toBe(123_456);
    expect(payload.data.accel?.z).toBeCloseTo(9.81);
    expect(payload.data.gyro?.x).toBeCloseTo(0.01);
    expect(payload.data.id).toBe(1);
  });
});
