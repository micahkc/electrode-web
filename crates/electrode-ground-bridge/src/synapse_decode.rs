//! Best-effort decoding of Synapse FlatBuffer payloads observed on Zenoh.
//!
//! Real Cerebri/Synapse vehicles (and the SIL) publish Synapse messages on
//! compact catalog keys, `[<namespace>/]<key>[/<instance>]` (synapse_fbs
//! 0.7.0), e.g. `cub1/att`. We classify by the Zenoh key and decode the
//! topics we understand. Anything we do not recognize is still surfaced to the
//! operator as a raw payload so it remains discoverable.
//!
//! Catalog topics carry a mandatory value contract in the Zenoh encoding
//! (`application/x-synapse-struct;type=…;schema=sha256-128:…` for fixed-layout
//! topics, `application/x-flatbuffers;…` for root tables). Samples whose
//! encoding is missing or does not match the catalog contract are surfaced
//! raw, never decoded.
//!
//! Wire encoding: every fixed-layout topic is transmitted as the *bare*
//! `*Data` struct (raw fixed-size bytes), not a FlatBuffers root table. We
//! decode those exactly like `synapse_fbs::topic_decode::decode_struct`
//! (size-check then `Follow::follow`). Only `mocap` is a root table.

use flatbuffers::root;
use serde_json::{json, Value};
use synapse_fbs::topic::{
    AttitudeCommandData, AttitudeEstimateData, AttitudeEstimateFlags, ControlLoopMetricsData,
    GnssFixData, GnssFixFlags, ManualControlData, ManualControlFlags, MocapFrame, MocapPoseFrame,
    MocapRawFlags, NavigationTargetData, PowerStatusData, PwmSignalOutputsData, RadioControlData,
    RawPoseData, SensorComponentFlags, VehicleHealthData, VehicleHealthFlags,
};
use synapse_fbs::types::{GnssFixType, RotationMatrix3f};

/// A payload decoded (or passed through) from a Zenoh sample.
pub(crate) struct Decoded {
    /// Human-facing message type, e.g. `AttitudeEstimate` or `Raw`.
    pub schema: &'static str,
    /// JSON payload forwarded to the browser.
    pub payload: Value,
}

/// Classify a Zenoh key into the Synapse schema we expect on it.
pub(crate) fn classify(key: &str) -> &'static str {
    // Resolve the canonical topic through the catalog's compact-key grammar,
    // `[<namespace>/]<key>[/<instance>]` (handles namespaces and instances).
    if let Some(parsed) = synapse_fbs::topic_catalog::parse_key(key) {
        if let Some(schema) = schema_for_topic(parsed.topic.name) {
            return schema;
        }
    }
    // The qualisys bridge's custom (non-catalog) mocap keys.
    if key.contains("mocap") {
        "MocapFrame"
    } else {
        "Raw"
    }
}

/// Map a catalog topic name to the schema name we decode it as. Optical flow
/// and everything else fall through to the raw passthrough.
fn schema_for_topic(name: &str) -> Option<&'static str> {
    Some(match name {
        "MocapFrame" => "MocapFrame",
        "MocapPoseFrame" => "MocapPoseFrame",
        "ManualControlCommand" => "ManualControl",
        "RadioControl" => "RadioControl",
        "GnssFix" => "GnssFix",
        "PwmSignalOutputs" => "PwmSignalOutputs",
        "AttitudeEstimate" => "AttitudeEstimate",
        "AttitudeCommand" => "AttitudeCommand",
        "NavigationTarget" => "NavigationTarget",
        "ControlLoopMetrics" => "ControlLoopMetrics",
        "VehicleHealth" => "VehicleHealth",
        "PowerStatus" => "PowerStatus",
        "RawPose" => "RawPose",
        _ => return None,
    })
}

/// Enforce the mandatory value contract for catalog topics: the sample's
/// Zenoh encoding must match the catalog contract exactly, and its wire type
/// must belong to the topic the key names. Non-catalog keys carry no contract.
fn contract_error(key: &str, encoding: Option<&str>) -> Option<String> {
    let parsed = synapse_fbs::topic_catalog::parse_key(key)?;
    let Some(encoding) = encoding else {
        return Some(format!(
            "missing value encoding for catalog topic {}",
            parsed.topic.name
        ));
    };
    // zenoh-pico stores an unregistered/custom media type as the schema on
    // its default byte encoding, which round-trips as
    // `zenoh/bytes;<original encoding>`.  Validate the complete inner
    // Synapse contract while retaining strict rejection of every other
    // prefix or suffix.
    let normalized = encoding.strip_prefix("zenoh/bytes;").unwrap_or(encoding);
    match synapse_fbs::value_contract::topic_for_encoding(normalized) {
        Ok(topic) if topic.name == parsed.topic.name => None,
        Ok(topic) => Some(format!(
            "key names {} but value contract is for {}",
            parsed.topic.name, topic.name
        )),
        Err(err) => Some(err.to_string()),
    }
}

/// Decode a Zenoh sample by key, falling back to a raw preview. Catalog
/// samples failing the value contract are surfaced raw with the reason, never
/// decoded.
pub(crate) fn decode(key: &str, encoding: Option<&str>, bytes: &[u8]) -> Decoded {
    if let Some(reason) = contract_error(key, encoding) {
        let mut payload = raw_payload(bytes);
        payload["contractError"] = Value::String(reason);
        return Decoded {
            schema: "Raw",
            payload,
        };
    }
    match classify(key) {
        "MocapFrame" => decode_or_raw("MocapFrame", bytes, decode_mocap_frame),
        "MocapPoseFrame" => decode_or_raw("MocapPoseFrame", bytes, decode_mocap_pose_frame),
        "ManualControl" => decode_or_raw("ManualControl", bytes, decode_manual_control),
        "RadioControl" => decode_or_raw("RadioControl", bytes, decode_radio_control),
        "GnssFix" => decode_or_raw("GnssFix", bytes, decode_gnss_fix),
        "PwmSignalOutputs" => decode_or_raw("PwmSignalOutputs", bytes, decode_pwm_signal_outputs),
        "AttitudeEstimate" => decode_or_raw("AttitudeEstimate", bytes, decode_attitude_estimate),
        "AttitudeCommand" => decode_or_raw("AttitudeCommand", bytes, decode_attitude_command),
        "NavigationTarget" => decode_or_raw("NavigationTarget", bytes, decode_navigation_target),
        "ControlLoopMetrics" => {
            decode_or_raw("ControlLoopMetrics", bytes, decode_control_loop_metrics)
        }
        "VehicleHealth" => decode_or_raw("VehicleHealth", bytes, decode_vehicle_health),
        "PowerStatus" => decode_or_raw("PowerStatus", bytes, decode_power_status),
        "RawPose" => decode_or_raw("RawPose", bytes, decode_raw_pose),
        schema => Decoded {
            schema,
            payload: raw_payload(bytes),
        },
    }
}

fn decode_or_raw(
    schema: &'static str,
    bytes: &[u8],
    decoder: fn(&[u8]) -> Option<Value>,
) -> Decoded {
    match decoder(bytes) {
        Some(payload) => Decoded { schema, payload },
        None => Decoded {
            schema,
            payload: raw_payload(bytes),
        },
    }
}

fn raw_payload(bytes: &[u8]) -> Value {
    let preview: String = bytes
        .iter()
        .take(32)
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join("");
    json!({ "bytes": bytes.len(), "hexPreview": preview })
}

/// Decode a bare fixed-layout struct topic. Mirrors
/// `synapse_fbs::topic_decode::decode_struct`: verify the payload is exactly
/// the catalog struct size, then follow it at offset 0.
fn decode_struct<'a, T>(payload: &'a [u8], expected: usize) -> Option<T::Inner>
where
    T: flatbuffers::Follow<'a>,
{
    if payload.len() != expected {
        return None;
    }
    // Safety: generated fixed-layout structs are repr(transparent) byte arrays
    // with unaligned accessors, and the exact-size check above guarantees the
    // buffer covers the struct.
    Some(unsafe { T::follow(payload, 0) })
}

fn decode_attitude_command(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<AttitudeCommandData>(bytes, 48)?;
    let attitude = data.attitude();
    let rates = data.body_rate_flu_rad_s();
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "attitude": {
                "w": attitude.w(),
                "x": attitude.x(),
                "y": attitude.y(),
                "z": attitude.z()
            },
            "body_rate_flu_rad_s": {
                "roll": rates.roll(),
                "pitch": rates.pitch(),
                "yaw": rates.yaw()
            },
            "thrust": data.thrust(),
            "type_mask": data.type_mask()
        }
    }))
}

fn decode_navigation_target(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<NavigationTargetData>(bytes, 32)?;
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "altitude_error_m": data.altitude_error_m(),
            "airspeed_error_m_s": data.airspeed_error_m_s(),
            "xtrack_error_m": data.xtrack_error_m(),
            "desired_roll_deg": f64::from(data.desired_roll_cdeg()) / 100.0,
            "desired_pitch_deg": f64::from(data.desired_pitch_cdeg()) / 100.0,
            "desired_yaw_deg": f64::from(data.desired_yaw_cdeg()) / 100.0,
            "target_yaw_deg": f64::from(data.target_yaw_cdeg()) / 100.0,
            "distance_to_waypoint_m": data.distance_to_waypoint_m()
        }
    }))
}

fn decode_control_loop_metrics(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<ControlLoopMetricsData>(bytes, 24)?;
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "period_us": data.period_us(),
            "latency_us": data.latency_us(),
            "overrun_count": data.overrun_count(),
            "load_pct": f64::from(data.load_dpermille()) / 10.0
        }
    }))
}

fn decode_attitude_estimate(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<AttitudeEstimateData>(bytes, 40)?;
    let attitude = data.attitude();
    let rates = data.angular_velocity_flu_rad_s();
    let flags = AttitudeEstimateFlags::from_bits_retain(data.flags());
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "attitude": {
                "w": attitude.w(),
                "x": attitude.x(),
                "y": attitude.y(),
                "z": attitude.z()
            },
            "angular_velocity": {
                "roll": rates.roll(),
                "pitch": rates.pitch(),
                "yaw": rates.yaw()
            },
            "attitude_valid": flags.contains(AttitudeEstimateFlags::AttitudeValid),
            "rates_valid": flags.contains(AttitudeEstimateFlags::RatesValid)
        }
    }))
}

/// `SensorComponentFlags` bit names, for reporting which components are down.
const SENSOR_COMPONENT_NAMES: [(SensorComponentFlags, &str); 16] = [
    (SensorComponentFlags::Gyro, "gyro"),
    (SensorComponentFlags::Accel, "accel"),
    (SensorComponentFlags::Mag, "mag"),
    (SensorComponentFlags::AbsolutePressure, "baro"),
    (SensorComponentFlags::DifferentialPressure, "airspeed"),
    (SensorComponentFlags::Gnss, "gnss"),
    (SensorComponentFlags::OpticalFlow, "optical flow"),
    (SensorComponentFlags::VisionPosition, "vision"),
    (SensorComponentFlags::Rangefinder, "rangefinder"),
    (SensorComponentFlags::RadioControl, "rc"),
    (SensorComponentFlags::MotorOutputs, "motors"),
    (SensorComponentFlags::Battery, "battery"),
    (SensorComponentFlags::Estimator, "estimator"),
    (SensorComponentFlags::Logging, "logging"),
    (SensorComponentFlags::CommandLink, "command link"),
    (SensorComponentFlags::Terrain, "terrain"),
];

fn sensor_component_names(mask: SensorComponentFlags) -> Vec<&'static str> {
    SENSOR_COMPONENT_NAMES
        .iter()
        .filter(|(bit, _)| mask.contains(*bit))
        .map(|(_, name)| *name)
        .collect()
}

/// Decode a `health` (VehicleHealth) struct.
///
/// The sensor bitmasks are the authoritative health signal: a component that is
/// enabled but missing from `sensors_health` has failed, and some producers
/// report that while never setting the `Failsafe` flag at all. Battery and load
/// are null unless the vehicle actually reports those subsystems, because a
/// producer with no battery monitor leaves the fields at zero, which would
/// otherwise render as a flat pack rather than as no data.
fn decode_vehicle_health(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<VehicleHealthData>(bytes, 48)?;
    let flags = VehicleHealthFlags::from_bits_retain(data.flags());
    let present = SensorComponentFlags::from_bits_retain(data.sensors_present());
    let enabled = SensorComponentFlags::from_bits_retain(data.sensors_enabled());
    let health = SensorComponentFlags::from_bits_retain(data.sensors_health());
    let unhealthy = enabled.difference(health);
    let battery_present = present.contains(SensorComponentFlags::Battery);
    let load_dpermille = data.load_dpermille();

    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "flight_mode": data.flight_mode(),
            "link_quality_pct": data.link_quality_pct(),
            "sensors_present": present.bits(),
            "sensors_enabled": enabled.bits(),
            "sensors_health": health.bits(),
            "unhealthy_sensors": sensor_component_names(unhealthy),
            "battery_present": battery_present,
            "voltage_battery_v": battery_present
                .then(|| f64::from(data.voltage_battery_cv()) / 100.0),
            "current_battery_a": battery_present
                .then(|| f64::from(data.current_battery_da()) / 10.0),
            "battery_remaining_pct": battery_present.then(|| data.battery_remaining_pct()),
            "armed": flags.contains(VehicleHealthFlags::Armed),
            // An enabled-but-unhealthy component is a failsafe condition even
            // when the producer never raises the flag itself.
            "failsafe": flags.contains(VehicleHealthFlags::Failsafe) || !unhealthy.is_empty(),
            "failsafe_flag": flags.contains(VehicleHealthFlags::Failsafe),
            "system_state": data.system_state(),
            // A running vehicle never reports exactly 0.0% load, so treat it as
            // "not reported" rather than as an idle CPU.
            "load_pct": (load_dpermille > 0).then(|| f64::from(load_dpermille) / 10.0)
        }
    }))
}

/// The schema defines 65535 as "at or above 65.535 m, unusable". Producers with
/// no accuracy estimate saturate rather than truncate, so this is a "no figure
/// available" sentinel, not a large-but-real one.
const UNUSABLE_ACCURACY: u16 = u16::MAX;

fn accuracy_or_null(milli: u16) -> Option<f64> {
    (milli != UNUSABLE_ACCURACY).then(|| f64::from(milli) / 1000.0)
}

fn gnss_fix_type_name(fix_type: GnssFixType) -> &'static str {
    match fix_type {
        GnssFixType::NoFix => "no fix",
        GnssFixType::TimeOnly => "time only",
        GnssFixType::Fix2d => "2D",
        GnssFixType::Fix3d => "3D",
        GnssFixType::Dgnss => "DGNSS",
        GnssFixType::RtkFloat => "RTK float",
        GnssFixType::RtkFixed => "RTK fixed",
        GnssFixType::DeadReckoning => "dead reckoning",
        _ => "unknown",
    }
}

/// Decode a `gnss` (GnssFix) struct.
///
/// Every field a producer may leave unpopulated is null rather than its zero
/// value, because a zero here is indistinguishable from a real measurement and
/// would render as one: 0 m accuracy reads as perfect, and a zeroed lat/lon
/// plots as a position in the Gulf of Guinea.
fn decode_gnss_fix(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<GnssFixData>(bytes, 64)?;
    let flags = GnssFixFlags::from_bits_retain(data.flags());
    let fix_type = data.fix_type();
    // There is no position-valid flag, and producers publish samples while the
    // receiver is still acquiring — deliberately, so that "receiver alive, no
    // lock yet" is distinguishable from "receiver silent". fix_type is the only
    // signal that the position fields mean anything.
    let position_valid = fix_type.0 >= GnssFixType::Fix2d.0;
    let yaw_valid = flags.contains(GnssFixFlags::YawValid);
    let satellites_used = data.satellites_used();
    let satellites_visible = data.satellites_visible();

    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "fix_type": fix_type.0,
            "fix_type_name": gnss_fix_type_name(fix_type),
            "position_valid": position_valid,
            "latitude_deg": position_valid.then(|| f64::from(data.latitude_deg_e7()) / 1e7),
            "longitude_deg": position_valid.then(|| f64::from(data.longitude_deg_e7()) / 1e7),
            "altitude_msl_m": position_valid.then(|| f64::from(data.altitude_msl_mm()) / 1000.0),
            "altitude_ellipsoid_m": position_valid
                .then(|| f64::from(data.altitude_ellipsoid_mm()) / 1000.0),
            "horizontal_accuracy_m": accuracy_or_null(data.horizontal_accuracy_mm()),
            "vertical_accuracy_m": accuracy_or_null(data.vertical_accuracy_mm()),
            "velocity_accuracy_mps": accuracy_or_null(data.velocity_accuracy_mm_s()),
            // A real fix never has a DOP below 1, so 0 means "not reported".
            "hdop": (data.hdop_centi() > 0).then(|| f64::from(data.hdop_centi()) / 100.0),
            "vdop": (data.vdop_centi() > 0).then(|| f64::from(data.vdop_centi()) / 100.0),
            "ground_speed_mps": f64::from(data.ground_speed_cm_s()) / 100.0,
            "course_over_ground_deg": flags
                .contains(GnssFixFlags::CourseValid)
                .then(|| f64::from(data.course_over_ground_cdeg()) / 100.0),
            "yaw_deg": yaw_valid.then(|| f64::from(data.yaw_cdeg()) / 100.0),
            // Gated on YawValid as well: a producer that does not compute
            // heading leaves the accuracy at 0 instead of saturating it, and 0
            // would read as a perfect heading rather than a missing one.
            "yaw_accuracy_deg": yaw_valid.then(|| f64::from(data.yaw_accuracy_cdeg()) / 100.0),
            "velocity_up_mps": flags
                .contains(GnssFixFlags::VelocityUpValid)
                .then(|| f64::from(data.velocity_up_cm_s()) / 100.0),
            "time_unix_us": flags.contains(GnssFixFlags::TimeValid).then(|| data.time_unix_us()),
            "satellites_used": satellites_used,
            // Visible can never be below used; a smaller value means the
            // producer does not report it.
            "satellites_visible": (satellites_visible >= satellites_used)
                .then_some(satellites_visible),
            "id": data.id()
        }
    }))
}

fn decode_power_status(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<PowerStatusData>(bytes, 64)?;
    let voltages = data.voltages();
    let cells_mv: Vec<u16> = vec![
        voltages.cell0_mv(),
        voltages.cell1_mv(),
        voltages.cell2_mv(),
        voltages.cell3_mv(),
        voltages.cell4_mv(),
        voltages.cell5_mv(),
        voltages.cell6_mv(),
        voltages.cell7_mv(),
        voltages.cell8_mv(),
        voltages.cell9_mv(),
        voltages.cell10_mv(),
        voltages.cell11_mv(),
        voltages.cell12_mv(),
        voltages.cell13_mv(),
        voltages.cell14_mv(),
        voltages.cell15_mv(),
    ];
    // Pack voltage is the sum of populated (non-zero) cells, millivolts to volts.
    let pack_mv: u32 = cells_mv.iter().map(|&cell| u32::from(cell)).sum();
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "voltage_v": f64::from(pack_mv) / 1000.0,
            "current_a": f64::from(data.current_battery_da()) / 10.0,
            "remaining_pct": data.remaining_pct(),
            "connected": data.connected(),
            "cells_mv": cells_mv,
            "temperature_c": f64::from(data.temperature_cdeg()) / 100.0
        }
    }))
}

fn decode_raw_pose(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<RawPoseData>(bytes, 40)?;
    let pose = data.pose();
    let position = pose.position_enu_m();
    let attitude = pose.attitude();
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "position": { "x": position.x(), "y": position.y(), "z": position.z() },
            "attitude": { "x": attitude.x(), "y": attitude.y(), "z": attitude.z(), "w": attitude.w() }
        }
    }))
}

fn decode_manual_control(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<ManualControlData>(bytes, 40)?;
    let flags = ManualControlFlags::from_bits_retain(data.flags());
    // Axes and aux channels are scaled shorts in -1000..1000 => normalized /1000.
    let milli = |value: i16| f64::from(value) / 1000.0;
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "axes": {
                "roll": milli(data.roll_milli()),
                "pitch": milli(data.pitch_milli()),
                "yaw": milli(data.yaw_milli()),
                "throttle": milli(data.throttle_milli())
            },
            "aux": [
                milli(data.aux0_milli()),
                milli(data.aux1_milli()),
                milli(data.aux2_milli()),
                milli(data.aux3_milli()),
                milli(data.aux4_milli()),
                milli(data.aux5_milli())
            ],
            "flight_mode": data.flight_mode(),
            "arm_switch": flags.contains(ManualControlFlags::ArmSwitch),
            "kill_switch": flags.contains(ManualControlFlags::KillSwitch),
            "active": flags.contains(ManualControlFlags::Active),
            "valid": flags.contains(ManualControlFlags::Valid),
            "buttons": data.buttons()
        }
    }))
}

fn decode_radio_control(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<RadioControlData>(bytes, 48)?;
    let channels: Vec<u16> = vec![
        data.chan0_raw_us(),
        data.chan1_raw_us(),
        data.chan2_raw_us(),
        data.chan3_raw_us(),
        data.chan4_raw_us(),
        data.chan5_raw_us(),
        data.chan6_raw_us(),
        data.chan7_raw_us(),
        data.chan8_raw_us(),
        data.chan9_raw_us(),
        data.chan10_raw_us(),
        data.chan11_raw_us(),
        data.chan12_raw_us(),
        data.chan13_raw_us(),
        data.chan14_raw_us(),
        data.chan15_raw_us(),
        data.chan16_raw_us(),
        data.chan17_raw_us(),
    ];
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "channel_count": data.channel_count(),
            "link_quality_pct": data.link_quality_pct()
        },
        "channels": {
            "ch0": channels[0],
            "ch1": channels[1],
            "ch2": channels[2],
            "ch3": channels[3],
            "ch4": channels[4],
            "ch5": channels[5],
            "ch6": channels[6],
            "ch7": channels[7],
            "ch8": channels[8],
            "ch9": channels[9],
            "ch10": channels[10],
            "ch11": channels[11],
            "ch12": channels[12],
            "ch13": channels[13],
            "ch14": channels[14],
            "ch15": channels[15],
            "ch16": channels[16],
            "ch17": channels[17]
        }
    }))
}

fn decode_pwm_signal_outputs(bytes: &[u8]) -> Option<Value> {
    let data = decode_struct::<PwmSignalOutputsData>(bytes, 48)?;
    let outputs_us: Vec<u16> = vec![
        data.output0_us(),
        data.output1_us(),
        data.output2_us(),
        data.output3_us(),
        data.output4_us(),
        data.output5_us(),
        data.output6_us(),
        data.output7_us(),
        data.output8_us(),
        data.output9_us(),
        data.output10_us(),
        data.output11_us(),
        data.output12_us(),
        data.output13_us(),
        data.output14_us(),
        data.output15_us(),
    ];
    Some(json!({
        "data": {
            "timestamp_us": data.timestamp_us(),
            "active_mask": data.active_mask(),
            "port": data.port(),
            "outputs_us": outputs_us
        },
        // First four outputs kept as a motors object so state-store's
        // parseMotorOutputs stays simple.
        "motors": {
            "m0": outputs_us[0],
            "m1": outputs_us[1],
            "m2": outputs_us[2],
            "m3": outputs_us[3]
        }
    }))
}

fn decode_mocap_frame(bytes: &[u8]) -> Option<Value> {
    let frame = root::<MocapFrame>(bytes).ok()?;
    // Report every tracked rigid body's pose (position + attitude), matching
    // the wire schema's `rigid_bodies` vector so downstream consumers can index
    // `rigid_bodies[0]`.
    let mut rigid_bodies: Vec<Value> = Vec::new();
    if let Some(bodies) = frame.rigid_bodies() {
        for body in bodies.iter() {
            let p = body.position_enu_m();
            let q = rotation_matrix_to_quaternion(body.rotation());
            let flags = MocapRawFlags::from_bits_retain(body.flags());
            rigid_bodies.push(json!({
                "id": body.id(),
                "position": { "x": p.x(), "y": p.y(), "z": p.z() },
                "attitude": { "x": q.0, "y": q.1, "z": q.2, "w": q.3 },
                "residual": body.residual_mm(),
                "tracking_valid": flags.contains(MocapRawFlags::Valid)
            }));
        }
    }
    let mut labeled_markers: Vec<Value> = Vec::new();
    if let Some(markers) = frame.markers() {
        for marker in markers.iter() {
            let p = marker.position_enu_m();
            labeled_markers.push(json!({
                "id": marker.id(),
                "position": { "x": p.x(), "y": p.y(), "z": p.z() },
                "residual": marker.residual_mm()
            }));
        }
    }
    Some(json!({
        "timestamp_us": frame.timestamp_us(),
        "frame_number": frame.frame_number(),
        "rigid_bodies": rigid_bodies,
        "labeled_markers": labeled_markers
    }))
}

fn decode_mocap_pose_frame(bytes: &[u8]) -> Option<Value> {
    let frame = root::<MocapPoseFrame>(bytes).ok()?;
    let mut rigid_bodies = Vec::new();
    if let Some(bodies) = frame.rigid_bodies() {
        for body in bodies.iter() {
            let pose = body.pose();
            let p = pose.position_enu_m();
            let q = pose.attitude();
            let flags = MocapRawFlags::from_bits_retain(body.flags());
            rigid_bodies.push(json!({
                "id": body.id(),
                "position": { "x": p.x(), "y": p.y(), "z": p.z() },
                "attitude": { "x": q.x(), "y": q.y(), "z": q.z(), "w": q.w() },
                "residual": body.residual_mm(),
                "tracking_valid": flags.contains(MocapRawFlags::Valid)
            }));
        }
    }
    let mut labeled_markers = Vec::new();
    if let Some(markers) = frame.markers() {
        for marker in markers.iter() {
            let p = marker.position_enu_m();
            labeled_markers.push(json!({
                "id": marker.id(),
                "position": { "x": p.x(), "y": p.y(), "z": p.z() },
                "residual": marker.residual_mm()
            }));
        }
    }
    Some(json!({
        "timestamp_us": frame.timestamp_us(),
        "frame_number": frame.frame_number(),
        "rigid_bodies": rigid_bodies,
        "labeled_markers": labeled_markers
    }))
}

fn rotation_matrix_to_quaternion(rotation: &RotationMatrix3f) -> (f32, f32, f32, f32) {
    let trace = rotation.r11() + rotation.r22() + rotation.r33();
    let quaternion = if trace > 0.0 {
        let scale = (trace + 1.0).sqrt() * 2.0;
        (
            (rotation.r32() - rotation.r23()) / scale,
            (rotation.r13() - rotation.r31()) / scale,
            (rotation.r21() - rotation.r12()) / scale,
            0.25 * scale,
        )
    } else if rotation.r11() > rotation.r22() && rotation.r11() > rotation.r33() {
        let scale = (1.0 + rotation.r11() - rotation.r22() - rotation.r33()).sqrt() * 2.0;
        (
            0.25 * scale,
            (rotation.r12() + rotation.r21()) / scale,
            (rotation.r13() + rotation.r31()) / scale,
            (rotation.r32() - rotation.r23()) / scale,
        )
    } else if rotation.r22() > rotation.r33() {
        let scale = (1.0 + rotation.r22() - rotation.r11() - rotation.r33()).sqrt() * 2.0;
        (
            (rotation.r12() + rotation.r21()) / scale,
            0.25 * scale,
            (rotation.r23() + rotation.r32()) / scale,
            (rotation.r13() - rotation.r31()) / scale,
        )
    } else {
        let scale = (1.0 + rotation.r33() - rotation.r11() - rotation.r22()).sqrt() * 2.0;
        (
            (rotation.r13() + rotation.r31()) / scale,
            (rotation.r23() + rotation.r32()) / scale,
            0.25 * scale,
            (rotation.r21() - rotation.r12()) / scale,
        )
    };
    normalize_quaternion(quaternion)
}

fn normalize_quaternion(quaternion: (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
    let norm = (quaternion.0.mul_add(
        quaternion.0,
        quaternion.1.mul_add(
            quaternion.1,
            quaternion
                .2
                .mul_add(quaternion.2, quaternion.3 * quaternion.3),
        ),
    ))
    .sqrt();
    if !norm.is_finite() || norm == 0.0 {
        return (0.0, 0.0, 0.0, 1.0);
    }
    (
        quaternion.0 / norm,
        quaternion.1 / norm,
        quaternion.2 / norm,
        quaternion.3 / norm,
    )
}

#[cfg(test)]
mod tests {
    use super::{contract_error, decode_gnss_fix, decode_vehicle_health};

    #[test]
    fn accepts_zenoh_pico_byte_wrapper_around_exact_contract() {
        let parsed = synapse_fbs::topic_catalog::parse_key("pwm").expect("pwm catalog topic");
        let expected = synapse_fbs::value_contract::encoding_for_topic(parsed.topic);
        let wrapped = format!("zenoh/bytes;{expected}");

        assert_eq!(contract_error("pwm", Some(&wrapped)), None);
    }

    /// Field offsets are the vehicle's own struct layout, padding included.
    fn gnss_bytes(fix_type: u8, latitude_deg_e7: i32, longitude_deg_e7: i32) -> Vec<u8> {
        let mut bytes = vec![0_u8; 64];
        bytes[16..20].copy_from_slice(&latitude_deg_e7.to_le_bytes());
        bytes[20..24].copy_from_slice(&longitude_deg_e7.to_le_bytes());
        // The onboard NMEA path saturates every accuracy field.
        bytes[32..34].copy_from_slice(&u16::MAX.to_le_bytes());
        bytes[34..36].copy_from_slice(&u16::MAX.to_le_bytes());
        bytes[36..38].copy_from_slice(&u16::MAX.to_le_bytes());
        bytes[53] = fix_type;
        bytes
    }

    #[test]
    fn decodes_a_locked_gnss_fix() {
        let value = decode_gnss_fix(&gnss_bytes(3, 377_749_000, -1_224_194_000)).expect("decoded");
        let data = &value["data"];

        assert_eq!(data["position_valid"], true);
        assert_eq!(data["fix_type_name"], "3D");
        assert_eq!(data["latitude_deg"], 37.7749);
        assert_eq!(data["longitude_deg"], -122.4194);
        // Saturated means unusable, not 65.5 m.
        assert!(data["horizontal_accuracy_m"].is_null());
    }

    /// The vehicle streams while the receiver acquires, and those samples carry
    /// a zeroed position that would otherwise plot off West Africa.
    #[test]
    fn reports_no_position_before_the_receiver_locks() {
        for fix_type in [0, 1] {
            let value = decode_gnss_fix(&gnss_bytes(fix_type, 0, 0)).expect("decoded");
            let data = &value["data"];

            assert_eq!(data["position_valid"], false);
            assert!(data["latitude_deg"].is_null());
            assert!(data["longitude_deg"].is_null());
        }
    }

    fn health_bytes(present: u32, enabled: u32, health: u32, flags: u8) -> Vec<u8> {
        let mut bytes = vec![0_u8; 48];
        bytes[8..12].copy_from_slice(&present.to_le_bytes());
        bytes[12..16].copy_from_slice(&enabled.to_le_bytes());
        bytes[16..20].copy_from_slice(&health.to_le_bytes());
        bytes[47] = flags;
        bytes
    }

    #[test]
    fn reports_no_battery_when_the_vehicle_has_no_battery_component() {
        const GYRO_ACCEL_MOTORS: u32 = 1 | 2 | 1024;
        let value = decode_vehicle_health(&health_bytes(
            GYRO_ACCEL_MOTORS,
            GYRO_ACCEL_MOTORS,
            GYRO_ACCEL_MOTORS,
            1,
        ))
        .expect("decoded");
        let data = &value["data"];

        assert_eq!(data["battery_present"], false);
        assert!(data["voltage_battery_v"].is_null());
        assert!(data["battery_remaining_pct"].is_null());
        assert!(data["load_pct"].is_null());
    }

    /// The sensor bitmask is the authoritative failsafe indicator: RDD2 raises
    /// Armed but never the Failsafe flag itself.
    #[test]
    fn raises_failsafe_when_an_enabled_component_drops_out_of_health() {
        const RADIO_CONTROL: u32 = 512;
        const ENABLED: u32 = 1 | 2 | RADIO_CONTROL | 1024 | 4096;
        let value =
            decode_vehicle_health(&health_bytes(ENABLED, ENABLED, ENABLED & !RADIO_CONTROL, 1))
                .expect("decoded");
        let data = &value["data"];

        assert_eq!(data["armed"], true);
        assert_eq!(data["failsafe_flag"], false);
        assert_eq!(data["failsafe"], true);
        assert_eq!(data["unhealthy_sensors"], serde_json::json!(["rc"]));
    }
}
