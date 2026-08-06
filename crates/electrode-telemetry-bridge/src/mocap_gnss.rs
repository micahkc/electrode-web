//! Mocap pose to `GnssFixData`, for vehicles that take their fix over the
//! telemetry radio instead of from an onboard receiver.
//!
//! This is a port of `sample_to_fix`/`build_gnss_fix` from the vehicle tree's
//! `test_scripts/publish_gps_zenoh.py` and `publish_gps_synapse.py`. The
//! arithmetic is kept identical field for field so an origin, `yaw_offset` or
//! `heading_offset` trimmed against those scripts stays correct here — a
//! calibration that silently shifted would put the vehicle's estimator
//! somewhere it is not.

use synapse_fbs::topic::{
    ExternalOdometryCovarianceData, ExternalOdometryData, ExternalOdometryFlags,
    ExternalOdometryStatus, GnssFixData, GnssFixFlags,
};
use synapse_fbs::types::GnssFixType;

/// WGS-84 ellipsoid.
const WGS84_A: f64 = 6_378_137.0;
const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// Below this ground speed a course differenced from position is noise, so the
/// fix goes out without `CourseValid` rather than with a meaningless heading.
const COURSE_VALID_MIN_SPEED_MS: f64 = 0.15;

/// Accuracy fields saturate at 65535 mm, which the schema documents as "at or
/// above 65.535 m, unusable". That is the honest value while tracking is lost,
/// and the receiver-native way to say so.
pub const ACCURACY_UNUSABLE_M: f64 = 65.535;

fn wgs84_e2() -> f64 {
    let b = WGS84_A * (1.0 - WGS84_F);
    1.0 - (b / WGS84_A).powi(2)
}

/// Geodetic origin the mocap frame is pinned to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Origin {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
}

/// Everything the conversion needs beyond the sample itself. Defaults match the
/// vehicle tree's scripts so a calibration carries over unchanged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UplinkConfig {
    pub origin: Origin,
    /// Rotates the facility frame onto true north.
    pub yaw_offset_deg: f64,
    pub heading_offset_deg: f64,
    pub hacc_m: f64,
    pub vacc_m: f64,
    /// Multiplies accuracy while tracking is degraded.
    pub degraded_scale: f64,
    pub hdop: f64,
    pub vdop: f64,
    pub satellites: u8,
    pub publish_yaw: bool,
    pub gnss_instance: u8,
}

impl Default for UplinkConfig {
    fn default() -> Self {
        Self {
            origin: Origin {
                lat_deg: 40.415_453_968_973_93,
                lon_deg: -86.932_758_662_594_37,
                alt_m: 0.0,
            },
            yaw_offset_deg: 230.0,
            heading_offset_deg: 138.0,
            hacc_m: 0.05,
            vacc_m: 0.08,
            degraded_scale: 4.0,
            hdop: 0.3,
            vdop: 0.4,
            satellites: 14,
            publish_yaw: false,
            gnss_instance: 0,
        }
    }
}

/// One decoded input sample, reduced to what a GNSS fix can carry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseSample {
    pub timestamp_us: u64,
    pub east: f64,
    pub north: f64,
    pub up: f64,
    /// w, x, y, z.
    pub quat: Option<[f64; 4]>,
    /// ve, vn, vu.
    pub velocity_enu: Option<[f64; 3]>,
    pub lost: bool,
    pub degraded: bool,
}

/// Position and time of the previous send, used only to difference a velocity
/// when the input carries none.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrevSend {
    pub east: f64,
    pub north: f64,
    pub up: f64,
    pub at_s: f64,
}

/// Flat-Earth ENU to WGS-84. Accurate to well under a centimetre over a
/// capture volume.
pub fn enu_to_geodetic(east: f64, north: f64, up: f64, origin: Origin) -> (f64, f64, f64) {
    let lat0 = origin.lat_deg.to_radians();
    let n = WGS84_A / (1.0 - wgs84_e2() * lat0.sin().powi(2)).sqrt();
    let d_lat = (north / (n + origin.alt_m)).to_degrees();
    let d_lon = (east / ((n + origin.alt_m) * lat0.cos())).to_degrees();
    (
        origin.lat_deg + d_lat,
        origin.lon_deg + d_lon,
        origin.alt_m + up,
    )
}

/// Rotate a 2-D vector counter-clockwise.
pub fn rotate_2d(x: f64, y: f64, angle_deg: f64) -> (f64, f64) {
    let a = angle_deg.to_radians();
    (x * a.cos() - y * a.sin(), x.mul_add(a.sin(), y * a.cos()))
}

/// Yaw (heading) in degrees from a quaternion.
pub fn quaternion_to_yaw(x: f64, y: f64, z: f64, w: f64) -> f64 {
    let siny_cosp = 2.0 * z.mul_add(w, x * y);
    let cosy_cosp = 1.0 - 2.0 * y.mul_add(y, z * z);
    siny_cosp.atan2(cosy_cosp).to_degrees().rem_euclid(360.0)
}

/// The schema requires producers to saturate, never truncate.
fn saturate_u16(value: f64) -> u16 {
    if value < 0.0 {
        return 0;
    }
    let rounded = value.round();
    if rounded >= f64::from(u16::MAX) {
        u16::MAX
    } else {
        rounded as u16
    }
}

fn clamp_i16(value: f64) -> i16 {
    value
        .round()
        .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16
}

/// Build the 64-byte `GnssFixData` image.
///
/// `GnssFixData` has no NED velocity vector — it carries ground speed, course
/// over ground and vertical velocity — so the horizontal components are folded
/// into speed and a course of `atan2(east, north)`, which is receiver-native:
/// zero at true north, positive clockwise.
#[allow(clippy::too_many_arguments)]
fn build_gnss_fix(
    east: f64,
    north: f64,
    up: f64,
    vn: f64,
    ve: f64,
    vd: f64,
    config: &UplinkConfig,
    boot_us: u64,
    time_unix_us: u64,
    yaw_deg: Option<f64>,
    fix_type: GnssFixType,
    h_acc_m: f64,
    v_acc_m: f64,
    have_velocity: bool,
) -> GnssFixData {
    let (lat, lon, alt) = enu_to_geodetic(east, north, up, config.origin);
    let speed = vn.hypot(ve);
    let course_deg = ve.atan2(vn).to_degrees().rem_euclid(360.0);

    let mut flags = GnssFixFlags::TimeValid;
    if have_velocity {
        flags |= GnssFixFlags::VelocityUpValid;
        if speed >= COURSE_VALID_MIN_SPEED_MS {
            flags |= GnssFixFlags::CourseValid;
        }
    }
    if yaw_deg.is_some() {
        flags |= GnssFixFlags::YawValid;
    }

    GnssFixData::new(
        boot_us,
        time_unix_us,
        (lat * 1e7).round() as i32,
        (lon * 1e7).round() as i32,
        (alt * 1000.0).round() as i32,
        (alt * 1000.0).round() as i32,
        saturate_u16(h_acc_m * 1000.0),
        saturate_u16(v_acc_m * 1000.0),
        saturate_u16((speed * 0.05).max(0.1) * 1000.0),
        yaw_deg.map_or(0, |_| saturate_u16(500.0)),
        saturate_u16(config.hdop * 100.0),
        saturate_u16(config.vdop * 100.0),
        saturate_u16(speed * 100.0),
        // Modulo 36000 so a course of 359.999 deg reports 35999 rather than
        // 36000, which is outside the range the vehicle's producer can emit.
        ((course_deg * 100.0) as u32 % 36_000) as u16,
        yaw_deg.map_or(0, |yaw| saturate_u16(yaw.rem_euclid(360.0) * 100.0)),
        clamp_i16(-vd * 100.0),
        flags.bits(),
        fix_type,
        config.satellites,
        config.satellites,
        config.gnss_instance,
    )
}

/// Result of converting one sample.
pub struct FixOutcome {
    pub payload: Vec<u8>,
    pub prev: PrevSend,
    pub fix_type: GnssFixType,
}

/// Convert one sample into a `GnssFixData` payload.
///
/// `accuracy` is `(horizontal_m, vertical_m)` taken from the covariance stream
/// when one is live; without it the configured fallbacks are used.
pub fn sample_to_fix(
    sample: &PoseSample,
    prev: Option<PrevSend>,
    config: &UplinkConfig,
    boot_us: u64,
    time_unix_us: u64,
    now_s: f64,
    accuracy: Option<(f64, f64)>,
) -> FixOutcome {
    let (mut east, mut north) = (sample.east, sample.north);
    let up = sample.up;
    if config.yaw_offset_deg != 0.0 {
        (east, north) = rotate_2d(east, north, config.yaw_offset_deg);
    }

    let mut have_velocity = false;
    let (mut ve, mut vn, mut vd) = (0.0, 0.0, 0.0);
    if let Some([sve, svn, svu]) = sample.velocity_enu {
        // Velocity lives in the same facility frame as position, so it takes
        // the same rotation.
        let (rve, rvn) = if config.yaw_offset_deg != 0.0 {
            rotate_2d(sve, svn, config.yaw_offset_deg)
        } else {
            (sve, svn)
        };
        ve = rve;
        vn = rvn;
        vd = -svu;
        have_velocity = true;
    } else if let Some(previous) = prev {
        let dt = now_s - previous.at_s;
        if dt > 0.001 {
            ve = (east - previous.east) / dt;
            vn = (north - previous.north) / dt;
            vd = -(up - previous.up) / dt;
            have_velocity = true;
        }
    }

    let yaw_deg = sample.quat.map(|[w, x, y, z]| {
        (quaternion_to_yaw(x, y, z, w) + config.heading_offset_deg).rem_euclid(360.0)
    });

    let (fix_type, h_acc, v_acc) = if sample.lost {
        (GnssFixType::NoFix, ACCURACY_UNUSABLE_M, ACCURACY_UNUSABLE_M)
    } else {
        let (mut h, mut v) = accuracy.unwrap_or((config.hacc_m, config.vacc_m));
        if sample.degraded {
            h *= config.degraded_scale;
            v *= config.degraded_scale;
        }
        (GnssFixType::Fix3d, h, v)
    };

    let fix = build_gnss_fix(
        east,
        north,
        up,
        vn,
        ve,
        vd,
        config,
        boot_us,
        time_unix_us,
        config.publish_yaw.then_some(yaw_deg).flatten(),
        fix_type,
        h_acc,
        v_acc,
        have_velocity,
    );

    FixOutcome {
        payload: fix.0.to_vec(),
        prev: PrevSend {
            east,
            north,
            up,
            at_s: now_s,
        },
        fix_type,
    }
}

/// A frozen position sent at full confidence is worse than no fix: the
/// estimator has no way to tell it is stale. Say so instead.
pub fn mark_stale(sample: &PoseSample) -> PoseSample {
    PoseSample {
        lost: true,
        degraded: false,
        quat: None,
        velocity_enu: None,
        ..*sample
    }
}

fn flags_lost(flags: ExternalOdometryFlags) -> bool {
    flags.contains(ExternalOdometryFlags::Lost)
}

fn flags_degraded(flags: ExternalOdometryFlags) -> bool {
    flags.intersects(
        ExternalOdometryFlags::Extrapolated
            | ExternalOdometryFlags::OutlierRejected
            | ExternalOdometryFlags::Degraded,
    )
}

fn status_lost(status: ExternalOdometryStatus) -> bool {
    status == ExternalOdometryStatus::Lost
}

fn status_degraded(status: ExternalOdometryStatus) -> bool {
    matches!(
        status,
        ExternalOdometryStatus::ExtrapolatedShort
            | ExternalOdometryStatus::ExtrapolatedLong
            | ExternalOdometryStatus::OutlierRejected
            | ExternalOdometryStatus::Degraded
    )
}

/// Wire size of a bare `synapse.topic.ExternalOdometryData` struct.
pub const EXTERNAL_ODOMETRY_SIZE: usize = 64;
/// Wire size of a bare `synapse.topic.ExternalOdometryCovarianceData` struct.
pub const EXTERNAL_ODOMETRY_COVARIANCE_SIZE: usize = 328;

/// Decode an `external_pose` (`ExternalOdometryData`) payload.
pub fn decode_external_odometry(payload: &[u8], instance: Option<u8>) -> Option<PoseSample> {
    if payload.len() != EXTERNAL_ODOMETRY_SIZE {
        return None;
    }
    // SAFETY: exact-size check above covers the fixed-layout struct.
    let data = unsafe { <ExternalOdometryData as flatbuffers::Follow>::follow(payload, 0) };
    if instance.is_some_and(|want| data.id() != want) {
        return None;
    }

    let flags = data.flags();
    if !flags.contains(ExternalOdometryFlags::PositionValid) {
        return None;
    }

    let position = data.position_enu_m();
    let status = data.status();
    let mut sample = PoseSample {
        timestamp_us: data.timestamp_us(),
        east: f64::from(position.x()),
        north: f64::from(position.y()),
        up: f64::from(position.z()),
        quat: None,
        velocity_enu: None,
        lost: flags_lost(flags) || status_lost(status),
        degraded: flags_degraded(flags) || status_degraded(status),
    };
    if flags.contains(ExternalOdometryFlags::AttitudeValid) {
        let q = data.attitude();
        sample.quat = Some([
            f64::from(q.w()),
            f64::from(q.x()),
            f64::from(q.y()),
            f64::from(q.z()),
        ]);
    }
    if flags.contains(ExternalOdometryFlags::LinearVelocityValid) {
        let v = data.linear_velocity_enu_m_s();
        sample.velocity_enu = Some([f64::from(v.x()), f64::from(v.y()), f64::from(v.z())]);
    }
    Some(sample)
}

/// Decode `external_pose_cov` into `(horizontal_m, vertical_m)` 1-sigma
/// accuracy, taken from the position block of the covariance.
///
/// The state vector is 12 long, so the upper triangle has 78 entries and the
/// east/north/up variances are its 6th, 7th and 8th diagonal entries — `c57`,
/// `c63` and `c68`.
pub fn decode_covariance(payload: &[u8], instance: Option<u8>) -> Option<(f64, f64)> {
    if payload.len() != EXTERNAL_ODOMETRY_COVARIANCE_SIZE {
        return None;
    }
    // SAFETY: exact-size check above covers the fixed-layout struct.
    let data =
        unsafe { <ExternalOdometryCovarianceData as flatbuffers::Follow>::follow(payload, 0) };
    if instance.is_some_and(|want| data.id() != want) {
        return None;
    }

    let covariance = data.covariance();
    let (var_e, var_n, var_u) = (covariance.c57(), covariance.c63(), covariance.c68());
    if var_e < 0.0 || var_n < 0.0 || var_u < 0.0 {
        return None;
    }
    Some((f64::from(var_e + var_n).sqrt(), f64::from(var_u).sqrt()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Values produced by the vehicle tree's `publish_gps_synapse.build_gnss_fix`
    /// for the same inputs; the conversion must not drift from the calibration
    /// those scripts were trimmed against.
    #[test]
    fn enu_origin_maps_to_the_origin_itself() {
        let origin = UplinkConfig::default().origin;
        let (lat, lon, alt) = enu_to_geodetic(0.0, 0.0, 0.0, origin);

        assert!((lat - origin.lat_deg).abs() < 1e-12);
        assert!((lon - origin.lon_deg).abs() < 1e-12);
        assert!((alt - origin.alt_m).abs() < 1e-12);
    }

    /// Golden values computed by the vehicle tree's own `enu_to_geodetic`, at
    /// the wire's 1e-7 degree resolution. A drift here is a calibration shift:
    /// the vehicle's estimator would be told it is somewhere it is not.
    #[test]
    fn enu_conversion_matches_the_reference_implementation() {
        let origin = UplinkConfig::default().origin;
        for (east, north, up, want_lat_e7, want_lon_e7, want_alt_mm) in [
            (0.0, 10.0, 0.0, 404_155_437_i64, -869_327_587_i64, 0_i64),
            (10.0, 0.0, 0.0, 404_154_540, -869_326_408, 0),
            (3.5, -2.25, 1.75, 404_154_338, -869_327_174, 1_750),
        ] {
            let (lat, lon, alt) = enu_to_geodetic(east, north, up, origin);

            assert_eq!((lat * 1e7).round() as i64, want_lat_e7);
            assert_eq!((lon * 1e7).round() as i64, want_lon_e7);
            assert_eq!((alt * 1000.0).round() as i64, want_alt_mm);
        }
    }

    /// The whole 64-byte payload, byte for byte against the vehicle tree's
    /// `build_gnss_fix` for the same inputs. Every field is covered at once —
    /// the geodetic conversion, the speed/course folding, the accuracy
    /// saturation, the flags and the packing.
    #[test]
    fn payload_matches_the_reference_implementation_byte_for_byte() {
        const REFERENCE: &str = "\
87d612000000000000f1536500000000e2e71618ba1e2fccd6060000d606000032005000\
640000001e0028008200521a00001e000b030e0e0000000000000000";
        let expected: Vec<u8> = (0..REFERENCE.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&REFERENCE[i..i + 2], 16).expect("hex"))
            .collect();

        let config = UplinkConfig {
            // The reference call takes already-rotated coordinates.
            yaw_offset_deg: 0.0,
            ..UplinkConfig::default()
        };
        let sample = PoseSample {
            timestamp_us: 0,
            east: 3.5,
            north: -2.25,
            up: 1.75,
            quat: None,
            // vn=0.5, ve=1.2, vd=-0.3 — the schema carries up, so vu = -vd.
            velocity_enu: Some([1.2, 0.5, 0.3]),
            lost: false,
            degraded: false,
        };

        let outcome = sample_to_fix(&sample, None, &config, 1_234_567, 1_700_000_000, 0.0, None);

        assert_eq!(outcome.payload, expected);
    }

    #[test]
    fn course_is_zero_at_true_north_and_positive_clockwise() {
        let config = UplinkConfig {
            yaw_offset_deg: 0.0,
            ..UplinkConfig::default()
        };
        let sample = PoseSample {
            timestamp_us: 0,
            east: 0.0,
            north: 0.0,
            up: 0.0,
            quat: None,
            // Moving due east at 2 m/s.
            velocity_enu: Some([2.0, 0.0, 0.0]),
            lost: false,
            degraded: false,
        };

        let outcome = sample_to_fix(&sample, None, &config, 0, 0, 0.0, None);
        let fix = unsafe { <GnssFixData as flatbuffers::Follow>::follow(&outcome.payload, 0) };

        assert_eq!(fix.course_over_ground_cdeg(), 9_000); // 90 deg = east
        assert_eq!(fix.ground_speed_cm_s(), 200);
        let flags = GnssFixFlags::from_bits_retain(fix.flags());
        assert!(flags.contains(GnssFixFlags::CourseValid));
        assert!(flags.contains(GnssFixFlags::VelocityUpValid));
    }

    #[test]
    fn slow_motion_publishes_without_course_valid() {
        let config = UplinkConfig {
            yaw_offset_deg: 0.0,
            ..UplinkConfig::default()
        };
        let sample = PoseSample {
            timestamp_us: 0,
            east: 0.0,
            north: 0.0,
            up: 0.0,
            quat: None,
            velocity_enu: Some([0.05, 0.0, 0.0]),
            lost: false,
            degraded: false,
        };

        let outcome = sample_to_fix(&sample, None, &config, 0, 0, 0.0, None);
        let fix = unsafe { <GnssFixData as flatbuffers::Follow>::follow(&outcome.payload, 0) };
        let flags = GnssFixFlags::from_bits_retain(fix.flags());

        assert!(!flags.contains(GnssFixFlags::CourseValid));
        assert!(flags.contains(GnssFixFlags::VelocityUpValid));
    }

    /// Lost tracking must go out as NoFix with saturated accuracy, never as a
    /// frozen position at full confidence.
    #[test]
    fn lost_tracking_sends_nofix_with_unusable_accuracy() {
        let config = UplinkConfig::default();
        let sample = PoseSample {
            timestamp_us: 0,
            east: 1.0,
            north: 2.0,
            up: 3.0,
            quat: None,
            velocity_enu: None,
            lost: true,
            degraded: false,
        };

        let outcome = sample_to_fix(&sample, None, &config, 0, 0, 0.0, None);
        let fix = unsafe { <GnssFixData as flatbuffers::Follow>::follow(&outcome.payload, 0) };

        assert_eq!(fix.fix_type(), GnssFixType::NoFix);
        assert_eq!(fix.horizontal_accuracy_mm(), u16::MAX);
        assert_eq!(fix.vertical_accuracy_mm(), u16::MAX);
    }

    #[test]
    fn degraded_tracking_scales_the_reported_accuracy() {
        let config = UplinkConfig::default();
        let sample = PoseSample {
            timestamp_us: 0,
            east: 0.0,
            north: 0.0,
            up: 0.0,
            quat: None,
            velocity_enu: None,
            lost: false,
            degraded: true,
        };

        let outcome = sample_to_fix(&sample, None, &config, 0, 0, 0.0, None);
        let fix = unsafe { <GnssFixData as flatbuffers::Follow>::follow(&outcome.payload, 0) };

        // 0.05 m * 4.0 = 0.2 m
        assert_eq!(fix.horizontal_accuracy_mm(), 200);
        assert_eq!(fix.vertical_accuracy_mm(), 320);
        assert_eq!(fix.fix_type(), GnssFixType::Fix3d);
    }

    #[test]
    fn a_stale_sample_becomes_a_lost_one() {
        let sample = PoseSample {
            timestamp_us: 7,
            east: 1.0,
            north: 2.0,
            up: 3.0,
            quat: Some([1.0, 0.0, 0.0, 0.0]),
            velocity_enu: Some([1.0, 1.0, 1.0]),
            lost: false,
            degraded: true,
        };

        let stale = mark_stale(&sample);

        assert!(stale.lost);
        assert_eq!(stale.east, 1.0);
        // Velocity and attitude from a stale sample are not republished.
        assert!(stale.velocity_enu.is_none());
        assert!(stale.quat.is_none());
    }

    #[test]
    fn yaw_offset_rotates_position_and_velocity_together() {
        let config = UplinkConfig {
            yaw_offset_deg: 90.0,
            ..UplinkConfig::default()
        };
        let sample = PoseSample {
            timestamp_us: 0,
            east: 1.0,
            north: 0.0,
            up: 0.0,
            quat: None,
            velocity_enu: Some([1.0, 0.0, 0.0]),
            lost: false,
            degraded: false,
        };

        let outcome = sample_to_fix(&sample, None, &config, 0, 0, 0.0, None);

        // (1, 0) rotated 90 deg CCW is (0, 1): due east becomes due north.
        assert!((outcome.prev.east - 0.0).abs() < 1e-9);
        assert!((outcome.prev.north - 1.0).abs() < 1e-9);

        let fix = unsafe { <GnssFixData as flatbuffers::Follow>::follow(&outcome.payload, 0) };
        assert_eq!(fix.course_over_ground_cdeg(), 0); // north
    }

    #[test]
    fn velocity_is_differenced_when_the_input_carries_none() {
        let config = UplinkConfig {
            yaw_offset_deg: 0.0,
            ..UplinkConfig::default()
        };
        let sample = PoseSample {
            timestamp_us: 0,
            east: 0.0,
            north: 2.0,
            up: 0.0,
            quat: None,
            velocity_enu: None,
            lost: false,
            degraded: false,
        };
        let prev = PrevSend {
            east: 0.0,
            north: 0.0,
            up: 0.0,
            at_s: 0.0,
        };

        let outcome = sample_to_fix(&sample, Some(prev), &config, 0, 0, 1.0, None);
        let fix = unsafe { <GnssFixData as flatbuffers::Follow>::follow(&outcome.payload, 0) };

        // 2 m north in 1 s.
        assert_eq!(fix.ground_speed_cm_s(), 200);
        assert_eq!(fix.course_over_ground_cdeg(), 0);
    }

    #[test]
    fn covariance_supplies_accuracy_when_present() {
        let config = UplinkConfig::default();
        let sample = PoseSample {
            timestamp_us: 0,
            east: 0.0,
            north: 0.0,
            up: 0.0,
            quat: None,
            velocity_enu: None,
            lost: false,
            degraded: false,
        };

        let outcome = sample_to_fix(&sample, None, &config, 0, 0, 0.0, Some((1.25, 2.5)));
        let fix = unsafe { <GnssFixData as flatbuffers::Follow>::follow(&outcome.payload, 0) };

        assert_eq!(fix.horizontal_accuracy_mm(), 1_250);
        assert_eq!(fix.vertical_accuracy_mm(), 2_500);
    }

    #[test]
    fn quaternion_yaw_matches_the_reference_convention() {
        // 90 deg about Z.
        let yaw = quaternion_to_yaw(
            0.0,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        );
        assert!((yaw - 90.0).abs() < 1e-9);
    }

    #[test]
    fn accuracy_saturates_rather_than_wrapping() {
        assert_eq!(saturate_u16(ACCURACY_UNUSABLE_M * 1000.0), u16::MAX);
        assert_eq!(saturate_u16(1e9), u16::MAX);
        assert_eq!(saturate_u16(-5.0), 0);
    }
}
