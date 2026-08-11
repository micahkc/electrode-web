//! Telemetry-radio link settings.
//!
//! The `electrode-telemetry-bridge` child owns the radio's serial port in both
//! directions: it decodes what the vehicle streams and, when the mocap GNSS
//! uplink is on, sends fixes back up the same port. A serial port cannot be
//! shared, so this is one process and the uplink is a flag on it rather than a
//! second child.

use serde::{Deserialize, Serialize};

/// How the bridge reaches the vehicle: the telemetry radio's serial port, or
/// UDP to a WiFi bridge (the micro-quad's ESP32) carrying the same framing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub(crate) enum LinkMode {
    #[default]
    Serial,
    Udp,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct TelemetryProfile {
    #[serde(default)]
    pub link_mode: LinkMode,
    /// UDP address of the vehicle's WiFi bridge (link_mode = udp).
    #[serde(default = "default_udp_address")]
    pub udp_address: String,
    /// Frame the Zenoh `manual` topic up the link as RC (link_mode = udp).
    #[serde(default)]
    pub manual_uplink: bool,
    pub serial_device: String,
    pub baud_rate: u32,
    /// Convert mocap pose to GnssFix and send it up the radio.
    pub mocap_gnss: bool,
    /// Namespace the mocap bridge publishes under; `**` matches any.
    pub mocap_namespace: String,
    /// Deployment namespace prefixed to republished telemetry keys.
    pub namespace: String,
    /// Geodetic origin the mocap frame is pinned to. Published rather than left
    /// to the bridge's defaults so the UI can invert the uplink's transform and
    /// draw telemetry and mocap in one frame.
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub origin_alt: f64,
    /// Degrees the uplink rotates the facility frame onto true north.
    pub yaw_offset_deg: f64,
}

/// The ESP32 bridge's access-point address and port.
fn default_udp_address() -> String {
    std::env::var("ELECTRODE_TELEMETRY_UDP_ADDRESS")
        .unwrap_or_else(|_| "192.168.4.1:14550".to_string())
}

impl Default for TelemetryProfile {
    fn default() -> Self {
        Self {
            link_mode: LinkMode::Serial,
            udp_address: default_udp_address(),
            manual_uplink: false,
            serial_device: std::env::var("ELECTRODE_TELEMETRY_SERIAL_DEVICE")
                .unwrap_or_else(|_| "/dev/ttyUSB0".to_string()),
            baud_rate: 57_600,
            mocap_gnss: false,
            mocap_namespace: "**".to_string(),
            namespace: String::new(),
            origin_lat: 40.415_453_968_973_93,
            origin_lon: -86.932_758_662_594_37,
            origin_alt: 0.0,
            yaw_offset_deg: 230.0,
        }
    }
}

impl TelemetryProfile {
    /// Arguments for the bridge child. The uplink's geodetic calibration is
    /// deliberately left to the bridge's own defaults, which are the values the
    /// vehicle tree's scripts were trimmed against.
    pub(crate) fn bridge_args(&self) -> Vec<String> {
        let mut args = match self.link_mode {
            LinkMode::Serial => vec![
                "--serial-device".to_string(),
                self.serial_device.clone(),
                "--baud-rate".to_string(),
                self.baud_rate.to_string(),
            ],
            LinkMode::Udp => {
                let mut args = vec!["--udp-address".to_string(), self.udp_address.clone()];
                if self.manual_uplink {
                    args.push("--manual-uplink".to_string());
                }
                args
            }
        };
        if !self.namespace.trim_matches('/').is_empty() {
            args.push("--namespace".to_string());
            args.push(self.namespace.clone());
        }
        // Always passed, so the bridge and the UI cannot drift onto different
        // calibrations without it being visible.
        args.extend([
            "--origin-lat".to_string(),
            self.origin_lat.to_string(),
            "--origin-lon".to_string(),
            self.origin_lon.to_string(),
            "--origin-alt".to_string(),
            self.origin_alt.to_string(),
            "--yaw-offset".to_string(),
            self.yaw_offset_deg.to_string(),
        ]);
        if self.mocap_gnss {
            args.push("--mocap-gnss".to_string());
            args.push("--mocap-namespace".to_string());
            args.push(self.mocap_namespace.clone());
        }
        args
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_profile_only_receives() {
        let args = TelemetryProfile::default().bridge_args();

        assert!(args.contains(&"--serial-device".to_string()));
        assert!(!args.contains(&"--mocap-gnss".to_string()));
    }

    #[test]
    fn enabling_mocap_gnss_adds_the_uplink_flags() {
        let profile = TelemetryProfile {
            mocap_gnss: true,
            mocap_namespace: "cub1".to_string(),
            ..TelemetryProfile::default()
        };

        let args = profile.bridge_args();

        assert!(args.contains(&"--mocap-gnss".to_string()));
        let index = args.iter().position(|arg| arg == "--mocap-namespace");
        assert_eq!(
            args.get(index.expect("flag") + 1),
            Some(&"cub1".to_string())
        );
    }

    /// An empty deployment namespace must not become a literal empty argument;
    /// the bridge would then publish onto a leading-slash key.
    /// The UI inverts this exact calibration to put telemetry and mocap in one
    /// frame, so it must reach the bridge rather than relying on its defaults.
    #[test]
    fn calibration_is_always_passed_to_the_bridge() {
        let args = TelemetryProfile::default().bridge_args();

        for flag in [
            "--origin-lat",
            "--origin-lon",
            "--origin-alt",
            "--yaw-offset",
        ] {
            assert!(args.contains(&flag.to_string()), "{flag} missing");
        }
    }

    #[test]
    fn empty_namespace_is_omitted_rather_than_passed_blank() {
        let args = TelemetryProfile::default().bridge_args();

        assert!(!args.contains(&"--namespace".to_string()));
    }

    #[test]
    fn udp_mode_swaps_the_transport_flags() {
        let profile = TelemetryProfile {
            link_mode: LinkMode::Udp,
            udp_address: "192.168.4.1:14550".to_string(),
            manual_uplink: true,
            ..TelemetryProfile::default()
        };

        let args = profile.bridge_args();

        let index = args.iter().position(|arg| arg == "--udp-address");
        assert_eq!(
            args.get(index.expect("flag") + 1),
            Some(&"192.168.4.1:14550".to_string())
        );
        assert!(args.contains(&"--manual-uplink".to_string()));
        assert!(!args.contains(&"--serial-device".to_string()));
        assert!(!args.contains(&"--baud-rate".to_string()));
    }

    /// Old persisted profiles predate the link fields; they must deserialize
    /// as serial rather than fail.
    #[test]
    fn profiles_without_link_fields_default_to_serial() {
        let profile: TelemetryProfile = serde_json::from_str(
            r#"{"serialDevice":"/dev/ttyUSB1","baudRate":57600,"mocapGnss":false,
                "mocapNamespace":"**","namespace":"","originLat":0.0,"originLon":0.0,
                "originAlt":0.0,"yawOffsetDeg":0.0}"#,
        )
        .expect("deserialize");

        assert_eq!(profile.link_mode, LinkMode::Serial);
        assert!(!profile.manual_uplink);
    }
}
