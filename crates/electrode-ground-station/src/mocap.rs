//! Which machine on the LAN publishes motion capture.
//!
//! Mocap does not arrive over the telemetry radio: a capture system (Qualisys)
//! publishes it on its own Zenoh router, and the command authority subscribes
//! to that router across the network. The address of that machine is the one
//! piece of the mocap path an operator has to set, and it changes with the
//! facility, so it is a stored profile rather than a launch-time environment
//! variable.

use std::path::Path;

use serde::{Deserialize, Serialize};

/// Port the capture machine's Zenoh router listens on unless told otherwise.
const DEFAULT_PORT: &str = "7447";
/// Transport assumed for a bare address. Mocap crosses a real network, where
/// TCP is what the capture bridges listen on.
const DEFAULT_PROTOCOL: &str = "tcp";

#[derive(Clone, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(default, rename_all = "camelCase")]
pub(crate) struct MocapProfile {
    /// As the operator typed it: an address (`192.168.10.2`), an address and
    /// port, or a full Zenoh locator. Empty means no capture machine.
    pub address: String,
}

impl MocapProfile {
    pub(crate) fn load_or_default(path: &Path) -> Self {
        std::fs::read_to_string(path)
            .ok()
            .and_then(|text| serde_json::from_str::<Self>(&text).ok())
            .unwrap_or_default()
    }

    pub(crate) fn save(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let text = serde_json::to_string_pretty(self).unwrap_or_default();
        std::fs::write(path, text)
    }

    /// The Zenoh locator to subscribe on, or `None` when unset.
    pub(crate) fn endpoint(&self) -> Option<String> {
        locator_for(&self.address)
    }
}

/// Turn what the operator typed into a Zenoh locator.
///
/// Asking for an IP address should mean typing an IP address, but a locator
/// pasted from a working config has to survive untouched — so anything that
/// already names a protocol is left exactly as it is.
pub(crate) fn locator_for(address: &str) -> Option<String> {
    let address = address.trim();
    if address.is_empty() {
        return None;
    }
    // A protocol is already named (`tcp/...`, `udp/...`, `ws/...`).
    if address.contains('/') {
        return Some(address.to_string());
    }
    let (host, port) = split_host_port(address);
    let host = if host.contains(':') && !host.starts_with('[') {
        // Bare IPv6 literal: Zenoh wants it bracketed before the port.
        format!("[{host}]")
    } else {
        host.to_string()
    };
    Some(format!("{DEFAULT_PROTOCOL}/{host}:{port}"))
}

/// Split a trailing `:port`, leaving IPv6 literals (which are all colons)
/// intact. Only an all-digit tail counts as a port.
fn split_host_port(address: &str) -> (&str, &str) {
    match address.rsplit_once(':') {
        Some((host, port))
            if !host.is_empty()
                && !port.is_empty()
                && port.chars().all(|c| c.is_ascii_digit())
                && !host.ends_with(':') =>
        {
            (host, port)
        }
        _ => (address, DEFAULT_PORT),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bare_address_gets_the_default_transport_and_port() {
        assert_eq!(
            locator_for("192.168.10.2").as_deref(),
            Some("tcp/192.168.10.2:7447")
        );
    }

    #[test]
    fn an_explicit_port_is_kept() {
        assert_eq!(
            locator_for("192.168.10.2:7448").as_deref(),
            Some("tcp/192.168.10.2:7448")
        );
    }

    #[test]
    fn a_hostname_works_the_same_way() {
        assert_eq!(locator_for(" qualisys.local ").as_deref(), Some("tcp/qualisys.local:7447"));
    }

    /// A locator pasted from a working configuration must reach Zenoh byte for
    /// byte; guessing a transport for it would silently break UDP or WebSocket
    /// capture links.
    #[test]
    fn a_full_locator_passes_through_untouched() {
        assert_eq!(
            locator_for("udp/192.168.10.2:7447").as_deref(),
            Some("udp/192.168.10.2:7447")
        );
    }

    #[test]
    fn an_ipv6_literal_is_bracketed_before_the_port_is_appended() {
        assert_eq!(locator_for("fd00::2").as_deref(), Some("tcp/[fd00::2]:7447"));
        assert_eq!(
            locator_for("[fd00::2]:7448").as_deref(),
            Some("tcp/[fd00::2]:7448")
        );
    }

    #[test]
    fn a_blank_address_means_no_capture_machine() {
        assert_eq!(locator_for("   "), None);
        assert_eq!(MocapProfile::default().endpoint(), None);
    }
}
