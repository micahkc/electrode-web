//! Local hardware discovery: joystick and serial device enumeration.

use serde::Serialize;

#[derive(Serialize)]
pub(crate) struct Device {
    pub kind: &'static str,
    pub path: String,
    pub name: String,
}

#[derive(Serialize)]
pub(crate) struct Devices {
    pub joysticks: Vec<Device>,
    pub serial: Vec<Device>,
}

pub(crate) fn list() -> Devices {
    Devices {
        joysticks: list_joysticks(),
        serial: list_serial(),
    }
}

/// Enumerate Linux joystick nodes (`/dev/input/js*`), naming each from sysfs.
fn list_joysticks() -> Vec<Device> {
    let mut devices = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/dev/input") {
        for entry in entries.flatten() {
            let file_name = entry.file_name();
            let node = file_name.to_string_lossy();
            let is_js = node
                .strip_prefix("js")
                .is_some_and(|rest| !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit()));
            if is_js {
                devices.push(Device {
                    kind: "joystick",
                    name: joystick_name(&node).unwrap_or_else(|| "Joystick".to_string()),
                    path: format!("/dev/input/{node}"),
                });
            }
        }
    }
    devices.sort_by(|a, b| a.path.cmp(&b.path));
    devices
}

/// Best-effort name for a joystick node name (e.g. `js0`) from sysfs.
fn joystick_name(node: &str) -> Option<String> {
    let path = format!("/sys/class/input/{node}/device/name");
    std::fs::read_to_string(path)
        .ok()
        .map(|name| name.trim().to_string())
        .filter(|name| !name.is_empty())
}

/// Best-effort name for a full joystick path (e.g. `/dev/input/js0`).
pub(crate) fn joystick_name_for(path: &str) -> Option<String> {
    let node = path.strip_prefix("/dev/input/")?;
    joystick_name(node)
}

/// Enumerate USB/ACM serial ports, both the kernel nodes (`/dev/ttyACM*`,
/// `/dev/ttyUSB*`) and the stable `/dev/serial/by-id/` aliases.
///
/// The by-id paths are listed first and are the ones to prefer: `ttyUSB`
/// numbering is assignment order, so it swaps between boots whenever more than
/// one adapter is attached, and picking the wrong one of two identical FTDI
/// adapters is otherwise easy to do.
fn list_serial() -> Vec<Device> {
    let mut devices = list_serial_by_id();
    devices.extend(list_serial_nodes());
    devices
}

fn list_serial_by_id() -> Vec<Device> {
    let mut devices = Vec::new();
    let Ok(entries) = std::fs::read_dir("/dev/serial/by-id") else {
        return devices;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        // Name the alias by the node it resolves to, so the two listings can be
        // matched up by eye.
        let node = std::fs::canonicalize(&path)
            .ok()
            .and_then(|target| {
                target
                    .file_name()
                    .map(|name| name.to_string_lossy().to_string())
            })
            .unwrap_or_default();
        let alias = entry.file_name().to_string_lossy().to_string();
        devices.push(Device {
            kind: "serial",
            path: path.to_string_lossy().to_string(),
            name: if node.is_empty() {
                alias
            } else {
                format!("{alias} → {node}")
            },
        });
    }
    devices.sort_by(|a, b| a.path.cmp(&b.path));
    devices
}

fn list_serial_nodes() -> Vec<Device> {
    let mut devices = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/dev") {
        for entry in entries.flatten() {
            let file_name = entry.file_name();
            let node = file_name.to_string_lossy();
            if node.starts_with("ttyACM") || node.starts_with("ttyUSB") {
                devices.push(Device {
                    kind: "serial",
                    path: format!("/dev/{node}"),
                    name: node.to_string(),
                });
            }
        }
    }
    devices.sort_by(|a, b| a.path.cmp(&b.path));
    devices
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The stable aliases must be offered, and offered first: picking the wrong
    /// one of two identical FTDI adapters by `ttyUSB` number is otherwise easy.
    #[test]
    fn by_id_aliases_are_listed_before_kernel_nodes() {
        if std::fs::metadata("/dev/serial/by-id").is_err() {
            // No USB serial adapters attached on this host.
            return;
        }
        let devices = list_serial();
        let by_id = devices
            .iter()
            .position(|device| device.path.starts_with("/dev/serial/by-id/"));
        let node = devices
            .iter()
            .position(|device| device.path.starts_with("/dev/tty"));

        assert!(by_id.is_some(), "no by-id aliases listed");
        if let (Some(by_id), Some(node)) = (by_id, node) {
            assert!(by_id < node, "by-id aliases must sort before kernel nodes");
        }
    }

    /// The alias names the node it resolves to, so the two listings can be
    /// matched up by eye.
    #[test]
    fn by_id_entries_name_their_resolved_node() {
        for device in list_serial_by_id() {
            assert!(
                device.name.contains(" → tty"),
                "alias {} did not resolve to a tty node",
                device.name
            );
        }
    }
}
