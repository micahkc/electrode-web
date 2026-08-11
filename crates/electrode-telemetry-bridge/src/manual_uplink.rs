//! ManualControl uplink: RC over the telemetry link.
//!
//! The micro-quad has no RC receiver; the ground station's `manual` topic is
//! the pilot. Each bare `ManualControlData` struct published on Zenoh (by the
//! joystick bridge or the browser's virtual transmitter, both already policy-
//! gated) is framed with the catalog id and transmitted immediately from the
//! subscriber callback on a cloned UDP socket. There is no queue: RC wants
//! the newest sample with the least latency, and a datagram either fits or
//! is dropped whole.
//!
//! UDP-only by design: a serial radio's port cannot be shared with the read
//! loop, and on those vehicles RC is the CRSF/PPM path anyway.

use std::net::UdpSocket;
use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::Arc;

use synapse_fbs::topic_catalog;

use crate::encode_frame;

/// Compact catalog key the ground station publishes RC on, shared with the
/// vehicle's `zros_serial` inbound table.
const MANUAL_KEY: &str = "manual";

pub struct ManualUplink {
    socket: UdpSocket,
    topic_id: u16,
    payload_size: usize,
    seq: AtomicU8,
    pub sent: AtomicU64,
    pub rejected: AtomicU64,
}

impl ManualUplink {
    /// `socket` is a clone of the UDP link's socket (`UdpLink::sender`).
    pub fn new(socket: UdpSocket) -> Option<Arc<Self>> {
        let info = topic_catalog::topic_by_key(MANUAL_KEY)?;
        Some(Arc::new(Self {
            socket,
            topic_id: info.id,
            payload_size: info.payload_size?,
            seq: AtomicU8::new(0),
            sent: AtomicU64::new(0),
            rejected: AtomicU64::new(0),
        }))
    }

    /// Frame and transmit one Zenoh payload. Anything that is not exactly a
    /// bare `ManualControlData` struct is counted and dropped; the vehicle
    /// would reject it by length anyway.
    pub fn forward(&self, payload: &[u8]) {
        if payload.len() != self.payload_size {
            self.rejected.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let seq = self.seq.fetch_add(1, Ordering::Relaxed);
        let frame = encode_frame(self.topic_id, payload, seq);
        match self.socket.send(&frame) {
            Ok(_) => {
                self.sent.fetch_add(1, Ordering::Relaxed);
            }
            Err(_) => {
                self.rejected.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FrameDecoder;

    #[test]
    fn forwards_exact_payloads_and_rejects_others() {
        let socket = UdpSocket::bind("127.0.0.1:0").expect("socket");
        let peer = UdpSocket::bind("127.0.0.1:0").expect("peer");
        socket
            .connect(peer.local_addr().expect("peer addr"))
            .expect("connect");
        let uplink = ManualUplink::new(socket).expect("manual in catalog");

        let payload = vec![0xa5_u8; uplink.payload_size];
        uplink.forward(&payload);
        uplink.forward(&payload[..10]);

        assert_eq!(uplink.sent.load(Ordering::Relaxed), 1);
        assert_eq!(uplink.rejected.load(Ordering::Relaxed), 1);

        let mut buf = [0_u8; 256];
        let received = peer.recv_from(&mut buf).expect("recv").0;
        let mut decoder = FrameDecoder::new();
        let frames = decoder.feed(&buf[..received]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].topic_id, uplink.topic_id);
        assert_eq!(frames[0].payload, payload);
    }
}
