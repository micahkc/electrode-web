//! The vehicle link as a byte stream.
//!
//! The synapse serial framing is self-synchronizing, so the decoder does not
//! care what carries the bytes. Serial is the telemetry radio; UDP is the
//! micro-quad's ESP32 WiFi bridge, which shovels the identical framing over
//! datagrams. A read timeout (or an empty window) is the normal idle state on
//! both and paces the caller's loop.

use std::io::ErrorKind;
use std::net::UdpSocket;
use std::time::Duration;

/// What `run_loop` needs from a link: nothing a serial port and a connected
/// UDP socket do not both provide.
pub trait Link {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize>;
    fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()>;
}

impl<T: serialport::SerialPort + ?Sized> Link for T {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        std::io::Read::read(self, buf)
    }

    fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
        std::io::Write::write_all(self, buf)
    }
}

/// `true` for the error kinds that mean "nothing arrived", which the caller
/// treats as pacing rather than failure. `ConnectionRefused` is included
/// because a connected UDP socket surfaces ICMP unreachable that way while
/// the WiFi bridge is still booting.
pub fn is_idle(kind: ErrorKind) -> bool {
    matches!(
        kind,
        ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::ConnectionRefused
    )
}

/// Connected UDP socket to the vehicle's WiFi bridge.
pub struct UdpLink {
    socket: UdpSocket,
}

impl UdpLink {
    pub fn connect(address: &str, read_timeout: Duration) -> std::io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.connect(address)?;
        socket.set_read_timeout(Some(read_timeout))?;
        Ok(Self { socket })
    }

    /// A second handle for a sender on another thread. UDP sockets, unlike
    /// serial ports, are safely shared, which is what lets the manual-control
    /// uplink transmit from the Zenoh callback with no queue in between.
    pub fn sender(&self) -> std::io::Result<UdpSocket> {
        self.socket.try_clone()
    }
}

impl Link for UdpLink {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.socket.recv(buf)
    }

    fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
        self.socket.send(buf).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn udp_link_round_trips_and_times_out() {
        let peer = UdpSocket::bind("127.0.0.1:0").expect("peer");
        let peer_addr = peer.local_addr().expect("peer addr");
        let mut link =
            UdpLink::connect(&peer_addr.to_string(), Duration::from_millis(20)).expect("link");

        link.write_all(b"hello").expect("send");
        let mut buf = [0_u8; 16];
        let (received, from) = peer.recv_from(&mut buf).expect("recv");
        assert_eq!(&buf[..received], b"hello");

        peer.send_to(b"world", from).expect("reply");
        let received = link.read(&mut buf).expect("read");
        assert_eq!(&buf[..received], b"world");

        let idle = link.read(&mut buf).expect_err("timeout");
        assert!(is_idle(idle.kind()), "unexpected kind {:?}", idle.kind());
    }
}
