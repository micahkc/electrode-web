//! Ground-side half of the RDD2 telemetry link.
//!
//! The vehicle (`subsys/csyn_serial/`) streams unsolicited Synapse topics over
//! a transparent serial link, normally a SiK radio pair at 57600 8N1. There is
//! no handshake, no heartbeat and no request/response, so "a decoded frame in
//! the last N seconds" is the only liveness signal available.
//!
//! Payloads are bare fixed-layout struct images, not FlatBuffers tables, which
//! is what the compact framing in `fbs/transport.fbs` prescribes for
//! constrained byte streams. That means a decoded payload can be republished on
//! Zenoh byte-for-byte: it is already the exact image
//! `synapse_fbs::topic::*Data` produces, and the same image every other
//! Electrode consumer already decodes.
//!
//! ```text
//! off  size  field
//!  0     2   sync      0x53 0x59  ('S','Y')
//!  2     2   len       u16, payload byte count
//!  4     2   topic_id  u16, synapse catalog TopicId
//!  6     1   seq       u8, wraps at 256, increments per frame sent
//!  7     1   flags     u8, reserved, currently always 0
//!  8     N   payload   the struct image, exactly `len` bytes
//! 8+N    2   crc16     over bytes [2, 8+N) — header after sync, plus payload
//! ```
//!
//! The reference implementation this is checked against is
//! `tools/synapse_serial/synapse_serial.py` in the vehicle tree.

pub mod link;
pub mod manual_uplink;
pub mod mocap_gnss;

use synapse_fbs::topic_catalog::{self, TopicInfo};

/// Frame delimiter, `'S' 'Y'`.
pub const SYNC: [u8; 2] = [0x53, 0x59];
/// Bytes before the payload: sync, len, topic_id, seq, flags.
pub const HEADER_LEN: usize = 8;
/// Trailing CRC-16.
pub const TRAILER_LEN: usize = 2;
/// Total non-payload bytes per frame.
pub const FRAME_OVERHEAD: usize = HEADER_LEN + TRAILER_LEN;
/// Must match `CONFIG_RDD2_CSYN_SERIAL_MAX_PAYLOAD`. Accepting a larger length
/// than the vehicle does is not permissive, it is wrong: the vehicle rejects an
/// over-long length immediately and recovers the frame behind it, so a decoder
/// that waits for the bogus payload swallows that frame instead.
pub const MAX_PAYLOAD: usize = 128;
/// SiK radio default.
pub const DEFAULT_BAUD: u32 = 57600;

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
///
/// This is Zephyr's `crc16_itu_t(0xffff, ...)`, which is what the vehicle
/// calls. Check value for `"123456789"` is `0x29B1`.
pub fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// One CRC-validated frame lifted off the link.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub topic_id: u16,
    pub seq: u8,
    pub payload: Vec<u8>,
}

/// Link counters, deliberately shaped like the vehicle's `csyn_serial status`
/// so the two ends can be compared directly when diagnosing a bad link.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkCounters {
    /// Frames that passed CRC.
    pub frames: u64,
    /// Candidates rejected because the CRC did not match.
    pub crc_errors: u64,
    /// Candidates rejected because `len` was zero or above [`MAX_PAYLOAD`].
    pub bad_len: u64,
    /// Times a rejected candidate forced a rescan from the next byte.
    pub resyncs: u64,
    /// Number of times `seq` skipped ahead, i.e. distinct drop events.
    pub seq_gaps: u64,
    /// Total frames the link lost, summed across every gap.
    pub dropped: u64,
}

/// Incremental frame decoder. Feed it arbitrary byte chunks; frames do not
/// align with read boundaries, so nothing may assume a read yields whole
/// frames.
#[derive(Debug, Default)]
pub struct FrameDecoder {
    buf: Vec<u8>,
    counters: LinkCounters,
    last_seq: Option<u8>,
}

/// Outcome of one pass over the head of the buffer.
enum Step {
    Frame(Frame),
    /// A candidate was rejected; its bytes are back in play, scan again.
    Rescan,
    /// Not enough bytes to decide yet.
    NeedMore,
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn counters(&self) -> LinkCounters {
        self.counters
    }

    /// Consume a chunk and return the frames it completed.
    ///
    /// Eager rather than lazy on purpose: as an iterator the buffer append
    /// would not happen until the caller iterated, so ignoring the result —
    /// or breaking out of the loop early — would silently discard bytes that
    /// can never be recovered.
    pub fn feed(&mut self, chunk: &[u8]) -> Vec<Frame> {
        self.buf.extend_from_slice(chunk);
        let mut frames = Vec::new();
        loop {
            match self.step() {
                Step::Frame(frame) => frames.push(frame),
                Step::Rescan => {}
                Step::NeedMore => return frames,
            }
        }
    }

    fn step(&mut self) -> Step {
        let Some(start) = find_sync(&self.buf) else {
            // Keep the last byte: a sync word may straddle two chunks.
            let discard = self.buf.len().saturating_sub(SYNC.len() - 1);
            self.buf.drain(..discard);
            return Step::NeedMore;
        };
        self.buf.drain(..start);
        if self.buf.len() < HEADER_LEN {
            return Step::NeedMore;
        }

        let len = usize::from(u16::from_le_bytes([self.buf[2], self.buf[3]]));
        if len == 0 || len > MAX_PAYLOAD {
            self.counters.bad_len += 1;
            return self.rescan();
        }

        let total = HEADER_LEN + len + TRAILER_LEN;
        if self.buf.len() < total {
            return Step::NeedMore;
        }

        let want = u16::from_le_bytes([self.buf[HEADER_LEN + len], self.buf[HEADER_LEN + len + 1]]);
        if crc16_ccitt_false(&self.buf[2..HEADER_LEN + len]) != want {
            self.counters.crc_errors += 1;
            return self.rescan();
        }

        let frame = Frame {
            topic_id: u16::from_le_bytes([self.buf[4], self.buf[5]]),
            seq: self.buf[6],
            payload: self.buf[HEADER_LEN..HEADER_LEN + len].to_vec(),
        };
        self.buf.drain(..total);
        self.counters.frames += 1;
        self.track_seq(frame.seq);
        Step::Frame(frame)
    }

    /// Drop only the sync word, so the bytes the rejected candidate covered are
    /// rescanned. A corrupted `len` makes the parser over-read across a frame
    /// boundary, so those bytes may contain the start of a real frame; a
    /// decoder that discarded them would silently lose the frame behind every
    /// corrupted one.
    fn rescan(&mut self) -> Step {
        self.counters.resyncs += 1;
        self.buf.drain(..SYNC.len());
        Step::Rescan
    }

    /// `seq` is a single counter across all topics, so a gap means the link
    /// dropped a frame, not that one topic went quiet.
    fn track_seq(&mut self, seq: u8) {
        if let Some(last) = self.last_seq {
            let gap = seq.wrapping_sub(last).wrapping_sub(1);
            if gap > 0 {
                self.counters.seq_gaps += 1;
                self.counters.dropped += u64::from(gap);
            }
        }
        self.last_seq = Some(seq);
    }
}

fn find_sync(buf: &[u8]) -> Option<usize> {
    buf.windows(SYNC.len()).position(|window| window == SYNC)
}

/// Build a frame. Used by the tests and available for the uplink direction
/// (`ManualControlCommand`, or `GnssFix` on a `mocap-gnss` vehicle build).
pub fn encode_frame(topic_id: u16, payload: &[u8], seq: u8) -> Vec<u8> {
    let mut body = Vec::with_capacity(HEADER_LEN - SYNC.len() + payload.len());
    body.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    body.extend_from_slice(&topic_id.to_le_bytes());
    body.push(seq);
    body.push(0); // flags: reserved
    body.extend_from_slice(payload);

    let mut frame = Vec::with_capacity(SYNC.len() + body.len() + TRAILER_LEN);
    frame.extend_from_slice(&SYNC);
    frame.extend_from_slice(&body);
    frame.extend_from_slice(&crc16_ccitt_false(&body).to_le_bytes());
    frame
}

/// Why a CRC-valid frame could not be turned into a Zenoh publication.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RouteError {
    /// The vehicle's topic list is application-owned and may grow, so an
    /// unknown id is skipped gracefully rather than treated as a fault.
    UnknownTopic(u16),
    /// Fixed-layout payloads must match the catalog size exactly; short frames
    /// are rejected rather than zero-extended.
    PayloadSize {
        topic: &'static str,
        expected: usize,
        actual: usize,
    },
}

impl std::fmt::Display for RouteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownTopic(id) => write!(f, "unknown topic id {id}"),
            Self::PayloadSize {
                topic,
                expected,
                actual,
            } => write!(f, "{topic} expects {expected} byte payload, got {actual}"),
        }
    }
}

impl std::error::Error for RouteError {}

/// A frame resolved against the Synapse catalog.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Route {
    pub topic: &'static TopicInfo,
    /// Canonical key: `[<namespace>/]<key>[/<instance>]`.
    pub key: String,
}

/// Resolve a frame to the catalog topic and the Zenoh key it publishes on.
pub fn route(namespace: &str, frame: &Frame) -> Result<Route, RouteError> {
    let topic = topic_catalog::topic_by_id(frame.topic_id)
        .ok_or(RouteError::UnknownTopic(frame.topic_id))?;

    if let Some(expected) = topic.payload_size {
        if topic.fixed_layout && frame.payload.len() != expected {
            return Err(RouteError::PayloadSize {
                topic: topic.name,
                expected,
                actual: frame.payload.len(),
            });
        }
    }

    let mut key = String::new();
    let namespace = namespace.trim_matches('/');
    if !namespace.is_empty() {
        key.push_str(namespace);
        key.push('/');
    }
    key.push_str(topic.key);
    // Multi-instance topics carry the producer instance in the key so two
    // receivers never collide on one key.
    if let Some(instance) = instance_of(topic, &frame.payload) {
        key.push('/');
        key.push_str(&instance.to_string());
    }

    Ok(Route { topic, key })
}

/// Instance id for multi-instance topics. `GnssFix` is the only one RDD2
/// telemeters; its `id` is the receiver instance.
fn instance_of(topic: &TopicInfo, payload: &[u8]) -> Option<u32> {
    if !topic.multi_instance {
        return None;
    }
    match topic.name {
        // SAFETY: `route` has already checked the payload is exactly the
        // catalog's fixed size for this struct.
        "GnssFix" => Some(u32::from(
            unsafe { <synapse_fbs::topic::GnssFixData as flatbuffers::Follow>::follow(payload, 0) }
                .id(),
        )),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// synapse catalog ids for the six topics RDD2 telemeters.
    const HEALTH: u16 = 1;
    const GNSS: u16 = 8;
    const ATT: u16 = 11;
    const ATT_SP: u16 = 19;
    const PWM: u16 = 25;
    const LOOP: u16 = 26;

    #[test]
    fn crc_matches_the_published_check_value() {
        assert_eq!(crc16_ccitt_false(b"123456789"), 0x29B1);
    }

    /// The interface contract pins payload and framed sizes per topic; drift in
    /// the pinned `synapse_fbs` catalog would break the link silently.
    #[test]
    fn catalog_matches_the_documented_downlink_sizes() {
        for (id, name, key, payload, framed) in [
            (HEALTH, "VehicleHealth", "health", 48, 58),
            (GNSS, "GnssFix", "gnss", 64, 74),
            (ATT, "AttitudeEstimate", "att", 40, 50),
            (ATT_SP, "AttitudeCommand", "att_sp", 48, 58),
            (PWM, "PwmSignalOutputs", "pwm", 48, 58),
            (LOOP, "ControlLoopMetrics", "loop", 24, 34),
        ] {
            let topic = topic_catalog::topic_by_id(id).expect("catalog id");
            assert_eq!(topic.name, name);
            assert_eq!(topic.key, key);
            assert_eq!(topic.payload_size, Some(payload));
            assert!(topic.fixed_layout);
            assert_eq!(payload + FRAME_OVERHEAD, framed);
        }
    }

    /// The vehicle build this decoder targets is pinned by the catalog hash,
    /// not by the crate version, so check the contract rather than the version.
    #[test]
    fn schema_set_hash_matches_the_vehicle_catalog() {
        assert_eq!(
            topic_catalog::SCHEMA_SET_HASH,
            "2fd857effb6c7558d6869f4307a5354c"
        );
    }

    #[test]
    fn decodes_a_frame_split_across_arbitrary_reads() {
        let payload: Vec<u8> = (0..40).collect();
        let frame = encode_frame(ATT, &payload, 42);
        assert_eq!(frame.len(), 50);

        let mut decoder = FrameDecoder::new();
        let mut decoded = Vec::new();
        // Leading garbage, then one byte at a time.
        decoded.extend(decoder.feed(b"\x00\xffnoise"));
        for byte in &frame {
            decoded.extend(decoder.feed(&[*byte]));
        }

        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].topic_id, ATT);
        assert_eq!(decoded[0].seq, 42);
        assert_eq!(decoded[0].payload, payload);
        assert_eq!(decoder.counters().frames, 1);
    }

    #[test]
    fn rejects_a_single_flipped_bit() {
        let mut frame = encode_frame(ATT, &[7; 40], 1);
        frame[20] ^= 0x01;

        let mut decoder = FrameDecoder::new();
        assert!(decoder.feed(&frame).is_empty());
        assert_eq!(decoder.counters().crc_errors, 1);
        assert_eq!(decoder.counters().frames, 0);
    }

    /// The requirement that separates a correct decoder from a lossy one: the
    /// frame behind a corrupted frame must still arrive.
    #[test]
    fn recovers_the_frame_behind_a_corrupted_one() {
        let mut corrupt = encode_frame(ATT, &[1; 40], 1);
        corrupt[9] ^= 0xFF;
        let good = encode_frame(PWM, &[2; 48], 2);

        let mut decoder = FrameDecoder::new();
        let mut stream = corrupt;
        stream.extend_from_slice(&good);
        let frames = decoder.feed(&stream);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].topic_id, PWM);
        assert_eq!(decoder.counters().crc_errors, 1);
    }

    /// A corrupted `len` makes the parser over-read across a frame boundary.
    /// Rejecting the length is not enough — the swallowed bytes must be
    /// rescanned, or the frame behind it is lost.
    #[test]
    fn recovers_the_frame_behind_an_over_long_length() {
        let mut stream = encode_frame(ATT, &[1; 40], 1);
        stream[2..4].copy_from_slice(&999_u16.to_le_bytes());
        stream.extend_from_slice(&encode_frame(LOOP, &[3; 24], 2));

        let mut decoder = FrameDecoder::new();
        let frames = decoder.feed(&stream);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].topic_id, LOOP);
        assert_eq!(decoder.counters().bad_len, 1);
    }

    #[test]
    fn rejects_a_zero_length_payload() {
        let mut decoder = FrameDecoder::new();
        let mut stream = Vec::from(SYNC);
        stream.extend_from_slice(&[0, 0, 1, 0, 0, 0, 0, 0]);
        assert!(decoder.feed(&stream).is_empty());
        assert_eq!(decoder.counters().bad_len, 1);
    }

    #[test]
    fn counts_dropped_frames_across_a_seq_wrap() {
        let mut decoder = FrameDecoder::new();
        for seq in [254_u8, 255, 3] {
            decoder.feed(&encode_frame(LOOP, &[0; 24], seq));
        }

        let counters = decoder.counters();
        assert_eq!(counters.frames, 3);
        assert_eq!(counters.seq_gaps, 1);
        assert_eq!(counters.dropped, 3); // 0, 1, 2 never arrived
    }

    #[test]
    fn garbage_never_grows_the_buffer_without_bound() {
        let mut decoder = FrameDecoder::new();
        for _ in 0..100 {
            assert!(decoder.feed(&[0xAB; 256]).is_empty());
        }
        assert!(decoder.buf.len() < SYNC.len());
    }

    /// Golden vector produced by the vehicle tree's own reference peer,
    /// `tools/synapse_serial/synapse_serial.py`, for a Fix3d at 37.7749,
    /// -122.4194, 12 m MSL, 11 satellites, seq 42. Decoding this proves the
    /// framing, the CRC and the `GnssFixData` field offsets all agree with the
    /// implementation the vehicle is tested against — not merely with our
    /// reading of the interface contract.
    const REFERENCE_GNSS_FRAME: &str = "\
5359400008002a000000000000000000000000000000000008fe8316304808b7e0\
2e0000000000000000000000000000000000000000000000000000000\
30b0000000000000000003279";

    fn reference_frame_bytes() -> Vec<u8> {
        let hex: String = REFERENCE_GNSS_FRAME.split_whitespace().collect();
        (0..hex.len())
            .step_by(2)
            .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).expect("hex"))
            .collect()
    }

    #[test]
    fn decodes_the_reference_implementations_frame() {
        let bytes = reference_frame_bytes();
        assert_eq!(bytes.len(), 74);

        let mut decoder = FrameDecoder::new();
        let frames = decoder.feed(&bytes);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].topic_id, 8);
        assert_eq!(frames[0].seq, 42);
        assert_eq!(frames[0].payload.len(), 64);
    }

    /// The bridge forwards payloads verbatim, so the bytes lifted off the link
    /// must be exactly what `synapse_fbs` decodes — that equivalence is the
    /// reason no re-encoding step exists.
    #[test]
    fn forwarded_payload_decodes_through_the_synapse_bindings() {
        let bytes = reference_frame_bytes();
        let frame = FrameDecoder::new().feed(&bytes).remove(0);
        let fix = unsafe {
            <synapse_fbs::topic::GnssFixData as flatbuffers::Follow>::follow(&frame.payload, 0)
        };

        assert_eq!(fix.latitude_deg_e7(), 377_749_000);
        assert_eq!(fix.longitude_deg_e7(), -1_224_194_000);
        assert_eq!(fix.altitude_msl_mm(), 12_000);
        assert_eq!(fix.fix_type(), synapse_fbs::types::GnssFixType::Fix3d);
        assert_eq!(fix.satellites_used(), 11);
    }

    #[test]
    fn encodes_byte_for_byte_like_the_reference_implementation() {
        let expected = reference_frame_bytes();
        let payload = &expected[HEADER_LEN..expected.len() - TRAILER_LEN];

        assert_eq!(encode_frame(GNSS, payload, 42), expected);
    }

    #[test]
    fn routes_topics_onto_canonical_catalog_keys() {
        let frame = Frame {
            topic_id: ATT,
            seq: 0,
            payload: vec![0; 40],
        };
        assert_eq!(route("", &frame).unwrap().key, "att");
        assert_eq!(route("cub1", &frame).unwrap().key, "cub1/att");
    }

    /// `GnssFix` is multi-instance, so the receiver id belongs in the key.
    #[test]
    fn routes_gnss_with_its_receiver_instance() {
        let mut payload = vec![0; 64];
        payload[56] = 2; // GnssFixData.id
        let frame = Frame {
            topic_id: GNSS,
            seq: 0,
            payload,
        };
        assert_eq!(route("", &frame).unwrap().key, "gnss/2");
    }

    #[test]
    fn rejects_a_short_fixed_layout_payload() {
        let frame = Frame {
            topic_id: HEALTH,
            seq: 0,
            payload: vec![0; 47],
        };
        assert!(matches!(
            route("", &frame),
            Err(RouteError::PayloadSize {
                expected: 48,
                actual: 47,
                ..
            })
        ));
    }

    #[test]
    fn skips_unknown_topic_ids() {
        let frame = Frame {
            topic_id: 60_000,
            seq: 0,
            payload: vec![0; 8],
        };
        assert_eq!(route("", &frame), Err(RouteError::UnknownTopic(60_000)));
    }
}
