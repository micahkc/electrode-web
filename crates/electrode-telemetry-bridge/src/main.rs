//! Bridge the RDD2 telemetry radio onto Zenoh.
//!
//! The vehicle streams six Synapse topics over a SiK pair, rate-limited to
//! 5 Hz each and sent only when their value changes — roughly 1.7 KB/s, about
//! 29% of a 57600 link. This binary decodes that stream and republishes each
//! payload on its canonical Synapse key, so the rest of Electrode (state store,
//! plots, map, MCAP logging) consumes hardware telemetry through exactly the
//! path it already uses for the simulator and for a directly-connected
//! autopilot.
//!
//! Payloads are forwarded byte-for-byte: the wire image is already a bare
//! `synapse.topic.*Data` struct, so re-encoding would only risk corrupting it.
//!
//!   electrode-telemetry-bridge --serial-device /dev/ttyUSB0
//!   electrode-telemetry-bridge --no-zenoh --verbose     # monitor only

use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::Parser;
use electrode_telemetry_bridge::mocap_gnss::{
    decode_covariance, decode_external_odometry, mark_stale, sample_to_fix, Origin, PoseSample,
    PrevSend, UplinkConfig,
};
use electrode_telemetry_bridge::{
    encode_frame, route, FrameDecoder, LinkCounters, Route, RouteError, DEFAULT_BAUD,
};
use synapse_fbs::types::GnssFixType;
use thiserror::Error;
use zenoh::{config::Config, pubsub::Publisher, Session, Wait};

/// Synapse catalog id for `GnssFix`, the one topic this bridge sends.
const GNSS_TOPIC_ID: u16 = 8;

#[derive(Debug, Parser)]
#[command(
    name = "electrode-telemetry-bridge",
    version,
    about = "Decode the RDD2 CSyn serial telemetry link and republish it as Synapse topics on Zenoh",
    long_about = "Reads the compact synapse serial framing the vehicle streams over a telemetry \
radio, validates each frame's CRC, resolves the topic id against the pinned synapse_fbs catalog, \
and republishes the payload verbatim on the canonical Zenoh key with the mandatory value \
contract. The link is unsolicited and one-way: there is no handshake, no heartbeat and no \
request/response, so liveness is inferred from frame arrival.",
    next_line_help = true,
    // A western longitude or a negative offset is a value, not a flag: without
    // this clap reads `--origin-lon -86.93` as an unknown argument `-8`.
    allow_negative_numbers = true,
    after_help = "\
Examples:
  electrode-telemetry-bridge --serial-device /dev/ttyUSB0
  electrode-telemetry-bridge --namespace cub1
  electrode-telemetry-bridge --no-zenoh --verbose

Environment:
  TELEMETRY_SERIAL_DEVICE, TELEMETRY_BAUD_RATE, ZENOH_CONNECT, ZENOH_NAMESPACE"
)]
struct Cli {
    #[arg(
        long = "serial-device",
        env = "TELEMETRY_SERIAL_DEVICE",
        value_name = "PATH",
        default_value = "/dev/ttyUSB0",
        help = "Serial device the telemetry radio presents"
    )]
    serial_device: String,

    #[arg(
        long = "baud-rate",
        env = "TELEMETRY_BAUD_RATE",
        default_value_t = DEFAULT_BAUD,
        help = "Link baud rate; SiK radios default to 57600 8N1"
    )]
    baud_rate: u32,

    #[arg(
        long = "serial-timeout-ms",
        default_value_t = 100,
        help = "Read timeout. A timeout is normal: topics are only sent when they change"
    )]
    serial_timeout_ms: u64,

    #[arg(
        long = "zenoh-connect",
        env = "ZENOH_CONNECT",
        value_name = "LOCATOR",
        default_value = "udp/127.0.0.1:7447",
        help = "Zenoh router/peer locator to connect to (matches the other ground-station bridges)"
    )]
    zenoh_connect: String,

    #[arg(
        long = "namespace",
        env = "ZENOH_NAMESPACE",
        value_name = "PREFIX",
        default_value = "",
        help = "Deployment namespace prefixed to every published key, e.g. `cub1`"
    )]
    namespace: String,

    #[arg(
        long = "no-zenoh",
        help = "Decode and report without publishing; pair with --verbose to monitor a link"
    )]
    no_zenoh: bool,

    #[arg(
        long,
        short,
        help = "Print every decoded frame, not just the periodic link summary"
    )]
    verbose: bool,

    #[arg(
        long = "status-interval-secs",
        default_value_t = 5,
        help = "Seconds between link summaries; 0 disables them"
    )]
    status_interval_secs: u64,

    #[arg(
        long = "link-timeout-ms",
        default_value_t = 3000,
        help = "Silence after which the link is reported down. The link has no heartbeat, so this \
                is the only liveness signal available"
    )]
    link_timeout_ms: u64,

    #[command(flatten)]
    uplink: UplinkArgs,
}

/// Mocap-sourced GNSS uplink. Defaults match the vehicle tree's
/// `test_scripts/publish_gps_zenoh.py`, so a calibration trimmed there carries
/// over unchanged.
#[derive(Debug, Parser)]
#[command(next_help_heading = "Mocap GNSS uplink")]
struct UplinkArgs {
    #[arg(
        long = "mocap-gnss",
        env = "TELEMETRY_MOCAP_GNSS",
        help = "Convert mocap pose to GnssFix and send it up the radio. Requires a vehicle built \
                -S mocap-gnss; the default build declares gnss outbound and drops injected fixes"
    )]
    mocap_gnss: bool,

    #[arg(
        long = "mocap-namespace",
        env = "TELEMETRY_MOCAP_NAMESPACE",
        value_name = "PREFIX",
        default_value = "**",
        help = "Namespace the mocap bridge publishes under; `**` matches any"
    )]
    mocap_namespace: String,

    #[arg(
        long = "mocap-instance",
        value_name = "ID",
        help = "Only accept samples whose producer instance matches"
    )]
    mocap_instance: Option<u8>,

    #[arg(
        long = "no-covariance",
        help = "Ignore external_pose_cov and use --hacc/--vacc instead"
    )]
    no_covariance: bool,

    #[arg(
        long = "uplink-rate-hz",
        default_value_t = 10.0,
        help = "Fixes per second sent"
    )]
    uplink_rate_hz: f64,

    #[arg(
        long = "uplink-timeout-ms",
        default_value_t = 500,
        help = "Silence after which the fix is sent as NoFix. A frozen position at full confidence \
                is worse than none: the estimator cannot tell it is stale"
    )]
    uplink_timeout_ms: u64,

    #[arg(long = "origin-lat", default_value_t = 40.415_453_968_973_93)]
    origin_lat: f64,

    #[arg(long = "origin-lon", default_value_t = -86.932_758_662_594_37)]
    origin_lon: f64,

    #[arg(long = "origin-alt", default_value_t = 0.0)]
    origin_alt: f64,

    #[arg(
        long = "yaw-offset",
        default_value_t = 230.0,
        help = "Degrees rotating the facility frame onto true north"
    )]
    yaw_offset: f64,

    #[arg(long = "heading-offset", default_value_t = 138.0)]
    heading_offset: f64,

    #[arg(
        long = "hacc",
        default_value_t = 0.05,
        help = "Fallback horizontal accuracy, metres"
    )]
    hacc: f64,

    #[arg(long = "vacc", default_value_t = 0.08)]
    vacc: f64,

    #[arg(
        long = "degraded-scale",
        default_value_t = 4.0,
        help = "Accuracy multiplier while tracking is degraded"
    )]
    degraded_scale: f64,

    #[arg(long = "hdop", default_value_t = 0.3)]
    hdop: f64,

    #[arg(long = "vdop", default_value_t = 0.4)]
    vdop: f64,

    #[arg(long = "satellites", default_value_t = 14)]
    satellites: u8,

    #[arg(long = "publish-yaw", help = "Send mocap heading as GNSS yaw")]
    publish_yaw: bool,

    #[arg(long = "gnss-instance", default_value_t = 0, help = "GnssFixData.id")]
    gnss_instance: u8,
}

impl UplinkArgs {
    fn config(&self) -> UplinkConfig {
        UplinkConfig {
            origin: Origin {
                lat_deg: self.origin_lat,
                lon_deg: self.origin_lon,
                alt_m: self.origin_alt,
            },
            yaw_offset_deg: self.yaw_offset,
            heading_offset_deg: self.heading_offset,
            hacc_m: self.hacc,
            vacc_m: self.vacc,
            degraded_scale: self.degraded_scale,
            hdop: self.hdop,
            vdop: self.vdop,
            satellites: self.satellites,
            publish_yaw: self.publish_yaw,
            gnss_instance: self.gnss_instance,
        }
    }
}

#[derive(Debug, Error)]
enum BridgeError {
    #[error("zenoh error: {0}")]
    Zenoh(String),
    #[error("serial error: {0}")]
    Serial(#[from] serialport::Error),
    #[error("serial read error: {0}")]
    SerialRead(#[from] std::io::Error),
    #[error("--mocap-gnss needs a zenoh session to read mocap pose from; drop --no-zenoh")]
    UplinkNeedsZenoh,
}

type Result<T> = std::result::Result<T, BridgeError>;

fn main() -> Result<()> {
    let cli = Cli::parse();
    zenoh::init_log_from_env_or("error");

    let mut port = serialport::new(&cli.serial_device, cli.baud_rate)
        .timeout(Duration::from_millis(cli.serial_timeout_ms))
        .open()?;
    print_startup(&cli);

    if cli.no_zenoh {
        if cli.uplink.mocap_gnss {
            return Err(BridgeError::UplinkNeedsZenoh);
        }
        return run_loop(&cli, &mut *port, &mut NullSink, None);
    }

    let session = open_session(&cli)?;
    // Subscribers are kept alive for the run; the callbacks feed `inbox`.
    let (inbox, _subscribers) = if cli.uplink.mocap_gnss {
        let inbox = Arc::new(Mutex::new(MocapInbox::default()));
        let subscribers = declare_mocap_subscribers(&session, &cli.uplink, &inbox)?;
        (Some(inbox), Some(subscribers))
    } else {
        (None, None)
    };
    let mut uplink = inbox.map(|inbox| Uplink::new(&cli.uplink, inbox));

    let mut sink = ZenohSink {
        session: &session,
        publishers: HashMap::new(),
    };
    run_loop(&cli, &mut *port, &mut sink, uplink.as_mut())
}

/// Latest mocap sample and accuracy, filled by the subscriber callbacks.
#[derive(Default)]
struct MocapInbox {
    sample: Option<(PoseSample, Instant)>,
    accuracy: Option<((f64, f64), Instant)>,
    received: u64,
    key: Option<String>,
}

type Subscribers = Vec<zenoh::pubsub::Subscriber<()>>;

/// Subscribe to the mocap pose (and its covariance, unless disabled) on both
/// the bare and instanced key forms.
fn declare_mocap_subscribers(
    session: &Session,
    args: &UplinkArgs,
    inbox: &Arc<Mutex<MocapInbox>>,
) -> Result<Subscribers> {
    let namespace = args.mocap_namespace.trim_matches('/');
    let key = |leaf: &str| {
        if namespace.is_empty() {
            leaf.to_string()
        } else {
            format!("{namespace}/{leaf}")
        }
    };
    let instance = args.mocap_instance;
    let mut subscribers = Vec::new();

    for expr in [key("external_pose"), format!("{}/*", key("external_pose"))] {
        let inbox = Arc::clone(inbox);
        subscribers.push(
            session
                .declare_subscriber(expr)
                .callback(move |sample| note_pose(&inbox, &sample, instance))
                .wait()
                .map_err(|error| BridgeError::Zenoh(error.to_string()))?,
        );
    }

    if args.no_covariance {
        return Ok(subscribers);
    }
    for expr in [
        key("external_pose_cov"),
        format!("{}/*", key("external_pose_cov")),
    ] {
        let inbox = Arc::clone(inbox);
        subscribers.push(
            session
                .declare_subscriber(expr)
                .callback(move |sample| note_covariance(&inbox, &sample, instance))
                .wait()
                .map_err(|error| BridgeError::Zenoh(error.to_string()))?,
        );
    }

    Ok(subscribers)
}

fn note_pose(inbox: &Mutex<MocapInbox>, sample: &zenoh::sample::Sample, instance: Option<u8>) {
    let Some(pose) = decode_external_odometry(&sample.payload().to_bytes(), instance) else {
        return;
    };
    let mut inbox = inbox.lock().expect("mocap inbox");
    inbox.sample = Some((pose, Instant::now()));
    inbox.received += 1;
    inbox.key = Some(sample.key_expr().as_str().to_string());
}

fn note_covariance(
    inbox: &Mutex<MocapInbox>,
    sample: &zenoh::sample::Sample,
    instance: Option<u8>,
) {
    let Some(accuracy) = decode_covariance(&sample.payload().to_bytes(), instance) else {
        return;
    };
    inbox.lock().expect("mocap inbox").accuracy = Some((accuracy, Instant::now()));
}

/// Sends mocap-derived GNSS fixes up the radio at a fixed rate.
struct Uplink {
    inbox: Arc<Mutex<MocapInbox>>,
    config: UplinkConfig,
    period: Duration,
    timeout: Duration,
    started: Instant,
    next_send: Instant,
    prev: Option<PrevSend>,
    /// Outgoing frame counter, independent of the vehicle's own `seq`.
    seq: u8,
    sent: u64,
    lost_sent: u64,
}

impl Uplink {
    fn new(args: &UplinkArgs, inbox: Arc<Mutex<MocapInbox>>) -> Self {
        let now = Instant::now();
        Self {
            inbox,
            config: args.config(),
            period: Duration::from_secs_f64(1.0 / args.uplink_rate_hz.max(0.1)),
            timeout: Duration::from_millis(args.uplink_timeout_ms),
            started: now,
            next_send: now,
            prev: None,
            seq: 0,
            sent: 0,
            lost_sent: 0,
        }
    }

    /// Build the frame due now, if any.
    fn tick(&mut self) -> Option<Vec<u8>> {
        let now = Instant::now();
        if now < self.next_send {
            return None;
        }
        self.next_send = now + self.period;

        let (sample, accuracy) = {
            let inbox = self.inbox.lock().expect("mocap inbox");
            let sample = inbox.sample?;
            let accuracy = inbox
                .accuracy
                .filter(|(_, at)| at.elapsed() < self.timeout)
                .map(|(value, _)| value);
            (sample, accuracy)
        };

        let (pose, at) = sample;
        // A frozen position sent at full confidence is worse than no fix: the
        // estimator has no way to tell it is stale.
        let (pose, accuracy) = if at.elapsed() > self.timeout {
            (mark_stale(&pose), None)
        } else {
            (pose, accuracy)
        };

        let outcome = sample_to_fix(
            &pose,
            self.prev,
            &self.config,
            self.started.elapsed().as_micros() as u64,
            unix_micros(),
            now.duration_since(self.started).as_secs_f64(),
            accuracy,
        );
        self.prev = Some(outcome.prev);
        self.sent += 1;
        if outcome.fix_type == GnssFixType::NoFix {
            self.lost_sent += 1;
        }

        let frame = encode_frame(GNSS_TOPIC_ID, &outcome.payload, self.seq);
        self.seq = self.seq.wrapping_add(1);
        Some(frame)
    }
}

fn unix_micros() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since| since.as_micros() as u64)
}

/// Where decoded payloads go. Publishing is behind a trait so `--no-zenoh`
/// monitoring runs the identical decode path, rather than a second one that
/// could drift from it.
trait FrameSink {
    fn publish(&mut self, route: &Route, payload: &[u8]) -> Result<()>;
}

struct NullSink;

impl FrameSink for NullSink {
    fn publish(&mut self, _route: &Route, _payload: &[u8]) -> Result<()> {
        Ok(())
    }
}

struct ZenohSink<'a> {
    session: &'a Session,
    publishers: HashMap<String, Publisher<'a>>,
}

impl FrameSink for ZenohSink<'_> {
    fn publish(&mut self, route: &Route, payload: &[u8]) -> Result<()> {
        let session = self.session;
        let publisher = match self.publishers.entry(route.key.clone()) {
            Entry::Occupied(entry) => entry.into_mut(),
            Entry::Vacant(entry) => {
                // The value contract is mandatory and stamped once at declare
                // time; consumers validate it before decoding payload bytes.
                let encoding = synapse_fbs::value_contract::encoding_for_topic(route.topic);
                let publisher = session
                    .declare_publisher(route.key.clone())
                    .encoding(zenoh::bytes::Encoding::from(encoding.as_str()))
                    .wait()
                    .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
                entry.insert(publisher)
            }
        };
        publisher
            .put(payload.to_vec())
            .wait()
            .map_err(|error| BridgeError::Zenoh(error.to_string()))
    }
}

/// Per-topic arrival counts plus the routing rejections, which the periodic
/// summary reports next to the decoder's own counters.
#[derive(Default)]
struct Stats {
    per_topic: HashMap<&'static str, u64>,
    unknown: u64,
    size_errors: u64,
    last_frame: Option<Instant>,
    gnss_note_printed: bool,
}

fn run_loop(
    cli: &Cli,
    port: &mut dyn serialport::SerialPort,
    sink: &mut dyn FrameSink,
    mut uplink: Option<&mut Uplink>,
) -> Result<()> {
    let mut decoder = FrameDecoder::new();
    let mut stats = Stats::default();
    let mut buf = [0_u8; 512];
    let status_interval = Duration::from_secs(cli.status_interval_secs);
    let mut next_status = Instant::now() + status_interval;

    loop {
        match port.read(&mut buf) {
            Ok(0) => {}
            Ok(read) => consume(cli, &mut decoder, &mut stats, sink, &buf[..read])?,
            // A read timeout is the normal idle state, not an error: a topic
            // that never updates is never sent. It is also what paces this
            // loop, so the uplink below runs on the same thread as the read
            // and the port never needs a lock.
            Err(error) if error.kind() == std::io::ErrorKind::TimedOut => {}
            Err(error) => return Err(BridgeError::SerialRead(error)),
        }

        if let Some(uplink) = uplink.as_deref_mut() {
            if let Some(frame) = uplink.tick() {
                port.write_all(&frame)?;
            }
        }

        if cli.status_interval_secs > 0 && Instant::now() >= next_status {
            print_status(cli, &decoder.counters(), &mut stats);
            print_uplink_status(uplink.as_deref());
            next_status = Instant::now() + status_interval;
        }
    }
}

/// The vehicle never acknowledges an injected fix, so the only honest report is
/// what was sent — acceptance has to be confirmed on the vehicle shell.
fn print_uplink_status(uplink: Option<&Uplink>) {
    let Some(uplink) = uplink else {
        return;
    };
    let inbox = uplink.inbox.lock().expect("mocap inbox");
    let source = inbox.key.as_deref().unwrap_or("waiting for mocap");
    println!(
        "tx  gnss sent={} nofix={} from {source} (received={})",
        uplink.sent, uplink.lost_sent, inbox.received
    );
}

fn consume(
    cli: &Cli,
    decoder: &mut FrameDecoder,
    stats: &mut Stats,
    sink: &mut dyn FrameSink,
    chunk: &[u8],
) -> Result<()> {
    for frame in decoder.feed(chunk) {
        match route(&cli.namespace, &frame) {
            Ok(resolved) => {
                *stats.per_topic.entry(resolved.topic.name).or_default() += 1;
                stats.last_frame = Some(Instant::now());
                if cli.verbose {
                    print_frame(&resolved, frame.seq, &frame.payload);
                }
                sink.publish(&resolved, &frame.payload)?;
            }
            // The vehicle's topic list is application-owned and may grow, so an
            // unrecognised id is skipped rather than treated as a fault.
            Err(RouteError::UnknownTopic(id)) => {
                stats.unknown += 1;
                if cli.verbose {
                    println!("[{:3}] unknown topic id {id}", frame.seq);
                }
            }
            Err(error) => {
                stats.size_errors += 1;
                eprintln!("[{:3}] {error}", frame.seq);
            }
        }
    }
    Ok(())
}

fn print_frame(resolved: &Route, seq: u8, payload: &[u8]) {
    match synapse_fbs::topic_decode::decode_topic_debug(resolved.topic, payload) {
        Ok(text) => println!("[{seq:3}] {:<18} {text}", resolved.key),
        Err(error) => println!(
            "[{seq:3}] {:<18} {} B (undecodable: {error})",
            resolved.key,
            payload.len()
        ),
    }
}

/// Shaped like the vehicle shell's `csyn_serial status` so both ends of a bad
/// link can be compared line for line.
fn print_status(cli: &Cli, counters: &LinkCounters, stats: &mut Stats) {
    println!(
        "rx  frames={} crc_err={} bad_len={} unknown={} size_err={}",
        counters.frames, counters.crc_errors, counters.bad_len, stats.unknown, stats.size_errors
    );
    println!(
        "rx  seq_gaps={} dropped={} resyncs={} link={}",
        counters.seq_gaps,
        counters.dropped,
        counters.resyncs,
        link_state(cli, stats)
    );

    if !stats.per_topic.is_empty() {
        let mut topics: Vec<_> = stats.per_topic.iter().collect();
        topics.sort_unstable();
        let summary: Vec<String> = topics
            .into_iter()
            .map(|(name, count)| format!("{name}={count}"))
            .collect();
        println!("rx  {}", summary.join(" "));
    }

    maybe_note_absent_gnss(stats);
}

/// The link has no heartbeat, so liveness is "a decoded frame in the last
/// `--link-timeout-ms`".
fn link_state(cli: &Cli, stats: &Stats) -> String {
    let timeout = Duration::from_millis(cli.link_timeout_ms);
    match stats.last_frame {
        Some(at) if at.elapsed() <= timeout => {
            format!("up ({:.1}s ago)", at.elapsed().as_secs_f32())
        }
        Some(at) => format!("down ({:.1}s silent)", at.elapsed().as_secs_f32()),
        None => "down (no frames yet)".to_string(),
    }
}

/// A vehicle built `-S mocap-gnss` expects a fix to be supplied and telemeters
/// none, so absent GNSS must be surfaced as a build variant, not a fault.
fn maybe_note_absent_gnss(stats: &mut Stats) {
    if stats.gnss_note_printed || stats.per_topic.is_empty() {
        return;
    }
    if !stats.per_topic.contains_key("GnssFix") {
        println!(
            "note: no GnssFix telemetry — expected if the vehicle is built -S mocap-gnss, \
             where the ground station supplies the fix instead"
        );
        stats.gnss_note_printed = true;
    }
}

fn print_startup(cli: &Cli) {
    println!(
        "electrode-telemetry-bridge: {} @ {} baud",
        cli.serial_device, cli.baud_rate
    );
    if cli.no_zenoh {
        println!("  publishing disabled (--no-zenoh)");
    } else {
        let namespace = cli.namespace.trim_matches('/');
        let prefix = if namespace.is_empty() {
            String::from("<bare catalog keys>")
        } else {
            format!("{namespace}/")
        };
        println!("  → zenoh {} as {prefix}", cli.zenoh_connect);
    }
}

fn open_session(cli: &Cli) -> Result<Session> {
    zenoh::open(zenoh_config(cli)?)
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))
}

fn zenoh_config(cli: &Cli) -> Result<Config> {
    let mut config = Config::default();
    config
        .insert_json5("mode", "\"client\"")
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    config
        .insert_json5("connect/endpoints", &format!("[\"{}\"]", cli.zenoh_connect))
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    // Match the other ground-station bridges: explicit endpoint, no multicast
    // discovery, so every component deterministically connects to the same hub.
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    Ok(config)
}

#[cfg(test)]
mod cli_tests {
    use super::Cli;
    use clap::Parser;

    /// The ground station always passes the calibration, and a western
    /// longitude is negative. Without `allow_negative_numbers` clap rejects it
    /// and the bridge exits the moment it is launched.
    #[test]
    fn accepts_a_negative_origin_longitude() {
        let cli = Cli::try_parse_from([
            "electrode-telemetry-bridge",
            "--origin-lat",
            "40.41545396897393",
            "--origin-lon",
            "-86.93275866259437",
            "--yaw-offset",
            "230",
        ])
        .expect("negative longitude must parse");

        assert!((cli.uplink.origin_lon - -86.932_758_662_594_37).abs() < 1e-12);
        assert!((cli.uplink.origin_lat - 40.415_453_968_973_93).abs() < 1e-12);
    }
}
