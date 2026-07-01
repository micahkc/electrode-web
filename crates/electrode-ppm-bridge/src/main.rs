use std::io::Write;
use std::time::Duration;

use clap::Parser;
use electrode_ppm_bridge::{
    build_packet, channel_map_from_slice, manual_control_to_channels,
    rc_channels_payload_to_channels, ChannelMap, PpmChannels, FAILSAFE_CHANNELS,
};
use synapse_fbs::topic::{ManualControl, ManualControlData};
use thiserror::Error;
use zenoh::{config::Config, Wait};

#[derive(Debug, Parser)]
#[command(
    name = "electrode-ppm-bridge",
    version,
    about = "Bridge Synapse manual/autopilot outputs on Zenoh to a PPM encoder serial link",
    long_about = "Subscribes to Synapse ManualControl FlatBuffers and CUBS2 RcChannels16 \
control_output samples over Zenoh. The ManualControl active/mode switch selects either manual \
passthrough or autopilot output, then the bridge sends the same 14-byte serial packet used by \
the ROS ppm_bridge Arduino encoder.",
    next_line_help = true,
    after_help = "\
Examples:
  electrode-ppm-bridge --serial-device /dev/ttyACM0
  electrode-ppm-bridge --manual-topic synapse/manual_control --control-output-topic synapse/control_output --channel-map 1,2,0,3,4

Environment:
  ZENOH_CONNECT, ZENOH_TOPIC, ZENOH_CONTROL_OUTPUT_TOPIC
  PPM_SERIAL_DEVICE, PPM_BAUD_RATE, PPM_CHANNEL_MAP"
)]
struct Cli {
    #[command(flatten)]
    zenoh: ZenohArgs,

    #[command(flatten)]
    serial: SerialArgs,

    #[command(flatten)]
    ppm: PpmArgs,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Zenoh")]
struct ZenohArgs {
    #[arg(
        long = "zenoh-connect",
        env = "ZENOH_CONNECT",
        value_name = "LOCATOR",
        default_value = "udp/127.0.0.1:7447",
        help = "Zenoh router locator"
    )]
    zenoh_connect: String,

    #[arg(
        long = "manual-topic",
        alias = "zenoh-topic",
        alias = "topic",
        env = "ZENOH_TOPIC",
        value_name = "KEYEXPR",
        default_value = "synapse/manual_control",
        help = "Zenoh key expression carrying synapse.topic.ManualControl payloads"
    )]
    manual_topic: String,

    #[arg(
        long = "control-output-topic",
        env = "ZENOH_CONTROL_OUTPUT_TOPIC",
        value_name = "KEYEXPR",
        default_value = "synapse/control_output",
        help = "Zenoh key expression carrying CUBS2 RcChannels16 controller output payloads"
    )]
    control_output_topic: String,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Serial")]
struct SerialArgs {
    #[arg(
        long = "serial-device",
        env = "PPM_SERIAL_DEVICE",
        value_name = "PATH",
        default_value = "/dev/ttyACM0",
        help = "Serial device connected to the PPM encoder"
    )]
    serial_device: String,

    #[arg(
        long = "baud-rate",
        env = "PPM_BAUD_RATE",
        value_name = "BAUD",
        default_value_t = 57_600,
        help = "Serial baud rate"
    )]
    baud_rate: u32,

    #[arg(
        long = "serial-timeout-ms",
        env = "PPM_SERIAL_TIMEOUT_MS",
        value_name = "MS",
        default_value_t = 100,
        help = "Serial write timeout"
    )]
    serial_timeout_ms: u64,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "PPM")]
struct PpmArgs {
    #[arg(
        long = "channel-map",
        env = "PPM_CHANNEL_MAP",
        value_delimiter = ',',
        value_name = "INDEXES",
        default_value = "0,1,2,3,4",
        help = "Comma-separated output channel map over base order throttle,roll,pitch,yaw,mode"
    )]
    channel_map: Vec<usize>,
}

#[derive(Debug, Error)]
enum BridgeError {
    #[error("zenoh error: {0}")]
    Zenoh(String),
    #[error("serial error: {0}")]
    Serial(#[from] serialport::Error),
    #[error("serial write error: {0}")]
    SerialWrite(#[from] std::io::Error),
    #[error("channel map error: {0}")]
    ChannelMap(#[from] electrode_ppm_bridge::ChannelMapError),
    #[error("manual control payload is missing data")]
    MissingManualControlData,
    #[error("invalid ManualControl flatbuffer: {0}")]
    InvalidFlatbuffer(#[from] flatbuffers::InvalidFlatbuffer),
}

type Result<T> = std::result::Result<T, BridgeError>;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let channel_map = channel_map_from_slice(&cli.ppm.channel_map)?;
    run(cli, channel_map)
}

fn run(cli: Cli, channel_map: ChannelMap) -> Result<()> {
    let session = zenoh::open(zenoh_config(&cli)?)
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    let manual_subscriber = session
        .declare_subscriber(cli.zenoh.manual_topic.clone())
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    let control_subscriber = session
        .declare_subscriber(cli.zenoh.control_output_topic.clone())
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;

    let mut serial = serialport::new(&cli.serial.serial_device, cli.serial.baud_rate)
        .timeout(Duration::from_millis(cli.serial.serial_timeout_ms))
        .open()?;

    println!(
        "listening on manual={} control_output={} and writing {} at {} baud with channel map {:?}",
        cli.zenoh.manual_topic,
        cli.zenoh.control_output_topic,
        cli.serial.serial_device,
        cli.serial.baud_rate,
        channel_map.0
    );

    let mut manual_mode = ManualMode::Failsafe;
    let mut manual_channels = PpmChannels(FAILSAFE_CHANNELS);
    let mut control_channels: Option<PpmChannels> = None;

    loop {
        let mut updated = false;

        if let Some(sample) = manual_subscriber
            .recv_timeout(Duration::from_millis(10))
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?
        {
            let payload = sample.payload().to_bytes();
            match manual_from_payload(&payload, channel_map) {
                Ok(manual) => {
                    manual_mode = manual.mode;
                    manual_channels = manual.channels;
                    updated = true;
                }
                Err(BridgeError::InvalidFlatbuffer(error)) => {
                    eprintln!("dropping invalid ManualControl flatbuffer: {error}");
                }
                Err(BridgeError::MissingManualControlData) => {
                    eprintln!("dropping ManualControl payload without data");
                }
                Err(error) => return Err(error),
            }
        }

        while let Some(sample) = control_subscriber
            .recv_timeout(Duration::ZERO)
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?
        {
            let payload = sample.payload().to_bytes();
            match rc_channels_payload_to_channels(&payload) {
                Some(channels) => {
                    control_channels = Some(channel_map.apply(channels));
                    updated = true;
                }
                None => {
                    eprintln!("dropping invalid RcChannels16 control_output payload");
                }
            }
        }

        if !updated {
            continue;
        }

        let channels = match manual_mode {
            ManualMode::Manual => manual_channels,
            ManualMode::Auto => control_channels.unwrap_or(PpmChannels(FAILSAFE_CHANNELS)),
            ManualMode::Failsafe => PpmChannels(FAILSAFE_CHANNELS),
        };
        serial.write_all(&build_packet(channels))?;
    }
}

#[derive(Debug, Clone, Copy)]
enum ManualMode {
    Manual,
    Auto,
    Failsafe,
}

#[derive(Debug, Clone, Copy)]
struct ManualSelection {
    mode: ManualMode,
    channels: PpmChannels,
}

fn manual_from_payload(payload: &[u8], channel_map: ChannelMap) -> Result<ManualSelection> {
    let manual_control = flatbuffers::root::<ManualControl>(payload)?;
    let data = manual_control
        .data()
        .ok_or(BridgeError::MissingManualControlData)?;
    Ok(manual_from_data(data, channel_map))
}

fn manual_from_data(data: &ManualControlData, channel_map: ChannelMap) -> ManualSelection {
    let mode = if !data.valid() || data.kill_switch() {
        ManualMode::Failsafe
    } else if data.active() {
        ManualMode::Manual
    } else {
        ManualMode::Auto
    };

    ManualSelection {
        mode,
        channels: channel_map.apply(manual_control_to_channels(data)),
    }
}

fn zenoh_config(cli: &Cli) -> Result<Config> {
    let mut config = Config::default();
    config
        .insert_json5("mode", "\"client\"")
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    config
        .insert_json5(
            "connect/endpoints",
            &format!("[\"{}\"]", cli.zenoh.zenoh_connect),
        )
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    Ok(config)
}
