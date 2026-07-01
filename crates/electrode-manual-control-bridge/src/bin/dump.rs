use clap::Parser;
use flatbuffers::root;
use synapse_fbs::topic::ManualControl;
use thiserror::Error;
use zenoh::{config::Config, Wait};

#[derive(Debug, Parser)]
#[command(
    name = "electrode-manual-control-dump",
    version,
    about = "Subscribe to Synapse ManualControl over Zenoh and print decoded fields"
)]
struct Cli {
    #[arg(
        long = "zenoh-connect",
        env = "ZENOH_CONNECT",
        value_name = "LOCATOR",
        default_value = "tcp/127.0.0.1:7447",
        help = "Zenoh router locator"
    )]
    zenoh_connect: String,

    #[arg(
        long = "topic",
        alias = "zenoh-topic",
        env = "ZENOH_TOPIC",
        value_name = "KEYEXPR",
        default_value = "synapse/manual_control",
        help = "Zenoh key expression for synapse.topic.ManualControl payloads"
    )]
    topic: String,
}

#[derive(Debug, Error)]
enum DumpError {
    #[error("zenoh error: {0}")]
    Zenoh(String),
    #[error("flatbuffer decode error: {0}")]
    Flatbuffer(String),
}

type Result<T> = std::result::Result<T, DumpError>;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let session = zenoh::open(zenoh_config(&cli)?)
        .wait()
        .map_err(|error| DumpError::Zenoh(error.to_string()))?;
    let subscriber = session
        .declare_subscriber(cli.topic)
        .wait()
        .map_err(|error| DumpError::Zenoh(error.to_string()))?;

    loop {
        let sample = subscriber
            .recv()
            .map_err(|error| DumpError::Zenoh(error.to_string()))?;
        let payload = sample.payload().to_bytes();
        let message = root::<ManualControl>(&payload)
            .map_err(|error| DumpError::Flatbuffer(error.to_string()))?;
        let Some(data) = message.data() else {
            return Err(DumpError::Flatbuffer(
                "ManualControl.data is missing".to_string(),
            ));
        };
        let axes = data.axes();

        println!(
            "roll={:+.3} pitch={:+.3} yaw={:+.3} throttle={:.3} mode={} arm={} kill={} active={} valid={} timestamp_us={}",
            axes.roll(),
            axes.pitch(),
            axes.yaw(),
            axes.throttle(),
            data.flight_mode(),
            data.arm_switch(),
            data.kill_switch(),
            data.active(),
            data.valid(),
            data.timestamp_us(),
        );
    }
}

fn zenoh_config(cli: &Cli) -> Result<Config> {
    let mut config = Config::default();
    config
        .insert_json5("mode", "\"client\"")
        .map_err(|error| DumpError::Zenoh(error.to_string()))?;
    config
        .insert_json5("connect/endpoints", &format!("[\"{}\"]", cli.zenoh_connect))
        .map_err(|error| DumpError::Zenoh(error.to_string()))?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|error| DumpError::Zenoh(error.to_string()))?;
    Ok(config)
}
