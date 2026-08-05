//! Electrode Ground Station daemon.
//!
//! Serves the same static Viewer bundle that GitHub Pages hosts, but on the
//! local machine and alongside a `gcs/*` HTTP API. Because the app probes
//! `gcs/health` on its own origin, being served by this daemon is exactly what
//! flips it from Viewer into Ground Station mode and unlocks the hardware
//! panels.
//!
//! Capabilities: device discovery, a live raw joystick inspector, and an
//! editable RC mapping profile applied by supervising the manual-control
//! bridge.

mod autopilot;
mod autopilot_link;
mod devices;
mod joystick;
mod mapping;
mod mocap;
mod sim_bridge;
mod simulation;
mod supervisor;
mod telemetry;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};

use axum::body::Body;
use axum::extract::State;
use axum::http::{Request, StatusCode};
use axum::response::IntoResponse;
use axum::routing::{get, post};
use axum::{Json, Router};
use clap::Parser;
use electrode_command_authority::{CommandAuthority, CommandAuthorityConfig};
use flatbuffers::FlatBufferBuilder;
use serde::{Deserialize, Serialize};
use synapse_fbs::cmd::{
    ParamGetReply, ParamGetRequest, ParamGetRequestArgs, ParamKind, ParamSetReply, ParamSetRequest,
    ParamSetRequestArgs, ParamValue, ParamValueArgs,
};
use synapse_fbs::types::CommandResultCode;
use tower::{service_fn, ServiceExt};

use autopilot::AutopilotProfile;
use autopilot_link::{AutopilotLink, AutopilotRunStatus};
use mapping::MappingProfile;
use mocap::MocapProfile;
use simulation::{ModelicaFile, ModelicaFileSave, SimulationProfile};
use supervisor::Supervisor;

#[derive(Parser, Debug)]
#[command(
    name = "electrode-ground-station",
    about = "Serve the electrode Viewer locally with Ground Station (hardware) capabilities"
)]
struct Cli {
    /// Address to listen on.
    #[arg(long, env = "ELECTRODE_GCS_ADDR", default_value = "127.0.0.1:8790")]
    addr: SocketAddr,

    /// Directory containing the built Viewer app (adapter-static output).
    #[arg(long, env = "ELECTRODE_GCS_WEB_DIR", default_value = "apps/web/build")]
    web_dir: PathBuf,

    /// Where the RC mapping profile is stored.
    #[arg(
        long,
        env = "ELECTRODE_GCS_MAPPING_FILE",
        default_value = "electrode-mapping.json"
    )]
    mapping_file: PathBuf,

    /// Where the autopilot stack profile is stored.
    #[arg(
        long,
        env = "ELECTRODE_GCS_AUTOPILOT_FILE",
        default_value = "electrode-autopilot.json"
    )]
    autopilot_file: PathBuf,

    /// Where the Rumoca simulation profile is stored.
    #[arg(
        long,
        env = "ELECTRODE_GCS_SIMULATION_FILE",
        default_value = "electrode-simulation.json"
    )]
    simulation_file: PathBuf,

    /// Where the motion-capture link profile is stored.
    #[arg(
        long,
        env = "ELECTRODE_GCS_MOCAP_FILE",
        default_value = "electrode-mocap.json"
    )]
    mocap_file: PathBuf,
}

struct AppState {
    mapping: RwLock<MappingProfile>,
    mapping_file: PathBuf,
    autopilot: RwLock<AutopilotProfile>,
    autopilot_file: PathBuf,
    simulation: RwLock<SimulationProfile>,
    simulation_file: PathBuf,
    sim_bridge: sim_bridge::SimBridge,
    supervisor: Supervisor,
    ppm_supervisor: Supervisor,
    telemetry: RwLock<telemetry::TelemetryProfile>,
    telemetry_supervisor: Supervisor,
    mocap: RwLock<MocapProfile>,
    mocap_file: PathBuf,
    autopilot_link: AutopilotLink,
    command_authority: CommandAuthority,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct RuntimeParameterRequest {
    name: String,
    value: Option<f64>,
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct RuntimeParameterResponse {
    name: String,
    value: f64,
}

async fn runtime_parameter(
    State(state): State<Shared>,
    Json(request): Json<RuntimeParameterRequest>,
) -> Result<Json<RuntimeParameterResponse>, (StatusCode, String)> {
    let name = request.name.trim().to_string();
    if name.is_empty() || name.len() > 128 {
        return Err((StatusCode::BAD_REQUEST, "invalid parameter name".into()));
    }
    if request.value.is_some_and(|value| !value.is_finite()) {
        return Err((
            StatusCode::BAD_REQUEST,
            "parameter value must be finite".into(),
        ));
    }
    let result = tokio::task::spawn_blocking(move || {
        let mut builder = FlatBufferBuilder::new();
        let name_offset = builder.create_string(&name);
        let (target, expected_set) = if let Some(value) = request.value {
            let parameter = ParamValue::create(
                &mut builder,
                &ParamValueArgs {
                    name: Some(name_offset),
                    kind: ParamKind::Float,
                    float_value: value,
                    ..Default::default()
                },
            );
            let root = ParamSetRequest::create(
                &mut builder,
                &ParamSetRequestArgs {
                    value: Some(parameter),
                },
            );
            builder.finish(root, None);
            ("cmd/param_set", true)
        } else {
            let root = ParamGetRequest::create(
                &mut builder,
                &ParamGetRequestArgs {
                    name: Some(name_offset),
                    offset: 0,
                    limit: 1,
                },
            );
            builder.finish(root, None);
            ("cmd/param_get", false)
        };
        let payload = state
            .command_authority
            .trusted_query(target, builder.finished_data().to_vec())?;
        let value = if expected_set {
            let reply = flatbuffers::root::<ParamSetReply<'_>>(&payload)?;
            anyhow::ensure!(
                reply.result() == CommandResultCode::Accepted,
                "parameter set rejected"
            );
            anyhow::ensure!(
                reply.value().is_some(),
                "parameter set reply contained no value"
            );

            // Accepted means CUBS2 staged the update. It applies staged
            // values atomically at the next 50 Hz controller boundary, so
            // read the live value back after one complete control period.
            std::thread::sleep(std::time::Duration::from_millis(25));
            let mut readback_builder = FlatBufferBuilder::new();
            let readback_name = readback_builder.create_string(&name);
            let readback_root = ParamGetRequest::create(
                &mut readback_builder,
                &ParamGetRequestArgs {
                    name: Some(readback_name),
                    offset: 0,
                    limit: 1,
                },
            );
            readback_builder.finish(readback_root, None);
            let readback_payload = state
                .command_authority
                .trusted_query("cmd/param_get", readback_builder.finished_data().to_vec())?;
            let readback = flatbuffers::root::<ParamGetReply<'_>>(&readback_payload)?;
            anyhow::ensure!(
                readback.result() == CommandResultCode::Accepted,
                "parameter read-back rejected"
            );
            readback
                .values()
                .and_then(|values| (!values.is_empty()).then(|| values.get(0).float_value()))
        } else {
            let reply = flatbuffers::root::<ParamGetReply<'_>>(&payload)?;
            anyhow::ensure!(
                reply.result() == CommandResultCode::Accepted,
                "parameter get rejected"
            );
            reply
                .values()
                .and_then(|values| (!values.is_empty()).then(|| values.get(0).float_value()))
        }
        .ok_or_else(|| anyhow::anyhow!("parameter reply contained no value"))?;
        Ok::<_, anyhow::Error>(RuntimeParameterResponse { name, value })
    })
    .await
    .map_err(|error| (StatusCode::INTERNAL_SERVER_ERROR, error.to_string()))?
    .map_err(|error| (StatusCode::BAD_GATEWAY, error.to_string()))?;
    Ok(Json(result))
}

fn should_serve_spa_fallback(path: &str) -> bool {
    let last_segment = path.rsplit('/').next().unwrap_or_default();

    !path.starts_with("/_app/") && !path.starts_with("/assets/") && !last_segment.contains('.')
}

async fn serve_spa_fallback(
    index: PathBuf,
    req: Request<Body>,
) -> Result<axum::response::Response, std::convert::Infallible> {
    if !should_serve_spa_fallback(req.uri().path()) {
        return Ok(StatusCode::NOT_FOUND.into_response());
    }

    let response = match tower_http::services::ServeFile::new(index)
        .oneshot(req)
        .await
    {
        Ok(response) => response.map(Body::new),
        Err(err) => match err {},
    };
    Ok(response)
}

#[cfg(test)]
mod tests {
    use super::should_serve_spa_fallback;

    #[test]
    fn spa_fallback_accepts_client_routes() {
        assert!(should_serve_spa_fallback("/"));
        assert!(should_serve_spa_fallback("/manual-control"));
        assert!(should_serve_spa_fallback("/nested/route"));
    }

    #[test]
    fn spa_fallback_rejects_asset_paths() {
        assert!(!should_serve_spa_fallback(
            "/_app/immutable/entry/start.missing.js"
        ));
        assert!(!should_serve_spa_fallback("/assets/models/missing.glb"));
        assert!(!should_serve_spa_fallback("/favicon.ico"));
    }
}

type Shared = Arc<AppState>;

#[derive(Serialize)]
struct Health {
    service: &'static str,
    version: &'static str,
    host: String,
}

async fn health() -> Json<Health> {
    Json(Health {
        service: "electrode-ground-station",
        version: env!("CARGO_PKG_VERSION"),
        host: hostname(),
    })
}

async fn devices() -> Json<devices::Devices> {
    Json(devices::list())
}

async fn get_mapping(State(state): State<Shared>) -> Json<MappingProfile> {
    Json(state.mapping.read().expect("mapping lock poisoned").clone())
}

async fn put_mapping(
    State(state): State<Shared>,
    Json(profile): Json<MappingProfile>,
) -> Result<Json<MappingProfile>, (StatusCode, String)> {
    // Persist, then relaunch the bridge if it's already running so mapping
    // edits take effect live.
    profile
        .save(&state.mapping_file)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    let args = profile.bridge_args();
    let ppm_args = profile.ppm_bridge_args();
    *state.mapping.write().expect("mapping lock poisoned") = profile.clone();
    state
        .supervisor
        .restart_if_running(&args)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    state
        .ppm_supervisor
        .restart_if_running(&ppm_args)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    Ok(Json(profile))
}

async fn get_autopilot(State(state): State<Shared>) -> Json<AutopilotProfile> {
    Json(
        state
            .autopilot
            .read()
            .expect("autopilot lock poisoned")
            .clone(),
    )
}

async fn put_autopilot(
    State(state): State<Shared>,
    Json(profile): Json<AutopilotProfile>,
) -> Result<Json<AutopilotProfile>, (StatusCode, String)> {
    let profile = profile.normalized();
    profile
        .save(&state.autopilot_file)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    state.sim_bridge.set_mocap_source(profile.mocap_source);
    *state.autopilot.write().expect("autopilot lock poisoned") = profile.clone();
    Ok(Json(profile))
}

async fn autopilot_run_status(State(state): State<Shared>) -> Json<AutopilotRunStatus> {
    Json(state.autopilot_link.status())
}

async fn autopilot_start(
    State(state): State<Shared>,
) -> Result<Json<AutopilotRunStatus>, (StatusCode, String)> {
    let profile = state
        .autopilot
        .read()
        .expect("autopilot lock poisoned")
        .clone();
    // The link blocks briefly on zenoh open; keep the async runtime free.
    let link = tokio::task::block_in_place(|| {
        let session = state.command_authority.vehicle_client()?;
        state.autopilot_link.start(&profile, Some(session))
    });
    link.map(Json)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))
}

async fn autopilot_stop(State(state): State<Shared>) -> Json<AutopilotRunStatus> {
    Json(tokio::task::block_in_place(|| state.autopilot_link.stop()))
}

async fn get_simulation(State(state): State<Shared>) -> Json<SimulationProfile> {
    Json(
        state
            .simulation
            .read()
            .expect("simulation lock poisoned")
            .clone(),
    )
}

async fn put_simulation(
    State(state): State<Shared>,
    Json(profile): Json<SimulationProfile>,
) -> Result<Json<SimulationProfile>, (StatusCode, String)> {
    let profile = profile.normalized();
    profile
        .save(&state.simulation_file)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    *state.simulation.write().expect("simulation lock poisoned") = profile.clone();
    Ok(Json(profile))
}

async fn simulation_model(
    State(state): State<Shared>,
) -> Result<Json<ModelicaFile>, (StatusCode, String)> {
    let profile = state
        .simulation
        .read()
        .expect("simulation lock poisoned")
        .clone();
    profile
        .read_model_file()
        .map(Json)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))
}

async fn put_simulation_model(
    State(state): State<Shared>,
    Json(file): Json<ModelicaFileSave>,
) -> Result<Json<ModelicaFile>, (StatusCode, String)> {
    let profile = state
        .simulation
        .read()
        .expect("simulation lock poisoned")
        .clone();
    profile
        .save_model_file(file)
        .map(Json)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct BridgeStatus {
    running: bool,
    bin: String,
    ppm_running: bool,
    ppm_bin: String,
}

async fn bridge_status(State(state): State<Shared>) -> Json<BridgeStatus> {
    let manual_running = state.supervisor.running();
    let ppm_running = state.ppm_supervisor.running();
    Json(BridgeStatus {
        running: manual_running,
        bin: state.supervisor.bin_display(),
        ppm_running,
        ppm_bin: state.ppm_supervisor.bin_display(),
    })
}

async fn bridge_start(
    State(state): State<Shared>,
) -> Result<Json<BridgeStatus>, (StatusCode, String)> {
    let args = state
        .mapping
        .read()
        .expect("mapping lock poisoned")
        .bridge_args();
    state
        .supervisor
        .start(&args)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    let manual_running = state.supervisor.running();
    let ppm_running = state.ppm_supervisor.running();
    Ok(Json(BridgeStatus {
        running: manual_running,
        bin: state.supervisor.bin_display(),
        ppm_running,
        ppm_bin: state.ppm_supervisor.bin_display(),
    }))
}

async fn bridge_stop(State(state): State<Shared>) -> Json<BridgeStatus> {
    state.supervisor.stop();
    let ppm_running = state.ppm_supervisor.running();
    Json(BridgeStatus {
        running: false,
        bin: state.supervisor.bin_display(),
        ppm_running,
        ppm_bin: state.ppm_supervisor.bin_display(),
    })
}

async fn ppm_bridge_start(
    State(state): State<Shared>,
) -> Result<Json<BridgeStatus>, (StatusCode, String)> {
    let ppm_args = state
        .mapping
        .read()
        .expect("mapping lock poisoned")
        .ppm_bridge_args();
    state
        .ppm_supervisor
        .start(&ppm_args)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    Ok(Json(BridgeStatus {
        running: state.supervisor.running(),
        bin: state.supervisor.bin_display(),
        ppm_running: state.ppm_supervisor.running(),
        ppm_bin: state.ppm_supervisor.bin_display(),
    }))
}

async fn ppm_bridge_stop(State(state): State<Shared>) -> Json<BridgeStatus> {
    state.ppm_supervisor.stop();
    Json(BridgeStatus {
        running: state.supervisor.running(),
        bin: state.supervisor.bin_display(),
        ppm_running: false,
        ppm_bin: state.ppm_supervisor.bin_display(),
    })
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct TelemetryStatus {
    running: bool,
    bin: String,
    profile: telemetry::TelemetryProfile,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct MocapStatus {
    /// As stored, in whatever form the operator typed it.
    address: String,
    /// The Zenoh locator that address resolves to, so the operator can see
    /// what the default transport and port filled in.
    endpoint: Option<String>,
    /// Something answered on the far end. A configured address with
    /// `connected: false` is an address nothing is listening at.
    connected: bool,
    peers: usize,
    error: Option<String>,
}

fn mocap_status_of(state: &Shared) -> MocapStatus {
    let address = state
        .mocap
        .read()
        .expect("mocap lock poisoned")
        .address
        .clone();
    let link = state.command_authority.mocap_status();
    MocapStatus {
        address,
        endpoint: link.endpoint,
        connected: link.connected,
        peers: link.peers,
        error: link.error,
    }
}

async fn mocap_status(State(state): State<Shared>) -> Json<MocapStatus> {
    Json(mocap_status_of(&state))
}

/// Repoint the mocap link at a different capture machine.
///
/// The link is reopened in place; the address is persisted only once the link
/// has been applied, so a stored address always names the machine the ground
/// station is actually subscribed to. An unreachable address is not an error —
/// the capture system is routinely powered on after the ground station — so it
/// is stored and reported as configured-but-not-connected.
async fn put_mocap(
    State(state): State<Shared>,
    Json(profile): Json<MocapProfile>,
) -> Result<Json<MocapStatus>, (StatusCode, String)> {
    let address = profile.address.trim().to_string();
    if address.len() > 256 {
        return Err((StatusCode::BAD_REQUEST, "address too long".into()));
    }
    let profile = MocapProfile { address };
    let endpoint = profile.endpoint();
    if let Err(error) = state
        .command_authority
        .set_mocap_endpoint(endpoint.as_deref())
    {
        tracing::warn!(%error, "mocap link could not be opened");
    }
    {
        let mut current = state.mocap.write().expect("mocap lock poisoned");
        *current = profile;
        if let Err(error) = current.save(&state.mocap_file) {
            tracing::warn!(%error, "could not persist the mocap profile");
        }
    }
    Ok(Json(mocap_status_of(&state)))
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct MocapGnssRequest {
    enabled: bool,
}

fn telemetry_status_of(state: &Shared) -> TelemetryStatus {
    TelemetryStatus {
        running: state.telemetry_supervisor.running(),
        bin: state.telemetry_supervisor.bin_display(),
        profile: state
            .telemetry
            .read()
            .expect("telemetry lock poisoned")
            .clone(),
    }
}

async fn telemetry_status(State(state): State<Shared>) -> Json<TelemetryStatus> {
    Json(telemetry_status_of(&state))
}

/// Replace the telemetry profile. A running bridge is relaunched so a device
/// or baud change takes effect without the operator restarting the daemon.
async fn put_telemetry(
    State(state): State<Shared>,
    Json(profile): Json<telemetry::TelemetryProfile>,
) -> Result<Json<TelemetryStatus>, (StatusCode, String)> {
    let args = {
        let mut current = state.telemetry.write().expect("telemetry lock poisoned");
        *current = profile;
        current.bridge_args()
    };
    state
        .telemetry_supervisor
        .restart_if_running(&args)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    Ok(Json(telemetry_status_of(&state)))
}

async fn telemetry_start(
    State(state): State<Shared>,
) -> Result<Json<TelemetryStatus>, (StatusCode, String)> {
    let args = state
        .telemetry
        .read()
        .expect("telemetry lock poisoned")
        .bridge_args();
    state
        .telemetry_supervisor
        .start(&args)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    Ok(Json(telemetry_status_of(&state)))
}

async fn telemetry_stop(State(state): State<Shared>) -> Json<TelemetryStatus> {
    state.telemetry_supervisor.stop();
    Json(telemetry_status_of(&state))
}

/// Toggle the mocap GNSS uplink. The radio's serial port cannot be shared, so
/// the setting lives on the one bridge process and takes effect by relaunching
/// it — telemetry drops for the moment that takes.
async fn telemetry_set_mocap_gnss(
    State(state): State<Shared>,
    Json(request): Json<MocapGnssRequest>,
) -> Result<Json<TelemetryStatus>, (StatusCode, String)> {
    let args = {
        let mut profile = state.telemetry.write().expect("telemetry lock poisoned");
        profile.mocap_gnss = request.enabled;
        profile.bridge_args()
    };
    state
        .telemetry_supervisor
        .restart_if_running(&args)
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    Ok(Json(telemetry_status_of(&state)))
}

fn hostname() -> String {
    std::fs::read_to_string("/etc/hostname")
        .ok()
        .map(|name| name.trim().to_string())
        .filter(|name| !name.is_empty())
        .unwrap_or_else(|| "localhost".to_string())
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "electrode_ground_station=info".into()),
        )
        .init();

    let cli = Cli::parse();
    // The stored capture-machine address wins over the launch environment:
    // it is what the operator last set from the UI, and the env var is only
    // the seed for a machine that has never had one set.
    let mut mocap_profile = MocapProfile::load_or_default(&cli.mocap_file);
    let mut authority_config = CommandAuthorityConfig::from_env();
    match mocap_profile.endpoint() {
        Some(endpoint) => authority_config.telemetry_connect = Some(endpoint),
        // Nothing stored, but the environment named a capture machine. Show
        // that as the address rather than leaving the field blank next to a
        // live link — an operator reading "not configured" while mocap streams
        // has no way to tell which of the two is lying.
        None => {
            if let Some(seed) = authority_config.telemetry_connect.clone() {
                mocap_profile.address = seed;
            }
        }
    }
    let command_authority = CommandAuthority::start(authority_config)?;
    let zenoh_listeners = command_authority.listeners().to_vec();
    let autopilot_profile = AutopilotProfile::load_or_default(&cli.autopilot_file);
    let sim_bridge = sim_bridge::SimBridge::start(
        command_authority.vehicle_client()?,
        autopilot_profile.mocap_source,
    )?;

    let state: Shared = Arc::new(AppState {
        mapping: RwLock::new(MappingProfile::load_or_default(&cli.mapping_file)),
        mapping_file: cli.mapping_file.clone(),
        autopilot: RwLock::new(autopilot_profile),
        autopilot_file: cli.autopilot_file.clone(),
        simulation: RwLock::new(SimulationProfile::load_or_default(&cli.simulation_file)),
        simulation_file: cli.simulation_file.clone(),
        sim_bridge,
        supervisor: Supervisor::manual_control(),
        ppm_supervisor: Supervisor::ppm_bridge(),
        telemetry: RwLock::new(telemetry::TelemetryProfile::default()),
        telemetry_supervisor: Supervisor::telemetry_bridge(),
        mocap: RwLock::new(mocap_profile),
        mocap_file: cli.mocap_file.clone(),
        autopilot_link: AutopilotLink::new(),
        command_authority,
    });

    // The gcs/* API is same-origin in production; permissive CORS lets the Vite
    // dev server (a different port) probe it during development.
    let shutdown_state = state.clone();
    let gcs = Router::new()
        .route("/health", get(health))
        .route("/devices", get(devices))
        .route("/joystick", get(joystick::joystick_ws))
        .route("/mapping", get(get_mapping).put(put_mapping))
        .route("/autopilot", get(get_autopilot).put(put_autopilot))
        .route("/autopilot/status", get(autopilot_run_status))
        .route("/autopilot/start", post(autopilot_start))
        .route("/autopilot/stop", post(autopilot_stop))
        .route("/runtime/parameter", post(runtime_parameter))
        .route("/simulation", get(get_simulation).put(put_simulation))
        .route(
            "/simulation/model",
            get(simulation_model).put(put_simulation_model),
        )
        .route("/bridge", get(bridge_status))
        .route("/bridge/start", post(bridge_start))
        .route("/bridge/stop", post(bridge_stop))
        .route("/telemetry", get(telemetry_status).put(put_telemetry))
        .route("/telemetry/start", post(telemetry_start))
        .route("/telemetry/stop", post(telemetry_stop))
        .route("/telemetry/mocap-gnss", post(telemetry_set_mocap_gnss))
        .route("/mocap", get(mocap_status).put(put_mocap))
        .route("/ppm", get(bridge_status))
        .route("/ppm/start", post(ppm_bridge_start))
        .route("/ppm/stop", post(ppm_bridge_stop))
        .layer(tower_http::cors::CorsLayer::permissive())
        .with_state(state);

    // SPA static hosting: serve built assets and fall back to index.html for
    // client-rendered routes without masking missing JS/CSS/model assets.
    let index = cli.web_dir.join("index.html");
    let static_service = tower_http::services::ServeDir::new(&cli.web_dir).fallback(service_fn(
        move |req: Request<Body>| {
            let index = index.clone();
            serve_spa_fallback(index, req)
        },
    ));

    let app = Router::new()
        .nest("/gcs", gcs)
        .fallback_service(static_service);

    let listener = tokio::net::TcpListener::bind(cli.addr).await?;
    tracing::info!(addr = %cli.addr, web_dir = %cli.web_dir.display(), "electrode-ground-station listening");
    println!("\n  electrode Ground Station up:");
    println!(
        "    Ground Station (viewer + hardware):  http://{}/",
        cli.addr
    );
    if !zenoh_listeners.is_empty() {
        println!(
            "    Zenoh command authority: {}\n",
            zenoh_listeners.join(", ")
        );
    }
    axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal())
        .await?;
    shutdown_state.autopilot_link.stop_for_shutdown();
    shutdown_state.supervisor.stop();
    shutdown_state.ppm_supervisor.stop();
    Ok(())
}

async fn shutdown_signal() {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};

        let mut terminate = signal(SignalKind::terminate()).ok();
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {}
            _ = async {
                if let Some(signal) = terminate.as_mut() {
                    signal.recv().await;
                } else {
                    std::future::pending::<()>().await;
                }
            } => {}
        }
    }
    #[cfg(not(unix))]
    {
        let _ = tokio::signal::ctrl_c().await;
    }
}
