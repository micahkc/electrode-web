use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
};
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::time::{interval, Duration};

use crate::{
    commands::{handle_command, CommandIntent},
    state::AppState,
};

pub async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

async fn handle_socket(socket: WebSocket, state: AppState) {
    let (mut sender, mut receiver) = socket.split();
    let mut tick = interval(Duration::from_millis(100));
    let mut command_sequence = 0_u64;

    loop {
        tokio::select! {
            _ = tick.tick() => {
                for frame in simulated_frames(&state) {
                    match serde_json::to_string(&frame) {
                        Ok(text) => {
                            if sender.send(Message::Text(text.into())).await.is_err() {
                                return;
                            }
                        }
                        Err(err) => {
                            tracing::warn!(%err, "failed to serialize telemetry frame");
                        }
                    }
                }
            }
            incoming = receiver.next() => {
                let Some(Ok(message)) = incoming else {
                    return;
                };

                match message {
                    Message::Text(text) => {
                        let now_ms = now_ms();
                        let ack = match serde_json::from_str::<CommandIntent>(&text) {
                            Ok(intent) => handle_command(&state, intent, now_ms),
                            Err(err) => {
                                command_sequence += 1;
                                crate::commands::CommandAck {
                                    kind: "commandAck",
                                    command_id: format!("parse-error-{command_sequence}"),
                                    command: "unknown".to_string(),
                                    status: "rejected",
                                    reason: err.to_string(),
                                    sequence: command_sequence,
                                    received_at_ms: now_ms,
                                }
                            }
                        };

                        if let Ok(text) = serde_json::to_string(&ack) {
                            if sender.send(Message::Text(text.into())).await.is_err() {
                                return;
                            }
                        }
                    }
                    Message::Close(_) => return,
                    _ => {}
                }
            }
        }
    }
}

fn simulated_frames(state: &AppState) -> Vec<Value> {
    let now_ms = now_ms();
    let elapsed_s = state.started_at.elapsed().as_secs_f64();
    let radius_m = 42.0;
    let x_m = (elapsed_s / 8.0).cos() * radius_m;
    let y_m = (elapsed_s / 8.0).sin() * radius_m;
    let alt_m = 18.0 + (elapsed_s / 5.0).sin() * 3.0;
    let lat = 40.4237 + y_m / 111_111.0;
    let lon = -86.9212 + x_m / (111_111.0 * 40.4237_f64.to_radians().cos());
    let vehicle_id = &state.vehicle_id;
    let runtime = state.runtime();

    vec![
        frame(
            state,
            "pose",
            "Pose",
            format!("vehicle/{vehicle_id}/state/pose"),
            now_ms,
            250,
            json!({
                "lat": lat,
                "lon": lon,
                "altM": alt_m,
                "xM": x_m,
                "yM": y_m,
                "zM": -alt_m
            }),
        ),
        frame(
            state,
            "velocity",
            "Velocity",
            format!("vehicle/{vehicle_id}/state/velocity"),
            now_ms,
            250,
            json!({
                "northMps": (elapsed_s / 4.0).cos() * 1.8,
                "eastMps": -(elapsed_s / 4.0).sin() * 1.8,
                "downMps": (elapsed_s / 3.0).cos() * 0.2,
                "groundSpeedMps": 1.8
            }),
        ),
        frame(
            state,
            "attitude",
            "Attitude",
            format!("vehicle/{vehicle_id}/state/attitude"),
            now_ms,
            220,
            json!({
                "rollDeg": (elapsed_s * 1.3).sin() * 12.0,
                "pitchDeg": (elapsed_s * 0.9).cos() * 8.0,
                "yawDeg": (elapsed_s * 18.0).rem_euclid(360.0)
            }),
        ),
        frame(
            state,
            "battery",
            "Battery",
            format!("vehicle/{vehicle_id}/state/battery"),
            now_ms,
            2500,
            json!({
                "voltageV": 15.9 - elapsed_s * 0.002,
                "currentA": 4.5 + elapsed_s.sin() * 1.2,
                "remainingPct": (94.0 - elapsed_s * 0.05).max(22.0)
            }),
        ),
        frame(
            state,
            "link",
            "LinkStatus",
            format!("vehicle/{vehicle_id}/state/link"),
            now_ms,
            2500,
            json!({
                "rssiDbm": -49.0 - (elapsed_s / 4.0).sin() * 5.0,
                "latencyMs": 23.0 + (elapsed_s / 2.0).sin() * 5.0,
                "packetLossPct": ((elapsed_s / 5.0).sin() * 1.8).max(0.0)
            }),
        ),
        frame(
            state,
            "mode",
            "ModeState",
            format!("vehicle/{vehicle_id}/state/mode"),
            now_ms,
            2500,
            json!({
                "name": runtime.mode,
                "armed": runtime.armed,
                "failsafe": runtime.failsafe
            }),
        ),
        frame(
            state,
            "localization",
            "LocalizationState",
            format!("vehicle/{vehicle_id}/state/localization"),
            now_ms,
            1200,
            json!({
                "source": "mocap+ekf",
                "fresh": true,
                "quality": 0.91 + (elapsed_s / 6.0).sin() * 0.05,
                "updatedAtMs": now_ms
            }),
        ),
    ]
}

fn frame(
    state: &AppState,
    stream_id: &str,
    message_type: &str,
    topic: String,
    now_ms: u64,
    ttl_ms: u64,
    payload: Value,
) -> Value {
    let sequence = state.next_telemetry_sequence();
    json!({
        "kind": "telemetry",
        "topic": topic,
        "header": {
            "sequence": sequence,
            "sourceTimeNs": now_ms * 1_000_000,
            "receiveTimeNs": now_ms * 1_000_000,
            "expireTimeNs": (now_ms + ttl_ms) * 1_000_000,
            "vehicleId": state.vehicle_id,
            "schemaVersion": electrode_web_core::SCHEMA_VERSION,
            "messageType": message_type,
            "priority": "normal",
            "streamId": stream_id
        },
        "payload": payload
    })
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock is before unix epoch")
        .as_millis() as u64
}
