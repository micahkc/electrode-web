mod commands;
mod config;
mod log;
mod state;
mod topics;
mod websocket;

use axum::{extract::State, routing::get, Json, Router};
use tower_http::cors::CorsLayer;

use crate::{config::BridgeConfig, state::AppState};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    log::init_tracing();

    let config = BridgeConfig::from_env()?;
    let state = AppState::new(config.vehicle_id);
    let app = Router::new()
        .route("/health", get(health))
        .route("/api/topics", get(list_topics))
        .route("/ws", get(websocket::ws_handler))
        .layer(CorsLayer::permissive())
        .with_state(state);

    let listener = tokio::net::TcpListener::bind(config.bind_addr).await?;
    tracing::info!("electrode bridge listening on http://{}", config.bind_addr);
    axum::serve(listener, app).await?;
    Ok(())
}

async fn health() -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "status": "ok",
        "service": "electrode-ground-bridge"
    }))
}

async fn list_topics(State(state): State<AppState>) -> Json<Vec<topics::TopicDefinition>> {
    Json(topics::topic_definitions(&state.vehicle_id))
}
