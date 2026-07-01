use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const SCHEMA_VERSION: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Priority {
    Low,
    Normal,
    High,
    Critical,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MessageHeader {
    pub sequence: u64,
    pub source_time_ns: u64,
    pub receive_time_ns: u64,
    pub expire_time_ns: u64,
    pub vehicle_id: String,
    pub schema_version: u16,
    pub message_type: String,
    pub priority: Priority,
    pub stream_id: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TelemetryEnvelope<T = serde_json::Value> {
    pub kind: String,
    pub topic: String,
    pub header: MessageHeader,
    pub payload: T,
}

#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ValidationError {
    #[error("schema version {actual} is not supported; expected {expected}")]
    UnsupportedSchema { expected: u16, actual: u16 },
    #[error("message expired at {expire_time_ns}; now is {now_ns}")]
    Expired { expire_time_ns: u64, now_ns: u64 },
    #[error("vehicle id is empty")]
    MissingVehicleId,
    #[error("stream id is empty")]
    MissingStreamId,
}

pub fn validate_header(header: &MessageHeader, now_ns: u64) -> Result<(), ValidationError> {
    if header.schema_version != SCHEMA_VERSION {
        return Err(ValidationError::UnsupportedSchema {
            expected: SCHEMA_VERSION,
            actual: header.schema_version,
        });
    }

    if header.expire_time_ns > 0 && header.expire_time_ns < now_ns {
        return Err(ValidationError::Expired {
            expire_time_ns: header.expire_time_ns,
            now_ns,
        });
    }

    if header.vehicle_id.trim().is_empty() {
        return Err(ValidationError::MissingVehicleId);
    }

    if header.stream_id.trim().is_empty() {
        return Err(ValidationError::MissingStreamId);
    }

    Ok(())
}

pub fn validate_json_frame(frame: &str, now_ns: u64) -> Result<(), String> {
    let envelope: TelemetryEnvelope = serde_json::from_str(frame).map_err(|err| err.to_string())?;
    validate_header(&envelope.header, now_ns).map_err(|err| err.to_string())
}

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(js_name = validateJsonFrame)]
pub fn validate_json_frame_wasm(frame: &str, now_ns: f64) -> String {
    match validate_json_frame(frame, now_ns as u64) {
        Ok(()) => String::new(),
        Err(err) => err,
    }
}
