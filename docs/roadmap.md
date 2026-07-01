# electrode Roadmap

This workspace implements the first practical vertical slice from the Cerebri web GCS roadmap under the project name `electrode`.

## Phase 0

- Architecture, topic conventions, message lifecycle, and safety documentation.

## Phase 1

- npm and Cargo workspaces.
- SvelteKit app.
- Rust/WASM protocol crate.
- TypeScript SDK package.
- Native bridge crate.
- FlatBuffer schema generation hook.

## Phase 2

- Common, state, command, event, mission, and parameter schemas.
- Typed topic registry.
- Schema-version validation in Rust and TypeScript.

## Phase 3

- Native bridge MVP with browser-safe WebSocket endpoint.
- Direct browser command publishing with `@cognipilot/zenoh-wasm`.
- Topic and command allowlists in bridge mode.
- Synthetic bridge telemetry until native Zenoh telemetry integration or WASM subscriber support is wired.

## Phase 4

- Browser worker for WebSocket, replay, latest-wins state, stale-topic tracking, message rates, and latency.

## Phase 5

- Operator-useful GCS: connection, vehicle status, map, events, topic inspector, and command panel.

Later phases add real Zenoh integration, log persistence, mission editing, parameters, PURT workflows, Tauri packaging, and measured low-latency side channels.
