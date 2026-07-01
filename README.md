# electrode-web

`electrode-web` is the web-first ground station workspace for the `electrode` project. It follows the Cerebri GCS roadmap with a browser UI, a Rust/WASM protocol core, a TypeScript SDK, FlatBuffer schemas, and a native Rust bridge that exposes a browser-safe WebSocket endpoint.

## Current MVP

- SvelteKit web app with connection, status, map, commands, replay, events, plots, and topic inspector panels.
- TypeScript SDK with topic registry, latest-wins state store, command preconditions, Synapse `.sylg` log recording/replay, Zenoh WASM publishing, and transport types.
- Web Worker that runs simulator, Zenoh, bridge, and replay pipelines off the UI thread.
- Rust `electrode-web-core` crate for shared message validation and future WASM exports.
- Browser `@cognipilot/zenoh-wasm` integration for direct command-intent publishing to a Zenoh WebSocket endpoint.
- Rust `electrode-ground-bridge` crate with a WebSocket telemetry/command MVP fallback.
- Rust `electrode-manual-control-bridge` crate for Linux joystick/gamepad input to Synapse `ManualControl` over Zenoh.
- Rust `electrode-ppm-bridge` crate for Synapse manual/autopilot outputs to serial PPM encoder output.
- FlatBuffer schema files, BFBS schema assets, and generator script hooks.

## Quick Start

```bash
npm install
cargo build
npm run build
npm run dev
```

Run the bridge in another terminal when using bridge mode:

```bash
npm run bridge
```

Run the native manual-control bridge when using a Linux joystick/gamepad:

```bash
npm run manual:bridge -- --device /dev/input/js0
```

Inspect the published manual-control stream:

```bash
npm run manual:dump -- --topic synapse/manual_control
```

Run the native PPM bridge when using hardware-in-the-loop receiver output:

```bash
npm run ppm:bridge -- \
  --manual-topic synapse/manual_control \
  --control-output-topic synapse/control_output \
  --serial-device /dev/ttyACM0
```

For direct Zenoh WASM command publishing, run a Zenoh router with a WebSocket listener:

```bash
zenohd -l ws/0.0.0.0:7447
```

Then select `zenoh` mode in the app and use `ws/127.0.0.1:7447`.

The web app defaults to simulator mode, so it is useful without vehicle hardware.
