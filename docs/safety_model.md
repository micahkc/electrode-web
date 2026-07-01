# Command Safety Model

The web app sends operator intent. It does not directly publish raw Zenoh messages or actuator authority.

Command path:

```text
UI intent
  -> command builder
  -> schema and precondition checks
  -> operator confirmation when required
  -> bridge allowlist
  -> command publish
  -> ack, reject, or timeout shown in UI
```

The browser checks obvious operator-facing preconditions. The bridge remains the authority for allowlists, vehicle selection, command sequence monotonicity, expiration, and disconnect behavior.

In direct `zenoh` mode, the browser publishes command intents with `@cognipilot/zenoh-wasm` to the registered command key expression. That path is useful for early integration against a Zenoh WebSocket router, but it does not replace bridge-side authorization or vehicle-side ack/reject handling.

Initial commands:

- arm
- disarm
- set mode
- land
- return
- clear mission
- upload mission placeholder
- set parameter placeholder
