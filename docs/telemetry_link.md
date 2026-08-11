# RDD2 Telemetry Link

Electrode's ground-side half of the CogniPilot `csyn_serial` transport: the
vehicle streams Synapse topics over a telemetry radio, and
`electrode-telemetry-bridge` puts them onto Zenoh.

```text
RDD2 vehicle (subsys/csyn_serial)
  -> SiK radio pair, 57600 8N1, transparent serial
  -> electrode-telemetry-bridge
  -> Synapse bare structs on canonical Zenoh keys: health, gnss, att, att_sp, pwm, loop
  -> the same state store, plots, map and MCAP logging the simulator feeds
```

Payloads cross the bridge byte-for-byte. The wire payload is already a bare
fixed-layout `synapse.topic.*Data` struct — the exact image every other
Electrode consumer decodes — so there is no re-encoding step to corrupt.

## Running It

```bash
npm run telemetry:bridge -- --serial-device /dev/ttyUSB0
```

Useful flags:

| Flag | Purpose |
|---|---|
| `--namespace cub1` | Prefix every published key, e.g. `cub1/att` |
| `--no-zenoh` | Decode and report without publishing |
| `--verbose` | Print every decoded frame |
| `--status-interval-secs` | Seconds between link summaries (0 disables) |

`--no-zenoh --verbose` is the monitor mode: it runs the identical decode path,
so what it prints is exactly what would have been published.

## Frame Format

Little-endian throughout. Overhead is 10 bytes per frame.

```text
off  size  field
 0     2   sync      0x53 0x59  ('S','Y')
 2     2   len       u16, payload byte count
 4     2   topic_id  u16, synapse catalog TopicId
 6     1   seq       u8, wraps at 256, increments per frame sent
 7     1   flags     u8, reserved, currently always 0
 8     N   payload   the struct image, exactly `len` bytes
8+N    2   crc16     over bytes [2, 8+N)
```

The CRC is CRC-16/CCITT-FALSE: polynomial `0x1021`, init `0xFFFF`, no
reflection, no final XOR. Check value for `"123456789"` is `0x29B1`.

Three decoder properties matter more than they look:

- **The CRC is validated before any field is read.** The vehicle does this too;
  a decoder that skips it eventually acts on line noise.
- **A rejected candidate is rescanned, not discarded.** A corrupted `len` makes
  the parser over-read across a frame boundary, so the bytes it swallowed may
  contain the start of a real frame. Discarding them loses the frame behind
  every corrupted one.
- **A `len` above 128 is rejected immediately** (`CONFIG_RDD2_CSYN_SERIAL_MAX_PAYLOAD`).
  Waiting for the bogus payload would swallow the frame behind it, so being
  permissive here is not permissive, it is lossy.

`seq` is a single counter across all topics, so a gap means the link dropped a
frame, not that one topic went quiet.

## Downlink Topics

Six topics in the default vehicle build, each rate-limited to 5 Hz and sent only
when its value changes — roughly 1.7 KB/s, about 29% of a 57600 link.

| Topic | id | Key | Payload | Notes |
|---|---|---|---|---|
| `VehicleHealth` | 1 | `health` | 48 B | arming, mode, RC link, sensor bitmasks |
| `GnssFix` | 8 | `gnss/<id>` | 64 B | only when built for the onboard receiver |
| `AttitudeEstimate` | 11 | `att` | 40 B | estimated attitude and rates |
| `AttitudeCommand` | 19 | `att_sp` | 48 B | desired attitude and rates |
| `PwmSignalOutputs` | 25 | `pwm` | 48 B | motor outputs |
| `ControlLoopMetrics` | 26 | `loop` | 24 B | hot-path timing |

Unknown topic ids are skipped and counted, never treated as a fault: the
vehicle's topic list is application-owned and may grow.

## Reading The Telemetry Honestly

The wire format has no "this field is populated" bit, so several fields sit at
zero on a vehicle that does not measure them. Zero is indistinguishable from a
real reading, and renders as one. Electrode's decoders return `null` for these
instead, and the UI shows "not reported" rather than a plausible number.

**GNSS position is gated on `fix_type`, not on a flag.** There is no
position-valid bit. The vehicle publishes fixes while the receiver is still
acquiring — deliberately, so "receiver alive, acquiring" is distinguishable from
"receiver silent" — and those samples carry a meaningless latitude and
longitude, usually zero, which plots off West Africa. Any sample below `Fix2d`
is treated as carrying no position at all.

**Saturated accuracies mean unusable, not large.** The schema defines `65535` as
"at or above 65.535 m". The generic NMEA driver reports no accuracy and
saturates `horizontal_accuracy_mm`, `vertical_accuracy_mm` and
`velocity_accuracy_mm_s`. `yaw_accuracy_cdeg` is the exception: it stays at 0,
which would read as a *perfect* heading, so it is reported only when `YawValid`
is set.

**The sensor bitmask is the authoritative failsafe indicator.** RDD2 sets the
`Armed` flag but never `Failsafe`. What it does report is `sensors_health`:
`MotorOutputs` and `Estimator` are always set, `Gyro|Accel` only while the IMU
delivers samples, and `RadioControl` only while RC is valid and not stale. Any
enabled component missing from `sensors_health` raises failsafe in Electrode,
and the failed components are named in `unhealthy_sensors`.

**There is no battery telemetry.** RDD2 has no battery monitor in its published
state, so voltage, current and remaining percent are absent. They are reported
only when the vehicle advertises a `Battery` component, so the gauge shows no
data rather than a flat pack. CPU load, comm drop rate, error counters,
`vehicle_type`, `system_state`, `thrust`, `type_mask` and `overrun_count` are
likewise unpopulated.

## Liveness And Diagnosis

The link has no handshake, no heartbeat and no request/response. The vehicle
streams unsolicited frames, so liveness is "a decoded frame in the last
`--link-timeout-ms`" and nothing more.

The periodic summary is shaped like the vehicle shell's `csyn_serial status`, so
both ends of a bad link can be compared line for line:

```text
rx  frames=60 crc_err=0 bad_len=0 unknown=0 size_err=0
rx  seq_gaps=0 dropped=0 resyncs=0 link=up (0.2s ago)
rx  AttitudeEstimate=20 ControlLoopMetrics=20 GnssFix=5 VehicleHealth=5
```

Absent `GnssFix` is not a fault. A vehicle built `-S mocap-gnss` expects the
ground station to supply a fix and telemeters none; the bridge says so once
rather than reporting a failure.

## Build Variants

The vehicle picks its GNSS source at build time, and the direction of the `gnss`
topic follows:

| Vehicle build | `GnssFix` direction | Effect on the ground station |
|---|---|---|
| default (onboard receiver) | outbound | GNSS arrives as telemetry; injection rejected |
| `-S mocap-gnss` | inbound | GNSS must be supplied; none is telemetered |

The uplink set in the default build is `ManualControlCommand` (id 4) and
`InertialSample` (id 5, simulation only). Manual control is wired for UDP
links only — see below; inertial injection is not wired here.

## UDP Link Mode (WiFi bridge)

`--udp-address HOST:PORT` (or `TELEMETRY_UDP_ADDRESS`) replaces the serial
port with a connected UDP socket, for vehicles whose "radio" is a transparent
UART↔UDP WiFi bridge such as the micro-quad's ESP32 access point
(`192.168.4.1:14550` by default). The framing is identical; the decoder does
not care what carries the bytes.

Because a UDP socket, unlike a serial port, can be shared across threads,
`--manual-uplink` additionally subscribes to the Zenoh `manual` topic (bare
`ManualControlData`, already policy-gated by the command authority) and
frames each sample up the link as RC, transmitted directly from the
subscriber callback for minimum latency. The vehicle side needs
`CONFIG_RDD2_RC_SYNAPSE`; its RC staleness failsafe expects a steady stream,
so keep the joystick bridge or the browser's virtual transmitter running
while armed. `--manual-uplink` is refused on serial links, where RC is the
CRSF/PPM path.

## Mocap GNSS Uplink

A serial port cannot be shared, so the bridge owns the radio in both
directions: it decodes what the vehicle streams and, with `--mocap-gnss`,
converts mocap pose into `GnssFixData` and sends it back up the same port.

```bash
npm run telemetry:bridge -- --serial-device /dev/ttyUSB1 \
  --mocap-gnss --mocap-namespace cub1 --mocap-instance 0
```

It subscribes to `external_pose` (and `external_pose_cov` unless
`--no-covariance`), and sends a fix at `--uplink-rate-hz`. The conversion is a
port of `sample_to_fix`/`build_gnss_fix` from the vehicle tree's
`test_scripts/publish_gps_zenoh.py`, checked byte for byte against that
implementation in the tests — so an origin, `yaw_offset` or `heading_offset`
trimmed against those scripts stays correct here. The defaults are those
scripts' defaults for the same reason.

Three behaviours are worth knowing, because each is the honest answer to a
failure rather than a convenience:

- **Tracking loss is reported, not hidden.** `Lost` — or no sample for
  `--uplink-timeout-ms` — goes out as `fix_type=NoFix` with accuracy saturated
  to 65535 mm. A frozen position at full confidence is worse than no fix,
  because the estimator has no way to tell it is stale.
- **Degraded tracking scales the accuracy** by `--degraded-scale` rather than
  being dropped.
- **Accuracy is real when it can be.** It comes from the position block of
  `external_pose_cov`, falling back to `--hacc`/`--vacc`.

Nothing acknowledges an injected fix, so the bridge reports only what it sent:

```text
tx  gnss sent=15 nofix=0 from cub1/external_pose/0 (received=149)
```

Acceptance is confirmed on the vehicle, with `csyn topic echo gnss` and the
`wrong_dir` counter in `csyn_serial status`.

**This needs a vehicle built `-S mocap-gnss`.** The default build declares
`gnss` outbound and drops an injected fix into `wrong_dir` rather than
accepting it.

### The toggle

Under `electrode-ground-station` the daemon supervises the bridge and the
Ground Station strip carries a **Mocap GPS** button. Turning it on when no
bridge is running starts one, since the uplink is a flag on the process that
owns the radio rather than a separate child. For the same reason toggling
relaunches the bridge, so telemetry drops for the moment that takes.

| Route | Purpose |
|---|---|
| `GET gcs/telemetry` | bridge state and profile |
| `POST gcs/telemetry/start` \| `/stop` | run or stop the bridge |
| `POST gcs/telemetry/mocap-gnss` | `{"enabled": bool}` |

## Reference Implementation

The vehicle tree ships `tools/synapse_serial/synapse_serial.py`, the peer this
decoder is checked against. Its encoder output is committed as a golden vector
in the bridge's tests, so a framing or field-offset regression fails the build
rather than the flight.
