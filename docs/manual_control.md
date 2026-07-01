# Manual Control and PPM Bridge

Electrode treats manual control as a first-class ground-station workflow with a native hardware boundary.

```text
gamepad / joystick / touch input
  -> electrode-manual-control-bridge
  -> Synapse ManualControl FlatBuffer
  -> Zenoh key expression: synapse/manual_control

autopilot control output
  -> CUBS2 RcChannels16 payload
  -> Zenoh key expression: synapse/control_output

synapse/manual_control + synapse/control_output
  -> electrode-ppm-bridge
  -> serial PPM encoder packet
  -> RC receiver
```

The browser or desktop UI owns input selection, arming of the manual-control session, calibration display, and operator feedback. Native bridge binaries own Linux joystick access, Zenoh publishing/subscribing, serial access, and the packet contract used by the encoder hardware.

## Manual Input Bridge

Run the joystick/gamepad bridge from the Electrode workspace:

```bash
npm run manual:bridge -- \
  --device /dev/input/js0 \
  --zenoh-connect udp/127.0.0.1:7447 \
  --topic synapse/manual_control
```

Useful options:

```text
--publish-hz 50
--stale-ms 250
--roll-axis 1
--pitch-axis 2
--yaw-axis 3
--throttle-axis 0
--mode-axis 4
--active-axis 5
--invert-active true
--arm-button INDEX
--kill-button INDEX
```

The bridge publishes `synapse.topic.ManualControl` FlatBuffers. The `active` switch selects manual passthrough in the PPM bridge; inactive but valid input selects autopilot output.

Use the dump tool to inspect published values:

```bash
npm run manual:dump -- --topic synapse/manual_control
```

## PPM Bridge

Run the PPM bridge from the Electrode workspace:

```bash
npm run ppm:bridge -- \
  --zenoh-connect udp/127.0.0.1:7447 \
  --manual-topic synapse/manual_control \
  --control-output-topic synapse/control_output \
  --serial-device /dev/ttyACM0 \
  --baud-rate 57600
```

Useful environment variables:

```bash
JOYSTICK_DEVICE=/dev/input/js0
ZENOH_CONNECT=udp/127.0.0.1:7447
ZENOH_TOPIC=synapse/manual_control
ZENOH_CONTROL_OUTPUT_TOPIC=synapse/control_output
PPM_SERIAL_DEVICE=/dev/ttyACM0
PPM_BAUD_RATE=57600
PPM_CHANNEL_MAP=1,2,0,3,4
```

The PPM bridge selection rules are:

```text
ManualControl valid=true, kill_switch=false, active=true  -> manual channels
ManualControl valid=true, kill_switch=false, active=false -> latest control_output channels
ManualControl valid=false or kill_switch=true             -> failsafe channels
```

Base channel order before `PPM_CHANNEL_MAP`:

```text
0 throttle
1 aileron / roll
2 elevator / pitch
3 rudder / yaw
4 mode
```

`synapse/control_output` is interpreted as the first five little-endian `i32` PWM values from a CUBS2 `RcChannels16` payload in this order:

```text
0 roll
1 pitch
2 throttle
3 yaw
4 mode
```

Those values are mapped into the PPM base order above. Pitch and yaw are inverted for the serial encoder convention, and all PWM values are clamped to `1000..=2000`.

The serial packet is 14 bytes:

```text
0xffff header, five little-endian u16 channels, little-endian u16 checksum
```

The checksum is the wrapping sum of the five transmitted channel values.

## Safety Boundary

Failsafe channels are:

```text
throttle 1000
roll     1500
pitch    1500
yaw      1500
mode     2000
```

Serial device access should remain in the native bridge or Tauri shell, not in the browser.
