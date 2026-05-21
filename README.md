# E7012E Simulator

Standalone simulator for the RP2350 controller. It runs a kinematic bicycle model, emulates the controller's encoder, ultrasonic, and camera inputs, pushes those inputs over serial as `sim ...` and `align ...` commands, and exposes a browser UI plus Prometheus metrics.

## Current Behavior

- Fixed-rate simulation loop on the main thread
- Separate serial thread for USB I/O and controller telemetry parsing
- Separate HTTP thread serving the UI, API endpoints, and Prometheus metrics
- Live map loading from TOML files
- UI controls for steer/throttle command overrides, reset, serial port selection, fake-car mode, and raw command injection

The simulator is intended to pair with `rp2350_controller` built with the `simulation` feature.

## What It Emits

The serial output path can generate:

- `sim encoder <period_us>`
- `sim encoder-timeout`
- `sim dist <left_cm> <center_cm> <right_cm>`
- `align <angle_deg> <confidence>`

The serial input path also parses controller telemetry so the UI can show what the firmware is commanding and estimating.

## Run

```sh
cargo run
```

Useful overrides:

```sh
SIM_TICK_HZ=1000 \
SIM_SERIAL_PORT=/dev/ttyACM0 \
SIM_MAP_PATH=maps/default_map.toml \
SIM_UI_BIND=0.0.0.0:9093 \
cargo run
```

## Environment Variables

- `SIM_TICK_HZ`: simulation rate, clamped to `1..1000`, default `500`
- `SIM_SERIAL_PORT`: serial device path, default `/dev/ttyACM0`
- `SIM_MAP_PATH`: map file path, default `maps/default_map.toml`
- `SIM_UI_BIND`: HTTP bind address, default `0.0.0.0:9093`

## UI And API

Default endpoints:

- `/`: browser UI
- `/api/state`: current simulator snapshot
- `/api/config`: current config plus available serial ports and map files
- `/api/map`: current loaded map geometry and bounds
- `/api/control`: set steer and throttle override PWM values
- `/api/command`: send an arbitrary raw command to the controller
- `/api/load_map`: load a different TOML map at runtime
- `/api/reset`: reset the simulator state and send `reset` to the controller
- `/metrics`: Prometheus metrics

## Fake-Car Mode

When fake-car mode is enabled in the UI, the serial thread stops trying to open a real serial port. Commands are still consumed from the queue and counted, which makes it useful for exercising the UI and metrics path without hardware connected.

## Sensor Models

- Encoder: converts traveled distance to pulse periods, adds Gaussian jitter, drops some pulses, and emits timeout events after inactivity
- Distance: casts three rays at `45 deg`, `0 deg`, and `-45 deg`, adds noise, and emits `inf` for missing returns
- Camera: measures heading relative to the nearest major axis, clamps to `+/-30 deg`, adds noise, computes confidence, and can drop frames

## Map Format

Maps are TOML files with repeated `[[line]]` entries.

Fields:

- `id`: optional string
- `kind`: `generic`, `vertical`, or `horizontal`
- `x1`, `y1`, `x2`, `y2`: segment endpoints in metres

Example maps live in `maps/`.

## Metrics

The simulator exports Prometheus metrics under the `e7012e_sim` namespace, including:

- vehicle position, heading, and speed
- current distance readings
- last encoder period
- last camera angle and confidence
- dropped encoder, distance, and camera events
- serial connection state, sent commands, and serial errors

## Source Layout

- `src/main.rs`: startup and simulation loop
- `src/bicycle.rs`: vehicle dynamics and PWM-to-motion mapping
- `src/sensors.rs`: encoder, distance, and camera emulators
- `src/serial_out.rs`: serial writer plus controller telemetry parser
- `src/ui.rs`: HTTP server and JSON endpoints
- `src/state.rs`: shared UI snapshot
- `src/map.rs`: TOML map loading and raycasting support
- `src/metrics.rs`: Prometheus metric registration and updates
- `src/constants.rs`: tuning and environment defaults

## Related Docs

- [HOW_IT_WORKS.md](HOW_IT_WORKS.md): thread model, update loop, serial flow, and UI/API behavior