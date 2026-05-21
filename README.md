# E7012E Simulator

Standalone bicycle-model simulator for generating RP2350-compatible sensor inputs over USB serial.

## What it does

- Runs a kinematic bicycle simulation at configurable high rate (default 500 Hz, max 1000 Hz)
- Loads a map from TOML line geometry
- Emulates:
  - Encoder timing measurements with distortion and timeout behavior
  - 3-ray distance sensor measurements with raycasting, range/spec filtering, noise, and random dropouts
  - Camera alignment measurements clamped to plus/minus 30 degrees using nearest vertical/horizontal map line
- Sends commands to the RP2350 simulation interface:
  - sim encoder
  - sim encoder-timeout
  - sim dist
  - align
- Exposes:
  - HTTP UI (default 127.0.0.1:9093)
  - Prometheus metrics at /metrics
  - Grafana dashboard template in monitor/grafana/dashboards/simulator.json

## Constants

All simulator constants and environment overrides are in src/constants.rs for quick access.

## Map format

Map file is TOML and consists of [[line]] entries:

- id: optional string
- kind: generic | vertical | horizontal
- x1, y1, x2, y2: endpoints in meters

See maps/default_map.toml.

## Run

1. Build:

```sh
cargo build
```

2. Run with default settings:

```sh
cargo run
```

3. Run with custom settings:

```sh
SIM_TICK_HZ=1000 SIM_SERIAL_PORT=/dev/ttyACM0 SIM_MAP_PATH=maps/default_map.toml cargo run
```

## Metrics

Scrape:

- http://127.0.0.1:9093/metrics

Key metrics:

- e7012e_sim_speed_mps
- e7012e_sim_heading_deg
- e7012e_sim_distance_left_cm
- e7012e_sim_distance_center_cm
- e7012e_sim_distance_right_cm
- e7012e_sim_encoder_period_us
- e7012e_sim_camera_angle_deg
- e7012e_sim_camera_confidence
- e7012e_sim_serial_connected
- e7012e_sim_serial_sent_commands_total
- e7012e_sim_serial_errors_total

## Notes

- UI and serial I/O run on separate threads to avoid blocking the simulation loop.
- Sensor distortion/noise/dropout behavior is isolated in sensor emulators for easy tuning.
