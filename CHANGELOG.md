# Changelog

This file tracks notable simulator repository changes.

## spooky-firefox 2026-05-21

### Wall-Correction Telemetry Through Metrics Stack

- Extended controller telemetry parsing to ingest:
	- `wall_left_deg`
	- `wall_right_deg`
	- `wall_combined_deg`
- Added shared snapshot fields for wall-correction telemetry so the UI/API can expose them.
- Added Prometheus gauges for each wall-correction component.
- Updated Grafana dashboard panels/queries to chart the new wall-correction metrics.
- Added additional track presets for simulator runs:
	- `maps/loop_90deg_corridor.toml`
	- `maps/race_square_35.toml`
	- `maps/race_square_35_blocks.toml`

Files changed:

- `src/serial_out.rs`
- `src/state.rs`
- `src/metrics.rs`
- `monitor/grafana/dashboards/simulator.json`
- `maps/loop_90deg_corridor.toml`
- `maps/race_square_35.toml`
- `maps/race_square_35_blocks.toml`
- `README.md`
- `HOW_IT_WORKS.md`
- `CHANGELOG.md`

### Camera Delay Modeling And UI Path Trail

- Added delayed camera frame emission in `CameraEmulator` using a sampled latency per frame.
- Extended emitted `align` command payloads to include the simulated `delay_ms`.
- Updated steering conversion constant to `0.24 deg/us` for simulator dynamics and UI consistency.
- Added breadcrumb path-trail rendering in the UI with heading and steering vectors.
- Switched vehicle rendering to physical meter-based geometry projected by map scale.
- Updated simulator docs to reflect current behavior.

Files changed:

- `src/constants.rs`
- `src/sensors.rs`
- `src/ui_index.html`
- `README.md`
- `HOW_IT_WORKS.md`
- `CHANGELOG.md`
