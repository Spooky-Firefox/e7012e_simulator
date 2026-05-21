# Changelog

This file tracks notable simulator repository changes.

## spooky-firefox 2026-05-21

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
