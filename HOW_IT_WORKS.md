# How the Simulator Works

## Thread Model

The simulator splits work across three concurrent paths.

- Main thread: deterministic simulation loop
- Serial thread: serial reconnect, command output, and controller telemetry parsing
- HTTP thread: browser UI, JSON API, and Prometheus metrics

This keeps the simulation tick independent from serial stalls and HTTP polling.

## Startup Flow

On boot the simulator:

1. Reads the configured tick rate, map path, serial port, and UI bind address.
2. Loads the selected TOML map.
3. Creates a shared `SimSnapshot` protected by `RwLock`.
4. Starts the serial thread.
5. Starts the HTTP/UI thread.
6. Enters the fixed-rate simulation loop.

## Main Simulation Loop

`run_sim_loop` advances time at `1 / SIM_TICK_HZ` seconds per step.

Each tick does the same sequence:

1. Check whether the UI requested a reset.
2. Read the current commanded steer/throttle PWM values from the shared snapshot.
3. Step the vehicle model with `step_vehicle`.
4. Update encoder emulation and publish any resulting serial commands.
5. Update distance emulation against the current loaded map.
6. Update camera emulation from current vehicle heading.
7. Write the latest vehicle and sensor state back into the snapshot.
8. Sync Prometheus gauges from the snapshot.
9. Sleep until the next absolute tick deadline.

The loop uses an absolute `next_tick` deadline, so short timing slip does not accumulate into permanent drift.

## Vehicle Model

The simulated car state tracks:

- position `(x, y)` in metres
- heading in radians
- speed in metres per second
- current steer PWM and throttle PWM
- simulation time

The dynamics are implemented in `bicycle.rs` using a kinematic bicycle model with throttle acceleration, braking, and drag limits driven from PWM commands.

## Encoder Emulation

`EncoderEmulator` accumulates traveled distance until it crosses `ENCODER_DISTANCE_PER_PULSE_M`.

For each pulse:

- compute the period from the time since the last pulse
- add Gaussian jitter
- randomly drop some pulses
- emit `sim encoder <period_us>` for the pulses that survive

If too much time passes without another pulse, it emits `sim encoder-timeout` once.

## Distance Emulation

`DistanceEmulator` emits at `DIST_SENSOR_RATE_HZ` rather than every simulation tick.

For each emission:

- cast three rays from the vehicle heading using sensor angles `45 deg`, `0 deg`, and `-45 deg`
- intersect them against loaded map segments
- discard hits outside the configured HC-SR04 range window
- randomly drop some returns
- add distance noise
- emit `sim dist <left_cm> <center_cm> <right_cm>`

Missing values are represented as `f32::INFINITY` internally and serialized as `inf` in the outgoing command string.

## Camera Emulation

`CameraEmulator` also runs at its own rate.

The current implementation does not raycast to a tagged wall target. Instead it uses the vehicle heading itself:

- find the nearest major axis, meaning a multiple of `90 deg`
- compute signed heading offset from that axis in `[-45, 45]`
- suppress output when the offset exceeds `CAMERA_MAX_ANGLE_DEG`
- randomly drop some frames
- add angle noise
- compute confidence from proximity to the axis
- emit `align <angle_deg> <confidence>`

For UI/debugging it also records the selected major-axis angle as `camera_line_angle_deg`.

## Serial Thread

The serial thread has two jobs.

### Outbound path

- receive command strings from the bounded channel
- reconnect automatically when the selected serial port changes or the port drops
- write each command to the controller serial port
- count sent commands and errors in the snapshot and metrics

### Inbound path

The thread also reads controller output and parses several formats:

- `steer_us:` and `throttle_us:` key-value telemetry
- `steer_pwm_us=` and `throttle_pwm_us=` logging output
- CSV controller rows when the firmware uses `simple_csv`

From that stream it updates UI-visible controller state, including:

- commanded steering and throttle PWM seen from the controller
- steering setpoint
- PID error
- heading estimate
- observer covariance
- PID P term
- PID D term
- numeric drive mode

That makes the simulator UI useful as a controller introspection panel, not just a sensor generator.

## Fake-Car Mode

When `fake_car_enabled` is true, the serial thread skips opening a real serial port and simply consumes outbound commands from the channel. This exercises the simulator path without attached hardware.

## HTTP UI

The HTTP thread serves the embedded HTML UI and a small JSON API.

Important routes:

- `GET /api/state`: full serialized `SimSnapshot`
- `GET /api/config`: current port, fake-car flag, map path, map error, available ports, available maps
- `GET /api/map`: current map bounds and line geometry
- `POST /api/config`: update serial port and fake-car mode
- `POST /api/control`: update manual steer/throttle command values
- `POST /api/command`: inject an arbitrary command string
- `POST /api/load_map`: reload a map from disk at runtime
- `POST /api/reset`: reset simulation state and enqueue `reset` for the controller
- `GET /metrics`: Prometheus exposition output

## Shared Snapshot

`SimSnapshot` is the bridge across threads. It carries:

- current vehicle pose and speed
- latest sensor values
- drop counters
- serial connection and error counters
- controller telemetry parsed from serial input
- UI-selected command values
- serial port and fake-car configuration
- current map metadata and load errors
- reset request flag

The UI reads it, the serial thread updates serial-related fields, and the simulation loop updates the physics and sensor fields.

## Maps

Maps are loaded from TOML into `LoadedMap`.

Each `[[line]]` entry becomes a `MapLine` with:

- an `id`
- a `kind`
- a 2D segment

The line `kind` is mainly exposed to the UI today. Distance sensing uses all lines generically through segment raycasting.