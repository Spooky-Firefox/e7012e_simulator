# How It Works

## Thread model

The simulator runs three concurrent paths:

1. Simulation loop thread (main thread)
2. Serial output thread
3. HTTP UI/metrics thread

This keeps time-critical simulation updates independent from serial stalls and web polling.

## Simulation loop

The loop runs at fixed dt = 1 / SIM_TICK_HZ from src/constants.rs.

At each tick:

1. Compute control input profile (steer and throttle PWM)
2. Integrate kinematic bicycle model
3. Update sensor emulators (encoder, distance, camera)
4. Publish command strings to serial queue
5. Update shared snapshot and Prometheus gauges

## Bicycle model

State:

- x, y position (m)
- heading (rad)
- speed (m/s)

Model:

- x_dot = v cos(theta)
- y_dot = v sin(theta)
- theta_dot = v / L * tan(delta)
- v_dot = a_cmd - drag(v)

Where:

- L is wheelbase
- delta comes from steering PWM mapping
- a_cmd comes from throttle PWM mapping

## Map geometry

Map is a TOML list of line segments in meters.

Distance sensors raycast against segments and keep nearest hit within range limits.

Camera emulation uses nearest vertical/horizontal tagged segment to produce alignment angle.

## Sensor emulation

### Encoder

- Converts traveled distance to pulses using ENCODER_DISTANCE_PER_PULSE_M
- Emits sim encoder <period_us>
- Applies period jitter and random drop probability
- Emits sim encoder-timeout when pulse silence exceeds ENCODER_TIMEOUT_S

### Distance

- 3 rays at configurable mounting angles
- Raycast nearest line hit
- Drops value when out of sensor spec range
- Applies random dropout and additive noise
- Emits sim dist <left_cm> <center_cm> <right_cm>
- Missing measurements are represented as inf

### Camera

- Chooses nearest vertical/horizontal line
- Computes relative heading angle
- Clamps to plus/minus 30 degrees
- Applies noise and optional dropout
- Emits align <angle_deg> <confidence>

## RP2350 integration

The simulator serial stream is intended for rp2350_controller built with simulation feature.

Expected commands:

- sim encoder
- sim encoder-timeout
- sim dist
- align

## Metrics and dashboard

Prometheus metrics are exported at /metrics.

A starter Grafana dashboard is provided in monitor/grafana/dashboards/simulator.json.
