mod bicycle;
mod constants;
mod geometry;
mod map;
mod metrics;
mod sensors;
mod serial_out;
mod state;
mod ui;

use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use anyhow::Result;
use crossbeam_channel::bounded;
use rand::thread_rng;

use bicycle::{ControlInput, VehicleState, step_vehicle};
use map::LoadedMap;
use metrics::SimMetrics;
use sensors::{CameraEmulator, DistanceEmulator, EncoderEmulator, SimCommand};
use state::SimSnapshot;

fn main() -> Result<()> {
    let tick_hz = constants::sim_tick_hz();
    let map_path = constants::map_path();
    let map = LoadedMap::load_from_file(&map_path)?;
    let serial_port_name = constants::serial_port_name();
    let map_state = Arc::new(RwLock::new(map.clone()));

    let snapshot = Arc::new(RwLock::new(SimSnapshot::new(
        tick_hz,
        map.lines.len(),
        serial_port_name,
        map_path,
    )));
    let metrics = Arc::new(SimMetrics::new()?);

    let (tx_cmd, rx_cmd) = bounded::<String>(4_096);
    serial_out::spawn_serial_thread(rx_cmd, Arc::clone(&snapshot), Arc::clone(&metrics));
    ui::spawn_ui_thread(
        Arc::clone(&snapshot),
        Arc::clone(&metrics),
        Arc::clone(&map_state),
        tx_cmd.clone(),
    );

    run_sim_loop(map_state, snapshot, metrics, tx_cmd)
}

fn run_sim_loop(
    map: Arc<RwLock<LoadedMap>>,
    snapshot: Arc<RwLock<SimSnapshot>>,
    metrics: Arc<SimMetrics>,
    tx_cmd: crossbeam_channel::Sender<String>,
) -> Result<()> {
    let tick_hz = constants::sim_tick_hz();
    let dt_s = 1.0 / tick_hz as f32;
    let dt = Duration::from_secs_f64(dt_s as f64);

    let mut rng = thread_rng();
    let mut vehicle = VehicleState::default();
    let mut encoder = EncoderEmulator::new();
    let mut distance = DistanceEmulator::new();
    let mut camera = CameraEmulator::new();

    let mut next_tick = Instant::now();

    loop {
        let mut reset_now = false;
        if let Ok(mut lock) = snapshot.write() {
            if lock.reset_requested {
                lock.reset_requested = false;
                reset_now = true;
            }
        }

        if reset_now {
            vehicle = VehicleState::default();
            encoder = EncoderEmulator::new();
            distance = DistanceEmulator::new();
            camera = CameraEmulator::new();

            if let Ok(mut lock) = snapshot.write() {
                lock.sim_time_s = 0.0;
                lock.x_m = vehicle.pos.x;
                lock.y_m = vehicle.pos.y;
                lock.heading_deg = 0.0;
                lock.speed_mps = 0.0;
                lock.steer_pwm_us = vehicle.steer_pwm_us;
                lock.throttle_pwm_us = vehicle.throttle_pwm_us;
                lock.encoder_period_us = None;
                lock.dist_left_cm = f32::INFINITY;
                lock.dist_center_cm = f32::INFINITY;
                lock.dist_right_cm = f32::INFINITY;
                lock.camera_angle_deg = None;
                lock.camera_confidence = None;
                lock.camera_line_angle_deg = None;
                lock.dropped_encoder = 0;
                lock.dropped_distance = 0;
                lock.dropped_camera = 0;
                lock.serial_sent_commands = 0;
                lock.cmd_steer_pwm_us = constants::STEER_NEUTRAL_PWM_US;
                lock.cmd_throttle_pwm_us = constants::THROTTLE_NEUTRAL_PWM_US;
            }

            next_tick = Instant::now();
            continue;
        }

        let control = snapshot
            .read()
            .map(|lock| ControlInput {
                steer_pwm_us: lock.cmd_steer_pwm_us,
                throttle_pwm_us: lock.cmd_throttle_pwm_us,
            })
            .unwrap_or(ControlInput {
                steer_pwm_us: constants::STEER_NEUTRAL_PWM_US,
                throttle_pwm_us: constants::THROTTLE_NEUTRAL_PWM_US,
            });
        let ds = step_vehicle(&mut vehicle, control, dt_s);

        let (encoder_cmds, dropped_encoder) = encoder.update(vehicle.sim_time_s, ds, &mut rng);
        for cmd in encoder_cmds {
            publish_command(&tx_cmd, cmd, &snapshot);
        }
        if dropped_encoder > 0 {
            metrics.inc_dropped_encoder_by(dropped_encoder);
            if let Ok(mut lock) = snapshot.write() {
                lock.dropped_encoder = lock.dropped_encoder.saturating_add(dropped_encoder);
            }
        }

        let (distance_cmd, distance_values, dropped_distance) = match map.read() {
            Ok(map_lock) => distance.update(&map_lock, &vehicle, &mut rng),
            Err(_) => (None, [f32::INFINITY; 3], 0),
        };
        if let Some(cmd) = distance_cmd.clone() {
            publish_command(&tx_cmd, cmd, &snapshot);
        }
        if dropped_distance > 0 {
            metrics.inc_dropped_distance_by(dropped_distance);
            if let Ok(mut lock) = snapshot.write() {
                lock.dropped_distance = lock.dropped_distance.saturating_add(dropped_distance);
            }
        }

        let (camera_cmd, camera_angle, camera_confidence, camera_line_angle, dropped_camera) =
            camera.update(&vehicle, &mut rng);
        if let Some(cmd) = camera_cmd {
            publish_command(&tx_cmd, cmd, &snapshot);
        }
        if dropped_camera > 0 {
            metrics.inc_dropped_camera_by(dropped_camera);
            if let Ok(mut lock) = snapshot.write() {
                lock.dropped_camera = lock.dropped_camera.saturating_add(dropped_camera);
            }
        }

        if let Ok(mut lock) = snapshot.write() {
            lock.sim_time_s = vehicle.sim_time_s;
            lock.x_m = vehicle.pos.x;
            lock.y_m = vehicle.pos.y;
            lock.heading_deg = vehicle.heading_rad.to_degrees();
            lock.speed_mps = vehicle.speed_mps;
            lock.steer_pwm_us = vehicle.steer_pwm_us;
            lock.throttle_pwm_us = vehicle.throttle_pwm_us;
            if distance_cmd.is_some() {
                lock.dist_left_cm = distance_values[0];
                lock.dist_center_cm = distance_values[1];
                lock.dist_right_cm = distance_values[2];
            }
            lock.camera_angle_deg = camera_angle;
            lock.camera_confidence = camera_confidence;
            if let Some(la) = camera_line_angle {
                lock.camera_line_angle_deg = Some(la);
            }
        }

        if let Ok(lock) = snapshot.read() {
            metrics.sync_from_snapshot(&lock);
        }

        next_tick += dt;
        let now = Instant::now();
        if next_tick > now {
            std::thread::sleep(next_tick - now);
        } else {
            next_tick = now;
        }
    }
}

fn publish_command(
    tx_cmd: &crossbeam_channel::Sender<String>,
    cmd: SimCommand,
    snapshot: &Arc<RwLock<SimSnapshot>>,
) {
    let wire = cmd.to_wire();

    if let SimCommand::Encoder { period_us } = cmd {
        if let Ok(mut lock) = snapshot.write() {
            lock.encoder_period_us = Some(period_us);
        }
    }

    let _ = tx_cmd.try_send(wire);
}
