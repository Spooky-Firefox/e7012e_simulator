use std::env;

pub const SIM_TICK_HZ_DEFAULT: u32 = 500;
pub const SIM_TICK_HZ_MAX: u32 = 1_000;

pub const WHEELBASE_M: f32 = 0.26;
pub const STEER_NEUTRAL_PWM_US: u16 = 1_500;
pub const STEER_ANGLE_PER_US_DEG: f32 = 0.24;
pub const STEER_MAX_DEG: f32 = 35.0;

pub const THROTTLE_NEUTRAL_PWM_US: u16 = 1_500;
pub const THROTTLE_DEADBAND_US: u16 = 25;
pub const THROTTLE_MAX_ACCEL_MPS2: f32 = 2.2;
pub const THROTTLE_MAX_BRAKE_MPS2: f32 = 3.0;
pub const THROTTLE_DRAG_LINEAR: f32 = 0.60;
pub const VEHICLE_MAX_SPEED_MPS: f32 = 2.5;

pub const ENCODER_DISTANCE_PER_PULSE_M: f32 = 3.03 / 100.0;
pub const ENCODER_PERIOD_JITTER_STD_US: f32 = 250.0;
pub const ENCODER_DROP_PROB: f32 = 0.01;
pub const ENCODER_TIMEOUT_S: f32 = 0.25;

pub const DIST_SENSOR_RATE_HZ: f32 = 20.0;
pub const DIST_SENSOR_ANGLES_DEG: [f32; 3] = [45.0, 0.0, -45.0];
pub const DIST_SENSOR_MIN_RANGE_M: f32 = 0.02;
pub const DIST_SENSOR_MAX_RANGE_M: f32 = 4.0;
pub const DIST_SENSOR_NOISE_STD_M: f32 = 0.01;
pub const DIST_SENSOR_DROP_PROB: f32 = 0.03;

pub const CAMERA_RATE_HZ: f32 = 7.0;
pub const CAMERA_ANGLE_NOISE_STD_DEG: f32 = 1.0;
pub const CAMERA_DROP_PROB: f32 = 0.05;
pub const CAMERA_MAX_ANGLE_DEG: f32 = 30.0;

pub const SERIAL_BAUD: u32 = 115_200;
pub const SERIAL_RETRY_MS: u64 = 1_000;

pub const METRICS_NAMESPACE: &str = "e7012e_sim";
pub const UI_BIND_DEFAULT: &str = "0.0.0.0:9093";

pub const DEFAULT_MAP_PATH: &str = "maps/default_map.toml";

pub fn sim_tick_hz() -> u32 {
    env::var("SIM_TICK_HZ")
        .ok()
        .and_then(|value| value.parse::<u32>().ok())
        .map(|value| value.clamp(1, SIM_TICK_HZ_MAX))
        .unwrap_or(SIM_TICK_HZ_DEFAULT)
}

pub fn map_path() -> String {
    env::var("SIM_MAP_PATH").unwrap_or_else(|_| DEFAULT_MAP_PATH.to_string())
}

pub fn ui_bind() -> String {
    env::var("SIM_UI_BIND").unwrap_or_else(|_| UI_BIND_DEFAULT.to_string())
}

pub fn serial_port_name() -> String {
    env::var("SIM_SERIAL_PORT").unwrap_or_else(|_| "/dev/ttyACM0".to_string())
}
