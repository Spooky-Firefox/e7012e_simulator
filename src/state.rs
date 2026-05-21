use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct SimSnapshot {
    pub tick_hz: u32,
    pub sim_time_s: f32,
    pub x_m: f32,
    pub y_m: f32,
    pub heading_deg: f32,
    pub speed_mps: f32,
    pub steer_pwm_us: u16,
    pub throttle_pwm_us: u16,
    pub encoder_period_us: Option<f32>,
    pub dist_left_cm: f32,
    pub dist_center_cm: f32,
    pub dist_right_cm: f32,
    pub camera_angle_deg: Option<f32>,
    pub camera_confidence: Option<f32>,
    /// Angle of the nearest reference map line used for camera alignment [deg]
    pub camera_line_angle_deg: Option<f32>,
    pub dropped_encoder: u64,
    pub dropped_distance: u64,
    pub dropped_camera: u64,
    pub serial_connected: bool,
    pub serial_errors: u64,
    pub serial_sent_commands: u64,
    pub serial_rx_lines: u64,
    pub serial_rx_steer_pwm_us: Option<u16>,
    pub serial_rx_throttle_pwm_us: Option<u16>,
    /// Angle setpoint the controller is targeting [deg] (from `setpoint` field in telemetry)
    pub serial_rx_angle_setpoint_deg: Option<f32>,
    /// PID error (setpoint - measured) from controller telemetry [deg]
    pub serial_rx_pid_error: Option<f32>,
    /// Heading observer estimate from controller telemetry [deg]
    pub serial_rx_heading_estimate_deg: Option<f32>,
    /// Heading observer covariance from controller telemetry [deg²]
    pub serial_rx_observer_covariance: Option<f32>,
    /// PID proportional term from controller telemetry
    pub serial_rx_pid_p: Option<f32>,
    /// PID derivative term from controller telemetry
    pub serial_rx_pid_d: Option<f32>,
    /// Left-wall centering correction contribution from controller telemetry [deg]
    pub serial_rx_wall_left_correction_deg: Option<f32>,
    /// Right-wall centering correction contribution from controller telemetry [deg]
    pub serial_rx_wall_right_correction_deg: Option<f32>,
    /// Combined wall-centering correction from controller telemetry [deg]
    pub serial_rx_wall_combined_correction_deg: Option<f32>,
    /// Drive mode reported by controller: 0=Startup, 1=Straight, 2=Turning
    pub serial_rx_drive_mode: Option<u8>,
    pub serial_port_name: String,
    pub fake_car_enabled: bool,
    pub cmd_steer_pwm_us: u16,
    pub cmd_throttle_pwm_us: u16,
    pub map_line_count: usize,
    pub map_path: String,
    pub map_load_error: Option<String>,
    pub reset_requested: bool,
}

impl SimSnapshot {
    pub fn new(
        tick_hz: u32,
        map_line_count: usize,
        serial_port_name: String,
        map_path: String,
    ) -> Self {
        Self {
            tick_hz,
            sim_time_s: 0.0,
            x_m: 0.0,
            y_m: 0.0,
            heading_deg: 0.0,
            speed_mps: 0.0,
            steer_pwm_us: 1500,
            throttle_pwm_us: 1500,
            encoder_period_us: None,
            dist_left_cm: f32::INFINITY,
            dist_center_cm: f32::INFINITY,
            dist_right_cm: f32::INFINITY,
            camera_angle_deg: None,
            camera_confidence: None,
            camera_line_angle_deg: None,
            dropped_encoder: 0,
            dropped_distance: 0,
            dropped_camera: 0,
            serial_connected: false,
            serial_errors: 0,
            serial_sent_commands: 0,
            serial_rx_lines: 0,
            serial_rx_steer_pwm_us: None,
            serial_rx_throttle_pwm_us: None,
            serial_rx_angle_setpoint_deg: None,
            serial_rx_pid_error: None,
            serial_rx_heading_estimate_deg: None,
            serial_rx_observer_covariance: None,
            serial_rx_pid_p: None,
            serial_rx_pid_d: None,
            serial_rx_wall_left_correction_deg: None,
            serial_rx_wall_right_correction_deg: None,
            serial_rx_wall_combined_correction_deg: None,
            serial_rx_drive_mode: None,
            serial_port_name,
            fake_car_enabled: false,
            cmd_steer_pwm_us: 1500,
            cmd_throttle_pwm_us: 1500,
            map_line_count,
            map_path,
            map_load_error: None,
            reset_requested: false,
        }
    }
}
