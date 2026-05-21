use anyhow::Result;
use prometheus::{Counter, Encoder, Gauge, Opts, Registry, TextEncoder};

use crate::constants;
use crate::state::SimSnapshot;

pub struct SimMetrics {
    registry: Registry,
    sim_time_s: Gauge,
    sim_tick_hz: Gauge,
    pose_x_m: Gauge,
    pose_y_m: Gauge,
    heading_deg: Gauge,
    speed_mps: Gauge,
    steer_pwm_us: Gauge,
    throttle_pwm_us: Gauge,
    encoder_period_us: Gauge,
    dist_left_cm: Gauge,
    dist_center_cm: Gauge,
    dist_right_cm: Gauge,
    camera_angle_deg: Gauge,
    camera_confidence: Gauge,
    camera_line_angle_deg: Gauge,
    serial_connected: Gauge,
    serial_errors: Counter,
    serial_sent_commands: Counter,
    dropped_encoder: Counter,
    dropped_distance: Counter,
    dropped_camera: Counter,
    // Controller telemetry received over serial
    ctrl_steer_pwm_us: Gauge,
    ctrl_throttle_pwm_us: Gauge,
    ctrl_angle_setpoint_deg: Gauge,
    ctrl_pid_error: Gauge,
    ctrl_heading_estimate_deg: Gauge,
    ctrl_observer_covariance: Gauge,
    ctrl_pid_p: Gauge,
    ctrl_pid_d: Gauge,
}

impl SimMetrics {
    pub fn new() -> Result<Self> {
        let registry = Registry::new();

        let sim_time_s = Gauge::with_opts(
            Opts::new("sim_time_seconds", "Simulation time in seconds")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let sim_tick_hz = Gauge::with_opts(
            Opts::new("sim_tick_hz", "Configured simulation tick rate")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let pose_x_m = Gauge::with_opts(
            Opts::new("pose_x_m", "Vehicle X position in meters")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let pose_y_m = Gauge::with_opts(
            Opts::new("pose_y_m", "Vehicle Y position in meters")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let heading_deg = Gauge::with_opts(
            Opts::new("heading_deg", "Vehicle heading in degrees")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let speed_mps = Gauge::with_opts(
            Opts::new("speed_mps", "Vehicle speed in m/s").namespace(constants::METRICS_NAMESPACE),
        )?;
        let steer_pwm_us = Gauge::with_opts(
            Opts::new(
                "steer_pwm_us",
                "Steering PWM commanded by simulator to controller [µs]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let throttle_pwm_us = Gauge::with_opts(
            Opts::new(
                "throttle_pwm_us",
                "Throttle PWM commanded by simulator to controller [µs]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let encoder_period_us = Gauge::with_opts(
            Opts::new("encoder_period_us", "Latest emulated encoder period")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let dist_left_cm = Gauge::with_opts(
            Opts::new("distance_left_cm", "Latest emulated left distance")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let dist_center_cm = Gauge::with_opts(
            Opts::new("distance_center_cm", "Latest emulated center distance")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let dist_right_cm = Gauge::with_opts(
            Opts::new("distance_right_cm", "Latest emulated right distance")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let camera_angle_deg = Gauge::with_opts(
            Opts::new("camera_angle_deg", "Latest emulated camera angle")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let camera_confidence = Gauge::with_opts(
            Opts::new("camera_confidence", "Latest emulated camera confidence")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let camera_line_angle_deg = Gauge::with_opts(
            Opts::new(
                "camera_line_angle_deg",
                "Angle of nearest reference map line used for camera alignment [deg]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let serial_connected = Gauge::with_opts(
            Opts::new("serial_connected", "1 when serial output is connected")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let serial_errors = Counter::with_opts(
            Opts::new("serial_errors_total", "Total serial write/open errors")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let serial_sent_commands = Counter::with_opts(
            Opts::new(
                "serial_sent_commands_total",
                "Total commands sent to controller",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let dropped_encoder = Counter::with_opts(
            Opts::new("dropped_encoder_total", "Dropped encoder samples")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let dropped_distance = Counter::with_opts(
            Opts::new("dropped_distance_total", "Dropped distance samples")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let dropped_camera = Counter::with_opts(
            Opts::new("dropped_camera_total", "Dropped camera samples")
                .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_steer_pwm_us = Gauge::with_opts(
            Opts::new(
                "ctrl_steer_pwm_us",
                "Steering PWM reported back by controller over serial [µs]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_throttle_pwm_us = Gauge::with_opts(
            Opts::new(
                "ctrl_throttle_pwm_us",
                "Throttle PWM reported back by controller over serial [µs]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_angle_setpoint_deg = Gauge::with_opts(
            Opts::new(
                "ctrl_angle_setpoint_deg",
                "Angle setpoint the controller PID is targeting [deg]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_pid_error = Gauge::with_opts(
            Opts::new(
                "ctrl_pid_error",
                "Steering PID error (setpoint - measured) from controller [deg]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_heading_estimate_deg = Gauge::with_opts(
            Opts::new(
                "ctrl_heading_estimate_deg",
                "Heading observer estimate from controller [deg]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_observer_covariance = Gauge::with_opts(
            Opts::new(
                "ctrl_observer_covariance",
                "Heading observer covariance from controller [deg²]",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_pid_p = Gauge::with_opts(
            Opts::new(
                "ctrl_pid_p",
                "Steering PID proportional term from controller",
            )
            .namespace(constants::METRICS_NAMESPACE),
        )?;
        let ctrl_pid_d = Gauge::with_opts(
            Opts::new("ctrl_pid_d", "Steering PID derivative term from controller")
                .namespace(constants::METRICS_NAMESPACE),
        )?;

        registry.register(Box::new(sim_time_s.clone()))?;
        registry.register(Box::new(sim_tick_hz.clone()))?;
        registry.register(Box::new(pose_x_m.clone()))?;
        registry.register(Box::new(pose_y_m.clone()))?;
        registry.register(Box::new(heading_deg.clone()))?;
        registry.register(Box::new(speed_mps.clone()))?;
        registry.register(Box::new(encoder_period_us.clone()))?;
        registry.register(Box::new(dist_left_cm.clone()))?;
        registry.register(Box::new(dist_center_cm.clone()))?;
        registry.register(Box::new(dist_right_cm.clone()))?;
        registry.register(Box::new(camera_angle_deg.clone()))?;
        registry.register(Box::new(camera_confidence.clone()))?;
        registry.register(Box::new(camera_line_angle_deg.clone()))?;
        registry.register(Box::new(serial_connected.clone()))?;
        registry.register(Box::new(serial_errors.clone()))?;
        registry.register(Box::new(serial_sent_commands.clone()))?;
        registry.register(Box::new(dropped_encoder.clone()))?;
        registry.register(Box::new(dropped_distance.clone()))?;
        registry.register(Box::new(dropped_camera.clone()))?;
        registry.register(Box::new(ctrl_steer_pwm_us.clone()))?;
        registry.register(Box::new(ctrl_throttle_pwm_us.clone()))?;
        registry.register(Box::new(ctrl_angle_setpoint_deg.clone()))?;
        registry.register(Box::new(ctrl_pid_error.clone()))?;
        registry.register(Box::new(ctrl_heading_estimate_deg.clone()))?;
        registry.register(Box::new(ctrl_observer_covariance.clone()))?;
        registry.register(Box::new(ctrl_pid_p.clone()))?;
        registry.register(Box::new(ctrl_pid_d.clone()))?;

        Ok(Self {
            registry,
            sim_time_s,
            sim_tick_hz,
            pose_x_m,
            pose_y_m,
            heading_deg,
            speed_mps,
            steer_pwm_us,
            throttle_pwm_us,
            encoder_period_us,
            dist_left_cm,
            dist_center_cm,
            dist_right_cm,
            camera_angle_deg,
            camera_confidence,
            camera_line_angle_deg,
            serial_connected,
            serial_errors,
            serial_sent_commands,
            dropped_encoder,
            dropped_distance,
            dropped_camera,
            ctrl_steer_pwm_us,
            ctrl_throttle_pwm_us,
            ctrl_angle_setpoint_deg,
            ctrl_pid_error,
            ctrl_heading_estimate_deg,
            ctrl_observer_covariance,
            ctrl_pid_p,
            ctrl_pid_d,
        })
    }

    pub fn sync_from_snapshot(&self, snapshot: &SimSnapshot) {
        self.sim_time_s.set(snapshot.sim_time_s as f64);
        self.sim_tick_hz.set(snapshot.tick_hz as f64);
        self.pose_x_m.set(snapshot.x_m as f64);
        self.pose_y_m.set(snapshot.y_m as f64);
        self.heading_deg.set(snapshot.heading_deg as f64);
        self.speed_mps.set(snapshot.speed_mps as f64);
        self.steer_pwm_us.set(snapshot.steer_pwm_us as f64);
        self.throttle_pwm_us.set(snapshot.throttle_pwm_us as f64);
        self.encoder_period_us
            .set(snapshot.encoder_period_us.unwrap_or(0.0) as f64);
        self.dist_left_cm.set(snapshot.dist_left_cm as f64);
        self.dist_center_cm.set(snapshot.dist_center_cm as f64);
        self.dist_right_cm.set(snapshot.dist_right_cm as f64);
        self.camera_angle_deg
            .set(snapshot.camera_angle_deg.unwrap_or(0.0) as f64);
        self.camera_confidence
            .set(snapshot.camera_confidence.unwrap_or(0.0) as f64);
        self.camera_line_angle_deg
            .set(snapshot.camera_line_angle_deg.unwrap_or(0.0) as f64);
        self.serial_connected
            .set(if snapshot.serial_connected { 1.0 } else { 0.0 });
        self.ctrl_steer_pwm_us
            .set(snapshot.serial_rx_steer_pwm_us.unwrap_or(0) as f64);
        self.ctrl_throttle_pwm_us
            .set(snapshot.serial_rx_throttle_pwm_us.unwrap_or(0) as f64);
        self.ctrl_angle_setpoint_deg
            .set(snapshot.serial_rx_angle_setpoint_deg.unwrap_or(0.0) as f64);
        self.ctrl_pid_error
            .set(snapshot.serial_rx_pid_error.unwrap_or(0.0) as f64);
        self.ctrl_heading_estimate_deg
            .set(snapshot.serial_rx_heading_estimate_deg.unwrap_or(0.0) as f64);
        self.ctrl_observer_covariance
            .set(snapshot.serial_rx_observer_covariance.unwrap_or(0.0) as f64);
        self.ctrl_pid_p
            .set(snapshot.serial_rx_pid_p.unwrap_or(0.0) as f64);
        self.ctrl_pid_d
            .set(snapshot.serial_rx_pid_d.unwrap_or(0.0) as f64);
    }

    pub fn inc_serial_error(&self) {
        self.serial_errors.inc();
    }

    pub fn inc_serial_sent(&self) {
        self.serial_sent_commands.inc();
    }

    pub fn inc_dropped_encoder_by(&self, count: u64) {
        self.dropped_encoder.inc_by(count as f64);
    }

    pub fn inc_dropped_distance_by(&self, count: u64) {
        self.dropped_distance.inc_by(count as f64);
    }

    pub fn inc_dropped_camera_by(&self, count: u64) {
        self.dropped_camera.inc_by(count as f64);
    }

    pub fn encode(&self) -> String {
        let encoder = TextEncoder::new();
        let families = self.registry.gather();
        let mut buffer = Vec::new();
        if encoder.encode(&families, &mut buffer).is_ok() {
            String::from_utf8_lossy(&buffer).into_owned()
        } else {
            "# failed to encode metrics\n".to_string()
        }
    }
}
