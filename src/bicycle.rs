use crate::geometry::{Vec2, wrap_angle_rad};
use crate::constants;

#[derive(Clone, Copy, Debug)]
pub struct VehicleState {
    pub pos: Vec2,
    pub heading_rad: f32,
    pub speed_mps: f32,
    pub sim_time_s: f32,
    pub steer_pwm_us: u16,
    pub throttle_pwm_us: u16,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            pos: Vec2::new(0.5, 0.5),
            heading_rad: 0.0,
            speed_mps: 0.0,
            sim_time_s: 0.0,
            steer_pwm_us: constants::STEER_NEUTRAL_PWM_US,
            throttle_pwm_us: constants::THROTTLE_NEUTRAL_PWM_US,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ControlInput {
    pub steer_pwm_us: u16,
    pub throttle_pwm_us: u16,
}

pub fn step_vehicle(state: &mut VehicleState, control: ControlInput, dt_s: f32) -> f32 {
    state.steer_pwm_us = control.steer_pwm_us;
    state.throttle_pwm_us = control.throttle_pwm_us;

    let steer_delta_deg = (control.steer_pwm_us as f32 - constants::STEER_NEUTRAL_PWM_US as f32)
        * constants::STEER_ANGLE_PER_US_DEG;
    let steer_deg = steer_delta_deg.clamp(-constants::STEER_MAX_DEG, constants::STEER_MAX_DEG);
    let steer_rad = steer_deg.to_radians();

    let throttle_delta = control.throttle_pwm_us as i32 - constants::THROTTLE_NEUTRAL_PWM_US as i32;
    let accel_cmd = if throttle_delta.unsigned_abs() <= constants::THROTTLE_DEADBAND_US as u32 {
        0.0
    } else if throttle_delta > 0 {
        (throttle_delta as f32 / 500.0).clamp(0.0, 1.0) * constants::THROTTLE_MAX_ACCEL_MPS2
    } else {
        -(throttle_delta.unsigned_abs() as f32 / 500.0).clamp(0.0, 1.0) * constants::THROTTLE_MAX_BRAKE_MPS2
    };

    let drag = constants::THROTTLE_DRAG_LINEAR * state.speed_mps;
    let accel = accel_cmd - drag;

    state.speed_mps += accel * dt_s;
    state.speed_mps = state
        .speed_mps
        .clamp(-constants::VEHICLE_MAX_SPEED_MPS, constants::VEHICLE_MAX_SPEED_MPS);

    let yaw_rate = if steer_rad.abs() < 1e-4 {
        0.0
    } else {
        (state.speed_mps / constants::WHEELBASE_M) * steer_rad.tan()
    };

    state.heading_rad = wrap_angle_rad(state.heading_rad + yaw_rate * dt_s);

    let ds = state.speed_mps * dt_s;
    state.pos.x += ds * state.heading_rad.cos();
    state.pos.y += ds * state.heading_rad.sin();
    state.sim_time_s += dt_s;
    ds
}
