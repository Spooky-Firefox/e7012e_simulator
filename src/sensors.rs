use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::collections::VecDeque;

use crate::bicycle::VehicleState;
use crate::constants;
use crate::geometry::Vec2;
use crate::map::LoadedMap;

#[derive(Clone, Debug)]
pub enum SimCommand {
    Encoder {
        period_us: f32,
    },
    EncoderTimeout,
    Dist {
        left_cm: f32,
        center_cm: f32,
        right_cm: f32,
    },
    Align {
        angle_deg: f32,
        confidence: f32,
        delay_ms: u32,
    },
}

impl SimCommand {
    pub fn to_wire(&self) -> String {
        match self {
            SimCommand::Encoder { period_us } => format!("sim encoder {period_us:.3}\n"),
            SimCommand::EncoderTimeout => "sim encoder-timeout\n".to_string(),
            SimCommand::Dist {
                left_cm,
                center_cm,
                right_cm,
            } => format!("sim dist {left_cm:.3} {center_cm:.3} {right_cm:.3}\n"),
            SimCommand::Align {
                angle_deg,
                confidence,
                delay_ms,
            } => format!("align {angle_deg:.3} {confidence:.3} {delay_ms}\n"),
        }
    }
}

struct QueuedCameraMeasurement {
    available_at_s: f32,
    angle_deg: f32,
    confidence: f32,
    nearest_axis_deg: f32,
    delay_ms: u32,
}

pub struct EncoderEmulator {
    accum_distance_m: f32,
    last_pulse_time_s: Option<f32>,
    timeout_emitted: bool,
}

impl EncoderEmulator {
    pub fn new() -> Self {
        Self {
            accum_distance_m: 0.0,
            last_pulse_time_s: None,
            timeout_emitted: false,
        }
    }

    pub fn update<R: Rng>(
        &mut self,
        time_s: f32,
        ds_m: f32,
        rng: &mut R,
    ) -> (Vec<SimCommand>, u64) {
        let mut out = Vec::new();
        let mut dropped = 0u64;
        self.accum_distance_m += ds_m.abs();

        while self.accum_distance_m >= constants::ENCODER_DISTANCE_PER_PULSE_M {
            self.accum_distance_m -= constants::ENCODER_DISTANCE_PER_PULSE_M;
            self.timeout_emitted = false;

            if let Some(prev) = self.last_pulse_time_s {
                let base_us = ((time_s - prev) * 1_000_000.0).max(50.0);
                let jitter = Normal::new(0.0, constants::ENCODER_PERIOD_JITTER_STD_US as f64)
                    .map(|n| n.sample(rng) as f32)
                    .unwrap_or(0.0);
                let period_us = (base_us + jitter).max(50.0);
                if rng.r#gen::<f32>() < constants::ENCODER_DROP_PROB {
                    dropped = dropped.saturating_add(1);
                } else {
                    out.push(SimCommand::Encoder { period_us });
                }
            }
            self.last_pulse_time_s = Some(time_s);
        }

        if let Some(last) = self.last_pulse_time_s
            && !self.timeout_emitted && time_s - last >= constants::ENCODER_TIMEOUT_S {
                self.timeout_emitted = true;
                out.push(SimCommand::EncoderTimeout);
            }

        (out, dropped)
    }
}

pub struct DistanceEmulator {
    last_emit_s: f32,
}

impl DistanceEmulator {
    pub fn new() -> Self {
        Self { last_emit_s: 0.0 }
    }

    pub fn update<R: Rng>(
        &mut self,
        map: &LoadedMap,
        state: &VehicleState,
        rng: &mut R,
    ) -> (Option<SimCommand>, [f32; 3], u64) {
        if state.sim_time_s - self.last_emit_s < 1.0 / constants::DIST_SENSOR_RATE_HZ {
            return (None, [f32::INFINITY; 3], 0);
        }
        self.last_emit_s = state.sim_time_s;

        let mut dropped = 0u64;
        let mut values_cm = [f32::INFINITY; 3];

        for (idx, sensor_angle_deg) in constants::DIST_SENSOR_ANGLES_DEG.iter().enumerate() {
            let ray_angle = state.heading_rad + sensor_angle_deg.to_radians();
            let dir = Vec2::from_angle_rad(ray_angle);
            let mut maybe_distance =
                map.raycast_distance(state.pos, dir, constants::DIST_SENSOR_MAX_RANGE_M);

            if let Some(distance) = maybe_distance
                && (!(constants::DIST_SENSOR_MIN_RANGE_M..=constants::DIST_SENSOR_MAX_RANGE_M).contains(&distance))
                {
                    maybe_distance = None;
                }

            if rng.r#gen::<f32>() < constants::DIST_SENSOR_DROP_PROB {
                maybe_distance = None;
                dropped = dropped.saturating_add(1);
            }

            let distance_cm = match maybe_distance {
                Some(distance_m) => {
                    let noise_m = Normal::new(0.0, constants::DIST_SENSOR_NOISE_STD_M as f64)
                        .map(|n| n.sample(rng) as f32)
                        .unwrap_or(0.0);
                    ((distance_m + noise_m).max(0.0)) * 100.0
                }
                None => f32::INFINITY,
            };
            values_cm[idx] = distance_cm;
        }

        (
            Some(SimCommand::Dist {
                left_cm: values_cm[0],
                center_cm: values_cm[1],
                right_cm: values_cm[2],
            }),
            values_cm,
            dropped,
        )
    }
}

pub struct CameraEmulator {
    last_emit_s: f32,
    pending: VecDeque<QueuedCameraMeasurement>,
}

impl CameraEmulator {
    pub fn new() -> Self {
        Self {
            last_emit_s: 0.0,
            pending: VecDeque::new(),
        }
    }

    pub fn update<R: Rng>(
        &mut self,
        state: &VehicleState,
        rng: &mut R,
    ) -> (
        Option<SimCommand>,
        Option<f32>,
        Option<f32>,
        Option<f32>,
        u64,
    ) {
        let mut dropped = 0u64;
        let mut line_angle_for_ui = None;

        if state.sim_time_s - self.last_emit_s >= 1.0 / constants::CAMERA_RATE_HZ {
            self.last_emit_s = state.sim_time_s;

            let heading_deg = state.heading_rad.to_degrees();

            // Angle from the nearest major axis (multiples of 90deg), in range [-45, 45]
            let modulo = ((heading_deg % 90.0) + 90.0) % 90.0;
            let from_axis = if modulo > 45.0 { modulo - 90.0 } else { modulo };

            // Nearest major axis angle (for debug/display)
            let nearest_axis_deg = heading_deg - from_axis;
            line_angle_for_ui = Some(nearest_axis_deg);

            // Only enqueue when within +/-CAMERA_MAX_ANGLE_DEG of a major axis.
            if from_axis.abs() <= constants::CAMERA_MAX_ANGLE_DEG {
                if rng.r#gen::<f32>() < constants::CAMERA_DROP_PROB {
                    dropped = 1;
                } else {
                    let noise = Normal::new(0.0, constants::CAMERA_ANGLE_NOISE_STD_DEG as f64)
                        .map(|n| n.sample(rng) as f32)
                        .unwrap_or(0.0);
                    let angle_deg = (from_axis + noise).clamp(
                        -constants::CAMERA_MAX_ANGLE_DEG,
                        constants::CAMERA_MAX_ANGLE_DEG,
                    );

                    // Confidence: 1.0 at 0deg offset, 0.0 at +/-CAMERA_MAX_ANGLE_DEG
                    let confidence =
                        (1.0 - from_axis.abs() / constants::CAMERA_MAX_ANGLE_DEG).clamp(0.0, 1.0);

                    let jitter = rng.gen_range(
                        (1.0 - constants::CAMERA_DELAY_JITTER_FRACTION)
                            ..=(1.0 + constants::CAMERA_DELAY_JITTER_FRACTION),
                    );
                    let delay_ms =
                        ((constants::CAMERA_DELAY_BASE_MS as f32) * jitter).round().max(0.0) as u32;

                    self.pending.push_back(QueuedCameraMeasurement {
                        available_at_s: state.sim_time_s + delay_ms as f32 / 1000.0,
                        angle_deg,
                        confidence,
                        nearest_axis_deg,
                        delay_ms,
                    });
                }
            }
        }

        if self
            .pending
            .front()
            .is_some_and(|next| next.available_at_s <= state.sim_time_s)
            && let Some(measurement) = self.pending.pop_front()
        {
            return (
                Some(SimCommand::Align {
                    angle_deg: measurement.angle_deg,
                    confidence: measurement.confidence,
                    delay_ms: measurement.delay_ms,
                }),
                Some(measurement.angle_deg),
                Some(measurement.confidence),
                Some(measurement.nearest_axis_deg),
                dropped,
            );
        }

        (None, None, None, line_angle_for_ui, dropped)
    }
}
