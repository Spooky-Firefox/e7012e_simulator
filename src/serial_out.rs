use std::io::{ErrorKind, Read, Write};
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

use crossbeam_channel::Receiver;

use crate::constants;
use crate::metrics::SimMetrics;
use crate::state::SimSnapshot;

pub fn spawn_serial_thread(
    rx: Receiver<String>,
    snapshot: Arc<RwLock<SimSnapshot>>,
    metrics: Arc<SimMetrics>,
) {
    thread::spawn(move || {
        let mut maybe_port: Option<Box<dyn serialport::SerialPort>> = None;
        let mut current_port_name = String::new();
        let mut fake_car_enabled = false;
        let mut read_buf = [0u8; 512];
        let mut line_buf = String::new();

        loop {
            let (desired_port_name, desired_fake_car) = snapshot
                .read()
                .map(|lock| (lock.serial_port_name.clone(), lock.fake_car_enabled))
                .unwrap_or_else(|_| (constants::serial_port_name(), false));

            if desired_port_name != current_port_name || desired_fake_car != fake_car_enabled {
                current_port_name = desired_port_name;
                fake_car_enabled = desired_fake_car;
                maybe_port = None;
                line_buf.clear();

                if let Ok(mut lock) = snapshot.write() {
                    lock.serial_connected = fake_car_enabled;
                }
            }

            if fake_car_enabled {
                let Ok(_command) = rx.recv_timeout(Duration::from_millis(20)) else {
                    continue;
                };

                metrics.inc_serial_sent();
                if let Ok(mut lock) = snapshot.write() {
                    lock.serial_connected = true;
                    lock.serial_sent_commands = lock.serial_sent_commands.saturating_add(1);
                }
                continue;
            }

            if maybe_port.is_none() {
                match serialport::new(&current_port_name, constants::SERIAL_BAUD)
                    .timeout(Duration::from_millis(5))
                    .open()
                {
                    Ok(port) => {
                        maybe_port = Some(port);
                        if let Ok(mut lock) = snapshot.write() {
                            lock.serial_connected = true;
                        }
                    }
                    Err(_) => {
                        if let Ok(mut lock) = snapshot.write() {
                            lock.serial_connected = false;
                            lock.serial_errors = lock.serial_errors.saturating_add(1);
                        }
                        metrics.inc_serial_error();
                        thread::sleep(Duration::from_millis(constants::SERIAL_RETRY_MS));
                        continue;
                    }
                }
            }

            let Some(port) = maybe_port.as_mut() else {
                continue;
            };

            loop {
                match port.read(&mut read_buf) {
                    Ok(0) => break,
                    Ok(count) => {
                        if let Ok(chunk) = std::str::from_utf8(&read_buf[..count]) {
                            line_buf.push_str(chunk);
                            process_rx_buffer(&mut line_buf, &snapshot);
                        }
                    }
                    Err(err)
                        if matches!(
                            err.kind(),
                            ErrorKind::WouldBlock | ErrorKind::TimedOut | ErrorKind::Interrupted
                        ) =>
                    {
                        break;
                    }
                    Err(_) => {
                        metrics.inc_serial_error();
                        if let Ok(mut lock) = snapshot.write() {
                            lock.serial_connected = false;
                            lock.serial_errors = lock.serial_errors.saturating_add(1);
                        }
                        maybe_port = None;
                        line_buf.clear();
                        break;
                    }
                }
            }

            let Ok(command) = rx.recv_timeout(Duration::from_millis(5)) else {
                continue;
            };

            let Some(port) = maybe_port.as_mut() else {
                continue;
            };

            if port.write_all(command.as_bytes()).is_ok() {
                metrics.inc_serial_sent();
                if let Ok(mut lock) = snapshot.write() {
                    lock.serial_sent_commands = lock.serial_sent_commands.saturating_add(1);
                }
            } else {
                metrics.inc_serial_error();
                if let Ok(mut lock) = snapshot.write() {
                    lock.serial_connected = false;
                    lock.serial_errors = lock.serial_errors.saturating_add(1);
                }
                maybe_port = None;
            }
        }
    });
}

fn process_rx_buffer(buffer: &mut String, snapshot: &Arc<RwLock<SimSnapshot>>) {
    while let Some(newline_pos) = buffer.find('\n') {
        let mut line = buffer.drain(..=newline_pos).collect::<String>();
        line.truncate(line.trim_end_matches(['\r', '\n']).len());
        if line.is_empty() {
            continue;
        }

        if let Some((steer, throttle)) = parse_pwm_from_line(&line)
            && let Ok(mut lock) = snapshot.write()
        {
            let s = steer.clamp(1000, 2000);
            let t = throttle.clamp(1000, 2000);
            lock.cmd_steer_pwm_us = s;
            lock.cmd_throttle_pwm_us = t;
            lock.serial_rx_steer_pwm_us = Some(s);
            lock.serial_rx_throttle_pwm_us = Some(t);
            lock.serial_rx_lines = lock.serial_rx_lines.saturating_add(1);
        }

        // Parse PID / heading observer telemetry from the plotter-format line.
        if let Ok(mut lock) = snapshot.write() {
            if let Some(v) = parse_f32_after_marker(&line, "setpoint_mps:") {
                lock.serial_rx_angle_setpoint_deg = Some(v);
            }
            if let Some(v) = parse_f32_after_marker(&line, "error:") {
                lock.serial_rx_pid_error = Some(v);
            }
            if let Some(v) = parse_f32_after_marker(&line, "kalman0:") {
                lock.serial_rx_heading_estimate_deg = Some(v);
            }
            if let Some(v) = parse_f32_after_marker(&line, "kalman1:") {
                lock.serial_rx_observer_covariance = Some(v);
            }
            if let Some(v) = parse_f32_after_marker(&line, "kalman2:") {
                lock.serial_rx_pid_p = Some(v);
            }
            if let Some(v) = parse_f32_after_marker(&line, "kalman3:") {
                lock.serial_rx_pid_d = Some(v);
            }
        }
    }

    if buffer.len() > 4096 {
        buffer.clear();
    }
}

fn parse_pwm_from_line(line: &str) -> Option<(u16, u16)> {
    if let Some((steer, throttle)) = parse_key_value_pwm(line) {
        return Some((steer, throttle));
    }

    if let Some((steer, throttle)) = parse_equals_pwm(line) {
        return Some((steer, throttle));
    }

    parse_csv_pwm(line)
}

fn parse_key_value_pwm(line: &str) -> Option<(u16, u16)> {
    let steer = parse_u16_after_marker(line, "steer_us:")?;
    let throttle = parse_u16_after_marker(line, "throttle_us:")?;
    Some((steer, throttle))
}

fn parse_equals_pwm(line: &str) -> Option<(u16, u16)> {
    let steer = parse_u16_after_marker(line, "steer_pwm_us=")?;
    let throttle = parse_u16_after_marker(line, "throttle_pwm_us=")?;
    Some((steer, throttle))
}

fn parse_csv_pwm(line: &str) -> Option<(u16, u16)> {
    let mut columns = line.split(',');
    let _time_us = columns.next()?;
    let event = columns.next()?;
    if !event.eq_ignore_ascii_case("controller") {
        return None;
    }

    let steer = columns.next()?.parse::<u16>().ok()?;
    let throttle = columns.next()?.parse::<u16>().ok()?;
    Some((steer, throttle))
}

fn parse_u16_after_marker(line: &str, marker: &str) -> Option<u16> {
    let start = line.find(marker)? + marker.len();
    let tail = &line[start..];
    let end = tail
        .find(|ch: char| !ch.is_ascii_digit())
        .unwrap_or(tail.len());
    if end == 0 {
        return None;
    }

    tail[..end].parse::<u16>().ok()
}

fn parse_f32_after_marker(line: &str, marker: &str) -> Option<f32> {
    let start = line.find(marker)? + marker.len();
    let tail = &line[start..];
    let end = tail
        .find(|ch: char| !ch.is_ascii_digit() && ch != '.' && ch != '-')
        .unwrap_or(tail.len());
    if end == 0 {
        return None;
    }
    tail[..end].parse::<f32>().ok()
}
