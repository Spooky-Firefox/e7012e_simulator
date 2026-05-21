use std::sync::{Arc, RwLock};
use std::thread;

use serde::{Deserialize, Serialize};
use tiny_http::{Header, Method, Response, Server, StatusCode};

use crate::constants;
use crate::map::LoadedMap;
use crate::metrics::SimMetrics;
use crate::state::SimSnapshot;

const INDEX_HTML: &str = include_str!("ui_index.html");

#[derive(Serialize)]
struct MapLineView {
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    kind: &'static str,
}

#[derive(Serialize)]
struct MapView {
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
    lines: Vec<MapLineView>,
}

#[derive(Serialize)]
struct ConfigView {
    serial_port_name: String,
    fake_car_enabled: bool,
    map_path: String,
    map_load_error: Option<String>,
    available_ports: Vec<String>,
    available_maps: Vec<String>,
}

#[derive(Deserialize)]
struct ConfigUpdateRequest {
    serial_port_name: Option<String>,
    fake_car_enabled: Option<bool>,
}

#[derive(Deserialize)]
struct ControlUpdateRequest {
    steer_pwm_us: Option<u16>,
    throttle_pwm_us: Option<u16>,
}

#[derive(Deserialize)]
struct MapLoadRequest {
    path: String,
}

#[derive(Deserialize)]
struct SendCommandRequest {
    command: String,
}

pub fn spawn_ui_thread(
    snapshot: Arc<RwLock<SimSnapshot>>,
    metrics: Arc<SimMetrics>,
    map: Arc<RwLock<LoadedMap>>,
    tx_cmd: crossbeam_channel::Sender<String>,
) {
    thread::spawn(move || {
        let bind = constants::ui_bind();
        let Ok(server) = Server::http(&bind) else {
            return;
        };

        for request in server.incoming_requests() {
            let mut request = request;
            match (request.method(), request.url()) {
                (&Method::Get, "/") => {
                    let response = Response::from_string(INDEX_HTML)
                        .with_header(content_type("text/html; charset=utf-8"));
                    let _ = request.respond(response);
                }
                (&Method::Get, "/api/state") => {
                    let body = snapshot
                        .read()
                        .ok()
                        .and_then(|state| serde_json::to_string(&*state).ok())
                        .unwrap_or_else(|| "{}".to_string());
                    let response =
                        Response::from_string(body).with_header(content_type("application/json"));
                    let _ = request.respond(response);
                }
                (&Method::Get, "/api/config") => {
                    let available_ports = serialport::available_ports()
                        .ok()
                        .unwrap_or_default()
                        .into_iter()
                        .map(|port| port.port_name)
                        .collect::<Vec<_>>();

                    let mut available_maps = std::fs::read_dir("maps")
                        .ok()
                        .into_iter()
                        .flatten()
                        .filter_map(|e| {
                            let e = e.ok()?;
                            let name = e.file_name().to_string_lossy().into_owned();
                            if name.ends_with(".toml") {
                                Some(format!("maps/{}", name))
                            } else {
                                None
                            }
                        })
                        .collect::<Vec<_>>();
                    available_maps.sort();

                    let body = snapshot
                        .read()
                        .ok()
                        .and_then(|state| {
                            serde_json::to_string(&ConfigView {
                                serial_port_name: state.serial_port_name.clone(),
                                fake_car_enabled: state.fake_car_enabled,
                                map_path: state.map_path.clone(),
                                map_load_error: state.map_load_error.clone(),
                                available_ports,
                                available_maps,
                            })
                            .ok()
                        })
                        .unwrap_or_else(|| "{}".to_string());

                    let response =
                        Response::from_string(body).with_header(content_type("application/json"));
                    let _ = request.respond(response);
                }
                (&Method::Get, "/api/map") => {
                    let body = map
                        .read()
                        .ok()
                        .and_then(|loaded| {
                            let (min_x, min_y, max_x, max_y) = loaded.bounds();
                            let lines = loaded
                                .lines
                                .iter()
                                .map(|line| MapLineView {
                                    x1: line.segment.a.x,
                                    y1: line.segment.a.y,
                                    x2: line.segment.b.x,
                                    y2: line.segment.b.y,
                                    kind: line.kind.as_str(),
                                })
                                .collect::<Vec<_>>();
                            serde_json::to_string(&MapView {
                                min_x,
                                min_y,
                                max_x,
                                max_y,
                                lines,
                            })
                            .ok()
                        })
                        .unwrap_or_else(|| "{}".to_string());

                    let response =
                        Response::from_string(body).with_header(content_type("application/json"));
                    let _ = request.respond(response);
                }
                (&Method::Post, "/api/config") => {
                    let Some(payload) = read_json::<ConfigUpdateRequest>(&mut request) else {
                        let _ =
                            request.respond(json_error(StatusCode(400), "invalid config payload"));
                        continue;
                    };

                    if let Ok(mut state) = snapshot.write() {
                        if let Some(port) = payload.serial_port_name {
                            let cleaned = port.trim();
                            if !cleaned.is_empty() {
                                state.serial_port_name = cleaned.to_string();
                            }
                        }
                        if let Some(fake) = payload.fake_car_enabled {
                            state.fake_car_enabled = fake;
                        }
                    }

                    let _ = request.respond(
                        Response::from_string("{\"ok\":true}")
                            .with_header(content_type("application/json")),
                    );
                }
                (&Method::Post, "/api/load_map") => {
                    let Some(payload) = read_json::<MapLoadRequest>(&mut request) else {
                        let _ = request.respond(json_error(StatusCode(400), "invalid map payload"));
                        continue;
                    };

                    let path = payload.path.trim().to_string();
                    if path.is_empty() {
                        let _ = request.respond(json_error(StatusCode(400), "map path is empty"));
                        continue;
                    }

                    match LoadedMap::load_from_file(&path) {
                        Ok(loaded_map) => {
                            let line_count = loaded_map.lines.len();

                            if let Ok(mut map_lock) = map.write() {
                                *map_lock = loaded_map;
                            }
                            if let Ok(mut state) = snapshot.write() {
                                state.map_path = path;
                                state.map_line_count = line_count;
                                state.map_load_error = None;
                            }

                            let _ = request.respond(
                                Response::from_string("{\"ok\":true}")
                                    .with_header(content_type("application/json")),
                            );
                        }
                        Err(err) => {
                            if let Ok(mut state) = snapshot.write() {
                                state.map_load_error = Some(err.to_string());
                            }

                            let body = format!(
                                "{{\"ok\":false,\"error\":{}}}",
                                serde_json::to_string(&err.to_string())
                                    .unwrap_or_else(|_| "\"map load failed\"".to_string())
                            );
                            let response = Response::from_string(body)
                                .with_status_code(StatusCode(400))
                                .with_header(content_type("application/json"));
                            let _ = request.respond(response);
                        }
                    }
                }
                (&Method::Post, "/api/reset") => {
                    if let Ok(mut state) = snapshot.write() {
                        state.reset_requested = true;
                        state.map_load_error = None;
                    }
                    let _ = tx_cmd.try_send("reset\n".to_string());

                    let _ = request.respond(
                        Response::from_string("{\"ok\":true}")
                            .with_header(content_type("application/json")),
                    );
                }
                (&Method::Post, "/api/command") => {
                    let Some(payload) = read_json::<SendCommandRequest>(&mut request) else {
                        let _ =
                            request.respond(json_error(StatusCode(400), "invalid command payload"));
                        continue;
                    };
                    let cmd = payload.command.trim().to_string();
                    if cmd.is_empty() {
                        let _ = request.respond(json_error(StatusCode(400), "empty command"));
                        continue;
                    }
                    let wire = if cmd.ends_with('\n') {
                        cmd
                    } else {
                        format!("{}\n", cmd)
                    };
                    match tx_cmd.try_send(wire) {
                        Ok(()) => {
                            let _ = request.respond(
                                Response::from_string("{\"ok\":true}")
                                    .with_header(content_type("application/json")),
                            );
                        }
                        Err(_) => {
                            let _ =
                                request.respond(json_error(StatusCode(503), "command queue full"));
                        }
                    }
                }
                (&Method::Post, "/api/control") => {
                    let Some(payload) = read_json::<ControlUpdateRequest>(&mut request) else {
                        let _ =
                            request.respond(json_error(StatusCode(400), "invalid control payload"));
                        continue;
                    };

                    if let Ok(mut state) = snapshot.write() {
                        if let Some(steer) = payload.steer_pwm_us {
                            state.cmd_steer_pwm_us = steer.clamp(1000, 2000);
                        }
                        if let Some(throttle) = payload.throttle_pwm_us {
                            state.cmd_throttle_pwm_us = throttle.clamp(1000, 2000);
                        }
                    }

                    let _ = request.respond(
                        Response::from_string("{\"ok\":true}")
                            .with_header(content_type("application/json")),
                    );
                }
                (&Method::Get, "/metrics") => {
                    let response = Response::from_string(metrics.encode())
                        .with_header(content_type("text/plain; version=0.0.4"));
                    let _ = request.respond(response);
                }
                _ => {
                    let response = Response::from_string("not found")
                        .with_status_code(StatusCode(404))
                        .with_header(content_type("text/plain"));
                    let _ = request.respond(response);
                }
            }
        }
    });
}

fn content_type(value: &str) -> Header {
    Header::from_bytes("Content-Type", value).unwrap_or_else(|_| {
        Header::from_bytes("Content-Type", "text/plain").expect("valid fallback content type")
    })
}

fn read_json<T>(request: &mut tiny_http::Request) -> Option<T>
where
    T: for<'de> Deserialize<'de>,
{
    let mut body = String::new();
    request.as_reader().read_to_string(&mut body).ok()?;
    serde_json::from_str::<T>(&body).ok()
}

fn json_error(status: StatusCode, message: &str) -> Response<std::io::Cursor<Vec<u8>>> {
    let body = format!(
        "{{\"ok\":false,\"error\":{}}}",
        serde_json::to_string(message).unwrap_or_else(|_| "\"request failed\"".to_string())
    );
    Response::from_string(body)
        .with_status_code(status)
        .with_header(content_type("application/json"))
}
