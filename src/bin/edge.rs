//! Edge device entry point.
//!
//! Reads JSON-lines from stdin (one [`Input`] per line, as the sensor gateway
//! forwards them), runs the engine, and writes JSON-lines [`Event`]s to stdout.
//! No network, no clock beyond the monotonic tick supplied by the caller.
//!
//! ```text
//! edge --config config.json < inputs.jsonl > events.jsonl
//! edge --replay scene.json --config config.json     # run a recorded scene
//! ```

use std::io::{self, BufRead, Write};

use door_tailgate::badge::{BadgeDecision, BadgeRead};
use door_tailgate::beam::BeamSignal;
use door_tailgate::door::DoorSignal;
use door_tailgate::engine::{Event, Input};
use door_tailgate::radar::RadarPoint;
use door_tailgate::time::Ts;
use door_tailgate::{Config, Engine};
use serde::Deserialize;

/// Wire format for an input line.
#[derive(Debug, Deserialize)]
#[serde(tag = "t", rename_all = "snake_case")]
enum Wire {
    Radar {
        t_us: i64,
        #[serde(default)]
        points: Vec<PointWire>,
    },
    Door {
        t_us: i64,
        open: bool,
    },
    Badge {
        t_us: i64,
        credential: String,
        #[serde(default)]
        reader: String,
        #[serde(default)]
        decision: Option<String>,
    },
    Beam {
        t_us: i64,
        blocked: bool,
        #[serde(default)]
        direction: Option<String>,
        #[serde(default)]
        sensor: u8,
    },
    Tick {
        t_us: i64,
    },
}

#[derive(Debug, Deserialize)]
struct PointWire {
    x: f32,
    y: f32,
    #[serde(default)]
    z: f32,
    #[serde(default = "default_rcs")]
    rcs: f32,
    #[serde(default)]
    vr: Option<f32>,
}

fn default_rcs() -> f32 {
    -6.0
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut config_path: Option<String> = None;
    let mut echo_status = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--config" | "-c" => config_path = args.next(),
            "--status" => echo_status = true,
            "--help" | "-h" => {
                println!("usage: edge [--config FILE] [--status] < inputs.jsonl");
                return;
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }

    let cfg = match &config_path {
        Some(p) => match std::fs::read_to_string(p) {
            Ok(s) => match Config::from_json_str(&s) {
                Ok(c) => c,
                Err(e) => {
                    eprintln!("config parse error: {e}");
                    std::process::exit(2);
                }
            },
            Err(e) => {
                eprintln!("cannot read config {p}: {e}");
                std::process::exit(2);
            }
        },
        None => Config::default(),
    };
    let problems = cfg.validate();
    if !problems.is_empty() {
        for p in &problems {
            eprintln!("config: {p}");
        }
        std::process::exit(2);
    }

    let mut engine = Engine::new(cfg);
    let stdin = io::stdin();
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    let mut last_t = 0i64;

    let emit = |events: Vec<Event>, out: &mut io::BufWriter<io::StdoutLock<'_>>| {
        for e in events {
            // Writing JSON lines; a failed write means the gateway is gone, so
            // stop quietly rather than spin.
            if serde_json::to_writer(&mut *out, &e).is_err() {
                return;
            }
            let _ = out.write_all(b"\n");
        }
    };

    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let wire: Wire = match serde_json::from_str(line) {
            Ok(w) => w,
            Err(e) => {
                eprintln!("bad input line: {e}");
                continue;
            }
        };
        // Tick between messages so timers fire even when an input arrives late.
        let t = wire_time(&wire);
        while last_t + 40_000 <= t {
            last_t += 40_000;
            emit(engine.tick(Ts::from_micros(last_t)), &mut stdout);
        }
        last_t = last_t.max(t);
        let events = match wire {
            Wire::Radar { t_us, points } => engine.feed(Input::Radar {
                ts: Ts::from_micros(t_us),
                points: points
                    .into_iter()
                    .map(|p| {
                        let mut rp = RadarPoint::new(p.x, p.y, p.z).with_rcs(p.rcs);
                        if let Some(vr) = p.vr {
                            rp = rp.with_velocity(vr);
                        }
                        rp
                    })
                    .collect(),
            }),
            Wire::Door { t_us, open } => {
                engine.feed(Input::Door(DoorSignal::new(Ts::from_micros(t_us), open)))
            }
            Wire::Badge {
                t_us,
                credential,
                reader,
                decision,
            } => {
                let dec = match decision.as_deref() {
                    Some("granted") | Some("allow") => Some(BadgeDecision::Granted),
                    Some("denied") | Some("deny") => Some(BadgeDecision::Denied),
                    _ => None,
                };
                engine.feed(Input::Badge(BadgeRead {
                    ts: Ts::from_micros(t_us),
                    reader,
                    credential,
                    decision: dec,
                }))
            }
            Wire::Beam {
                t_us,
                blocked,
                direction,
                sensor,
            } => {
                let dir = match direction.as_deref() {
                    Some("in") => Some(door_tailgate::geom::Travel::In),
                    Some("out") => Some(door_tailgate::geom::Travel::Out),
                    _ => None,
                };
                engine.feed(Input::Beam(BeamSignal {
                    ts: Ts::from_micros(t_us),
                    sensor,
                    blocked,
                    direction: dir,
                }))
            }
            Wire::Tick { t_us } => engine.tick(Ts::from_micros(t_us)),
        };
        emit(events, &mut stdout);

        if echo_status {
            let status = serde_json::json!({
                "t_us": t,
                "health": engine.health(),
                "lane_stopped": engine.lane_stopped(),
                "alarm": engine.alarm_relay_on(),
                "episode": engine.current_episode().map(|e| e.id),
            });
            let _ = writeln!(stdout, "{status}");
        }
        let _ = stdout.flush();
    }
}

fn wire_time(w: &Wire) -> i64 {
    match w {
        Wire::Radar { t_us, .. }
        | Wire::Door { t_us, .. }
        | Wire::Badge { t_us, .. }
        | Wire::Beam { t_us, .. }
        | Wire::Tick { t_us } => *t_us,
    }
}
