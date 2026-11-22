//! Replay a recorded or generated scene and print a human-readable timeline.
//!
//! ```text
//! sim --scenario tailgate
//! sim --scenario cart_with_tailgater --json
//! sim --list
//! ```

use door_tailgate::badge::BadgeRead;
use door_tailgate::beam::BeamSignal;
use door_tailgate::door::DoorSignal;
use door_tailgate::engine::{Action, Event, SensorHealth, Severity};
use door_tailgate::geom::Travel;
use door_tailgate::radar::RadarPoint;
use door_tailgate::time::Ts;
use door_tailgate::{Config, Engine, Input};

const FRAME_MS: i64 = 40;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Person,
    CartWithOperator,
    #[allow(dead_code)]
    CartAlone,
}

#[derive(Debug, Clone)]
struct Actor {
    x: f32,
    y0: f32,
    y1: f32,
    t0: i64,
    t1: i64,
    kind: Kind,
    badge: Option<(i64, &'static str)>,
}

impl Actor {
    fn person(x: f32, y0: f32, y1: f32, t0: i64, t1: i64) -> Actor {
        Actor {
            x,
            y0,
            y1,
            t0,
            t1,
            kind: Kind::Person,
            badge: None,
        }
    }
    fn badge(mut self, ms: i64, cred: &'static str) -> Actor {
        self.badge = Some((ms, cred));
        self
    }
    fn cart(mut self, kind: Kind, y0: f32, y1: f32, t0: i64, t1: i64) -> Actor {
        self.kind = kind;
        self.y0 = y0;
        self.y1 = y1;
        self.t0 = t0;
        self.t1 = t1;
        self
    }
    fn y_at(&self, ms: f64) -> Option<f32> {
        if ms < self.t0 as f64 || ms > self.t1 as f64 {
            return None;
        }
        let f = (ms - self.t0 as f64) / (self.t1 - self.t0) as f64;
        Some(self.y0 + (self.y1 - self.y0) * f as f32)
    }
    fn points(&self, ms: f64) -> Vec<RadarPoint> {
        let Some(y) = self.y_at(ms) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        match self.kind {
            Kind::Person | Kind::CartWithOperator => {
                // torso: a realistic spread of person-height returns
                for (dx, dy) in [
                    (-0.10, 0.02),
                    (-0.05, -0.06),
                    (-0.02, 0.07),
                    (0.0, 0.0),
                    (0.03, -0.05),
                    (0.06, 0.05),
                    (0.10, -0.01),
                    (0.0, 0.10),
                    (0.0, -0.10),
                ] {
                    out.push(RadarPoint::new(self.x + dx, y + dy, 1.45).with_rcs(-5.0));
                }
                if self.kind == Kind::CartWithOperator {
                    for i in 0..9 {
                        let cx = self.x - 0.55 + 0.1375 * i as f32;
                        out.push(RadarPoint::new(cx, y - 0.55 - 0.10, 0.45).with_rcs(-2.0));
                        out.push(RadarPoint::new(cx, y - 0.55 + 0.10, 0.60).with_rcs(-2.0));
                    }
                }
            }
            Kind::CartAlone => {
                for i in 0..9 {
                    let cx = self.x - 0.55 + 0.1375 * i as f32;
                    out.push(RadarPoint::new(cx, y - 0.10, 0.45).with_rcs(-2.0));
                    out.push(RadarPoint::new(cx, y + 0.10, 0.60).with_rcs(-2.0));
                }
            }
        }
        out
    }
}

struct Scenario {
    name: &'static str,
    about: &'static str,
    actors: Vec<Actor>,
    door: Vec<(i64, i64)>,
    until: i64,
    /// Extra beam trips: (start, end, direction).
    beams: Vec<(i64, i64, Option<Travel>)>,
}

fn scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "single",
            about: "One employee badges and walks in. Baseline: no alarm.",
            actors: vec![Actor::person(0.0, 3.0, -1.2, 1000, 3200).badge(600, "alice")],
            door: vec![(900, 3400)],
            until: 7000,
            beams: vec![(1800, 2050, Some(Travel::In))],
        },
        Scenario {
            name: "tailgate",
            about: "Alice badges; a stranger follows through the same opening.",
            actors: vec![
                Actor::person(0.0, 3.0, -1.2, 1000, 3200).badge(700, "alice"),
                Actor::person(0.05, 3.5, -1.2, 1500, 3700),
            ],
            door: vec![(900, 3900)],
            until: 8000,
            beams: vec![
                (1800, 2020, Some(Travel::In)),
                (2250, 2470, Some(Travel::In)),
            ],
        },
        Scenario {
            name: "cart_alone",
            about: "One technician pushes a wide cart through. Baseline: one person, no alarm.",
            actors: vec![Actor::person(0.0, 3.0, -1.4, 1000, 3600)
                .cart(Kind::CartWithOperator, 2.45, -1.95, 1000, 3600)
                .badge(650, "tech")],
            door: vec![(900, 4000)],
            until: 8000,
            beams: vec![(1400, 2250, Some(Travel::In))],
        },
        Scenario {
            name: "cart_with_tailgater",
            about: "A technician with a cart; a stranger slips in right behind. The cart occludes the second person for the radar, so the infrared person trip is the evidence.",
            actors: vec![
                Actor::person(0.0, 3.0, -1.4, 1000, 3600)
                    .cart(Kind::CartWithOperator, 2.45, -1.95, 1000, 3600)
                    .badge(650, "tech"),
                Actor::person(0.03, 2.2, -1.4, 2250, 4200),
            ],
            door: vec![(900, 4400)],
            until: 9000,
            beams: vec![
                (1400, 2300, Some(Travel::In)),
                (2500, 2720, Some(Travel::In)),
            ],
        },
        Scenario {
            name: "abreast_ok",
            about: "Two colleagues walk side by side; each badges. Baseline: no alarm.",
            actors: vec![
                Actor::person(-0.38, 3.0, -1.2, 1000, 3200).badge(600, "a"),
                Actor::person(0.38, 3.0, -1.2, 1040, 3240).badge(680, "b"),
            ],
            door: vec![(900, 3500)],
            until: 7000,
            beams: vec![(1800, 2150, Some(Travel::In))],
        },
        Scenario {
            name: "abreast_piggyback",
            about: "Two people abreast on a single badge.",
            actors: vec![
                Actor::person(-0.38, 3.0, -1.2, 1000, 3200).badge(600, "a"),
                Actor::person(0.38, 3.0, -1.2, 1040, 3240),
            ],
            door: vec![(900, 3500)],
            until: 7000,
            beams: vec![(1800, 2150, Some(Travel::In))],
        },
        Scenario {
            name: "rush",
            about: "Morning rush: six employees, each badges, 1.4 s apart. Baseline: no alarm.",
            actors: (0..6)
                .map(|i| {
                    let t0 = 1000 + i * 1400;
                    Actor::person(if i % 2 == 0 { -0.15 } else { 0.15 }, 3.0, -1.2, t0, t0 + 2857)
                        .badge(t0 - 300, "emp")
                })
                .collect(),
            door: vec![(800, 11000)],
            until: 15000,
            beams: (0..6)
                .map(|i| {
                    let s = 1500 + i * 1400;
                    (s, s + 240, Some(Travel::In))
                })
                .collect(),
        },
        Scenario {
            name: "zero_badge",
            about: "Nobody badges; somebody walks through a door left open.",
            actors: vec![Actor::person(0.0, 3.0, -1.2, 1000, 3200)],
            door: vec![(900, 3400)],
            until: 7000,
            beams: vec![(1800, 2050, Some(Travel::In))],
        },
        Scenario {
            name: "radar_blind_beam",
            about: "Radar is down (no frames at all). One badge, but the infrared sees two people pass. Degraded sensing still catches the tailgate.",
            actors: vec![], // no radar returns at all
            door: vec![(900, 4200)],
            until: 9000,
            beams: vec![
                (1500, 1740, Some(Travel::In)),
                (2400, 2640, Some(Travel::In)),
            ],
        },
        Scenario {
            name: "one_credential_crowd",
            about: "Five people follow a single badge read through a propped lane. The per-episode cap and the count expose the group.",
            actors: (0..5)
                .map(|i| {
                    let t0 = 1000 + i * 900;
                    Actor::person(0.0, 3.0, -1.2, t0, t0 + 2000).badge(700, "shared")
                })
                .collect(),
            door: vec![(850, 6000)],
            until: 10000,
            beams: (0..5)
                .map(|i| {
                    let s = 1500 + i * 900;
                    (s, s + 240, Some(Travel::In))
                })
                .collect(),
        },
    ]
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut scenario_name = "tailgate".to_string();
    let mut json = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--scenario" | "-s" => scenario_name = args.next().unwrap_or_default(),
            "--json" => json = true,
            "--list" | "-l" => {
                for s in scenarios() {
                    println!("{:22} {}", s.name, s.about);
                }
                return;
            }
            "--help" | "-h" => {
                println!("usage: sim [--scenario NAME] [--json] [--list]");
                return;
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    let Some(sc) = scenarios().into_iter().find(|s| s.name == scenario_name) else {
        eprintln!("unknown scenario: {scenario_name} (try --list)");
        std::process::exit(2);
    };

    println!("=== {} ===", sc.name);
    println!("{}", sc.about);
    println!();

    let mut engine = Engine::new(Config::default());
    let mut ms = 0i64;
    let mut last_tick = 0i64;
    let mut verdict_count = 0;

    while ms <= sc.until {
        let ts = Ts::from_micros(ms * 1000);

        // Ticks between messages.
        while last_tick + FRAME_MS <= ms {
            last_tick += FRAME_MS;
            let events = engine.tick(Ts::from_micros(last_tick * 1000));
            report(&events, last_tick, json, &mut verdict_count);
        }
        // Radar frame.
        let mut pts = Vec::new();
        for a in &sc.actors {
            pts.extend(a.points(ms as f64));
        }
        report(
            &engine.feed(Input::Radar { ts, points: pts }),
            ms,
            json,
            &mut verdict_count,
        );
        // Door contact.
        let open = sc.door.iter().any(|(o, c)| ms >= *o && ms < *c);
        report(
            &engine.feed(Input::Door(DoorSignal::new(ts, open))),
            ms,
            json,
            &mut verdict_count,
        );
        // Badges (reader burst: 3 reads 30 ms apart).
        for a in &sc.actors {
            if let Some((b, cred)) = a.badge {
                if ms == b || ms == b + 30 || ms == b + 60 {
                    report(
                        &engine.feed(Input::Badge(BadgeRead::granted(ts, cred))),
                        ms,
                        json,
                        &mut verdict_count,
                    );
                }
            }
        }
        // Beam.
        for (s, e, dir) in &sc.beams {
            if ms == *s {
                let mut sig = BeamSignal::new(ts, true);
                sig.direction = *dir;
                report(&engine.feed(Input::Beam(sig)), ms, json, &mut verdict_count);
            }
            if ms == *e {
                report(
                    &engine.feed(Input::Beam(BeamSignal::new(ts, false))),
                    ms,
                    json,
                    &mut verdict_count,
                );
            }
        }
        ms += FRAME_MS;
    }
    // Drain timers.
    for _ in 0..200 {
        last_tick += FRAME_MS;
        report(
            &engine.tick(Ts::from_micros(last_tick * 1000)),
            last_tick,
            json,
            &mut verdict_count,
        );
    }

    println!();
    println!("--- result ---");
    for ep in engine.recent_episodes() {
        let v = ep.verdict.as_ref();
        println!(
            "episode {}: authorized={} passed={}{}",
            ep.id,
            ep.authorized,
            ep.passed(engine.config()),
            match v {
                Some(v) => format!(
                    "  -> {} (excess {})",
                    match v.severity {
                        Severity::Alarm => "ALARM",
                        Severity::Suspect => "SUSPECT",
                    },
                    v.excess
                ),
                None => "  -> clear".to_string(),
            }
        );
        if let Some(v) = v {
            for r in &v.reasons {
                println!("    · {r}");
            }
        }
    }
    if verdict_count == 0 {
        println!("(no verdicts: all episodes clear)");
    }
}

fn report(events: &[Event], ms: i64, json: bool, verdict_count: &mut usize) {
    for e in events {
        if json {
            println!("{}", serde_json::to_string(&(ms, e)).unwrap());
            continue;
        }
        match e {
            Event::Passage { episode, passage } => println!(
                "{:>7}ms  passage  ep{} pe={} {:?}{}",
                ms,
                episode,
                passage.pe,
                passage.source,
                passage
                    .note
                    .as_ref()
                    .map(|n| format!("  ({n})"))
                    .unwrap_or_default()
            ),
            Event::Badge {
                episode,
                authorized_total,
                ..
            } => println!(
                "{:>7}ms  badge    ep{} authorized_total={}",
                ms, episode, authorized_total
            ),
            Event::EpisodeOpened { episode, .. } => {
                println!("{:>7}ms  episode  ep{} opened", ms, episode)
            }
            Event::EpisodeClosed {
                episode,
                authorized,
                passed,
                ..
            } => println!(
                "{:>7}ms  episode  ep{} closed  authorized={} passed={}",
                ms, episode, authorized, passed
            ),
            Event::Verdict(v) => {
                *verdict_count += 1;
                println!(
                    "{:>7}ms  VERDICT  ep{} {:?} passed={} authorized={} excess={} health={:?}",
                    ms, v.episode, v.severity, v.passed, v.authorized, v.excess, v.health
                );
                for r in &v.reasons {
                    println!("            · {r}");
                }
            }
            Event::Action(a) => match a {
                Action::StopLane { .. } => println!("{:>7}ms  action   STOP LANE", ms),
                Action::ReleaseLane { .. } => println!("{:>7}ms  action   release lane", ms),
                Action::AlarmRelay { on, .. } => {
                    println!("{:>7}ms  action   alarm relay {}", ms, if *on { "ON" } else { "off" })
                }
                Action::RequestClip { from, to, .. } => println!(
                    "{:>7}ms  action   clip {:?}..{:?}",
                    ms, from, to
                ),
                Action::LocalVoicedAlert { text, .. } => {
                    println!("{:>7}ms  action   voice: {text}", ms)
                }
                Action::Degraded { health, reason } => {
                    println!("{:>7}ms  action   sensor {:?}: {reason}", ms, health)
                }
                Action::ProppedDoor { .. } => println!("{:>7}ms  action   PROPPED DOOR", ms),
                Action::Cleared { episode } => println!("{:>7}ms  action   ep{episode} cleared", ms),
            },
        }
    }
    let _ = SensorHealth::Nominal;
}
