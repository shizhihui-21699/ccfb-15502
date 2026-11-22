//! Shared scene builders: turn abstract people/carts into an input stream.

use door_tailgate::radar::RadarPoint;
use door_tailgate::time::Ts;
use door_tailgate::Input;

/// 25 Hz frame period in microseconds.
pub const FRAME_US: i64 = 40_000;
pub const TICK_US: i64 = 40_000;

/// A torso as a radar sees it: a few person-height returns.
/// A torso as a radar sees it: a spread of person-height returns. Real
/// millimetre-wave clusters carry tens of points per person; four are enough to
/// exercise the split logic but leave the cloud artificially sparse.
pub fn torso(x: f32, y: f32) -> Vec<RadarPoint> {
    let mut v = Vec::new();
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
        v.push(RadarPoint::new(x + dx, y + dy, 1.45).with_rcs(-5.0));
    }
    v
}

/// A cart body: a wide, low return set.
pub fn cart(x: f32, y: f32) -> Vec<RadarPoint> {
    let mut v = Vec::new();
    for i in 0..9 {
        let cx = x - 0.55 + 0.1375 * i as f32;
        v.push(RadarPoint::new(cx, y - 0.10, 0.45).with_rcs(-2.0));
        v.push(RadarPoint::new(cx, y + 0.10, 0.60).with_rcs(-2.0));
    }
    v
}

/// One walking actor in a scripted scene.
#[derive(Debug, Clone)]
pub struct Actor {
    /// Starting X, metres.
    pub x: f32,
    /// Starting Y, metres (positive = outside).
    pub y0: f32,
    /// Y at the end of the actor's walk.
    pub y1: f32,
    /// Start and end time of the walk, milliseconds.
    pub t0_ms: i64,
    pub t1_ms: i64,
    pub kind: ActorKind,
    pub badge_ms: Option<i64>,
    pub badge_cred: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorKind {
    Person,
    /// Cart plus its operator, travelling together; counts as one.
    CartWithOperator,
    /// A bare cart with no operator visible (rare; contributes no person).
    #[allow(dead_code)]
    CartAlone,
}

impl Actor {
    pub fn person(x: f32, y0: f32, y1: f32, t0_ms: i64, t1_ms: i64) -> Actor {
        Actor {
            x,
            y0,
            y1,
            t0_ms,
            t1_ms,
            kind: ActorKind::Person,
            badge_ms: None,
            badge_cred: "anon",
        }
    }

    pub fn with_badge(mut self, ms: i64, cred: &'static str) -> Actor {
        self.badge_ms = Some(ms);
        self.badge_cred = cred;
        self
    }

    pub fn cart(mut self, y0: f32, y1: f32, t0_ms: i64, t1_ms: i64) -> Actor {
        self.kind = ActorKind::CartWithOperator;
        self.y0 = y0;
        self.y1 = y1;
        self.t0_ms = t0_ms;
        self.t1_ms = t1_ms;
        self
    }

    fn y_at(&self, ms: f64) -> Option<f32> {
        if ms < self.t0_ms as f64 || ms > self.t1_ms as f64 {
            return None;
        }
        let f = (ms - self.t0_ms as f64) / (self.t1_ms - self.t0_ms) as f64;
        Some(self.y0 + (self.y1 - self.y0) * f as f32)
    }

    pub fn points(&self, ms: f64) -> Vec<RadarPoint> {
        let Some(y) = self.y_at(ms) else {
            return Vec::new();
        };
        match self.kind {
            ActorKind::Person => torso(self.x, y),
            // Cart body leading, pusher 0.55 m behind.
            ActorKind::CartWithOperator => {
                let mut p = cart(self.x, y);
                p.extend(torso(self.x, y + 0.55));
                p
            }
            ActorKind::CartAlone => cart(self.x, y),
        }
    }
}

/// Scripted door behaviour.
#[derive(Debug, Clone)]
pub struct DoorScript {
    /// (open_ms, close_ms) pairs.
    pub cycles: Vec<(i64, i64)>,
}

impl DoorScript {
    pub fn one(open_ms: i64, close_ms: i64) -> DoorScript {
        DoorScript {
            cycles: vec![(open_ms, close_ms)],
        }
    }
}

/// Build a full input timeline at 25 Hz from actors and a door script.
pub fn build_inputs(actors: &[Actor], door: &DoorScript, until_ms: i64) -> Vec<(i64, Input)> {
    let mut out: Vec<(i64, Input)> = Vec::new();
    let mut ms = 0i64;
    while ms <= until_ms {
        let us = ms * 1000;
        let ts = Ts::from_micros(us);
        // Radar frame.
        let mut pts = Vec::new();
        for a in actors {
            pts.extend(a.points(ms as f64));
        }
        out.push((ms, Input::Radar { ts, points: pts }));

        // Door contact: state sampled at this instant.
        let open = door.cycles.iter().any(|(o, c)| ms >= *o && ms < *c);
        out.push((ms, Input::Door(door_tailgate::door::DoorSignal::new(ts, open))));

        // Badges (one read per actor at its tap time; readers emit a burst).
        for a in actors {
            if let Some(b) = a.badge_ms {
                if ms == b || ms == b + 30 || ms == b + 60 {
                    out.push((
                        ms,
                        Input::Badge(door_tailgate::badge::BadgeRead::granted(
                            Ts::from_micros(ms * 1000),
                            a.badge_cred,
                        )),
                    ));
                }
            }
        }
        ms += FRAME_US / 1000;
    }
    out
}

/// Drive an engine with an input timeline, ticking between messages, and return
/// all events with their timestamps.
pub fn run(engine: &mut door_tailgate::Engine, timeline: Vec<(i64, Input)>) -> Vec<(i64, door_tailgate::Event)> {
    let mut all = Vec::new();
    let mut last_ms = 0i64;
    for (ms, input) in timeline {
        // Tick up to this timestamp in small steps so door debounce and alarm
        // scheduling see time pass.
        while last_ms + TICK_US / 1000 <= ms {
            last_ms += TICK_US / 1000;
            for e in engine.tick(Ts::from_micros(last_ms * 1000)) {
                all.push((last_ms, e));
            }
        }
        for e in engine.feed(input) {
            all.push((ms, e));
        }
        last_ms = last_ms.max(ms);
    }
    all
}

pub fn verdicts(events: &[(i64, door_tailgate::Event)]) -> Vec<(i64, door_tailgate::Verdict)> {
    events
        .iter()
        .filter_map(|(ms, e)| match e {
            door_tailgate::Event::Verdict(v) => Some((*ms, v.clone())),
            _ => None,
        })
        .collect()
}

pub fn actions(events: &[(i64, door_tailgate::Event)]) -> Vec<(i64, door_tailgate::Action)> {
    events
        .iter()
        .filter_map(|(ms, e)| match e {
            door_tailgate::Event::Action(a) => Some((*ms, a.clone())),
            _ => None,
        })
        .collect()
}
