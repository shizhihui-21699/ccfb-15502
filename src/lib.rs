//! # door_tailgate — edge tailgating detection for badge-controlled doors
//!
//! Runs **fully on-device**: no network, no wall clock, no cloud. Inputs are the
//! badge records, the door contact, an infrared beam, and a millimetre-wave
//! radar point cloud; the output is a per-passage trail and a tailgate verdict
//! with hardware actions (lane stop, alarm relay, camera clip request).
//!
//! ## Pipeline
//!
//! ```text
//!  radar cloud ─► cluster (DBSCAN) ─► split abreast ─► track (α-β) ─► inner-line crossings ─┐
//!  badge reader ─► debounce ───────────────────────────────────────────────────────────────┤
//!  door contact ─► debounce / episode boundaries ──────────────────────────────────────────┤─► episode
//!  infrared ─► trips (person / wide / noise) ──────────────────────────────────────────────┘   │
//!                                                                                             ▼
//!                                          passed (person equivalents) vs authorized (badges)
//!                                                                             │
//!                                          verdict ─► lane stop / relay / clip / voice
//! ```
//!
//! ## Design contract
//!
//! * **No allocation in the steady state** beyond the per-frame point and unit
//!   vectors (bounded by `tracker.max_tracks`); everything else is stack or
//!   reused buffers.
//! * **Deterministic.** Same input stream gives the same events, which makes the
//!   replay harness a regression test.
//! * **Fail-visible, not fail-open.** A blind sensor never yields a silent pass:
//!   the episode is marked degraded and any excess is at least `Suspect`.
//! * **Fail-safe at the door.** A hinged door with a closer is never locked by
//!   the interlock (egress). Only turnstiles and barriers can be stopped.

pub mod badge;
pub mod beam;
pub mod config;
pub mod door;
pub mod engine;
pub mod geom;
pub mod radar;
pub mod time;
pub mod tracker;

pub use config::Config;
pub use engine::{Action, Engine, Event, EvidenceSource, Input, Severity, Verdict};
pub use time::{Dur, Ts};

/// Replay-friendly high level facade used by the edge binary and the simulator.
///
/// Owns an [`Engine`] and exposes JSON-friendly entry points so the same code is
/// exercised in tests, the simulator, and the device.
#[derive(Debug)]
pub struct Controller {
    engine: Engine,
    /// Episodes seen so far, newest last, including the open one.
    pub episode_log: Vec<engine::Episode>,
}

impl Controller {
    pub fn new(cfg: Config) -> Controller {
        Controller {
            engine: Engine::new(cfg),
            episode_log: Vec::new(),
        }
    }

    pub fn engine(&self) -> &Engine {
        &self.engine
    }

    pub fn engine_mut(&mut self) -> &mut Engine {
        &mut self.engine
    }

    /// Feed an input and record any episode lifecycle events.
    pub fn feed(&mut self, input: Input) -> Vec<Event> {
        let events = self.engine.feed(input);
        self.capture(&events);
        events
    }

    pub fn tick(&mut self, now: Ts) -> Vec<Event> {
        let events = self.engine.tick(now);
        self.capture(&events);
        events
    }

    fn capture(&mut self, events: &[Event]) {
        for e in events {
            if let Event::EpisodeClosed { episode, .. } = e {
                if let Some(ep) = self
                    .engine
                    .recent_episodes()
                    .iter()
                    .find(|x| x.id == *episode)
                    .cloned()
                {
                    if !self.episode_log.iter().any(|x| x.id == ep.id) {
                        self.episode_log.push(ep);
                    }
                }
            }
        }
    }

    /// All verdicts that were emitted, in order.
    pub fn verdicts(&self) -> Vec<Verdict> {
        self.episode_log
            .iter()
            .filter_map(|e| e.verdict.clone())
            .collect()
    }
}
