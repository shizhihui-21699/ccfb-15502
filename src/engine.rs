//! Tailgating detection engine for a badge-controlled door.
//!
//! # Model
//!
//! The door is a **shared resource**: whoever taps opens it and the leaf stays
//! open long enough for several people. Attribution is therefore episode-based:
//!
//! ```text
//!   badge tap ──► door opens ──► passages ──► door closes + settle ──► verdict
//! ```
//!
//! An **episode** starts when the door opens (or when somebody crosses without
//! an episode, e.g. through a door left open), collects every badge read and
//! every counted passage, and is judged:
//!
//! * `authorized` — deduplicated granted badge reads attributable to the episode
//!   (a tap up to `attach_before_ms` before opening counts, a tap during counts,
//!   a tap up to `attach_after_ms` after the close counts because teammates tap
//!   while the leaf is still swinging).
//! * `passed` — person equivalents that crossed the inner line inwards, fused
//!   from radar and infrared (see [`Episode::passed`]).
//!
//! A tailgate alarm is raised when `passed > authorized`.
//!
//! # Why this survives the hard cases
//!
//! * **Carts.** The radar front end reports "worker pushing a trolley" as a
//!   single `Person` unit (a cart body becomes cargo only when nobody is visible
//!   with it), so a lone worker with a trolley gives `passed=1` against
//!   `authorized=1`. A second person behind the cart is a second `Person` and the
//!   alarm fires. If the cart hides that second person from the radar, the
//!   infrared person-trip behind the cart supplies the missing count.
//! * **Walking abreast.** Each colleague taps, so `authorized=2`, `passed=2`. Two
//!   people on one tap is a real tailgate.
//! * **Peak traffic.** Every teammate taps, so `authorized` tracks the crowd.
//!   The per-episode cap stops one stuck credential clearing a crowd; reader
//!   debounce folds repeated reads of a single card.
//! * **Immediate action.** The verdict does not wait for the door to close. Once
//!   `passed > authorized`, a verdict is scheduled `alarm_settle_ms` after the
//!   last passage (so a teammate tapping a beat later is counted) but never
//!   later than `alarm_max_delay_ms` after the first excess person — typically
//!   ~1 s after the tailgater crosses.

use crate::badge::{BadgeDecision, BadgeFilter, BadgeRead};
use crate::beam::{BeamMonitor, BeamSignal, BeamTrip, TripClass};
use crate::config::{Config, LaneKind};
use crate::door::{DoorMonitor, DoorSignal, DoorTransition};
use crate::geom::Travel;
use crate::radar::{process_frame, RadarPoint, Unit};
use crate::time::{Dur, Ts};
use crate::tracker::{Crossing, LineSet, Tracker};
use serde::{Deserialize, Serialize};

/// Health of the sensor set; drives degraded-mode behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorHealth {
    Nominal,
    /// Radar silent while the door or beam shows movement. Counts fall back to
    /// infrared and alarms become at least `Suspect`.
    RadarBlind,
    /// Neither radar nor infrared available.
    Blind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EpisodeState {
    Open,
    Settling,
    Closed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    /// High-confidence tailgate: counted people exceed authorizations.
    Alarm,
    /// Possible tailgate under degraded sensing or zero-badge entry.
    Suspect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceSource {
    Radar,
    Beam,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Passage {
    pub ts: Ts,
    pub track: Option<u32>,
    pub x: f32,
    pub pe: u8,
    pub source: EvidenceSource,
    pub note: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeamTripRecord {
    pub start: Ts,
    pub end: Ts,
    pub duration_ms: i64,
    pub class: TripClass,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TapRecord {
    pub ts: Ts,
    pub credential: String,
    pub reader: String,
    pub decision: Option<BadgeDecision>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Verdict {
    pub episode: u64,
    pub ts: Ts,
    pub severity: Severity,
    pub authorized: u32,
    pub passed: u32,
    /// `passed - authorized` when positive.
    pub excess: u32,
    pub health: SensorHealth,
    pub reasons: Vec<String>,
    /// Last crossing time, for a camera clip.
    pub last_crossing_at: Option<Ts>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Episode {
    pub id: u64,
    pub opened_at: Ts,
    pub closed_at: Option<Ts>,
    pub state: EpisodeState,
    pub authorized: u32,
    /// Radar person equivalents.
    pub radar_pe: u32,
    /// Infrared person-class trips in the entry direction.
    pub beam_person_trips: u32,
    /// Infrared wide (cart/queue) trips in the entry direction.
    pub beam_wide_trips: u32,
    pub passages: Vec<Passage>,
    pub beam_trips: Vec<BeamTripRecord>,
    pub badges: Vec<TapRecord>,
    pub health: SensorHealth,
    /// True once any radar frame was seen while this episode was open. If no
    /// radar ever contributed, infrared is the count of record for the whole
    /// episode even after `health` flips back to nominal on idle.
    pub radar_seen: bool,
    /// True once a verdict has been emitted for this episode.
    pub judged: bool,
    pub verdict: Option<Verdict>,
    /// Time of the last radar passage (for the alarm settle window).
    pub last_passage_at: Option<Ts>,
    /// Time the excess first appeared.
    pub excess_since: Option<Ts>,
}

impl Episode {
    /// Fused person count.
    ///
    /// * Infrared can only *add* people the radar missed (a pusher hidden behind
    ///   a cart, a blob that failed to split), and only up to `beam_raise_max`, so
    ///   a noisy beam cannot invent a crowd.
    /// * When the radar contributed **no unit at all** during the episode while
    ///   the infrared saw people — a dead sensor, or a sensor so degraded it sees
    ///   nothing — the infrared is the count of record. This keeps a blind radar
    ///   from silently passing a tailgater.
    pub fn passed(&self, cfg: &Config) -> u32 {
        if !self.radar_seen && self.radar_pe == 0 {
            // Radar saw nobody: trust the infrared.
            return self.beam_person_trips + self.beam_wide_trips;
        }
        let radar = self.radar_pe;
        if self.beam_person_trips == 0 {
            return radar;
        }
        let cap = radar.saturating_add(cfg.decision.beam_raise_max);
        radar.max(self.beam_person_trips.min(cap))
    }

    pub fn has_vehicle(&self) -> bool {
        self.passages.iter().any(|p| p.note.as_deref() == Some("vehicle"))
    }
}

/// Commands the engine emits. The edge binary maps these to hardware.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Action {
    /// Keep the turnstile/flap locked; refuse the next unlock. Never applies to
    /// a free-egress hinged door (see `interlock.lane_kind`).
    StopLane { episode: u64 },
    ReleaseLane { episode: u64 },
    /// Dry contact / relay for the alarm panel or a door-controller buzzer.
    AlarmRelay { episode: u64, on: bool },
    /// Ask the camera/NVR for a clip around the event.
    RequestClip { episode: u64, from: Ts, to: Ts },
    /// Loud local annunciation.
    LocalVoicedAlert { episode: u64, text: String },
    /// Sensor degraded; informational, forwarded to the ACS.
    Degraded { health: SensorHealth, reason: String },
    /// A door stayed open with no attribution.
    ProppedDoor { opened_at: Ts },
    /// Alarm cleared (operator or timeout).
    Cleared { episode: u64 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Event {
    Passage { episode: u64, passage: Passage },
    Badge { episode: u64, tap: TapRecord, authorized_total: u32 },
    EpisodeOpened { episode: u64, at: Ts, authorized: u32 },
    EpisodeClosed { episode: u64, at: Ts, authorized: u32, passed: u32 },
    Verdict(Verdict),
    Action(Action),
}

/// One input sample.
#[derive(Debug, Clone)]
pub enum Input {
    Radar { ts: Ts, points: Vec<RadarPoint> },
    Door(DoorSignal),
    Badge(BadgeRead),
    Beam(BeamSignal),
}

/// Which episode a late event is attributed to.
#[derive(Debug, Clone, Copy)]
enum Target {
    Current,
    Recent(usize),
}

/// How long without radar frames, while movement is visible, before declaring
/// the radar degraded.
const RADAR_BLIND_MS: i64 = 1200;
/// How long the "movement but no radar target" condition must hold before the
/// radar is declared degraded.
const DEGRADE_GRACE_MS: i64 = 600;

#[derive(Debug)]
pub struct Engine {
    cfg: Config,
    tracker: Tracker,
    lines: LineSet,
    door: DoorMonitor,
    beam: BeamMonitor,
    badges: BadgeFilter,
    episode: Option<Episode>,
    /// Closed episodes, kept so a late badge read can still clear a suspect.
    recent: Vec<Episode>,
    /// Granted taps seen before any episode, with expiry.
    pending_badges: Vec<(Ts, TapRecord)>,
    next_episode_id: u64,
    last_radar_ts: Option<Ts>,
    /// Last frame that produced at least one target (radar actually sensing).
    last_radar_unit_ts: Option<Ts>,
    degraded_since: Option<Ts>,
    last_beam_activity: Option<Ts>,
    now: Ts,
    health: SensorHealth,
    propped_reported: bool,
    lane_stopped: bool,
    lane_stopped_episode: Option<u64>,
    alarm_relay_on: bool,
    alarm_since: Option<Ts>,
    alarm_episode: Option<u64>,
    last_verdict: Option<Verdict>,
}

impl Engine {
    pub fn new(cfg: Config) -> Engine {
        let lines = LineSet {
            inner_y: cfg.lane.inner_line_m,
            outer_y: cfg.lane.outer_line_m,
            x_min: cfg.lane.x_min(),
            x_max: cfg.lane.x_max(),
        };
        Engine {
            tracker: Tracker::new(cfg.tracker.clone()),
            door: DoorMonitor::new(cfg.door.clone()),
            beam: BeamMonitor::new(cfg.beam.clone()),
            badges: BadgeFilter::new(cfg.badge.clone()),
            lines,
            cfg,
            episode: None,
            recent: Vec::new(),
            pending_badges: Vec::new(),
            next_episode_id: 1,
            last_radar_ts: None,
            last_radar_unit_ts: None,
            degraded_since: None,
            last_beam_activity: None,
            now: Ts::ZERO,
            health: SensorHealth::Nominal,
            propped_reported: false,
            lane_stopped: false,
            lane_stopped_episode: None,
            alarm_relay_on: false,
            alarm_since: None,
            alarm_episode: None,
            last_verdict: None,
        }
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn now(&self) -> Ts {
        self.now
    }

    pub fn health(&self) -> SensorHealth {
        self.health
    }

    pub fn current_episode(&self) -> Option<&Episode> {
        self.episode.as_ref()
    }

    pub fn recent_episodes(&self) -> &[Episode] {
        &self.recent
    }

    pub fn last_verdict(&self) -> Option<&Verdict> {
        self.last_verdict.as_ref()
    }

    pub fn lane_stopped(&self) -> bool {
        self.lane_stopped
    }

    pub fn alarm_relay_on(&self) -> bool {
        self.alarm_relay_on
    }

    pub fn tracker_snapshot(&self) -> Vec<crate::tracker::TrackSnapshot> {
        self.tracker.snapshots()
    }

    /// Feed one input; returns events to publish.
    pub fn feed(&mut self, input: Input) -> Vec<Event> {
        let mut out = Vec::new();
        match input {
            Input::Radar { ts, points } => self.on_radar(ts, points, &mut out),
            Input::Door(sig) => self.on_door(sig, &mut out),
            Input::Badge(read) => self.on_badge(read, &mut out),
            Input::Beam(sig) => self.on_beam(sig, &mut out),
        }
        out
    }

    /// Time-driven housekeeping: door debounce, propped-door timer, alarm
    /// settle, episode timeout, sensor health. Call at 10–25 Hz.
    pub fn tick(&mut self, now: Ts) -> Vec<Event> {
        let mut out = Vec::new();
        self.now = self.now.max(now);
        let now = self.now;

        if let Some(t) = self.door.poll(now) {
            self.on_door_transition(t, &mut out);
        }
        if let Some(trip) = self.beam.poll(now) {
            self.on_beam_trip(trip, &mut out);
        }
        self.update_health(now, &mut out);
        self.check_propped(now, &mut out);
        self.maybe_judge(now, &mut out);
        self.maybe_finalize(now, &mut out);
        self.expire_pending_badges(now);
        self.maybe_clear_alarm(now, &mut out);
        out
    }

    // ----------------------------------------------------------------- radar

    fn on_radar(&mut self, ts: Ts, points: Vec<RadarPoint>, out: &mut Vec<Event>) {
        self.now = self.now.max(ts);
        self.last_radar_ts = Some(ts);
        let units: Vec<Unit> = process_frame(ts, &points, &self.cfg.radar);
        // `radar_seen` means the radar reported at least one target, not merely
        // that a frame arrived: an empty frame from a dead or blinded sensor must
        // not make the radar look authoritative.
        if !units.is_empty() {
            self.last_radar_unit_ts = Some(ts);
            if let Some(ep) = self.episode.as_mut() {
                ep.radar_seen = true;
            }
        }
        let lines = self.lines.clone();
        // A cart in the lane can hide a person behind it from the radar; let
        // tracks coast through that dropout, and a wide infrared block is the
        // same signal when the radar itself saw nothing.
        let occluded = self.tracker.cart_present(ts)
            || self.episode.as_ref().map(|e| e.beam_wide_trips > 0).unwrap_or(false);
        let crossings = self.tracker.update(ts, &units, &lines, occluded);
        for c in crossings {
            self.on_crossing(c, out);
        }
    }

    fn on_crossing(&mut self, c: Crossing, out: &mut Vec<Event>) {
        if c.line_idx != 0 || c.travel != Travel::In || c.pe == 0 {
            return;
        }
        if self.episode.is_none() {
            // Someone crossed without an open episode (door left open).
            self.open_episode(c.ts, out);
        }
        let ep = self.episode.as_mut().expect("opened above");
        ep.radar_pe = ep.radar_pe.saturating_add(c.pe as u32);
        ep.last_passage_at = Some(c.ts);
        let note = match c.class {
            crate::tracker::UnitClass::Vehicle => Some("vehicle".to_string()),
            crate::tracker::UnitClass::Person => None,
        };
        let passage = Passage {
            ts: c.ts,
            track: Some(c.track),
            x: c.x,
            pe: c.pe,
            source: EvidenceSource::Radar,
            note,
        };
        ep.passages.push(passage.clone());
        let id = ep.id;
        out.push(Event::Passage { episode: id, passage });
    }

    // ------------------------------------------------------------------ door

    fn on_door(&mut self, sig: DoorSignal, out: &mut Vec<Event>) {
        self.now = self.now.max(sig.ts);
        if let Some(t) = self.door.update(sig) {
            self.on_door_transition(t, out);
        }
    }

    fn on_door_transition(&mut self, t: DoorTransition, out: &mut Vec<Event>) {
        match t {
            DoorTransition::Opened(at) => {
                self.propped_reported = false;
                match self.episode.as_mut() {
                    Some(ep) => {
                        // Re-open while settling keeps the same episode.
                        ep.state = EpisodeState::Open;
                        ep.closed_at = None;
                    }
                    None => {
                        self.open_episode(at, out);
                    }
                }
            }
            DoorTransition::Closed(at) => {
                if let Some(ep) = self.episode.as_mut() {
                    ep.closed_at = Some(at);
                    ep.state = EpisodeState::Settling;
                    // A close event with no pending movement can finalize after
                    // the settle window; handled in `maybe_finalize`.
                }
            }
        }
    }

    // ----------------------------------------------------------------- badge

    fn on_badge(&mut self, read: BadgeRead, out: &mut Vec<Event>) {
        self.now = self.now.max(read.ts);
        let granted = !matches!(read.decision, Some(BadgeDecision::Denied));
        let first_time = self.badges.accept(&read).is_some();
        let record = TapRecord {
            ts: read.ts,
            credential: read.credential,
            reader: read.reader,
            decision: read.decision,
        };
        let after = Dur::ms(self.cfg.badge.attach_after_ms);

        // Where does this tap belong?
        // 1. Open or still-settling episode, inside its window.
        let target = if let Some(ep) = self.episode.as_ref() {
            let within = match ep.state {
                EpisodeState::Open => true,
                EpisodeState::Settling => ep
                    .closed_at
                    .map(|c| read.ts.since(c) <= after)
                    .unwrap_or(true),
                EpisodeState::Closed => false,
            };
            within.then_some(Target::Current)
        } else {
            None
        };
        // 2. A just-closed episode still inside its after-window.
        let target = target.or_else(|| {
            let ep = self.recent.last()?;
            let closed = ep.closed_at?;
            (read.ts.since(closed) <= after).then_some(Target::Recent(self.recent.len() - 1))
        });

        if let Some(target) = target {
            self.record_badge(target, record, granted, first_time, out);
            return;
        }

        // 3. Nothing open: hold as a pre-authorization for an imminent opening,
        //    and open a provisional episode so the tap is not lost.
        self.pending_badges.push((read.ts, record));
        if self.episode.is_none() {
            self.open_episode(read.ts, out);
        }
    }

    fn record_badge(
        &mut self,
        target: Target,
        record: TapRecord,
        granted: bool,
        first_time: bool,
        out: &mut Vec<Event>,
    ) {
        let max_auth = self.cfg.decision.max_authorized_per_episode;
        let ep = match target {
            Target::Current => match self.episode.as_mut() {
                Some(e) => e,
                None => return,
            },
            Target::Recent(i) => &mut self.recent[i],
        };
        let was_judged = ep.judged;
        let cleared = ep.verdict.as_ref().map(|v| v.severity) == Some(Severity::Suspect);
        ep.badges.push(record.clone());
        if granted && first_time && ep.authorized < max_auth {
            ep.authorized += 1;
        }
        let total = ep.authorized;
        let id = ep.id;
        out.push(Event::Badge {
            episode: id,
            tap: record,
            authorized_total: total,
        });
        // A suspect verdict is cleared if the late authorization covers the
        // passages; a firm alarm stands until an operator clears it.
        if was_judged && cleared {
            let covered = ep.passed(&self.cfg) <= ep.authorized;
            if covered {
                ep.verdict = None;
                ep.judged = false;
            }
            if covered {
                out.push(Event::Action(Action::Cleared { episode: id }));
                self.clear_alarm(id, out);
            }
        }
    }

    // ------------------------------------------------------------------ beam

    fn on_beam(&mut self, sig: BeamSignal, out: &mut Vec<Event>) {
        self.now = self.now.max(sig.ts);
        if let Some(trip) = self.beam.update(sig) {
            self.on_beam_trip(trip, out);
        }
    }

    fn on_beam_trip(&mut self, trip: BeamTrip, out: &mut Vec<Event>) {
        self.last_beam_activity = Some(trip.end);
        if trip.direction == Some(Travel::Out) {
            return;
        }
        if trip.class == TripClass::Noise {
            return;
        }
        // A beam trip with no episode means movement through an open door before
        // radar saw it; open the episode so the trip is attributed.
        if self.episode.is_none() {
            self.open_episode(trip.start, out);
        }
        if let Some(ep) = self.episode.as_mut() {
            match trip.class {
                TripClass::Person => ep.beam_person_trips = ep.beam_person_trips.saturating_add(1),
                TripClass::Wide => ep.beam_wide_trips = ep.beam_wide_trips.saturating_add(1),
                TripClass::Noise => {}
            }
            ep.beam_trips.push(BeamTripRecord {
                start: trip.start,
                end: trip.end,
                duration_ms: trip.duration.millis_i64(),
                class: trip.class,
            });
            if ep.last_passage_at.map(|t| trip.end > t).unwrap_or(true) {
                ep.last_passage_at = Some(trip.end);
            }
        }
    }

    // -------------------------------------------------------------- episodes

    fn open_episode(&mut self, at: Ts, out: &mut Vec<Event>) -> u64 {
        let id = self.next_episode_id;
        self.next_episode_id += 1;
        let mut ep = Episode {
            id,
            opened_at: at,
            closed_at: None,
            state: EpisodeState::Open,
            authorized: 0,
            radar_pe: 0,
            beam_person_trips: 0,
            beam_wide_trips: 0,
            passages: Vec::new(),
            beam_trips: Vec::new(),
            badges: Vec::new(),
            health: self.health,
            judged: false,
            verdict: None,
            last_passage_at: None,
            excess_since: None,
            radar_seen: false,
        };
        // Attach pre-taps inside the before-window.
        let window = Dur::ms(self.cfg.badge.attach_before_ms);
        let mut adopted = 0u32;
        let mut used = Vec::new();
        for (i, (ts, rec)) in self.pending_badges.iter().enumerate() {
            if *ts <= at && at.since(*ts) <= window {
                ep.badges.push(rec.clone());
                if !matches!(rec.decision, Some(BadgeDecision::Denied))
                    && ep.authorized < self.cfg.decision.max_authorized_per_episode
                {
                    ep.authorized += 1;
                    adopted += 1;
                }
                used.push(i);
            }
        }
        for i in used.into_iter().rev() {
            self.pending_badges.remove(i);
        }
        let authorized = ep.authorized;
        self.episode = Some(ep);
        self.propped_reported = false;
        let _ = adopted;
        out.push(Event::EpisodeOpened {
            episode: id,
            at,
            authorized,
        });
        id
    }

    /// Emit a verdict as soon as the excess is confirmed and has settled.
    fn maybe_judge(&mut self, now: Ts, out: &mut Vec<Event>) {
        let Some(ep) = self.episode.as_mut() else { return };
        if ep.judged {
            return;
        }
        let passed = ep.passed(&self.cfg);
        if passed <= ep.authorized {
            ep.excess_since = None;
            return;
        }
        let first = *ep.excess_since.get_or_insert(now);
        let last = ep.last_passage_at.unwrap_or(first);
        let settle_due = last + Dur::ms(self.cfg.decision.alarm_settle_ms);
        let cap_due = first + Dur::ms(self.cfg.decision.alarm_max_delay_ms);
        if now < settle_due.min(cap_due) {
            return;
        }
        // One last chance: a teammate's badge might still attach this instant.
        let verdict = self.make_verdict(now);
        let Some(v) = verdict else { return };
        if let Some(ep) = self.episode.as_mut() {
            ep.verdict = Some(v.clone());
            ep.judged = true;
        }
        self.last_verdict = Some(v.clone());
        self.emit_verdict(v, out);
    }

    fn make_verdict(&self, now: Ts) -> Option<Verdict> {
        let ep = self.episode.as_ref()?;
        let passed = ep.passed(&self.cfg);
        let authorized = ep.authorized;
        if passed <= authorized {
            return None;
        }
        Some(self.build_verdict(ep, passed, authorized, now))
    }

    fn build_verdict(&self, ep: &Episode, passed: u32, authorized: u32, now: Ts) -> Verdict {
        let excess = passed - authorized;
        let health = ep.health;
        let mut reasons = Vec::new();
        let zero_badge = authorized == 0;

        let severity = if health != SensorHealth::Nominal
            || (zero_badge && !self.cfg.decision.zero_badge_alarm)
        {
            Severity::Suspect
        } else {
            Severity::Alarm
        };

        if zero_badge {
            reasons.push(format!("{passed} passage(s) with no granted badge"));
        } else {
            reasons.push(format!("{passed} person(s) passed, {authorized} badge(s) granted"));
        }
        if excess > 1 {
            reasons.push(format!("excess {excess}: possible group entry"));
        }
        if ep.beam_person_trips > ep.radar_pe {
            reasons.push(format!(
                "infrared saw {} person trip(s) vs {} by radar",
                ep.beam_person_trips, ep.radar_pe
            ));
        }
        if ep.beam_wide_trips > 0 {
            reasons.push("wide infrared block: cart or queue".into());
        }
        // Door-cycle duration as corroborating evidence: many people through a
        // very short opening, or a normal count through a long hold, both help
        // an operator triage the clip. The verdict itself stays count-based so a
        // slow closer or a held door cannot flip it.
        if let Some(closed) = ep.closed_at {
            let open_ms = closed.since(ep.opened_at).millis_i64();
            if open_ms > 0 {
                let per_person = open_ms / passed.max(1) as i64;
                if open_ms < 1200 && passed > 1 {
                    reasons.push(format!(
                        "{passed} people through a {open_ms} ms opening: fast, tight follow"
                    ));
                } else {
                    reasons.push(format!("door was open {open_ms} ms ({per_person} ms/person)"));
                }
            }
        }
        if health != SensorHealth::Nominal {
            reasons.push("degraded sensing during this episode".into());
        }
        Verdict {
            episode: ep.id,
            ts: now,
            severity,
            authorized,
            passed,
            excess,
            health,
            reasons,
            last_crossing_at: ep.last_passage_at,
        }
    }

    fn emit_verdict(&mut self, v: Verdict, out: &mut Vec<Event>) {
        let ep = v.episode;
        out.push(Event::Verdict(v.clone()));
        // Lanes that can be stopped are; a free-egress door never is. A suspect
        // verdict stops the lane too unless configured otherwise: under a blind
        // sensor a possible tailgate is still worth a short hold.
        let stop_for_severity = match v.severity {
            Severity::Alarm => self.cfg.interlock.stop_lane_on_alarm,
            Severity::Suspect => self.cfg.interlock.stop_lane_on_suspect,
        };
        if stop_for_severity && self.cfg.interlock.lane_kind != LaneKind::Door {
            self.lane_stopped = true;
            self.lane_stopped_episode = Some(ep);
            out.push(Event::Action(Action::StopLane { episode: ep }));
        }
        self.alarm_relay_on = true;
        self.alarm_since = Some(v.ts);
        self.alarm_episode = Some(ep);
        out.push(Event::Action(Action::AlarmRelay { episode: ep, on: true }));
        let last = v.last_crossing_at.unwrap_or(v.ts);
        out.push(Event::Action(Action::RequestClip {
            episode: ep,
            from: last.saturating_sub(Dur::ms(self.cfg.interlock.clip_pre_ms)).max(Ts::ZERO),
            to: last.saturating_add(Dur::ms(self.cfg.interlock.clip_post_ms)),
        }));
        if v.severity == Severity::Alarm {
            let text = if v.excess > 1 {
                format!("{} 人仅 {} 人刷卡，请重新刷卡", v.passed, v.authorized)
            } else {
                "请勿尾随，请单独刷卡".to_string()
            };
            out.push(Event::Action(Action::LocalVoicedAlert { episode: ep, text }));
        }
    }

    /// Close the episode once the door has been shut and settled.
    fn maybe_finalize(&mut self, now: Ts, out: &mut Vec<Event>) {
        let Some(ep) = self.episode.as_ref() else { return };
        let settled = match (ep.state, ep.closed_at) {
            (EpisodeState::Settling, Some(closed)) => {
                now.since(closed) >= Dur::ms(self.cfg.door.settle_ms)
            }
            (EpisodeState::Open, _) => {
                now.since(ep.opened_at) >= Dur::ms(self.cfg.door.max_episode_ms)
            }
            _ => false,
        };
        if settled {
            self.finalize_episode(now, out);
        }
    }

    fn finalize_episode(&mut self, now: Ts, out: &mut Vec<Event>) {
        let Some(mut ep) = self.episode.take() else { return };
        let passed = ep.passed(&self.cfg);
        // If no verdict was emitted yet, judge the whole episode now.
        if !ep.judged && passed > ep.authorized {
            let v = self.build_verdict(&ep, passed, ep.authorized, now);
            ep.verdict = Some(v.clone());
            ep.judged = true;
            self.last_verdict = Some(v.clone());
            self.emit_verdict(v, out);
        }
        ep.state = EpisodeState::Closed;
        ep.closed_at = Some(ep.closed_at.unwrap_or(now));
        let id = ep.id;
        let authorized = ep.authorized;
        out.push(Event::EpisodeClosed {
            episode: id,
            at: ep.closed_at.unwrap_or(now),
            authorized,
            passed,
        });
        self.recent.push(ep);
        if self.recent.len() > 16 {
            self.recent.remove(0);
        }
    }

    fn expire_pending_badges(&mut self, now: Ts) {
        let window = Dur::ms(self.cfg.badge.attach_before_ms);
        self.pending_badges
            .retain(|(ts, _)| now.since(*ts) <= window);
        // A provisional episode opened by a lone tap but never followed by a
        // door opening or any passage is cancelled.
        if let Some(ep) = self.episode.as_ref() {
            if ep.passages.is_empty()
                && ep.beam_person_trips == 0
                && ep.beam_wide_trips == 0
                && ep.closed_at.is_none()
                && !self.door.is_open()
                && now.since(ep.opened_at) > window
            {
                self.episode = None;
            }
        }
    }

    fn check_propped(&mut self, now: Ts, out: &mut Vec<Event>) {
        let Some(ep) = self.episode.as_ref() else { return };
        // A door held open past `propped_ms` is an alarm regardless of traffic:
        // propping defeats per-person attribution entirely.
        if self.door.is_open()
            && !self.propped_reported
            && now.since(ep.opened_at) >= Dur::ms(self.cfg.door.propped_ms)
        {
            self.propped_reported = true;
            out.push(Event::Action(Action::ProppedDoor {
                opened_at: ep.opened_at,
            }));
        }
    }

    fn maybe_clear_alarm(&mut self, now: Ts, out: &mut Vec<Event>) {
        if !self.alarm_relay_on || self.cfg.interlock.latch_alarm {
            return;
        }
        let Some(since) = self.alarm_since else { return };
        if now.since(since) >= Dur::ms(self.cfg.interlock.auto_clear_ms) {
            if let Some(ep) = self.alarm_episode {
                self.clear_alarm(ep, out);
                out.push(Event::Action(Action::Cleared { episode: ep }));
            }
        }
    }

    fn clear_alarm(&mut self, ep: u64, out: &mut Vec<Event>) {
        if self.alarm_relay_on {
            self.alarm_relay_on = false;
            self.alarm_since = None;
            self.alarm_episode = None;
            out.push(Event::Action(Action::AlarmRelay { episode: ep, on: false }));
        }
        if self.lane_stopped {
            self.lane_stopped = false;
            self.lane_stopped_episode = None;
            out.push(Event::Action(Action::ReleaseLane { episode: ep }));
        }
    }

    /// Operator/ACS clears the current alarm (e.g. after checking the camera).
    pub fn clear_alarm_external(&mut self, now: Ts) -> Vec<Event> {
        let mut out = Vec::new();
        self.now = self.now.max(now);
        if let Some(ep) = self.alarm_episode {
            self.clear_alarm(ep, &mut out);
        }
        out
    }

    // ---------------------------------------------------------------- health

    fn update_health(&mut self, now: Ts, out: &mut Vec<Event>) {
        let frames_fresh = self
            .last_radar_ts
            .map(|t| now.since(t).millis_i64() <= RADAR_BLIND_MS)
            .unwrap_or(false);
        // "Effective" radar means it produced at least one target recently; a
        // stream of empty frames from a blinded sensor is not effective.
        let units_fresh = self
            .last_radar_unit_ts
            .map(|t| now.since(t).millis_i64() <= RADAR_BLIND_MS)
            .unwrap_or(false);
        // "People are moving through the lane right now": the door is open, or
        // the beam is currently broken. This is deliberately a *current* signal,
        // not a trailing window — radar legitimately sees nothing in the instant
        // after a person has passed, and that must not read as a blind sensor.
        let movement = self.door.is_open() || self.beam.is_blocked();
        // Degraded when movement is visible but radar is not contributing:
        // either no frames at all, or frames that never resolve to a target.
        // A short grace period debounces the instant between a door opening and
        // the radar's first frame resolving a target.
        let radar_effective = frames_fresh && (units_fresh || !movement);
        let degraded_now = !radar_effective && movement;
        // Track how long the degraded condition has continuously held.
        let degraded_since = match (degraded_now, self.degraded_since) {
            (true, Some(s)) => Some(s),
            (true, None) => Some(now),
            (false, _) => None,
        };
        self.degraded_since = degraded_since;
        let sustained = degraded_since
            .map(|s| now.since(s) >= Dur::ms(DEGRADE_GRACE_MS))
            .unwrap_or(false);
        let health = if sustained {
            if self.cfg.beam.enabled {
                SensorHealth::RadarBlind
            } else {
                SensorHealth::Blind
            }
        } else {
            SensorHealth::Nominal
        };
        if health != self.health {
            let reason = match health {
                SensorHealth::Nominal => "radar sensing resumed".to_string(),
                SensorHealth::RadarBlind => {
                    if !frames_fresh {
                        "no radar frames while the door/beam show movement".to_string()
                    } else {
                        "radar frames carry no targets while the door/beam show movement".to_string()
                    }
                }
                SensorHealth::Blind => "radar not sensing and infrared disabled".to_string(),
            };
            self.health = health;
            out.push(Event::Action(Action::Degraded { health, reason }));
            if let Some(ep) = self.episode.as_mut() {
                if health != SensorHealth::Nominal {
                    ep.health = health;
                }
            }
        }
    }
}
