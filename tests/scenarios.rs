//! End-to-end scenarios against the full engine.
//!
//! These are the acceptance tests for the field problems: single normal entry,
//! tailgating, carts, walking abreast, peak traffic, badge abuse, propped doors,
//! degraded radar, and alarm latency.

mod common;

use common::*;
use door_tailgate::radar::RadarPoint;
use door_tailgate::time::Ts;
use door_tailgate::{Action, Config, Engine, Event, Input, Severity};

fn engine() -> Engine {
    Engine::new(Config::default())
}

// ---------------------------------------------------------------- baselines

#[test]
fn single_person_with_badge_is_not_an_alarm() {
    let a = Actor::person(0.0, 3.0, -1.0, 1000, 3200).with_badge(600, "alice");
    let door = DoorScript::one(900, 3400);
    let timeline = build_inputs(&[a], &door, 6000);
    let events = run(&mut engine(), timeline);
    assert!(verdicts(&events).is_empty(), "no alarm: {events:?}");
}

#[test]
fn tailgater_raises_alarm_and_stops_the_lane() {
    // Alice badges and walks in; Bob follows 0.6 s behind with no badge.
    let alice = Actor::person(0.0, 3.0, -1.0, 1000, 3200).with_badge(700, "alice");
    let bob = Actor::person(0.0, 3.4, -1.0, 1600, 3800);
    let door = DoorScript::one(900, 4000);
    let timeline = build_inputs(&[alice, bob], &door, 7000);
    let events = run(&mut engine(), timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "one verdict: {vs:?}");
    assert_eq!(vs[0].1.severity, Severity::Alarm);
    assert_eq!(vs[0].1.passed, 2);
    assert_eq!(vs[0].1.authorized, 1);
    assert!(actions(&events)
        .iter()
        .any(|(_, a)| matches!(a, Action::StopLane { .. })));
}

#[test]
fn alarm_is_raised_within_a_few_seconds_of_the_crossing() {
    let alice = Actor::person(0.0, 3.0, -1.0, 1000, 3000).with_badge(700, "alice");
    let bob = Actor::person(0.2, 3.2, -1.0, 1400, 3400);
    let door = DoorScript::one(900, 3600);
    let timeline = build_inputs(&[alice, bob], &door, 8000);
    let mut e = engine();
    let events = run(&mut e, timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1);
    // Bob crosses the inner line (y=-0.35) at about
    //   1400 + (3.2 + 0.35) / (4.2 m over 2000 ms) ≈ 3090 ms.
    // The alarm must fire well within "immediately": <= 1.5 s later, and long
    // before the door closes at 3600 ms + settle.
    let crossing_ms = 3090;
    let delay = vs[0].0 - crossing_ms;
    assert!(
        (0..=1500).contains(&delay),
        "verdict at {} ms, crossing ~{} ms, delay {} ms",
        vs[0].0,
        crossing_ms,
        delay
    );
}

// -------------------------------------------------------------------- carts

#[test]
fn worker_with_cart_alone_is_not_an_alarm() {
    let worker = Actor::person(0.0, 6.0, -1.0, 1000, 6500)
        .cart(5.45, -1.55, 1000, 5900)
        .with_badge(700, "worker");
    let door = DoorScript::one(900, 6700);
    let timeline = build_inputs(&[worker], &door, 9000);
    let events = run(&mut engine(), timeline);
    assert!(verdicts(&events).is_empty(), "cart alone must not alarm: {events:?}");
}

#[test]
fn cart_plus_hidden_tailgater_alarms_via_infrared() {
    // The cart occludes the radar for a moment and the pusher is not seen; the
    // beam sees a person trip the radar missed, which is what bumps the count.
    let worker = Actor::person(0.0, 5.0, -1.0, 1000, 5000)
        .cart(4.45, -1.55, 1000, 5000)
        .with_badge(700, "worker");
    let bob = Actor::person(0.02, 3.0, -1.0, 2600, 4600);
    let door = DoorScript::one(900, 5200);
    let timeline = build_inputs(&[worker, bob], &door, 8000);
    let events = run(&mut engine(), timeline);
    let vs = verdicts(&events);
    assert!(!vs.is_empty(), "should alarm: {events:?}");
    assert!(vs[0].1.passed > vs[0].1.authorized);
}

#[test]
fn a_cart_does_not_hide_the_person_who_follows_it() {
    // Regression for the hardest field case: worker pushes a cart, a second
    // person walks immediately behind them through the same opening. The cart
    // and its pusher must fold to ONE; the follower is a second person.
    let worker = Actor::person(0.0, 5.0, -1.0, 1000, 5000)
        .cart(4.45, -1.55, 1000, 5000)
        .with_badge(700, "worker");
    let follower = Actor::person(0.06, 3.6, -1.0, 2250, 4250);
    let door = DoorScript::one(900, 5400);
    let events = run(&mut engine(), build_inputs(&[worker, follower], &door, 9000));
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "{vs:?}");
    assert_eq!(vs[0].1.authorized, 1);
    assert_eq!(vs[0].1.passed, 2, "cart+operator is one, follower is the second");
}

#[test]
fn a_worker_with_cart_followed_by_two_people_counts_three() {
    let worker = Actor::person(0.0, 5.0, -1.0, 1000, 5000)
        .cart(4.45, -1.55, 1000, 5000)
        .with_badge(700, "worker");
    let f1 = Actor::person(-0.3, 3.6, -1.0, 2250, 4250);
    let f2 = Actor::person(0.3, 3.7, -1.0, 2300, 4300);
    let door = DoorScript::one(900, 5400);
    let events = run(&mut engine(), build_inputs(&[worker, f1, f2], &door, 9000));
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "{vs:?}");
    assert_eq!(vs[0].1.authorized, 1);
    assert_eq!(vs[0].1.passed, 3);
}

#[test]
fn two_people_walking_abreast_each_badge_is_fine() {
    let a = Actor::person(-0.35, 3.0, -1.0, 1000, 3200).with_badge(600, "a");
    let b = Actor::person(0.35, 3.0, -1.0, 1030, 3230).with_badge(650, "b");
    let door = DoorScript::one(900, 3500);
    let timeline = build_inputs(&[a, b], &door, 6000);
    let events = run(&mut engine(), timeline);
    assert!(verdicts(&events).is_empty(), "two badges, two people: {events:?}");
}

#[test]
fn two_people_walking_abreast_one_badge_alarms() {
    let a = Actor::person(-0.35, 3.0, -1.0, 1000, 3200).with_badge(600, "a");
    let b = Actor::person(0.35, 3.0, -1.0, 1030, 3230);
    let door = DoorScript::one(900, 3500);
    let timeline = build_inputs(&[a, b], &door, 6000);
    let events = run(&mut engine(), timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "{vs:?}");
    assert_eq!(vs[0].1.passed, 2);
    assert_eq!(vs[0].1.authorized, 1);
}

// ------------------------------------------------------------- peak traffic

#[test]
fn peak_traffic_of_individual_badges_never_alarms() {
    // Eight people, each taps, entries every 1.2 s: a morning rush through one
    // open door. None of them is a tailgater.
    let mut actors = Vec::new();
    for i in 0..8 {
        let t0 = 1000 + i * 1200;
        let x = if i % 2 == 0 { -0.2 } else { 0.2 };
        actors.push(
            Actor::person(x, 3.0, -1.0, t0, t0 + 2000)
                .with_badge(t0 - 300, Box::leak(format!("emp{i}").into_boxed_str())),
        );
    }
    let door = DoorScript::one(800, 11000);
    let timeline = build_inputs(&actors, &door, 16000);
    let events = run(&mut engine(), timeline);
    assert!(verdicts(&events).is_empty(), "peak traffic must not alarm: {events:?}");
}

#[test]
fn a_rush_with_one_walk_in_alarms_exactly_once() {
    let mut actors = Vec::new();
    for i in 0..6 {
        // 1.4 s headway at 1.4 m/s keeps 2 m between people: no overlap.
        let t0 = 1000 + i * 1400;
        actors.push(
            Actor::person(0.0, 3.0, -1.0, t0, t0 + 2857).with_badge(
                t0 - 300,
                Box::leak(format!("emp{i}").into_boxed_str()),
            ),
        );
    }
    // One unbadged walk-in between the third and fourth teammate.
    actors.push(Actor::person(0.0, 3.0, -1.0, 3900, 5757));
    let door = DoorScript::one(800, 10000);
    let timeline = build_inputs(&actors, &door, 16000);
    let events = run(&mut engine(), timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "exactly one alarm: {vs:?}");
    assert_eq!(vs[0].1.authorized, 6);
    // Verdict fires at the moment of excess; the final episode tally is 7.
    let closed = events.iter().find_map(|(_, e)| match e {
        Event::EpisodeClosed { passed, authorized, .. } => Some((*passed, *authorized)),
        _ => None,
    });
    assert_eq!(closed, Some((7, 6)));
}

// -------------------------------------------------------------- badge abuse

#[test]
fn one_credential_tapped_repeatedly_counts_once() {
    // A held card re-reads many times during the burst window.
    let a = Actor::person(0.0, 3.0, -1.0, 1000, 3200);
    let door = DoorScript::one(900, 3400);
    let mut timeline = build_inputs(&[a], &door, 6000);
    // Add extra reads at 700, 760, 820 ms (inside the 700 ms debounce window of
    // each other only if closer than debounce; these are 60 ms apart).
    for ms in [700i64, 760, 820, 880] {
        timeline.push((
            ms,
            Input::Badge(door_tailgate::badge::BadgeRead::granted(
                Ts::from_micros(ms * 1000),
                "alice",
            )),
        ));
    }
    timeline.sort_by_key(|(ms, _)| *ms);
    let events = run(&mut engine(), timeline);
    assert!(verdicts(&events).is_empty(), "repeated reads are one tap: {events:?}");
}

#[test]
fn per_episode_authorization_cap_holds() {
    // Five people each tap in one open-door episode, but the site caps a single
    // episode at 2 authorizations (e.g. a shared/public credential policy).
    // Only the first two clear the way; the rest exceed the cap.
    let mut cfg = Config::default();
    cfg.decision.max_authorized_per_episode = 2;
    let mut e = Engine::new(cfg);
    let mut actors = Vec::new();
    for i in 0..5 {
        let t0 = 1000 + i * 900;
        actors.push(Actor::person(0.0, 3.0, -1.0, t0, t0 + 2000).with_badge(
            t0 - 200,
            Box::leak("dup".to_string().into_boxed_str()),
        ));
    }
    let door = DoorScript::one(800, 7000);
    let timeline = build_inputs(&actors, &door, 11000);
    let events = run(&mut e, timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "{vs:?}");
    // The verdict fires as soon as the excess is provable, so its `passed` is a
    // snapshot "at least this many"; the cap holds at 2 authorizations.
    assert_eq!(vs[0].1.authorized, 2);
    assert!(vs[0].1.passed >= 3, "excess at trigger: {}", vs[0].1.passed);
    // The episode's final tally accounts for all five passages.
    let closed = events.iter().find_map(|(_, e)| match e {
        Event::EpisodeClosed { passed, authorized, .. } => Some((*passed, *authorized)),
        _ => None,
    });
    assert_eq!(closed, Some((5, 2)));
}

// ------------------------------------------------------------- door states

#[test]
fn zero_badge_entry_alarms() {
    let a = Actor::person(0.0, 3.0, -1.0, 1000, 3200);
    let door = DoorScript::one(900, 3400);
    let timeline = build_inputs(&[a], &door, 7000);
    let events = run(&mut engine(), timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "{vs:?}");
    assert_eq!(vs[0].1.authorized, 0);
    assert_eq!(vs[0].1.severity, Severity::Alarm);
}

#[test]
fn propped_door_alarms() {
    let a = Actor::person(0.0, 3.0, -1.0, 1000, 3200).with_badge(700, "alice");
    // Door never closes.
    let mut door = DoorScript::one(900, 100000);
    door.cycles[0].1 = 100000;
    let mut cfg = Config::default();
    cfg.door.propped_ms = 4000;
    let mut e = Engine::new(cfg);
    let timeline = build_inputs(&[a], &door, 12000);
    let events = run(&mut e, timeline);
    assert!(actions(&events).iter().any(
        |(_, a)| matches!(a, Action::ProppedDoor { .. })
    ));
}

#[test]
fn door_reopen_extends_the_same_episode() {
    // Alice badges; the door bounces/reopens; Bob follows without a badge
    // during the second open. It is still one tailgate.
    let alice = Actor::person(0.0, 3.0, -1.0, 1000, 3200).with_badge(700, "alice");
    let bob = Actor::person(0.0, 3.4, -1.0, 3600, 5800);
    let door = DoorScript {
        cycles: vec![(900, 3400), (3450, 6200)],
    };
    let timeline = build_inputs(&[alice, bob], &door, 9000);
    let events = run(&mut engine(), timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "{vs:?}");
    assert_eq!(vs[0].1.passed, 2);
}

// ------------------------------------------------------------ degradation

#[test]
fn radar_blind_with_beam_still_counts_and_marks_suspect() {
    // Radar delivers nothing; the beam sees one person per trip. Two people,
    // one badge => suspect alarm, not a silent pass.
    let mut cfg = Config::default();
    cfg.beam.enabled = true;
    cfg.decision.alarm_on_suspect = true;
    let mut e = Engine::new(cfg);
    let mut out: Vec<(i64, Event)> = Vec::new();
    // One badge, two beam person trips, door open 900..4000.
    let door = DoorSignalAt::new(900, 4000);
    let mut ms = 0;
    while ms <= 8000 {
        let ts = Ts::from_micros(ms * 1000);
        out.extend(e.tick(ts).into_iter().map(|e| (ms, e)));
        out.extend(e.feed(Input::Door(door.at(ms))).into_iter().map(|e| (ms, e)));
        if ms == 1000 {
            out.extend(
                e.feed(Input::Badge(door_tailgate::badge::BadgeRead::granted(ts, "alice")))
                    .into_iter()
                    .map(|e| (ms, e)),
            );
        }
        for (start, end) in [(1280i64, 1680i64), (2240, 2640)] {
            if ms == start {
                out.extend(
                    e.feed(Input::Beam(door_tailgate::beam::BeamSignal::new(ts, true)))
                        .into_iter()
                        .map(|e| (ms, e)),
                );
            }
            if ms == end {
                out.extend(
                    e.feed(Input::Beam(door_tailgate::beam::BeamSignal::new(ts, false)))
                        .into_iter()
                        .map(|e| (ms, e)),
                );
            }
        }
        ms += 40;
    }
    let vs = verdicts(&out);
    assert!(!vs.is_empty(), "degraded radar must still judge: {out:?}");
    assert_eq!(vs[0].1.health, door_tailgate::engine::SensorHealth::RadarBlind);
}

struct DoorSignalAt {
    open_ms: i64,
    close_ms: i64,
}

impl DoorSignalAt {
    fn new(open_ms: i64, close_ms: i64) -> Self {
        Self { open_ms, close_ms }
    }
    fn at(&self, ms: i64) -> door_tailgate::door::DoorSignal {
        door_tailgate::door::DoorSignal::new(
            Ts::from_micros(ms * 1000),
            ms >= self.open_ms && ms < self.close_ms,
        )
    }
}

// --------------------------------------------------------------- safety

#[test]
fn an_egress_door_is_never_locked_by_the_interlock() {
    // Even on a firm alarm, a hinged door is not stopped: people must be able
    // to leave in a fire.
    let mut cfg = Config::default();
    cfg.interlock.lane_kind = door_tailgate::config::LaneKind::Door;
    let mut e = Engine::new(cfg);
    let alice = Actor::person(0.0, 3.0, -1.0, 1000, 3200).with_badge(700, "alice");
    let bob = Actor::person(0.0, 3.4, -1.0, 1600, 3800);
    let door = DoorScript::one(900, 4000);
    let events = run(&mut e, build_inputs(&[alice, bob], &door, 7000));
    assert!(!actions(&events)
        .iter()
        .any(|(_, a)| matches!(a, Action::StopLane { .. })));
    assert!(actions(&events)
        .iter()
        .any(|(_, a)| matches!(a, Action::AlarmRelay { on: true, .. })));
}

#[test]
fn empty_lane_produces_no_events_beyond_health() {
    let e = engine();
    let mut e = e;
    let mut evs = Vec::new();
    for ms in 0..300 {
        evs.extend(e.tick(Ts::from_micros(ms * 40_000)));
    }
    assert!(evs.iter().all(|e| !matches!(e, Event::Verdict(_))));
}

#[test]
fn radar_points_outside_range_do_not_create_an_episode() {
    // Reflections 20 m away in the corridor must be ignored.
    let mut e = engine();
    let mut pts = Vec::new();
    for i in 0..5 {
        pts.push(RadarPoint::new(0.0, 12.0 + i as f32 * 0.1, 1.4));
    }
    let _ = e.feed(Input::Radar {
        ts: Ts::from_micros(0),
        points: pts,
    });
    assert!(e.current_episode().is_none());
}

#[test]
fn a_dead_radar_is_degraded_not_silently_clear() {
    // No radar frames at all; one badge, two infrared person trips. The engine
    // must report a degraded verdict rather than a nominal alarm or a silent
    // pass, because it cannot attribute the passage to the badge with certainty.
    use door_tailgate::beam::BeamSignal;
    use door_tailgate::door::DoorSignal;
    use door_tailgate::engine::SensorHealth;

    let mut e = engine();
    let mut ms = 0i64;
    let mut events: Vec<(i64, Event)> = Vec::new();
    while ms <= 9000 {
        let ts = Ts::from_micros(ms * 1000);
        events.extend(e.tick(ts).into_iter().map(|ev| (ms, ev)));
        let open = (900..4200).contains(&ms);
        events.extend(e.feed(Input::Door(DoorSignal::new(ts, open))).into_iter().map(|ev| (ms, ev)));
        if ms == 1000 {
            events.extend(
                e.feed(Input::Badge(door_tailgate::badge::BadgeRead::granted(ts, "alice")))
                    .into_iter()
                    .map(|ev| (ms, ev)),
            );
        }
        for (s, en) in [(1480i64, 1720i64), (2360, 2600)] {
            if ms == s {
                events.extend(
                    e.feed(Input::Beam(BeamSignal::new(ts, true)))
                        .into_iter()
                        .map(|ev| (ms, ev)),
                );
            }
            if ms == en {
                events.extend(
                    e.feed(Input::Beam(BeamSignal::new(ts, false)))
                        .into_iter()
                        .map(|ev| (ms, ev)),
                );
            }
        }
        ms += 40;
    }
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1, "{vs:?}");
    assert_eq!(vs[0].1.severity, Severity::Suspect);
    assert_eq!(vs[0].1.health, SensorHealth::RadarBlind);
    assert_eq!(vs[0].1.passed, 2, "both infrared trips counted");
    // A suspect alarm still actuates the lane (fail-visible).
    assert!(actions(&events)
        .iter()
        .any(|(_, a)| matches!(a, Action::StopLane { .. })));
}

#[test]
fn tailgate_alarm_clears_when_late_badges_cover_it_only_if_suspect() {
    // A suspect verdict (degraded) may be cleared by a late badge; a firm alarm
    // may not. This pins the "suspect cleared" path.
    let mut cfg = Config::default();
    // Zero-badge entry is "Suspect" rather than a firm alarm at this site.
    cfg.decision.zero_badge_alarm = false;
    let mut e = Engine::new(cfg);
    // Nobody badges during the episode.
    let alice = Actor::person(0.0, 3.0, -1.0, 1000, 3200);
    let bob = Actor::person(0.0, 3.4, -1.0, 1600, 3800);
    let door = DoorScript::one(900, 4000);
    // Two late badges (e.g. an operator swiping after reviewing the camera)
    // land while the door is still settling and cover both passages.
    let mut timeline = build_inputs(&[alice, bob], &door, 7000);
    timeline.push((
        4200,
        Input::Badge(door_tailgate::badge::BadgeRead::granted(
            Ts::from_micros(4200 * 1000),
            "alice",
        )),
    ));
    timeline.push((
        4250,
        Input::Badge(door_tailgate::badge::BadgeRead::granted(
            Ts::from_micros(4250 * 1000),
            "bob",
        )),
    ));
    timeline.sort_by_key(|(ms, _)| *ms);
    let events = run(&mut e, timeline);
    let vs = verdicts(&events);
    assert_eq!(vs.len(), 1);
    assert_eq!(vs[0].1.severity, Severity::Suspect);
    // Two late badges lift authorized to 2, matching passed, so the suspect
    // alarm is cleared.
    assert!(
        actions(&events)
            .iter()
            .any(|(_, a)| matches!(a, Action::Cleared { .. })),
        "suspect should be cleared by the late badges: {:?}",
        actions(&events)
    );
}
