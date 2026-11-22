//! Infrared beam (active infrared / light-curtain) processing.
//!
//! A beam produces one bit: clear or blocked. From that we derive **trips**
//! (blocked intervals) and classify them by duration:
//!
//! * `< min_person_block_ms` — noise (a swinging bag, an insect, a reflection).
//! * `min_person_block_ms .. cart_block_ms` — one walking person.
//! * `> cart_block_ms` — a cart/trolley/wide load, or several people in a row.
//!   It is **not** used as a person count, because a queue of people through a
//!   single beam is indistinguishable from one long object.
//!
//! The beam is deliberately a *secondary* sensor. Radar is the primary counter
//! because it separates people. The beam contributes in exactly three cases:
//! radar is blind (degraded mode), a cart may have occluded a second person, or
//! a wide blob could not be split. This is what stops a trolley or a crowd from
//! silently defeating the radar.

use crate::config::BeamConfig;
use crate::geom::Travel;
use crate::time::{Dur, Ts};

#[derive(Debug, Clone, Copy)]
pub struct BeamSignal {
    pub ts: Ts,
    /// Index of the beam in the lane (0 = nearest the door).
    pub sensor: u8,
    pub blocked: bool,
    /// Set by a directional (two-element) sensor: crossing direction.
    pub direction: Option<Travel>,
}

impl BeamSignal {
    pub fn new(ts: Ts, blocked: bool) -> BeamSignal {
        BeamSignal {
            ts,
            sensor: 0,
            blocked,
            direction: None,
        }
    }

    pub fn blocked_for(ts: Ts, dir: Travel) -> BeamSignal {
        BeamSignal {
            ts,
            sensor: 0,
            blocked: true,
            direction: Some(dir),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum TripClass {
    Noise,
    Person,
    /// Long block: a cart, or several people in single file.
    Wide,
}

#[derive(Debug, Clone, Copy)]
pub struct BeamTrip {
    pub start: Ts,
    pub end: Ts,
    pub duration: Dur,
    pub sensor: u8,
    /// Direction of travel if the hardware can tell; otherwise unknown.
    pub direction: Option<Travel>,
    pub class: TripClass,
}

#[derive(Debug, Default)]
pub struct BeamMonitor {
    cfg: BeamConfig,
    open: Option<(Ts, u8, Option<Travel>)>,
    last_seen: Ts,
}

impl BeamMonitor {
    pub fn new(cfg: BeamConfig) -> BeamMonitor {
        BeamMonitor {
            cfg,
            open: None,
            last_seen: Ts::ZERO,
        }
    }

    pub fn enabled(&self) -> bool {
        self.cfg.enabled
    }

    /// True while something is currently breaking the beam.
    pub fn is_blocked(&self) -> bool {
        self.open.is_some()
    }

    /// Feed one signal. Returns a trip when this signal ends one.
    pub fn update(&mut self, sig: BeamSignal) -> Option<BeamTrip> {
        if !self.cfg.enabled {
            return None;
        }
        self.last_seen = sig.ts;
        if sig.blocked {
            match &mut self.open {
                // Still the same block, but a directional sensor may firm up
                // the direction mid-block.
                Some((_, sensor, dir)) => {
                    if sig.direction.is_some() {
                        *dir = sig.direction;
                    }
                    *sensor = sig.sensor;
                    None
                }
                None => {
                    self.open = Some((sig.ts, sig.sensor, sig.direction));
                    None
                }
            }
        } else {
            let (start, sensor, dir) = self.open.take()?;
            let duration = sig.ts - start;
            if duration.micros_i64() < 0 {
                return None;
            }
            let class = self.classify(duration);
            Some(BeamTrip {
                start,
                end: sig.ts,
                duration,
                sensor,
                direction: if self.cfg.directional { dir } else { None },
                class,
            })
        }
    }

    /// Close a block that never cleared, e.g. a sensor that went silent with the
    /// beam blocked. Guards against a trip that would otherwise last forever.
    pub fn poll(&mut self, now: Ts) -> Option<BeamTrip> {
        let (start, sensor, dir) = self.open?;
        if (now - start).millis_i64() > self.cfg.cart_block_ms * 4 {
            self.open = None;
            return Some(BeamTrip {
                start,
                end: now,
                duration: now - start,
                sensor,
                direction: if self.cfg.directional { dir } else { None },
                class: TripClass::Wide,
            });
        }
        None
    }

    fn classify(&self, duration: Dur) -> TripClass {
        let ms = duration.millis_i64();
        if ms < self.cfg.min_person_block_ms {
            TripClass::Noise
        } else if ms > self.cfg.cart_block_ms {
            TripClass::Wide
        } else {
            TripClass::Person
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> BeamConfig {
        BeamConfig::default()
    }

    #[test]
    fn normal_pass_is_person() {
        let mut m = BeamMonitor::new(cfg());
        assert!(m.update(BeamSignal::new(Ts::from_ms(0.0), true)).is_none());
        let trip = m
            .update(BeamSignal::new(Ts::from_ms(300.0), false))
            .expect("trip");
        assert_eq!(trip.class, TripClass::Person);
    }

    #[test]
    fn short_blip_is_noise() {
        let mut m = BeamMonitor::new(cfg());
        m.update(BeamSignal::new(Ts::from_ms(0.0), true));
        let trip = m.update(BeamSignal::new(Ts::from_ms(40.0), false)).unwrap();
        assert_eq!(trip.class, TripClass::Noise);
    }

    #[test]
    fn long_block_is_wide() {
        let mut m = BeamMonitor::new(cfg());
        m.update(BeamSignal::new(Ts::from_ms(0.0), true));
        let trip = m.update(BeamSignal::new(Ts::from_ms(900.0), false)).unwrap();
        assert_eq!(trip.class, TripClass::Wide);
    }

    #[test]
    fn direction_is_kept_when_supported() {
        let mut m = BeamMonitor::new(cfg());
        m.update(BeamSignal::blocked_for(Ts::from_ms(0.0), Travel::In));
        let trip = m.update(BeamSignal::new(Ts::from_ms(300.0), false)).unwrap();
        assert_eq!(trip.direction, Some(Travel::In));
    }

    #[test]
    fn direction_is_dropped_when_unsupported() {
        let mut m = BeamMonitor::new(BeamConfig {
            directional: false,
            ..cfg()
        });
        m.update(BeamSignal::blocked_for(Ts::from_ms(0.0), Travel::In));
        let trip = m.update(BeamSignal::new(Ts::from_ms(300.0), false)).unwrap();
        assert_eq!(trip.direction, None);
    }

    #[test]
    fn stuck_block_is_closed_by_poll() {
        let mut m = BeamMonitor::new(cfg());
        m.update(BeamSignal::new(Ts::from_ms(0.0), true));
        let trip = m.poll(Ts::from_ms(5000.0)).expect("poll closes block");
        assert_eq!(trip.class, TripClass::Wide);
        assert!(m.poll(Ts::from_ms(6000.0)).is_none());
    }

    #[test]
    fn disabled_monitor_ignores_signals() {
        let mut m = BeamMonitor::new(BeamConfig {
            enabled: false,
            ..cfg()
        });
        m.update(BeamSignal::new(Ts::from_ms(0.0), true));
        assert!(m.update(BeamSignal::new(Ts::from_ms(300.0), false)).is_none());
    }
}
