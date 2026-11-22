//! Door contact (reed switch / hall sensor) debouncing and episode boundaries.
//!
//! The contact is noisy: a closing swing door bounces, people hold the leaf, and
//! a propped door never shows a close event at all. The monitor therefore emits
//! **stable** open/close transitions only, and the engine adds a settle window
//! on top.

use crate::config::DoorConfig;
use crate::time::{Dur, Ts};

#[derive(Debug, Clone, Copy)]
pub struct DoorSignal {
    pub ts: Ts,
    /// Raw contact state: `true` when the door is open.
    pub open: bool,
}

impl DoorSignal {
    pub fn new(ts: Ts, open: bool) -> DoorSignal {
        DoorSignal { ts, open }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DoorTransition {
    Opened(Ts),
    Closed(Ts),
}

#[derive(Debug)]
pub struct DoorMonitor {
    cfg: DoorConfig,
    raw: bool,
    raw_since: Ts,
    stable: bool,
    /// Last time the raw contact was seen open (for propped-door detection).
    pub last_raw_open: Option<Ts>,
    /// Stable-open since, for the propped-door timer.
    pub stable_open_since: Option<Ts>,
    /// Most recent stable close (used by the engine's settle timer).
    pub last_close: Option<Ts>,
    /// Number of raw edges rejected as bounce, for commissioning diagnostics.
    pub bounce_edges: u32,
}

impl DoorMonitor {
    pub fn new(cfg: DoorConfig) -> DoorMonitor {
        DoorMonitor {
            cfg,
            raw: false,
            raw_since: Ts::ZERO,
            stable: false,
            last_raw_open: None,
            stable_open_since: None,
            last_close: None,
            bounce_edges: 0,
        }
    }

    pub fn is_open(&self) -> bool {
        self.stable
    }

    pub fn settle(&self) -> Dur {
        Dur::ms(self.cfg.settle_ms)
    }

    /// Feed a contact reading; returns a stable transition if the debounce
    /// elapsed.
    pub fn update(&mut self, sig: DoorSignal) -> Option<DoorTransition> {
        if sig.open {
            self.last_raw_open = Some(sig.ts);
        }
        if sig.open != self.raw {
            self.raw = sig.open;
            self.raw_since = sig.ts;
        }
        self.promote(sig.ts)
    }

    /// Time-driven promotion: call on every tick so a stable transition is
    /// emitted even with no further contact messages.
    pub fn poll(&mut self, now: Ts) -> Option<DoorTransition> {
        self.promote(now)
    }

    fn promote(&mut self, now: Ts) -> Option<DoorTransition> {
        if self.raw == self.stable {
            return None;
        }
        let need = if self.raw {
            // Opening is reported quickly: holding the door open matters for
            // security, so a short debounce is used.
            Dur::ms(60)
        } else {
            Dur::ms(self.cfg.close_debounce_ms)
        };
        if (now - self.raw_since) < need {
            return None;
        }
        if !self.raw && (now - self.raw_since).millis_i64() > 0 {
            self.bounce_edges = self.bounce_edges.saturating_add(1);
        }
        self.stable = self.raw;
        if self.stable {
            self.stable_open_since = Some(self.raw_since);
            Some(DoorTransition::Opened(self.raw_since))
        } else {
            self.stable_open_since = None;
            self.last_close = Some(self.raw_since);
            Some(DoorTransition::Closed(self.raw_since))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mon() -> DoorMonitor {
        DoorMonitor::new(DoorConfig::default())
    }

    #[test]
    fn open_is_debounced_then_emitted() {
        let mut m = mon();
        assert!(m.update(DoorSignal::new(Ts::from_ms(0.0), true)).is_none());
        let t = m.poll(Ts::from_ms(80.0));
        assert_eq!(t, Some(DoorTransition::Opened(Ts::from_ms(0.0))));
        assert!(m.is_open());
    }

    #[test]
    fn bounce_during_close_is_ignored() {
        let mut m = mon();
        m.update(DoorSignal::new(Ts::from_ms(0.0), true));
        m.poll(Ts::from_ms(80.0));
        // Close, bounce open for 100 ms, close again.
        assert!(m.update(DoorSignal::new(Ts::from_ms(1000.0), false)).is_none());
        assert!(m.update(DoorSignal::new(Ts::from_ms(1100.0), true)).is_none());
        let t = m.update(DoorSignal::new(Ts::from_ms(1200.0), false));
        assert!(t.is_none());
        // Close debounce is 450 ms; the raw close started at 1200.
        let t = m.poll(Ts::from_ms(1700.0));
        assert_eq!(t, Some(DoorTransition::Closed(Ts::from_ms(1200.0))));
    }

    #[test]
    fn held_door_stays_open() {
        let mut m = mon();
        m.update(DoorSignal::new(Ts::from_ms(0.0), true));
        m.poll(Ts::from_ms(80.0));
        for i in 1..100 {
            // No close signal at all.
            assert!(m.poll(Ts::from_ms(i as f64 * 100.0)).is_none());
        }
        assert!(m.is_open());
    }
}
