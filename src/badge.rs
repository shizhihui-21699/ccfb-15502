//! Badge reader de-duplication and read history.

use crate::config::BadgeConfig;
use crate::time::{Dur, Ts};

#[derive(Debug, Clone)]
pub struct BadgeRead {
    pub ts: Ts,
    /// Reader id, e.g. `door-3-in`.
    pub reader: String,
    /// Opaque credential id. Never store the raw card number here.
    pub credential: String,
    /// Access controller decision, when the reader forwards it.
    pub decision: Option<BadgeDecision>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BadgeDecision {
    Granted,
    Denied,
    /// The controller did not evaluate locally (offline mode, door held by a
    /// schedule); treat the read as a request.
    Unknown,
}

impl BadgeRead {
    pub fn granted(ts: Ts, credential: impl Into<String>) -> BadgeRead {
        BadgeRead {
            ts,
            reader: String::new(),
            credential: credential.into(),
            decision: Some(BadgeDecision::Granted),
        }
    }

    pub fn denied(ts: Ts, credential: impl Into<String>) -> BadgeRead {
        BadgeRead {
            ts,
            reader: String::new(),
            credential: credential.into(),
            decision: Some(BadgeDecision::Denied),
        }
    }
}

#[derive(Debug, Clone)]
pub struct AcceptedTap {
    pub ts: Ts,
    pub credential: String,
    pub reader: String,
}

/// Deduplicates contactless re-reads: the same credential on the same reader
/// within `debounce_ms` is one tap, because holding a card near a reader
/// produces a burst of identical reads.
#[derive(Debug)]
pub struct BadgeFilter {
    cfg: BadgeConfig,
    last: Vec<(String, String, Ts)>,
}

impl BadgeFilter {
    pub fn new(cfg: BadgeConfig) -> BadgeFilter {
        BadgeFilter {
            cfg,
            last: Vec::new(),
        }
    }

    pub fn debounce(&self) -> Dur {
        Dur::ms(self.cfg.debounce_ms)
    }

    /// Returns `Some` for a tap that should count, `None` for a duplicate.
    ///
    /// A denied read never counts as an authorization but is still recorded by
    /// the caller if it wants a forced-door trail.
    pub fn accept(&mut self, read: &BadgeRead) -> Option<AcceptedTap> {
        let key_same = |k: &(String, String, Ts)| k.0 == read.reader && k.1 == read.credential;
        if let Some((_, _, ts)) = self.last.iter().find(|k| key_same(k)) {
            if read.ts.since(*ts) < self.debounce() {
                return None;
            }
        }
        self.last.retain(|k| read.ts.since(k.2) < Dur::secs(30.0));
        self.last.push((read.reader.clone(), read.credential.clone(), read.ts));
        Some(AcceptedTap {
            ts: read.ts,
            credential: read.credential.clone(),
            reader: read.reader.clone(),
        })
    }

    /// Forget a credential (episode boundary) so a deliberate re-tap in the next
    /// episode is not mistaken for a debounce duplicate.
    pub fn forget_older_than(&mut self, ts: Ts) {
        self.last.retain(|k| k.2 >= ts);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(ms: f64, cred: &str) -> BadgeRead {
        BadgeRead::granted(Ts::from_ms(ms), cred)
    }

    #[test]
    fn repeated_reads_of_the_same_card_are_one_tap() {
        let mut f = BadgeFilter::new(BadgeConfig::default());
        assert!(f.accept(&read(0.0, "c1")).is_some());
        assert!(f.accept(&read(300.0, "c1")).is_none());
        assert!(f.accept(&read(600.0, "c1")).is_none());
    }

    #[test]
    fn a_later_tap_counts() {
        let mut f = BadgeFilter::new(BadgeConfig::default());
        assert!(f.accept(&read(0.0, "c1")).is_some());
        assert!(f.accept(&read(1500.0, "c1")).is_some());
    }

    #[test]
    fn different_cards_count_separately() {
        let mut f = BadgeFilter::new(BadgeConfig::default());
        assert!(f.accept(&read(0.0, "c1")).is_some());
        assert!(f.accept(&read(100.0, "c2")).is_some());
    }

    #[test]
    fn different_readers_count_separately() {
        let mut f = BadgeFilter::new(BadgeConfig::default());
        let mut a = read(0.0, "c1");
        a.reader = "r1".into();
        let mut b = read(100.0, "c1");
        b.reader = "r2".into();
        assert!(f.accept(&a).is_some());
        assert!(f.accept(&b).is_some());
    }
}
