//! Monotonic timebase.
//!
//! The device has no reliable wall clock and must not depend on the network, so
//! every timestamp is microseconds since boot on a monotonic clock. A wall-clock
//! offset can be attached by the logger when records leave the device.

use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Sub};

/// Timestamp in microseconds on the monotonic device clock.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Ts(pub i64);

/// A duration in microseconds.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Dur(pub i64);

impl Ts {
    pub const ZERO: Ts = Ts(0);

    pub fn from_ms(ms: f64) -> Ts {
        Ts((ms * 1000.0).round() as i64)
    }

    pub fn from_micros(us: i64) -> Ts {
        Ts(us)
    }

    pub fn micros(self) -> i64 {
        self.0
    }

    pub fn millis(self) -> i64 {
        self.0 / 1000
    }

    pub fn secs_f64(self) -> f64 {
        self.0 as f64 / 1_000_000.0
    }

    pub fn saturating_add(self, d: Dur) -> Ts {
        Ts(self.0.saturating_add(d.0))
    }

    pub fn saturating_sub(self, d: Dur) -> Ts {
        Ts(self.0.saturating_sub(d.0))
    }

    /// Time elapsed since `earlier` (zero if `earlier` is in the future).
    pub fn since(self, earlier: Ts) -> Dur {
        Dur(self.0.saturating_sub(earlier.0))
    }

    pub fn is_after(self, other: Ts) -> bool {
        self.0 > other.0
    }

    pub fn is_before(self, other: Ts) -> bool {
        self.0 < other.0
    }

    pub fn min(self, other: Ts) -> Ts {
        if self.0 <= other.0 {
            self
        } else {
            other
        }
    }

    pub fn max(self, other: Ts) -> Ts {
        if self.0 >= other.0 {
            self
        } else {
            other
        }
    }
}

impl Dur {
    pub const ZERO: Dur = Dur(0);
    pub const MAX: Dur = Dur(i64::MAX);

    pub fn ms(ms: i64) -> Dur {
        Dur(ms.saturating_mul(1000))
    }

    pub fn secs(s: f64) -> Dur {
        Dur((s * 1_000_000.0).round() as i64)
    }

    pub fn micros(us: i64) -> Dur {
        Dur(us)
    }

    pub fn micros_i64(self) -> i64 {
        self.0
    }

    pub fn millis_i64(self) -> i64 {
        self.0 / 1000
    }

    pub fn secs_f64(self) -> f64 {
        self.0 as f64 / 1_000_000.0
    }

    pub fn is_zero(self) -> bool {
        self.0 == 0
    }

    pub fn saturating_mul(self, n: i64) -> Dur {
        Dur(self.0.saturating_mul(n))
    }

    pub fn clamped(self, lo: Dur, hi: Dur) -> Dur {
        Dur(self.0.clamp(lo.0, hi.0))
    }
}

impl Add<Dur> for Ts {
    type Output = Ts;
    fn add(self, rhs: Dur) -> Ts {
        Ts(self.0.saturating_add(rhs.0))
    }
}

impl AddAssign<Dur> for Ts {
    fn add_assign(&mut self, rhs: Dur) {
        self.0 = self.0.saturating_add(rhs.0);
    }
}

impl Sub<Dur> for Ts {
    type Output = Ts;
    fn sub(self, rhs: Dur) -> Ts {
        Ts(self.0.saturating_sub(rhs.0))
    }
}

impl Sub<Ts> for Ts {
    type Output = Dur;
    fn sub(self, rhs: Ts) -> Dur {
        Dur(self.0.saturating_sub(rhs.0))
    }
}

impl Add for Dur {
    type Output = Dur;
    fn add(self, rhs: Dur) -> Dur {
        Dur(self.0.saturating_add(rhs.0))
    }
}

impl fmt::Debug for Ts {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}ms", self.millis())
    }
}

impl fmt::Debug for Dur {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}ms", self.millis_i64())
    }
}
