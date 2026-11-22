//! Geometry helpers: positions and line crossings.

use serde::{Deserialize, Serialize};

/// 2-D position in metres. The door line is `y = 0`, the lane is centred on
/// `x = 0`, and **positive Y is outside** (the approach side). A person entering
/// moves towards negative Y.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Pos2 {
    pub x: f32,
    pub y: f32,
}

impl Pos2 {
    pub const ZERO: Pos2 = Pos2 { x: 0.0, y: 0.0 };

    pub fn new(x: f32, y: f32) -> Pos2 {
        Pos2 { x, y }
    }

    #[allow(clippy::should_implement_trait)]
    pub fn sub(self, other: Pos2) -> Pos2 {
        Pos2::new(self.x - other.x, self.y - other.y)
    }

    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: Pos2) -> Pos2 {
        Pos2::new(self.x + other.x, self.y + other.y)
    }

    pub fn scaled(self, k: f32) -> Pos2 {
        Pos2::new(self.x * k, self.y * k)
    }

    pub fn norm(self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn distance(self, other: Pos2) -> f32 {
        self.sub(other).norm()
    }

    pub fn dot(self, other: Pos2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// True when both positions are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

/// Direction of travel through a counting line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Travel {
    /// From the outside towards the inside of the protected area.
    In,
    /// From the inside towards the outside.
    Out,
}

impl Travel {
    pub fn sign(self) -> i8 {
        match self {
            Travel::In => -1,
            Travel::Out => 1,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LineCrossing {
    /// X coordinate at the crossing point.
    pub x: f32,
    pub travel: Travel,
    /// Interpolated Y of the crossing (normally equal to the line, kept for
    /// diagnostics of diagonal trajectories).
    pub y: f32,
}

/// Detects a crossing of the horizontal line `y = line_y` between `a` and `b`
/// when the interpolated x coordinate lies within `x_min..=x_max`.
///
/// Returns `None` when the segment does not cross or the crossing falls outside
/// the lane (the object skirted the doorway).
pub fn cross_horizontal(
    a: Pos2,
    b: Pos2,
    line_y: f32,
    x_min: f32,
    x_max: f32,
) -> Option<LineCrossing> {
    let da = a.y - line_y;
    let db = b.y - line_y;
    // Both on the same side (or exactly on the line): no crossing.
    if (da > 0.0) == (db > 0.0) {
        return None;
    }
    let denom = da - db;
    if denom.abs() < 1e-6 {
        return None;
    }
    let t = da / denom;
    if !(0.0..=1.0).contains(&t) {
        return None;
    }
    let x = a.x + (b.x - a.x) * t;
    if !x.is_finite() || x < x_min || x > x_max {
        return None;
    }
    Some(LineCrossing {
        x,
        travel: if da > 0.0 { Travel::In } else { Travel::Out },
        y: line_y,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entering_crossing_is_detected() {
        let c = cross_horizontal(Pos2::new(0.1, 1.0), Pos2::new(0.1, -1.0), 0.0, -1.0, 1.0)
            .expect("crossing");
        assert_eq!(c.travel, Travel::In);
        assert!((c.x - 0.1).abs() < 1e-6);
    }

    #[test]
    fn exiting_crossing_is_detected() {
        let c = cross_horizontal(Pos2::new(0.0, -1.0), Pos2::new(0.0, 1.0), 0.0, -1.0, 1.0)
            .expect("crossing");
        assert_eq!(c.travel, Travel::Out);
    }

    #[test]
    fn crossing_outside_the_lane_is_ignored() {
        assert!(cross_horizontal(
            Pos2::new(1.6, 1.0),
            Pos2::new(1.6, -1.0),
            0.0,
            -0.9,
            0.9
        )
        .is_none());
    }

    #[test]
    fn no_crossing_when_same_side() {
        assert!(cross_horizontal(Pos2::new(0.0, 2.0), Pos2::new(0.0, 1.0), 0.0, -1.0, 1.0).is_none());
    }
}
