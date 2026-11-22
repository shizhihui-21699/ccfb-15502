//! Multi-target tracking and line-crossing attribution.
//!
//! Blobs from consecutive radar frames are associated by a gated nearest-
//! neighbour rule with an α-β filter for smoothing. Each track keeps a **travel
//! record**: which counting lines it crossed, when, and whether its cluster
//! carried a cart.
//!
//! ## People, not carts, are the tracked target
//!
//! The radar front end already collapses "worker pushing a trolley" into a
//! single `Person` unit (`cart_evidence = true`); a cart body on its own becomes
//! a `Vehicle` unit that contributes zero person equivalents. This tracker
//! therefore counts people. It does **not** try to keep a cart and its pusher as
//! two coupled tracks and fold them later — that approach is brittle, because
//! the cart leads the pusher and their stale predictions make the two identities
//! swap frame to frame.
//!
//! The only trace of the cart here is `cart_recent`: a track whose cluster
//! showed a cart within the last moment is allowed to coast longer through a
//! brief radar dropout, and it is reported to the engine so the infrared can
//! vouch for a pusher hidden behind the cart.

use crate::config::TrackerConfig;
use crate::geom::{cross_horizontal, Pos2, Travel};
use crate::radar::{BlobKind, Unit};
use crate::time::Ts;

pub type TrackId = u32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnitClass {
    Person,
    Vehicle,
}

impl From<BlobKind> for UnitClass {
    fn from(k: BlobKind) -> UnitClass {
        match k {
            BlobKind::Person => UnitClass::Person,
            BlobKind::Vehicle => UnitClass::Vehicle,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackState {
    Tentative,
    Confirmed,
}

/// A crossing of a counting line by a confirmed track.
#[derive(Debug, Clone, Copy)]
pub struct Crossing {
    pub ts: Ts,
    pub track: TrackId,
    /// 0 = inner (through the door), 1 = outer (approach line).
    pub line_idx: usize,
    pub travel: Travel,
    pub x: f32,
    pub class: UnitClass,
    /// Person equivalents contributed by this crossing.
    pub pe: u8,
    /// The track was carrying a cart when it crossed.
    pub cart: bool,
}

#[derive(Debug, Clone)]
pub struct Track {
    pub id: TrackId,
    /// Smoothed position used for prediction and association.
    pub pos: Pos2,
    pub vel: Pos2,
    /// Latest raw measurement centroid; crossing detection uses this so filter
    /// lag cannot hide a fast crossing.
    pub raw: Pos2,
    prev_raw: Option<Pos2>,
    pub state: TrackState,
    pub class: UnitClass,
    pub hits: u16,
    pub misses: u16,
    pub first_seen: Ts,
    pub last_seen: Ts,
    /// Last time this track's cluster showed a cart body.
    pub cart_last_seen: Option<Ts>,
    pub crossings: Vec<Crossing>,
    pub counted: bool,
    pub width: f32,
    pub height: f32,
}

impl Track {
    pub fn speed(&self) -> f32 {
        self.vel.norm()
    }

    pub fn is_confirmed(&self) -> bool {
        self.state == TrackState::Confirmed
    }

    /// Person equivalents this track contributes if it crosses now.
    pub fn pe(&self) -> u8 {
        match self.class {
            UnitClass::Person => 1,
            UnitClass::Vehicle => 0,
        }
    }

    /// Whether the track is currently accompanied by a cart (within `co_travel`
    /// of the last cart sighting).
    pub fn has_cart(&self, now: Ts, cfg: &TrackerConfig) -> bool {
        self.cart_last_seen
            .map(|t| now.since(t).millis_i64() <= cfg.cart_memory_ms)
            .unwrap_or(false)
    }
}

#[derive(Debug, Clone)]
pub struct TrackSnapshot {
    pub id: TrackId,
    pub pos: Pos2,
    pub vel: Pos2,
    pub confirmed: bool,
    pub class: UnitClass,
    pub pe: u8,
    pub cart: bool,
}

/// Counting-line geometry, mirrored from `LaneConfig`.
#[derive(Debug, Clone)]
pub struct LineSet {
    pub inner_y: f32,
    pub outer_y: f32,
    pub x_min: f32,
    pub x_max: f32,
}

impl LineSet {
    pub fn y(&self, idx: usize) -> f32 {
        if idx == 0 {
            self.inner_y
        } else {
            self.outer_y
        }
    }
}

#[derive(Debug)]
pub struct Tracker {
    cfg: TrackerConfig,
    tracks: Vec<Track>,
    next_id: TrackId,
}

impl Tracker {
    pub fn new(cfg: TrackerConfig) -> Tracker {
        Tracker {
            cfg,
            tracks: Vec::new(),
            next_id: 1,
        }
    }

    pub fn tracks(&self) -> &[Track] {
        &self.tracks
    }

    pub fn snapshots(&self) -> Vec<TrackSnapshot> {
        let now = self.tracks.iter().map(|t| t.last_seen).max().unwrap_or(Ts::ZERO);
        self.tracks
            .iter()
            .map(|t| TrackSnapshot {
                id: t.id,
                pos: t.pos,
                vel: t.vel,
                confirmed: t.is_confirmed(),
                class: t.class,
                pe: t.pe(),
                cart: t.has_cart(now, &self.cfg),
            })
            .collect()
    }

    /// Whether any confirmed person track is currently associated with a cart.
    pub fn cart_present(&self, now: Ts) -> bool {
        self.tracks
            .iter()
            .any(|t| t.is_confirmed() && t.has_cart(now, &self.cfg))
    }

    /// Feed one frame of classified units. Returns crossings detected this
    /// frame (confirmed tracks only).
    ///
    /// `occluded` lets tracks coast through a radar dropout caused by a cart,
    /// instead of being pruned, so an operator following a cart is not lost.
    pub fn update(
        &mut self,
        ts: Ts,
        units: &[Unit],
        lines: &LineSet,
        occluded: bool,
    ) -> Vec<Crossing> {
        let n_tracks = self.tracks.len();
        let n_units = units.len();
        let mut used_tracks = vec![false; n_tracks];
        let mut used_units = vec![false; n_units];

        // Gated greedy association with a class gate: a confirmed track may only
        // continue onto a same-class unit while one is in its gate. This stops a
        // bare-cart track from being captured by a nearby torso blob.
        let mut pairs: Vec<(f32, usize, usize)> = Vec::new();
        let mut same_class_available = vec![false; n_tracks];
        for (ti, t) in self.tracks.iter().enumerate() {
            let dt = (ts - t.last_seen).secs_f64().max(0.001);
            let pred = t.pos.add(t.vel.scaled(dt as f32));
            for (ui, u) in units.iter().enumerate() {
                let gap = pred.distance(u.blob.centroid);
                let gate = self.cfg.gate_base_m + t.speed() * self.cfg.gate_speed_factor;
                if gap > gate {
                    continue;
                }
                if t.class == UnitClass::from(u.kind) {
                    same_class_available[ti] = true;
                }
                pairs.push((gap, ti, ui));
            }
        }
        pairs.retain(|&(_, ti, ui)| {
            let t = &self.tracks[ti];
            !t.is_confirmed()
                || !same_class_available[ti]
                || t.class == UnitClass::from(units[ui].kind)
        });
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (_, ti, ui) in pairs {
            if used_tracks[ti] || used_units[ui] {
                continue;
            }
            used_tracks[ti] = true;
            used_units[ui] = true;
            self.apply_measurement(ti, ts, &units[ui]);
        }

        // Misses. While occluded, confirmed tracks coast (bounded by an age cap).
        for (ti, t) in self.tracks.iter_mut().enumerate() {
            if used_tracks[ti] {
                continue;
            }
            let age_ms = (ts - t.last_seen).millis_i64();
            if occluded && age_ms <= self.cfg.max_miss_ms * 4 {
                continue;
            }
            t.misses += 1;
        }

        // New tracks.
        for (ui, u) in units.iter().enumerate() {
            if used_units[ui] {
                continue;
            }
            if self.tracks.len() >= self.cfg.max_tracks {
                if let Some(pos) = self.tracks.iter().position(|t| !t.is_confirmed()) {
                    self.tracks.swap_remove(pos);
                } else {
                    continue;
                }
            }
            self.spawn(ts, u);
        }

        let max_miss = self.cfg.max_miss;
        let max_miss_ms = self.cfg.max_miss_ms;
        self.tracks
            .retain(|t| t.misses <= max_miss && (ts - t.last_seen).millis_i64() <= max_miss_ms);

        self.detect_crossings(ts, lines)
    }

    fn spawn(&mut self, ts: Ts, u: &Unit) {
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1).max(1);
        self.tracks.push(Track {
            id,
            pos: u.blob.centroid,
            vel: Pos2::ZERO,
            raw: u.blob.centroid,
            prev_raw: None,
            state: TrackState::Tentative,
            class: UnitClass::from(u.kind),
            hits: 1,
            misses: 0,
            first_seen: ts,
            last_seen: ts,
            cart_last_seen: if u.cart_evidence { Some(ts) } else { None },
            crossings: Vec::new(),
            counted: false,
            width: u.blob.width,
            height: u.blob.max_height,
        });
    }

    fn apply_measurement(&mut self, ti: usize, ts: Ts, u: &Unit) {
        let cfg = &self.cfg;
        let t = &mut self.tracks[ti];
        let dt = (ts - t.last_seen).secs_f64().max(0.001) as f32;
        let predicted = t.pos.add(t.vel.scaled(dt));
        let residual = u.blob.centroid.sub(predicted);
        t.pos = predicted.add(residual.scaled(cfg.alpha));
        let mut vel = t.vel.add(residual.scaled(cfg.beta / dt));
        if vel.norm() > cfg.max_speed_mps {
            vel = vel.scaled(cfg.max_speed_mps / vel.norm());
        }
        t.vel = vel;
        t.prev_raw = Some(t.raw);
        t.raw = u.blob.centroid;
        t.last_seen = ts;
        t.hits += 1;
        t.misses = 0;
        if t.state == TrackState::Tentative && t.hits >= cfg.confirm_hits {
            t.state = TrackState::Confirmed;
        }
        if u.cart_evidence {
            t.cart_last_seen = Some(ts);
        }
        // A person blob never downgrades to a vehicle within one track; the
        // reverse can happen if a cart's operator steps into view.
        if UnitClass::from(u.kind) == UnitClass::Person {
            t.class = UnitClass::Person;
        }
        t.width = 0.7 * t.width + 0.3 * u.blob.width;
        t.height = 0.7 * t.height + 0.3 * u.blob.max_height;
    }

    fn detect_crossings(&mut self, ts: Ts, lines: &LineSet) -> Vec<Crossing> {
        let mut out = Vec::new();
        for t in self.tracks.iter_mut() {
            if !t.is_confirmed() || t.last_seen != ts || t.counted {
                continue;
            }
            let Some(prev) = t.prev_raw else { continue };
            // Only the inner line, only an entry, only people.
            if t.pe() == 0 {
                continue;
            }
            let Some(c) = cross_horizontal(prev, t.raw, lines.inner_y, lines.x_min, lines.x_max) else {
                continue;
            };
            if c.travel != Travel::In {
                continue;
            }
            let crossing = Crossing {
                ts,
                track: t.id,
                line_idx: 0,
                travel: c.travel,
                x: c.x,
                class: t.class,
                pe: t.pe(),
                cart: t.has_cart(ts, &self.cfg),
            };
            t.crossings.push(crossing);
            t.counted = true;
            out.push(crossing);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar::Blob;

    fn blob(ts: Ts, x: f32, y: f32, width: f32, height: f32) -> Blob {
        Blob {
            ts,
            centroid: Pos2::new(x, y),
            point_count: 8,
            width,
            depth: 0.25,
            max_height: height,
            min_height: 0.3,
            max_rcs: -3.0,
            mean_rcs: -5.0,
            mean_radial_velocity: 0.0,
            has_velocity: false,
            person_points: if height > 1.1 { 4 } else { 0 },
        }
    }

    fn person_unit(ts: Ts, x: f32, y: f32, cart: bool) -> Unit {
        Unit {
            blob: blob(ts, x, y, 0.45, 1.5),
            kind: BlobKind::Person,
            part: None,
            cart_evidence: cart,
        }
    }

    fn vehicle_unit(ts: Ts, x: f32, y: f32) -> Unit {
        Unit {
            blob: blob(ts, x, y, 0.9, 0.6),
            kind: BlobKind::Vehicle,
            part: None,
            cart_evidence: false,
        }
    }

    fn lines() -> LineSet {
        LineSet {
            inner_y: -0.35,
            outer_y: 1.2,
            x_min: -1.0,
            x_max: 1.0,
        }
    }

    fn walk(tk: &mut Tracker, units_at: impl Fn(Ts, i32) -> Vec<Unit>, steps: i32) -> Vec<Crossing> {
        let mut all = Vec::new();
        for step in 0..steps {
            let ts = Ts::from_ms(step as f64 * 40.0);
            let u = units_at(ts, step);
            all.extend(tk.update(ts, &u, &lines(), false));
        }
        all
    }

    #[test]
    fn one_person_walks_in_and_is_counted_once() {
        let mut tk = Tracker::new(TrackerConfig::default());
        let c = walk(
            &mut tk,
            |ts, step| vec![person_unit(ts, 0.0, 3.0 - 1.4 * (step as f32 * 0.04), false)],
            80,
        );
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].travel, Travel::In);
        assert_eq!(c[0].pe, 1);
    }

    #[test]
    fn dithering_near_the_line_does_not_double_count() {
        let mut tk = Tracker::new(TrackerConfig::default());
        let mut count = 0;
        for step in 0..40 {
            let ts = Ts::from_ms(step as f64 * 40.0);
            let y = -0.35 + if step % 2 == 0 { 0.06 } else { -0.06 };
            let u = vec![person_unit(ts, 0.0, y, false)];
            count += tk.update(ts, &u, &lines(), false).len();
        }
        assert!(count <= 1, "counted {count} times");
    }

    #[test]
    fn a_person_with_a_cart_counts_once_and_is_flagged() {
        let mut tk = Tracker::new(TrackerConfig::default());
        let c = walk(
            &mut tk,
            |ts, step| vec![person_unit(ts, 0.0, 3.0 - 1.0 * (step as f32 * 0.04), true)],
            100,
        );
        assert_eq!(c.len(), 1);
        assert!(c[0].cart, "cart evidence must ride along the crossing");
        assert_eq!(c[0].pe, 1);
    }

    #[test]
    fn a_bare_cart_contributes_no_person() {
        let mut tk = Tracker::new(TrackerConfig::default());
        let c = walk(
            &mut tk,
            |ts, step| vec![vehicle_unit(ts, 0.0, 3.0 - 1.0 * (step as f32 * 0.04))],
            100,
        );
        assert!(c.is_empty(), "a cart alone is not a person: {c:?}");
    }

    #[test]
    fn two_peers_abreast_count_as_two() {
        let mut tk = Tracker::new(TrackerConfig::default());
        let c = walk(
            &mut tk,
            |ts, step| {
                let y = 3.0 - 1.4 * (step as f32 * 0.04);
                vec![
                    person_unit(ts, -0.35, y, false),
                    person_unit(ts, 0.35, y, false),
                ]
            },
            100,
        );
        assert_eq!(c.len(), 2);
        assert_eq!(c.iter().map(|c| c.pe).sum::<u8>(), 2);
    }

    #[test]
    fn a_cart_does_not_capture_a_nearby_torso() {
        // A bare cart track and a person's torso near it must stay separate
        // classes, so the cart never contributes a person.
        let mut tk = Tracker::new(TrackerConfig::default());
        let c = walk(
            &mut tk,
            |ts, step| {
                let y = 3.0 - 1.0 * (step as f32 * 0.04);
                vec![vehicle_unit(ts, 0.0, y - 0.55), person_unit(ts, 0.0, y, false)]
            },
            100,
        );
        // Only the person crosses as a person.
        assert_eq!(c.iter().filter(|c| c.pe > 0).count(), 1, "{c:?}");
        assert_eq!(c.iter().map(|c| c.pe).sum::<u8>(), 1);
    }

    #[test]
    fn walking_back_out_does_not_count_as_entry() {
        let mut tk = Tracker::new(TrackerConfig::default());
        let c = walk(
            &mut tk,
            |ts, step| vec![person_unit(ts, 0.0, -1.0 + 1.4 * (step as f32 * 0.04), false)],
            100,
        );
        assert!(c.is_empty());
    }

    #[test]
    fn a_track_is_not_counted_until_confirmed() {
        let mut tk = Tracker::new(TrackerConfig::default());
        let mut c = Vec::new();
        for step in 0..2 {
            let ts = Ts::from_ms(step as f64 * 40.0);
            let u = vec![person_unit(ts, 0.0, 0.2 - 0.4 * step as f32, false)];
            c.extend(tk.update(ts, &u, &lines(), false));
        }
        assert!(c.is_empty());
    }
}
