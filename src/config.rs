//! Site configuration.
//!
//! Every value has a documented default so a new controller can be brought up
//! with an empty config file and tuned during commissioning. `Config::validate`
//! rejects physically impossible combinations (e.g. the inner counting line
//! outside the lane) before the engine starts.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub lane: LaneConfig,
    pub radar: RadarConfig,
    pub tracker: TrackerConfig,
    pub badge: BadgeConfig,
    pub door: DoorConfig,
    pub beam: BeamConfig,
    pub decision: DecisionConfig,
    pub interlock: InterlockConfig,
}

impl Config {
    pub fn from_json_str(s: &str) -> Result<Config, serde_json::Error> {
        serde_json::from_str(s)
    }

    /// Returns a list of human-readable problems; empty means the config is
    /// usable. Called at startup and by the simulator.
    pub fn validate(&self) -> Vec<String> {
        let mut e = Vec::new();
        let l = &self.lane;
        if !(0.6..=4.0).contains(&l.width_m) {
            e.push(format!("lane.width_m={} outside 0.6..=4.0", l.width_m));
        }
        if l.inner_line_m >= 0.0 {
            e.push("lane.inner_line_m must be negative (inside the protected area)".into());
        }
        if l.outer_line_m <= 0.0 {
            e.push("lane.outer_line_m must be positive (outside, approach side)".into());
        }
        if l.inner_zone_m.0 >= l.inner_zone_m.1 || l.inner_zone_m.1 > -0.05 {
            e.push("lane.inner_zone_m must be a range ending inside the door (y < -0.05)".into());
        }
        if l.outer_zone_m.0 <= 0.05 || l.outer_zone_m.0 >= l.outer_zone_m.1 {
            e.push("lane.outer_zone_m must be a positive ascending range".into());
        }
        if l.outer_line_m <= l.outer_zone_m.0 || l.outer_line_m >= l.outer_zone_m.1 {
            e.push("lane.outer_line_m must fall inside lane.outer_zone_m".into());
        }
        if !(0.1..=1.0).contains(&self.radar.cluster_eps_m) {
            e.push("radar.cluster_eps_m should be 0.1..=1.0 m".into());
        }
        if self.radar.cluster_min_points == 0 {
            e.push("radar.cluster_min_points must be >= 1".into());
        }
        if !(0.2..=2.0).contains(&self.tracker.gate_base_m) {
            e.push("tracker.gate_base_m should be 0.2..=2.0 m".into());
        }
        if self.tracker.alpha <= 0.0 || self.tracker.alpha > 1.0 {
            e.push("tracker.alpha must be in (0,1]".into());
        }
        if self.tracker.beta < 0.0 || self.tracker.beta > 1.0 {
            e.push("tracker.beta must be in [0,1]".into());
        }
        if self.badge.debounce_ms <= 0 {
            e.push("badge.debounce_ms must be > 0".into());
        }
        if self.door.settle_ms < 0 {
            e.push("door.settle_ms must be >= 0".into());
        }
        if self.decision.max_authorized_per_episode == 0 {
            e.push("decision.max_authorized_per_episode must be >= 1".into());
        }
        if self.decision.alarm_settle_ms < 0 {
            e.push("decision.alarm_settle_ms must be >= 0".into());
        }
        if self.beam.enabled && self.beam.cart_block_ms <= self.beam.min_person_block_ms {
            e.push("beam.cart_block_ms must exceed beam.min_person_block_ms".into());
        }
        e
    }
}

/// Doorway geometry. Y is metres along the walking direction; the door plane is
/// y=0 and outside is positive.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LaneConfig {
    /// Clear width of the lane. Passages outside this width are not attributed
    /// to the door (people walking past in the corridor).
    pub width_m: f32,
    /// Extra tolerance on each side when attributing a crossing to this lane.
    pub lane_margin_m: f32,
    /// Counting line just inside the door; crossing it in the entry direction
    /// is the definition of "went through".
    pub inner_line_m: f32,
    /// Counting line in the approach area; crossing it starts an approach.
    pub outer_line_m: f32,
    /// (min,max) of the inner presence zone, used to hold an episode open while
    /// somebody is still standing in the doorway.
    pub inner_zone_m: (f32, f32),
    /// (min,max) of the approach zone.
    pub outer_zone_m: (f32, f32),
}

impl Default for LaneConfig {
    fn default() -> Self {
        LaneConfig {
            width_m: 1.7,
            lane_margin_m: 0.15,
            inner_line_m: -0.35,
            outer_line_m: 1.20,
            inner_zone_m: (-2.6, -0.05),
            outer_zone_m: (0.80, 4.20),
        }
    }
}

impl LaneConfig {
    pub fn half_width(self: &LaneConfig) -> f32 {
        self.width_m * 0.5 + self.lane_margin_m
    }
    pub fn x_min(&self) -> f32 {
        -self.half_width()
    }
    pub fn x_max(&self) -> f32 {
        self.half_width()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RadarConfig {
    /// DBSCAN neighbourhood radius in the floor plane.
    pub cluster_eps_m: f32,
    /// Minimum points for a cluster to be reported.
    pub cluster_min_points: usize,
    /// Points farther than this from the door are ignored.
    pub max_range_m: f32,
    /// A cluster wider than this is either two people walking abreast or one
    /// cart. It is inspected for multiple torsos.
    pub split_width_m: f32,
    /// Neighbourhood radius (metres) for joining person-height returns into one
    /// torso. People closer than this in the floor plane cannot be separated by
    /// the sensor, so this is the physical resolution floor.
    pub torso_eps_m: f32,
    /// Minimum person-height returns for a torso.
    pub torso_min_points: usize,
    /// A cart is wide and low; these bound the "low" part.
    pub vehicle_min_width_m: f32,
    pub vehicle_max_height_m: f32,
    /// Person-height returns prove a cluster contains a person, so it must not
    /// be classified as a bare vehicle.
    pub person_min_height_m: f32,
    /// Set to false when the sensor does not report height (z); vehicle
    /// classification then relies on width alone and is less reliable.
    pub height_available: bool,
    /// Ignore clusters with a total RCS below this (noise floor), in dBsm.
    pub min_rcs: f32,
}

impl Default for RadarConfig {
    fn default() -> Self {
        RadarConfig {
            cluster_eps_m: 0.30,
            cluster_min_points: 2,
            max_range_m: 5.0,
            split_width_m: 0.62,
            torso_eps_m: 0.32,
            torso_min_points: 3,
            vehicle_min_width_m: 0.62,
            vehicle_max_height_m: 1.10,
            person_min_height_m: 1.15,
            height_available: true,
            min_rcs: -20.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TrackerConfig {
    /// Association gate at zero relative speed.
    pub gate_base_m: f32,
    /// Additional gate allowance per m/s of relative speed.
    pub gate_speed_factor: f32,
    /// α-β filter gains.
    pub alpha: f32,
    pub beta: f32,
    /// Frames before a track becomes actionable.
    pub confirm_hits: u16,
    /// Consecutive missed frames before a track is deleted.
    pub max_miss: u16,
    /// Hard age cap for an unmatched track.
    pub max_miss_ms: i64,
    pub max_tracks: usize,
    /// How long a cart sighting sticks to a person track, so a pusher briefly
    /// lost behind the cart is still treated as "with cart" and gets the
    /// infrared's help.
    pub cart_memory_ms: i64,
    /// Speeds above this are rejected as association noise.
    pub max_speed_mps: f32,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        TrackerConfig {
            gate_base_m: 0.55,
            gate_speed_factor: 0.45,
            alpha: 0.65,
            beta: 0.20,
            confirm_hits: 3,
            max_miss: 6,
            max_miss_ms: 700,
            max_tracks: 24,
            cart_memory_ms: 1500,
            max_speed_mps: 3.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BadgeConfig {
    /// Duplicate reads of the same credential on the same reader inside this
    /// window are one tap (contactless readers re-read while the card is held).
    pub debounce_ms: i64,
    /// A badge read this long before the door opens still authorizes the entry.
    pub attach_before_ms: i64,
    /// A badge read after the door closes still belongs to the episode.
    pub attach_after_ms: i64,
}

impl Default for BadgeConfig {
    fn default() -> Self {
        BadgeConfig {
            debounce_ms: 700,
            attach_before_ms: 3000,
            attach_after_ms: 1500,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DoorConfig {
    /// A close event only counts after the contact has been closed this long,
    /// which absorbs door bounce and people holding the leaf.
    pub close_debounce_ms: i64,
    /// After the door really closes, keep the episode open this long so a
    /// delayed follower who catches the door is still attributed to it.
    pub settle_ms: i64,
    /// Open this long with no passage and no badge is a propped-door alarm.
    pub propped_ms: i64,
    /// Safety valve: never let one episode grow without bound.
    pub max_episode_ms: i64,
}

impl Default for DoorConfig {
    fn default() -> Self {
        DoorConfig {
            close_debounce_ms: 450,
            settle_ms: 1200,
            propped_ms: 20000,
            max_episode_ms: 120000,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BeamConfig {
    pub enabled: bool,
    /// Trips shorter than this are noise (a swinging bag, reflections).
    pub min_person_block_ms: i64,
    /// Trips longer than this are a cart, trolley or wide load, not one
    /// walking person; they are excluded from the person count and instead
    /// prove that a vehicle passed (possible radar occlusion).
    pub cart_block_ms: i64,
    /// Directional limits: a trip is only used if the sensor declares support.
    pub directional: bool,
}

impl Default for BeamConfig {
    fn default() -> Self {
        BeamConfig {
            enabled: true,
            min_person_block_ms: 120,
            cart_block_ms: 650,
            directional: true,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DecisionConfig {
    /// Upper bound on authorizations one door-opening episode can carry; keeps
    /// a stuck/replayed credential from authorizing an arbitrary crowd.
    pub max_authorized_per_episode: u32,
    /// Wait this long after the latest crossing before deciding, so teammates
    /// tapping in quick succession are counted before an alarm.
    pub alarm_settle_ms: i64,
    /// …but never delay an alarm longer than this after the first excess
    /// person was seen.
    pub alarm_max_delay_ms: i64,
    /// Raise an alarm when only partial evidence exists (sensor degraded,
    /// single-source count). Turning this off keeps the alarm log only.
    pub alarm_on_suspect: bool,
    /// Treat a vehicle crossing without a detected operator as one person.
    /// True is right for a site where carts are always pushed.
    pub vehicle_assume_operator: bool,
    /// Allow infrared trips to raise the person count (bounded by
    /// `beam_raise_max`) when a vehicle was present or radar merged a blob.
    pub beam_raise_behind_vehicle: bool,
    pub beam_raise_max: u32,
    /// A door opening with zero badges still alarms when somebody walks in.
    pub zero_badge_alarm: bool,
}

impl Default for DecisionConfig {
    fn default() -> Self {
        DecisionConfig {
            max_authorized_per_episode: 8,
            alarm_settle_ms: 700,
            alarm_max_delay_ms: 2500,
            alarm_on_suspect: true,
            vehicle_assume_operator: true,
            beam_raise_behind_vehicle: true,
            beam_raise_max: 1,
            zero_badge_alarm: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneKind {
    /// Hinged/sliding door with a closer: egress must stay free.
    Door,
    /// Tripod or flap turnstile: the lane can be stopped.
    Turnstile,
    /// Vehicle/pedestrian barrier: only inhibit, never drop on presence.
    Barrier,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct InterlockConfig {
    pub lane_kind: LaneKind,
    /// Stop the turnstile lane / inhibit the next unlock on an alarm.
    pub stop_lane_on_alarm: bool,
    /// Also stop the lane on a `Suspect` verdict (degraded sensing, zero-badge).
    /// Fail-visible default: a possible tailgate under a blind sensor is still
    /// worth a short lane hold at a turnstile.
    pub stop_lane_on_suspect: bool,
    /// Latch the alarm until cleared locally or by the ACS.
    pub latch_alarm: bool,
    /// Auto-clear a non-latched alarm after this long.
    pub auto_clear_ms: i64,
    /// Camera clip requested around the event.
    pub clip_pre_ms: i64,
    pub clip_post_ms: i64,
    /// Keep the lane inhibited while the sensors are blind (turnstiles/barriers
    /// only; never applied to a free-egress door).
    pub fail_secure_on_blind: bool,
}

impl Default for InterlockConfig {
    fn default() -> Self {
        InterlockConfig {
            lane_kind: LaneKind::Turnstile,
            stop_lane_on_alarm: true,
            stop_lane_on_suspect: true,
            latch_alarm: true,
            auto_clear_ms: 60000,
            clip_pre_ms: 5000,
            clip_post_ms: 5000,
            fail_secure_on_blind: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_valid() {
        assert!(Config::default().validate().is_empty());
    }

    #[test]
    fn partial_json_uses_defaults() {
        let c = Config::from_json_str(r#"{"lane":{"width_m":2.4}}"#).unwrap();
        assert_eq!(c.lane.width_m, 2.4);
        assert_eq!(c.lane.inner_line_m, -0.35);
    }

    #[test]
    fn unknown_field_is_rejected() {
        assert!(Config::from_json_str(r#"{"lane":{"widht_m":2.4}}"#).is_err());
    }

    #[test]
    fn invalid_geometry_is_reported() {
        let mut c = Config::default();
        c.lane.inner_line_m = 0.5;
        assert!(!c.validate().is_empty());
    }
}
