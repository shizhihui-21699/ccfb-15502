//! Millimetre-wave radar front end.
//!
//! Input is a point cloud per frame (already range/Doppler processed by the
//! sensor firmware). The job here is to turn points into **blobs**: spatial
//! clusters annotated with the evidence needed to tell people from carts and to
//! separate people walking abreast.
//!
//! Everything is allocation-light and O(n²) in the *point* count per frame,
//! which is fine for a doorway (a few hundred points at 15–25 Hz).

use crate::config::RadarConfig;
use crate::geom::Pos2;
use crate::time::Ts;

/// One radar detection. `z` is height above the floor (sensor dependent;
/// `RadarConfig::height_available` says whether to trust it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadarPoint {
    pub pos: Pos2,
    pub z: f32,
    /// Radar cross-section in dBsm.
    pub rcs: f32,
    /// Radial velocity in m/s, positive away from the sensor. Optional.
    pub vr: f32,
    pub has_velocity: bool,
}

impl RadarPoint {
    pub fn new(x: f32, y: f32, z: f32) -> RadarPoint {
        RadarPoint {
            pos: Pos2::new(x, y),
            z,
            rcs: -6.0,
            vr: 0.0,
            has_velocity: false,
        }
    }

    pub fn with_rcs(mut self, rcs: f32) -> RadarPoint {
        self.rcs = rcs;
        self
    }

    pub fn with_velocity(mut self, vr: f32) -> RadarPoint {
        self.vr = vr;
        self.has_velocity = true;
        self
    }
}

#[derive(Debug, Clone, Default)]
pub struct RadarFrame {
    pub ts: Ts,
    pub points: Vec<RadarPoint>,
}

/// A spatial cluster of points, ready for classification.
#[derive(Debug, Clone)]
pub struct Blob {
    pub ts: Ts,
    pub centroid: Pos2,
    pub point_count: usize,
    pub width: f32,
    pub depth: f32,
    pub max_height: f32,
    pub min_height: f32,
    pub max_rcs: f32,
    pub mean_rcs: f32,
    pub mean_radial_velocity: f32,
    pub has_velocity: bool,
    /// Number of returns at person height.
    pub person_points: usize,
}

impl Blob {
    pub fn half_width(&self) -> f32 {
        self.width * 0.5
    }
}

/// DBSCAN over floor-plane positions.
///
/// Returns one label per point; `None` is noise. Implemented as breadth-first
/// expansion from core points (`neighbour count >= min_pts`) so dense clusters
/// stay contiguous.
pub fn dbscan(points: &[Pos2], eps: f32, min_pts: usize) -> Vec<Option<usize>> {
    let n = points.len();
    let min_pts = min_pts.max(1);
    let eps2 = eps * eps;
    let mut labels: Vec<Option<usize>> = vec![None; n];
    let mut visited = vec![false; n];
    let mut next_label = 0usize;
    let mut stack: Vec<usize> = Vec::new();
    let mut nbrs: Vec<usize> = Vec::new();

    for start in 0..n {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        nbrs.clear();
        collect_neighbours(points, start, eps2, &mut nbrs);
        if nbrs.len() < min_pts {
            continue; // noise for now
        }
        let label = next_label;
        next_label += 1;
        labels[start] = Some(label);
        stack.clear();
        stack.extend_from_slice(&nbrs);
        while let Some(j) = stack.pop() {
            if !visited[j] {
                visited[j] = true;
                nbrs.clear();
                collect_neighbours(points, j, eps2, &mut nbrs);
                if nbrs.len() >= min_pts {
                    for &m in &nbrs {
                        if labels[m].is_none() {
                            labels[m] = Some(label);
                            stack.push(m);
                        }
                    }
                }
            }
            if labels[j].is_none() {
                labels[j] = Some(label);
            }
        }
    }
    labels
}

fn collect_neighbours(points: &[Pos2], i: usize, eps2: f32, out: &mut Vec<usize>) {
    out.push(i);
    let pi = points[i];
    for (j, pj) in points.iter().enumerate() {
        if j == i {
            continue;
        }
        let d = pi.sub(*pj);
        if d.dot(d) <= eps2 {
            out.push(j);
        }
    }
}

fn summarise(ts: Ts, pts: &[&RadarPoint], cfg: &RadarConfig) -> Option<Blob> {
    if pts.len() < cfg.cluster_min_points {
        return None;
    }
    let n = pts.len() as f32;
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut minx = f32::MAX;
    let mut maxx = f32::MIN;
    let mut miny = f32::MAX;
    let mut maxy = f32::MIN;
    let mut maxh: f32 = 0.0;
    let mut minh = f32::MAX;
    let mut sum_rcs = 0.0;
    let mut max_rcs = f32::MIN;
    let mut sum_vr = 0.0;
    let mut vel_count = 0usize;
    let mut person_points = 0usize;
    for p in pts {
        sx += p.pos.x;
        sy += p.pos.y;
        minx = minx.min(p.pos.x);
        maxx = maxx.max(p.pos.x);
        miny = miny.min(p.pos.y);
        maxy = maxy.max(p.pos.y);
        maxh = maxh.max(p.z);
        minh = minh.min(p.z);
        sum_rcs += p.rcs;
        max_rcs = max_rcs.max(p.rcs);
        if p.has_velocity {
            sum_vr += p.vr;
            vel_count += 1;
        }
        if p.z >= cfg.person_min_height_m || !cfg.height_available {
            person_points += 1;
        }
    }
    let mean_rcs = sum_rcs / n;
    if mean_rcs < cfg.min_rcs && max_rcs < cfg.min_rcs {
        return None;
    }
    Some(Blob {
        ts,
        centroid: Pos2::new(sx / n, sy / n),
        point_count: pts.len(),
        width: maxx - minx,
        depth: maxy - miny,
        max_height: maxh,
        min_height: if minh == f32::MAX { 0.0 } else { minh },
        max_rcs,
        mean_rcs,
        mean_radial_velocity: if vel_count > 0 {
            sum_vr / vel_count as f32
        } else {
            0.0
        },
        has_velocity: vel_count > 0,
        person_points,
    })
}

/// Classification outcome for one blob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlobKind {
    Person,
    /// A cart / trolley / wide low load seen with **no person** in its cluster.
    ///
    /// A cart is not a counted target: it contributes zero person equivalents
    /// and instead tells the engine that a cart is present, which enables the
    /// infrared to vouch for a pusher the radar could not see behind it. This
    /// avoids the failure mode where a cart track and its operator's torso trade
    /// identities frame to frame.
    Vehicle,
}

/// A classified unit produced from a blob.
#[derive(Debug, Clone)]
pub struct Unit {
    pub blob: Blob,
    pub kind: BlobKind,
    /// For a split blob, `(index, total)` (e.g. `(2,3)`).
    pub part: Option<(u8, u8)>,
    /// True when the person's cluster contains a wide, low cart body — the
    /// person is pushing or walking with a cart.
    pub cart_evidence: bool,
}

/// Cluster and classify a whole frame.
///
/// The unit of interest is the **person**, not the cart. Classification is
/// driven by a torso search: person-height returns are joined into compact
/// groups (torsos) with DBSCAN.
///
/// * Two or more torsos in one cluster → that many people. This is how "two
///   colleagues side by side" is separated even when their point clouds merge.
/// * One torso → one person; if the cluster also carries a wide, low body, that
///   person is flagged as `cart_evidence` (pushing a trolley) but still counts
///   as exactly one.
/// * No torso, and the cluster is wide and low → a **bare cart** with nobody
///   visible. It contributes no person equivalent; it only tells the engine a
///   cart is present (see [`BlobKind::Vehicle`]).
pub fn process_frame(ts: Ts, points: &[RadarPoint], cfg: &RadarConfig) -> Vec<Unit> {
    let kept: Vec<&RadarPoint> = points
        .iter()
        .filter(|p| p.pos.is_finite() && p.pos.norm() <= cfg.max_range_m)
        .collect();
    if kept.is_empty() {
        return Vec::new();
    }
    let xs: Vec<Pos2> = kept.iter().map(|p| p.pos).collect();
    let labels = dbscan(&xs, cfg.cluster_eps_m, cfg.cluster_min_points);

    let mut members: Vec<Vec<&RadarPoint>> = Vec::new();
    for (idx, label) in labels.iter().enumerate() {
        let Some(l) = *label else { continue };
        if l >= members.len() {
            members.resize(l + 1, Vec::new());
        }
        members[l].push(kept[idx]);
    }

    let mut out = Vec::new();
    for mut group in members {
        if group.is_empty() {
            continue;
        }
        group.sort_by(|a, b| a.pos.x.total_cmp(&b.pos.x));
        let Some(blob) = summarise(ts, &group, cfg) else {
            continue;
        };
        let cart = has_cart_body(&group, cfg);
        let torsos = find_torsos(ts, &group, &blob, cfg);

        if torsos.len() >= 2 {
            // Two or more people sharing a cluster (abreast, or with a cart).
            let total = torsos.len().min(u8::MAX as usize) as u8;
            for (i, b) in torsos.into_iter().enumerate() {
                out.push(Unit {
                    blob: b,
                    kind: BlobKind::Person,
                    part: Some((i as u8 + 1, total)),
                    cart_evidence: cart,
                });
            }
            continue;
        }
        if torsos.len() == 1 {
            let b = torsos.into_iter().next().expect("len 1");
            out.push(Unit {
                blob: b,
                kind: BlobKind::Person,
                part: None,
                cart_evidence: cart,
            });
            continue;
        }
        // No torso at all.
        let wide_low = blob.width >= cfg.vehicle_min_width_m
            && (!cfg.height_available || blob.max_height <= cfg.vehicle_max_height_m);
        out.push(Unit {
            blob,
            kind: if wide_low {
                BlobKind::Vehicle
            } else {
                BlobKind::Person
            },
            part: None,
            cart_evidence: false,
        });
    }
    out
}

/// True when a cluster contains a wide, low body (a cart) — a horizontal band
/// of returns below person height spanning more than `vehicle_min_width_m`.
fn has_cart_body(group: &[&RadarPoint], cfg: &RadarConfig) -> bool {
    if !cfg.height_available {
        return false;
    }
    let low: Vec<&RadarPoint> = group
        .iter()
        .copied()
        .filter(|p| p.z <= cfg.vehicle_max_height_m)
        .collect();
    if low.len() < 3 {
        return false;
    }
    let minx = low.iter().map(|p| p.pos.x).fold(f32::MAX, f32::min);
    let maxx = low.iter().map(|p| p.pos.x).fold(f32::MIN, f32::max);
    maxx - minx >= cfg.vehicle_min_width_m
}


/// Locate torsos inside a cluster by DBSCAN over **person-height returns only**
/// in the floor plane.
///
/// Two people abreast merge into one cluster through their shoulders but their
/// torsos are still separate point groups; a cart body has no person-height
/// returns at all, so it never forms a torso. Returns one blob per torso
/// (partitioning *all* points of the cluster by nearest torso), or an empty
/// vector when there is no clear torso structure (a bare cart, or too many
/// points merged to resolve).
fn find_torsos(ts: Ts, group: &[&RadarPoint], _blob: &Blob, cfg: &RadarConfig) -> Vec<Blob> {
    if !cfg.height_available {
        // Without height the sensor cannot separate torsos; the tracker's
        // convoy logic and the beam still handle carts.
        return Vec::new();
    }
    let torso_pts: Vec<&RadarPoint> = group
        .iter()
        .copied()
        .filter(|p| p.z >= cfg.person_min_height_m)
        .collect();
    if torso_pts.len() < cfg.torso_min_points {
        return Vec::new();
    }
    let positions: Vec<Pos2> = torso_pts.iter().map(|p| p.pos).collect();
    let labels = dbscan(&positions, cfg.torso_eps_m, cfg.torso_min_points);
    let mut torsos: Vec<Vec<&RadarPoint>> = Vec::new();
    for (i, label) in labels.iter().enumerate() {
        let Some(l) = *label else { continue };
        if l >= torsos.len() {
            torsos.resize(l + 1, Vec::new());
        }
        torsos[l].push(torso_pts[i]);
    }
    if torsos.len() <= 1 {
        return Vec::new();
    }
    // Partition every point (cart body included) to the nearest torso centre,
    // so each person blob carries the part of the cart it is steering.
    let centres: Vec<Pos2> = torsos
        .iter()
        .map(|t| {
            let n = t.len() as f32;
            t.iter().fold(Pos2::ZERO, |a, p| a.add(p.pos)).scaled(1.0 / n)
        })
        .collect();
    partition_by_centres(ts, group, &centres, cfg)
}

fn partition_by_centres(
    ts: Ts,
    group: &[&RadarPoint],
    centres: &[Pos2],
    cfg: &RadarConfig,
) -> Vec<Blob> {
    let mut buckets: Vec<Vec<&RadarPoint>> = vec![Vec::new(); centres.len()];
    for p in group {
        let mut best = 0usize;
        let mut best_d = f32::MAX;
        for (i, c) in centres.iter().enumerate() {
            let d = p.pos.distance(*c);
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        buckets[best].push(*p);
    }
    buckets
        .into_iter()
        .filter_map(|b| summarise(ts, &b, cfg))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn person(x: f32, y: f32) -> RadarPoint {
        RadarPoint::new(x, y, 1.45).with_rcs(-4.0)
    }

    /// A torso: a compact group of person-height returns, one person.
    fn torso(x: f32, y: f32) -> Vec<RadarPoint> {
        vec![
            person(x - 0.06, y),
            person(x, y + 0.04),
            person(x + 0.06, y),
            person(x, y - 0.03),
        ]
    }

    fn cart_row(y: f32) -> Vec<RadarPoint> {
        (0..8)
            .map(|i| RadarPoint::new(-0.6 + 0.17 * i as f32, y, 0.5).with_rcs(-2.0))
            .collect()
    }

    #[test]
    fn single_person_is_one_person_unit() {
        let pts = torso(0.0, 2.0);
        let units = process_frame(Ts(0), &pts, &RadarConfig::default());
        assert_eq!(units.len(), 1);
        assert_eq!(units[0].kind, BlobKind::Person);
        assert!(units[0].part.is_none());
    }

    #[test]
    fn two_people_abreast_are_split() {
        // Two compact torsos 0.8 m apart; DBSCAN may merge them via the wide
        // cluster, and the torso search must recover two people.
        let mut pts = torso(-0.40, 2.0);
        pts.extend(torso(0.40, 2.0));
        // Bridge the gap with a couple of low (cart-height) returns so DBSCAN
        // sees a single cluster, as it does when two people brush shoulders.
        pts.push(RadarPoint::new(0.0, 2.0, 0.6).with_rcs(-2.0));
        let units = process_frame(Ts(0), &pts, &RadarConfig::default());
        let persons = units.iter().filter(|u| u.kind == BlobKind::Person).count();
        assert_eq!(persons, 2, "expected 2 persons, got {units:?}");
    }

    #[test]
    fn cart_is_vehicle() {
        let units = process_frame(Ts(0), &cart_row(2.0), &RadarConfig::default());
        assert_eq!(units.len(), 1);
        assert_eq!(units[0].kind, BlobKind::Vehicle);
    }

    #[test]
    fn person_pushing_wide_cart_is_one_person() {
        // A wide, low cart plus one pusher torso close behind it, so the point
        // clouds merge into a single cluster.
        let mut pts = cart_row(2.0);
        pts.extend(torso(0.0, 2.22));
        let units = process_frame(Ts(0), &pts, &RadarConfig::default());
        assert_eq!(units.len(), 1, "one entity: person with cart");
        assert_eq!(units[0].kind, BlobKind::Person);
    }

    #[test]
    fn two_people_walking_behind_a_cart_are_two() {
        let mut pts = cart_row(2.0);
        pts.extend(torso(-0.40, 2.25));
        pts.extend(torso(0.40, 2.25));
        let units = process_frame(Ts(0), &pts, &RadarConfig::default());
        let persons = units.iter().filter(|u| u.kind == BlobKind::Person).count();
        assert_eq!(persons, 2, "two pushers/companions behind the cart: {units:?}");
    }

    #[test]
    fn noise_cluster_is_dropped() {
        let p = person(0.0, 2.0).with_rcs(-40.0);
        let units = process_frame(Ts(0), &[p, p, p], &RadarConfig::default());
        assert!(units.is_empty());
    }

    #[test]
    fn out_of_range_points_are_ignored() {
        let units = process_frame(Ts(0), &[person(0.0, 9.0), person(0.1, 9.0)], &RadarConfig::default());
        assert!(units.is_empty());
    }

    #[test]
    fn dbscan_separates_distant_points() {
        let pts = vec![Pos2::new(0.0, 0.0), Pos2::new(0.1, 0.0), Pos2::new(2.0, 0.0)];
        let labels = dbscan(&pts, 0.3, 2);
        assert_eq!(labels[0], labels[1]);
        assert!(labels[1] != labels[2]);
    }

    #[test]
    fn dbscan_treats_singletons_as_noise() {
        let pts = vec![Pos2::new(0.0, 0.0), Pos2::new(3.0, 3.0)];
        let labels = dbscan(&pts, 0.3, 2);
        assert!(labels.iter().all(|l| l.is_none()));
    }
}
