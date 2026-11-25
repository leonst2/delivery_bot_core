//! Multi-object tracker: predict every track to the measurement time, associate detections (Hungarian on gated
//! Mahalanobis distances), update, start tentative tracks, confirm them N-of-M, drop lost ones.
//!
//! Only LiDAR updates are *cycles*: they advance the confirmation window and count misses. Optional sources (stereo,
//! side cameras) update tracks and start new ones but never count a miss, because they don't see all around.

use crate::assign::assign;
use crate::cluster::Cluster;
use crate::geometry::distance;
use crate::kalman::Imm;
use crate::params::Params;

/// A clock this far behind the last update is a reset (/sim/reset); smaller steps back are stale messages.
const RESET_JUMP_NS: i64 = 1_000_000_000;

#[derive(Clone, Debug, PartialEq)]
pub enum Detection {
    /// Points of one object, in `map`.
    Cluster(Cluster),
    /// Ground point of one object (side cameras), in `map`.
    Point([f64; 2]),
}

#[derive(Clone, Debug, PartialEq)]
pub struct TrackOutput {
    pub id: u32,
    pub position: [f64; 2],
    pub velocity: [f64; 2],
    pub position_covariance: [f64; 4],
    pub radius: f64,
    pub predicted_position: [f64; 2],
    pub prediction_horizon: f64,
    pub age: f64,
}

#[derive(Clone, Debug)]
struct Track {
    id: u32,
    kf: Imm,
    first_stamp: i64,
    birth: [f64; 2],
    updates: u32,
    /// Bit k = hit k cycles ago, bit 0 = the cycle that ended last.
    hit_history: u32,
    cycles: u32,
    hit_this_cycle: bool,
    misses: u32,
    confirmed: bool,
    moving: bool,
    /// Whether the last cluster measurement came from the footprint fit (else from the near surface).
    fitted: bool,
    radius: f64,
}

impl Track {
    fn speed(&self) -> f64 {
        let v = self.kf.velocity();
        v[0].hypot(v[1])
    }

    /// Heading of the object, once it is known well enough to fit the footprint along it.
    fn heading(&self, params: &Params) -> Option<f64> {
        let v = self.kf.velocity();
        let velocity_std = self.kf.velocity_variance().sqrt();
        let known = self.moving
            && self.speed() >= params.heading_min_speed
            && velocity_std <= params.heading_max_velocity_std;
        known.then(|| v[1].atan2(v[0]))
    }
}

pub struct Tracker {
    params: Params,
    tracks: Vec<Track>,
    next_id: u32,
    last_stamp: Option<i64>,
}

/// Centre estimate while the heading is unknown: along the bearing of the centroid, `offset` behind the nearest
/// surface on that bearing. The bearing follows an object that slides sideways (the nearest point of a long side seen
/// broadside doesn't), and the depth doesn't jump when a sparse point on a grazing side comes or goes (the
/// centroid's does).
fn behind_near_surface(cluster: &Cluster, sensor: [f64; 2], offset: f64) -> [f64; 2] {
    let (dx, dy) = (
        cluster.centroid[0] - sensor[0],
        cluster.centroid[1] - sensor[1],
    );
    let d = dx.hypot(dy);
    if d < 1e-9 {
        return cluster.centroid;
    }
    let bearing = [dx / d, dy / d];
    let depth = cluster
        .points
        .iter()
        .map(|q| (q[0] - sensor[0]) * bearing[0] + (q[1] - sensor[1]) * bearing[1])
        .fold(f64::INFINITY, f64::min);
    [
        sensor[0] + bearing[0] * (depth + offset),
        sensor[1] + bearing[1] * (depth + offset),
    ]
}

/// Centre of a `length` x `width` box with the given heading whose visible side produced `points`. Along each box
/// axis: if the sensor lies beyond the points, the nearest point is on the box face that looks at the sensor, so the
/// centre is half the box size farther away; otherwise the face along that axis is seen side-on and the centre is the
/// middle of the points.
pub fn box_center(
    points: &[[f64; 2]],
    heading: f64,
    sensor: [f64; 2],
    length: f64,
    width: f64,
) -> [f64; 2] {
    let (s, c) = heading.sin_cos();
    let axes = [([c, s], length), ([-s, c], width)];
    let mut center = [0.0, 0.0];
    for (axis, size) in axes {
        let project = |p: &[f64; 2]| p[0] * axis[0] + p[1] * axis[1];
        let (lo, hi) = points
            .iter()
            .map(project)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v), hi.max(v))
            });
        let at_sensor = project(&sensor);
        let along = if at_sensor < lo {
            lo + size / 2.0
        } else if at_sensor > hi {
            hi - size / 2.0
        } else {
            (lo + hi) / 2.0
        };
        center[0] += along * axis[0];
        center[1] += along * axis[1];
    }
    center
}

impl Tracker {
    pub fn new(params: Params) -> Self {
        Self {
            params,
            tracks: Vec::new(),
            next_id: 1,
            last_stamp: None,
        }
    }

    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Drops every track and starts the IDs again at 1.
    pub fn reset(&mut self) {
        self.tracks.clear();
        self.next_id = 1;
        self.last_stamp = None;
    }

    pub fn last_stamp(&self) -> Option<i64> {
        self.last_stamp
    }

    fn measurement(
        &self,
        track: Option<&Track>,
        detection: &Detection,
        sensor: [f64; 2],
    ) -> ([f64; 2], f64) {
        let p = &self.params;
        match detection {
            Detection::Point(point) => (*point, p.point_measurement_noise),
            Detection::Cluster(cluster) => {
                let z = match track.and_then(|t| t.heading(p)) {
                    Some(heading) => box_center(
                        &cluster.points,
                        heading,
                        sensor,
                        p.object_length,
                        p.object_width,
                    ),
                    None => behind_near_surface(cluster, sensor, p.surface_offset),
                };
                (z, p.measurement_noise)
            }
        }
    }

    /// When a track's cluster measurement switches between the near-surface estimate and the footprint fit, the two
    /// centres for the same points can be half a metre apart. Returns the new model and the shift that moves the
    /// track onto it, so the switch doesn't show up as motion.
    fn model_switch(
        &self,
        track: &Track,
        detection: &Detection,
        sensor: [f64; 2],
    ) -> Option<(bool, [f64; 2])> {
        let Detection::Cluster(cluster) = detection else {
            return None;
        };
        let p = &self.params;
        let fitted = track.heading(p).is_some();
        if fitted == track.fitted {
            return None;
        }
        let v = track.kf.velocity();
        let fit = box_center(
            &cluster.points,
            v[1].atan2(v[0]),
            sensor,
            p.object_length,
            p.object_width,
        );
        let surface = behind_near_surface(cluster, sensor, p.surface_offset);
        let (to, from) = if fitted {
            (fit, surface)
        } else {
            (surface, fit)
        };
        Some((fitted, [to[0] - from[0], to[1] - from[1]]))
    }

    /// Processes the detections of one sensor message. `sensor` is the robot position in `map`. `is_cycle` marks a
    /// LiDAR update. Returns false when the message is stale (older than the last update) and was ignored.
    pub fn step(
        &mut self,
        stamp: i64,
        sensor: [f64; 2],
        detections: &[Detection],
        is_cycle: bool,
    ) -> bool {
        if let Some(last) = self.last_stamp {
            if stamp < last - RESET_JUMP_NS {
                self.reset();
            } else if stamp < last {
                return false;
            }
        }
        let dt = self
            .last_stamp
            .map_or(0.0, |last| (stamp - last) as f64 * 1e-9);
        self.last_stamp = Some(stamp);
        for track in &mut self.tracks {
            track.kf.predict(dt);
        }

        // Gated cost matrix: rows = tracks (by id), columns = detections.
        let cost: Vec<Vec<f64>> = self
            .tracks
            .iter()
            .map(|track| {
                detections
                    .iter()
                    .map(|detection| {
                        let (z, noise) = self.measurement(Some(track), detection, sensor);
                        let d2 = track.kf.mahalanobis2(z, noise);
                        let gap = distance(z, track.kf.position());
                        let within = gap <= self.params.max_gate_distance
                            && (d2 <= self.params.gate_chi2
                                || gap <= self.params.min_gate_distance);
                        if within {
                            d2
                        } else {
                            f64::INFINITY
                        }
                    })
                    .collect()
            })
            .collect();
        let assignment = assign(&cost);

        let mut used = vec![false; detections.len()];
        for (row, column) in assignment.iter().enumerate() {
            let Some(column) = *column else { continue };
            used[column] = true;
            let (z, noise) = self.measurement(Some(&self.tracks[row]), &detections[column], sensor);
            let switch = self.model_switch(&self.tracks[row], &detections[column], sensor);
            let track = &mut self.tracks[row];
            if let Some((fitted, shift)) = switch {
                track.kf.shift(shift);
                track.fitted = fitted;
            }
            track.kf.update(z, noise);
            track.updates += 1;
            track.hit_this_cycle = true;
            // Only measured motion counts (a coasting track drifts on its velocity alone), and not before the third
            // update: two sparse measurements can be half a metre apart for a parked object.
            if track.updates >= 3
                && distance(track.kf.position(), track.birth) >= self.params.min_displacement
            {
                track.moving = true;
            }
            if let Detection::Cluster(cluster) = &detections[column] {
                track.radius = 0.8 * track.radius + 0.2 * cluster.radius;
            }
        }

        for (column, detection) in detections.iter().enumerate() {
            if used[column] {
                continue;
            }
            let (z, noise) = self.measurement(None, detection, sensor);
            let suppression = self.params.birth_suppression;
            let overlaps = |track: &Track| match detection {
                Detection::Cluster(cluster) => cluster
                    .points
                    .iter()
                    .any(|q| distance(track.kf.position(), *q) < suppression),
                Detection::Point(q) => distance(track.kf.position(), *q) < suppression,
            };
            if self.tracks.iter().any(overlaps) {
                continue;
            }
            let radius = match detection {
                Detection::Cluster(cluster) => cluster.radius,
                Detection::Point(_) => 0.0,
            };
            self.tracks.push(Track {
                id: self.next_id,
                kf: Imm::new(
                    z,
                    noise,
                    self.params.initial_speed_std,
                    [
                        self.params.accel_noise_smooth,
                        self.params.accel_noise_turning,
                    ],
                    self.params.model_switch_probability,
                ),
                first_stamp: stamp,
                birth: z,
                updates: 1,
                hit_history: 0,
                cycles: 0,
                hit_this_cycle: true,
                misses: 0,
                confirmed: false,
                moving: false,
                fitted: false,
                radius,
            });
            self.next_id = self.next_id.wrapping_add(1).max(1);
        }

        if is_cycle {
            self.end_cycle();
        }
        true
    }

    fn end_cycle(&mut self) {
        let p = &self.params;
        let window = if p.confirm_window >= 32 {
            u32::MAX
        } else {
            (1u32 << p.confirm_window) - 1
        };
        for track in &mut self.tracks {
            track.hit_history = ((track.hit_history << 1) | track.hit_this_cycle as u32) & window;
            track.cycles += 1;
            track.misses = if track.hit_this_cycle {
                0
            } else {
                track.misses + 1
            };
            track.hit_this_cycle = false;
            if !track.confirmed && track.hit_history.count_ones() >= p.confirm_hits {
                track.confirmed = true;
            }
        }
        let max_variance = 2.0 * p.max_position_std * p.max_position_std;
        self.tracks.retain(|t| {
            let c = t.kf.position_covariance();
            let alive = if t.confirmed {
                t.misses <= p.max_misses
            } else {
                t.cycles < p.confirm_window
            };
            alive && c[0] + c[3] <= max_variance
        });
    }

    /// Confirmed tracks at the time of the last update, sorted by id.
    pub fn outputs(&self) -> Vec<TrackOutput> {
        let p = &self.params;
        let now = self.last_stamp.unwrap_or(0);
        self.tracks
            .iter()
            .filter(|t| t.confirmed)
            .map(|t| {
                let position = t.kf.position();
                let velocity = if t.moving && t.speed() >= p.min_reported_speed {
                    t.kf.velocity()
                } else {
                    [0.0, 0.0]
                };
                TrackOutput {
                    id: t.id,
                    position,
                    velocity,
                    position_covariance: t.kf.position_covariance(),
                    radius: t.radius.max(p.default_radius),
                    predicted_position: [
                        position[0] + velocity[0] * p.prediction_horizon,
                        position[1] + velocity[1] * p.prediction_horizon,
                    ],
                    prediction_horizon: p.prediction_horizon,
                    age: (now - t.first_stamp) as f64 * 1e-9,
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cluster::{euclidean_clusters, Tolerance};

    const CYCLE: i64 = 100_000_000; // 10 Hz

    fn blob(center: [f64; 2]) -> Detection {
        let points: Vec<[f64; 2]> = (0..5)
            .map(|k| [center[0] + 0.1 * k as f64, center[1]])
            .collect();
        Detection::Cluster(euclidean_clusters(&points, Tolerance::fixed(0.35)).remove(0))
    }

    #[test]
    fn box_center_from_each_visible_side() {
        // Box 2 x 1 at the origin, heading +x. Sensor far to the left (+y): sees the long side y = 0.5.
        let side: Vec<[f64; 2]> = (0..11).map(|k| [-1.0 + 0.2 * k as f64, 0.5]).collect();
        let c = box_center(&side, 0.0, [0.0, 10.0], 2.0, 1.0);
        assert!(c[0].abs() < 1e-9 && c[1].abs() < 1e-9, "{c:?}");
        // Sensor straight ahead (+x): sees the short end x = 1.
        let end: Vec<[f64; 2]> = (0..6).map(|k| [1.0, -0.5 + 0.2 * k as f64]).collect();
        let c = box_center(&end, 0.0, [10.0, 0.0], 2.0, 1.0);
        assert!(c[0].abs() < 1e-9 && c[1].abs() < 1e-9, "{c:?}");
        // Sensor diagonal: sees the corner (1, 0.5) and parts of both faces.
        let corner = vec![[0.2, 0.5], [0.6, 0.5], [1.0, 0.5], [1.0, 0.1]];
        let c = box_center(&corner, 0.0, [10.0, 10.0], 2.0, 1.0);
        assert!(c[0].abs() < 1e-9 && c[1].abs() < 1e-9, "{c:?}");
    }

    #[test]
    fn confirms_on_the_second_hit_and_reports_velocity_once_moving() {
        let mut tracker = Tracker::new(Params::default());
        let mut confirmed_at = None;
        for k in 0..20 {
            let x = 5.0 + 0.5 * k as f64; // 5 m/s along +x
            tracker.step(k * CYCLE, [0.0, -6.0], &[blob([x, 0.0])], true);
            let out = tracker.outputs();
            if confirmed_at.is_none() && !out.is_empty() {
                confirmed_at = Some(k);
            }
        }
        assert_eq!(confirmed_at, Some(1));
        let out = tracker.outputs();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].id, 1);
        assert!((out[0].velocity[0] - 5.0).abs() < 0.3, "{:?}", out[0]);
        assert!((out[0].age - 1.9).abs() < 1e-9);
        assert!(
            (out[0].predicted_position[0] - out[0].position[0] - 1.5 * out[0].velocity[0]).abs()
                < 1e-9
        );
        assert!(out[0].radius >= Params::default().default_radius);
    }

    #[test]
    fn single_hits_never_become_tracks_and_lost_tracks_are_dropped() {
        let mut tracker = Tracker::new(Params::default());
        // Clutter: a different spot every cycle.
        for k in 0..10 {
            tracker.step(
                k * CYCLE,
                [0.0, 0.0],
                &[blob([3.0 * k as f64 - 10.0, 4.0])],
                true,
            );
            assert!(tracker.outputs().is_empty());
        }
        tracker.reset();
        for k in 0..5 {
            tracker.step(k * CYCLE, [0.0, 0.0], &[blob([4.0, 4.0])], true);
        }
        assert_eq!(tracker.outputs().len(), 1);
        for k in 5..20 {
            tracker.step(k * CYCLE, [0.0, 0.0], &[], true);
        }
        assert!(tracker.outputs().is_empty());
    }

    #[test]
    fn optional_sources_never_count_misses_and_stale_messages_are_ignored() {
        let mut tracker = Tracker::new(Params::default());
        let start = 100 * CYCLE;
        for k in 0..5 {
            tracker.step(start + k * CYCLE, [0.0, 0.0], &[blob([4.0, 4.0])], true);
        }
        for k in 0..30 {
            tracker.step(start + 5 * CYCLE + k * CYCLE / 10, [0.0, 0.0], &[], false);
        }
        assert_eq!(tracker.outputs().len(), 1);
        // Slightly older than the last update: stale, ignored.
        assert!(!tracker.step(start + CYCLE, [0.0, 0.0], &[blob([9.0, 9.0])], true));
        assert_eq!(tracker.outputs().len(), 1);
        // A big jump back is a reset.
        assert!(tracker.step(0, [0.0, 0.0], &[], true));
        assert!(tracker.outputs().is_empty());
    }

    #[test]
    fn a_static_object_is_never_moving() {
        let mut tracker = Tracker::new(Params::default());
        for k in 0..100 {
            let wobble = if k % 2 == 0 { 0.08 } else { -0.08 };
            tracker.step(
                k * CYCLE,
                [0.0, 0.0],
                &[blob([4.0 + wobble, 4.0 - wobble])],
                true,
            );
            for out in tracker.outputs() {
                assert_eq!(out.velocity, [0.0, 0.0]);
                assert_eq!(out.predicted_position, out.position);
            }
        }
    }
}
