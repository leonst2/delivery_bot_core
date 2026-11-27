//! Arc utilities for path tracking (same formulas as `arc.hpp` / `arc.cpp`).

use crate::angle::{angle_diff, normalize_angle, PI, TWO_PI};
use crate::path::{advance, Path, Segment};
use crate::se2::Pose2;

const STRAIGHT_CURVATURE: f64 = 1e-9; // below this a segment is treated as straight
const CENTRE_DISTANCE: f64 = 1e-12; // query point on the arc centre: every arc point is equally far

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ClosestPoint {
    /// Arc length of the closest point.
    pub s: f64,
    /// Path pose there.
    pub pose: Pose2,
    /// m.
    pub distance: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrackingError {
    /// Arc length of the reference (closest) point.
    pub s: f64,
    /// Path pose there.
    pub reference: Pose2,
    /// m, + = robot left of the reference heading.
    pub cross_track: f64,
    /// rad, robot yaw - reference yaw, in (-pi, pi].
    pub heading: f64,
}

// Candidate local arc lengths (from the segment start, >= 0) of the closest point on one segment, within [lo, hi].
// Always contains lo and hi; sorted ascending.
fn segment_candidates(
    start: &Pose2,
    seg: &Segment,
    point: (f64, f64),
    lo: f64,
    hi: f64,
) -> Vec<f64> {
    let mut local = vec![lo, hi];
    let dir = f64::from(seg.direction());
    if seg.curvature.abs() < STRAIGHT_CURVATURE {
        let along = start.yaw.cos() * (point.0 - start.x) + start.yaw.sin() * (point.1 - start.y);
        local.push((dir * along).clamp(lo, hi));
    } else {
        let k = seg.curvature;
        let cx = start.x - start.yaw.sin() / k;
        let cy = start.y + start.yaw.cos() / k;
        if (point.0 - cx).hypot(point.1 - cy) >= CENTRE_DISTANCE {
            let sign = if k > 0.0 { 1.0 } else { -1.0 };
            let yaw_q = (point.1 - cy).atan2(point.0 - cx) + sign * PI / 2.0;
            let sigma0 = normalize_angle(yaw_q - start.yaw) / k;
            let period = TWO_PI / k.abs();
            let (sigma_lo, sigma_hi) = if dir > 0.0 { (lo, hi) } else { (-hi, -lo) };
            // Integral lap counts; the range holds a handful of laps at most.
            #[allow(clippy::cast_possible_truncation)]
            let n_min = ((sigma_lo - sigma0) / period).ceil() as i64;
            #[allow(clippy::cast_possible_truncation)]
            let n_max = ((sigma_hi - sigma0) / period).floor() as i64;
            for n in n_min..=n_max {
                #[allow(clippy::cast_precision_loss)]
                let laps = n as f64;
                local.push((dir * (sigma0 + laps * period)).clamp(lo, hi));
            }
        }
    }
    local.sort_by(f64::total_cmp);
    local
}

/// The pose at arc length s (clamped to the path).
pub fn point_at(path: &Path, s: f64) -> Pose2 {
    path.pose_at(s)
}

/// The exact closest point of the path to `point`, searched only within [s_min, s_max] (clamped to the path; pass
/// `f64::INFINITY` for no upper limit). Ties go to the smallest s. An empty path gives its start pose at s = 0.
pub fn closest_point(path: &Path, point: (f64, f64), s_min: f64, s_max: f64) -> ClosestPoint {
    let distance_to = |pose: &Pose2| (point.0 - pose.x).hypot(point.1 - pose.y);
    if path.is_empty() {
        return ClosestPoint {
            s: 0.0,
            pose: *path.start(),
            distance: distance_to(path.start()),
        };
    }
    let total = path.length();
    let window_lo = s_min.clamp(0.0, total);
    let window_hi = s_max.clamp(window_lo, total);

    let mut best = ClosestPoint {
        s: 0.0,
        pose: Pose2::default(),
        distance: f64::INFINITY,
    };
    for (i, seg) in path.segments().iter().enumerate() {
        let seg_start = path.segment_start_s(i);
        let seg_end = path.segment_start_s(i + 1);
        let lo = window_lo.max(seg_start);
        let hi = window_hi.min(seg_end);
        if lo > hi {
            continue;
        }
        let start = path.segment_start_pose(i);
        for local in segment_candidates(start, seg, point, lo - seg_start, hi - seg_start) {
            let pose = advance(start, seg.curvature, f64::from(seg.direction()) * local);
            let d = distance_to(&pose);
            if d < best.distance {
                best = ClosestPoint {
                    s: seg_start + local,
                    pose,
                    distance: d,
                };
            }
        }
    }
    best
}

/// Signed lateral offset of `point` from the line through `reference` along its heading, + = left.
pub fn cross_track_error(reference: &Pose2, point: (f64, f64)) -> f64 {
    -reference.yaw.sin() * (point.0 - reference.x) + reference.yaw.cos() * (point.1 - reference.y)
}

/// yaw - reference.yaw, normalised to (-pi, pi].
pub fn heading_error(reference: &Pose2, yaw: f64) -> f64 {
    angle_diff(yaw, reference.yaw)
}

/// closest_point of the robot position, with the cross-track and heading errors against it.
pub fn tracking_error(path: &Path, robot: &Pose2, s_min: f64, s_max: f64) -> TrackingError {
    let closest = closest_point(path, robot.position(), s_min, s_max);
    TrackingError {
        s: closest.s,
        reference: closest.pose,
        cross_track: cross_track_error(&closest.pose, robot.position()),
        heading: heading_error(&closest.pose, robot.yaw),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_ends_and_signs() {
        let path = Path::new(Pose2::default(), vec![Segment::straight(5.0)]);
        assert!(
            closest_point(&path, (-3.0, 1.0), 0.0, f64::INFINITY)
                .s
                .abs()
                < 1e-12
        );
        let end = closest_point(&path, (7.0, -1.0), 0.0, f64::INFINITY);
        assert!((end.s - 5.0).abs() < 1e-12 && (end.distance - 5f64.sqrt()).abs() < 1e-12);
        let e = tracking_error(&path, &Pose2::new(2.0, 0.3, 0.1), 0.0, f64::INFINITY);
        assert!(
            (e.s - 2.0).abs() < 1e-12
                && (e.cross_track - 0.3).abs() < 1e-12
                && (e.heading - 0.1).abs() < 1e-12
        );
    }

    #[test]
    fn laps_and_windows() {
        let r = 1.25;
        let laps = Path::new(
            Pose2::new(r, 0.0, PI / 2.0),
            vec![Segment::left(TWO_PI * r * 1.3, r)],
        );
        let first = closest_point(&laps, (0.0, r), 0.0, f64::INFINITY);
        assert!((first.s - PI / 2.0 * r).abs() < 1e-9);
        let later = closest_point(&laps, (0.0, r), PI * r, f64::INFINITY);
        assert!((later.s - (TWO_PI + PI / 2.0) * r).abs() < 1e-9);
    }
}
