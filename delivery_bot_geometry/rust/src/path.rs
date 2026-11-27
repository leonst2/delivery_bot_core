//! Paths made of constant-curvature segments (same conventions and formulas as `path.hpp`).

use crate::angle::normalize_angle;
use crate::se2::Pose2;

const MIN_SEGMENT_LENGTH: f64 = 1e-12;

/// A constant-curvature segment.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Segment {
    /// 1/m, + = steering left.
    pub curvature: f64,
    /// m, signed: < 0 = reverse.
    pub length: f64,
}

impl Segment {
    pub const fn new(curvature: f64, length: f64) -> Self {
        Self { curvature, length }
    }

    pub const fn straight(length: f64) -> Self {
        Self::new(0.0, length)
    }

    pub fn left(length: f64, radius: f64) -> Self {
        Self::new(1.0 / radius, length)
    }

    pub fn right(length: f64, radius: f64) -> Self {
        Self::new(-1.0 / radius, length)
    }

    /// +1 forward, -1 reverse. A zero-length segment counts as forward.
    pub fn direction(&self) -> i32 {
        if self.length < 0.0 {
            -1
        } else {
            1
        }
    }
}

// sin(u) / u, with the Taylor series near 0 where the quotient loses precision.
fn sinc(u: f64) -> f64 {
    const SERIES_BELOW: f64 = 1e-4;
    if u.abs() < SERIES_BELOW {
        1.0 - u * u / 6.0
    } else {
        u.sin() / u
    }
}

/// The pose after driving the signed `length` with constant `curvature` from `start`.
pub fn advance(start: &Pose2, curvature: f64, length: f64) -> Pose2 {
    // The chord of an arc points along the mean heading and has length l * sinc(dtheta / 2).
    let dtheta = curvature * length;
    let half = 0.5 * dtheta;
    let chord = length * sinc(half);
    let heading = start.yaw + half;
    Pose2::new(
        start.x + chord * heading.cos(),
        start.y + chord * heading.sin(),
        normalize_angle(start.yaw + dtheta),
    )
}

/// A path from a start pose along a list of segments. Segments shorter than 1e-12 m are dropped. A point exactly on a
/// segment boundary belongs to the following segment (the last point to the last segment).
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    start: Pose2,
    segments: Vec<Segment>,
    cumulative_s: Vec<f64>,
    segment_start: Vec<Pose2>,
}

impl Default for Path {
    fn default() -> Self {
        Self::new(Pose2::default(), Vec::new())
    }
}

impl Path {
    pub fn new(start: Pose2, segments: Vec<Segment>) -> Self {
        let start = Pose2::new(start.x, start.y, normalize_angle(start.yaw));
        let segments: Vec<Segment> = segments
            .into_iter()
            .filter(|seg| seg.length.abs() >= MIN_SEGMENT_LENGTH)
            .collect();
        let mut cumulative_s = Vec::with_capacity(segments.len() + 1);
        let mut segment_start = Vec::with_capacity(segments.len() + 1);
        cumulative_s.push(0.0);
        segment_start.push(start);
        for seg in &segments {
            let s = cumulative_s[cumulative_s.len() - 1] + seg.length.abs();
            let pose = advance(
                &segment_start[segment_start.len() - 1],
                seg.curvature,
                seg.length,
            );
            cumulative_s.push(s);
            segment_start.push(pose);
        }
        Self {
            start,
            segments,
            cumulative_s,
            segment_start,
        }
    }

    pub fn start(&self) -> &Pose2 {
        &self.start
    }

    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Total travelled arc length.
    pub fn length(&self) -> f64 {
        self.cumulative_s[self.cumulative_s.len() - 1]
    }

    pub fn end(&self) -> &Pose2 {
        &self.segment_start[self.segment_start.len() - 1]
    }

    /// Arc length at which segment i starts; i == segments().len() gives length().
    pub fn segment_start_s(&self, i: usize) -> f64 {
        self.cumulative_s[i]
    }

    /// Pose at which segment i starts; i == segments().len() gives end().
    pub fn segment_start_pose(&self, i: usize) -> &Pose2 {
        &self.segment_start[i]
    }

    /// Index of the segment that contains s (clamped). Panics on an empty path.
    pub fn segment_index_at(&self, s: f64) -> usize {
        assert!(
            !self.segments.is_empty(),
            "Path::segment_index_at on an empty path"
        );
        // First boundary strictly greater than s, minus one: a boundary belongs to the following segment.
        let upper = self.cumulative_s.partition_point(|&boundary| boundary <= s);
        upper.saturating_sub(1).min(self.segments.len() - 1)
    }

    /// Pose at arc length s, clamped to [0, length].
    pub fn pose_at(&self, s: f64) -> Pose2 {
        if self.segments.is_empty() {
            return self.start;
        }
        let clamped = s.clamp(0.0, self.length());
        let i = self.segment_index_at(clamped);
        let seg = &self.segments[i];
        let local = clamped - self.cumulative_s[i];
        advance(
            &self.segment_start[i],
            seg.curvature,
            f64::from(seg.direction()) * local,
        )
    }

    /// Steering curvature at s (0 for an empty path).
    pub fn curvature_at(&self, s: f64) -> f64 {
        if self.segments.is_empty() {
            0.0
        } else {
            self.segments[self.segment_index_at(s)].curvature
        }
    }

    /// +1 forward, -1 reverse at s (+1 for an empty path).
    pub fn direction_at(&self, s: f64) -> i32 {
        if self.segments.is_empty() {
            1
        } else {
            self.segments[self.segment_index_at(s)].direction()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::angle::PI;

    #[test]
    fn quarter_circles_and_boundaries() {
        let r = 1.25;
        let q = PI / 2.0 * r;
        let left = advance(&Pose2::default(), 1.0 / r, q);
        assert!(
            (left.x - r).abs() < 1e-12
                && (left.y - r).abs() < 1e-12
                && (left.yaw - PI / 2.0).abs() < 1e-12
        );
        let back_left = advance(&Pose2::default(), 1.0 / r, -q);
        assert!((back_left.x + r).abs() < 1e-12 && (back_left.y - r).abs() < 1e-12);

        let path = Path::new(
            Pose2::default(),
            vec![
                Segment::left(1.0, r),
                Segment::straight(-2.0),
                Segment::straight(0.0),
                Segment::right(0.5, r),
            ],
        );
        assert_eq!(path.segments().len(), 3);
        assert!((path.length() - 3.5).abs() < 1e-12);
        assert_eq!(path.segment_index_at(0.999), 0);
        assert_eq!(path.segment_index_at(1.0), 1);
        assert_eq!(path.segment_index_at(10.0), 2);
        assert_eq!(path.direction_at(1.5), -1);
        assert_eq!(path.pose_at(-1.0), *path.start());
        assert_eq!(path.pose_at(100.0), path.pose_at(path.length()));
    }
}
