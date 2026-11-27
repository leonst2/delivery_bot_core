//! SE(2) poses.

use std::ops::Mul;

use crate::angle::normalize_angle;

/// A planar pose: position (x, y) and heading yaw. Every operation that produces a pose normalises the yaw to
/// (-pi, pi]; `new` stores it as given.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// The inverse transform: `p * p.inverse()` is the identity.
    #[must_use]
    pub fn inverse(&self) -> Self {
        let (s, c) = self.yaw.sin_cos();
        Self::new(
            -c * self.x - s * self.y,
            s * self.x - c * self.y,
            normalize_angle(-self.yaw),
        )
    }

    /// Maps a point given in this pose's frame into the parent frame.
    #[must_use]
    pub fn transform(&self, point: (f64, f64)) -> (f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        (
            self.x + c * point.0 - s * point.1,
            self.y + s * point.0 + c * point.1,
        )
    }

    /// Maps a point given in the parent frame into this pose's frame.
    #[must_use]
    pub fn inverse_transform(&self, point: (f64, f64)) -> (f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        let (dx, dy) = (point.0 - self.x, point.1 - self.y);
        (c * dx + s * dy, -s * dx + c * dy)
    }

    #[must_use]
    pub fn position(&self) -> (f64, f64) {
        (self.x, self.y)
    }

    /// Euclidean distance between the positions.
    #[must_use]
    pub fn distance(&self, other: &Self) -> f64 {
        (other.x - self.x).hypot(other.y - self.y)
    }
}

/// Composition: `a * b` is pose b (expressed in frame a) expressed in a's parent frame.
impl Mul for Pose2 {
    type Output = Pose2;

    fn mul(self, other: Pose2) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2::new(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            normalize_angle(self.yaw + other.yaw),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::angle::{angle_diff, PI};

    fn close(a: Pose2, b: Pose2) -> bool {
        (a.x - b.x).abs() < 1e-9
            && (a.y - b.y).abs() < 1e-9
            && angle_diff(a.yaw, b.yaw).abs() < 1e-9
    }

    #[test]
    fn compose_and_inverse() {
        let a = Pose2::new(1.0, 2.0, PI / 2.0);
        assert!(close(
            a * Pose2::new(1.0, 0.0, 0.0),
            Pose2::new(1.0, 3.0, PI / 2.0)
        ));
        let p = Pose2::new(-3.0, 4.5, 2.7);
        assert!(close(p * p.inverse(), Pose2::default()));
        assert!(close(p.inverse() * p, Pose2::default()));
        let local = (0.7, -1.2);
        let back = p.inverse_transform(p.transform(local));
        assert!((back.0 - local.0).abs() < 1e-12 && (back.1 - local.1).abs() < 1e-12);
    }
}
