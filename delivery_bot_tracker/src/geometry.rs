//! Planar poses. Everything the tracker does happens in the ground plane of the `map` frame.

use std::f64::consts::{PI, TAU};

/// Wraps an angle to (-pi, pi].
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % TAU;
    if a > PI {
        a -= TAU;
    } else if a <= -PI {
        a += TAU;
    }
    a
}

/// A rigid transform in the plane: rotate by `yaw`, then translate by (`x`, `y`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub const IDENTITY: Pose2 = Pose2 {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };

    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self {
            x,
            y,
            yaw: normalize_angle(yaw),
        }
    }

    /// Planar part of a 3D transform: translation x/y and the yaw of the quaternion.
    pub fn from_translation_quaternion(
        tx: f64,
        ty: f64,
        qx: f64,
        qy: f64,
        qz: f64,
        qw: f64,
    ) -> Self {
        let yaw = (2.0 * (qw * qz + qx * qy)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
        Self::new(tx, ty, yaw)
    }

    /// `self ∘ other`: with `self` = T_a_b and `other` = T_b_c, returns T_a_c.
    pub fn compose(&self, other: &Pose2) -> Pose2 {
        let p = self.transform([other.x, other.y]);
        Pose2::new(p[0], p[1], self.yaw + other.yaw)
    }

    pub fn inverse(&self) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2::new(
            -(c * self.x + s * self.y),
            s * self.x - c * self.y,
            -self.yaw,
        )
    }

    /// Maps a point from the child frame into the parent frame.
    pub fn transform(&self, p: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.yaw.sin_cos();
        [self.x + c * p[0] - s * p[1], self.y + s * p[0] + c * p[1]]
    }

    /// Linear in position, shortest arc in yaw. `t` = 0 gives `self`, 1 gives `other`.
    pub fn interpolate(&self, other: &Pose2, t: f64) -> Pose2 {
        Pose2::new(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
            self.yaw + normalize_angle(other.yaw - self.yaw) * t,
        )
    }
}

pub fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &Pose2, b: &Pose2) -> bool {
        (a.x - b.x).abs() < 1e-9
            && (a.y - b.y).abs() < 1e-9
            && normalize_angle(a.yaw - b.yaw).abs() < 1e-9
    }

    #[test]
    fn angles_wrap_to_half_open_interval() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(-0.5) + 0.5).abs() < 1e-12);
        assert!((normalize_angle(TAU + 0.25) - 0.25).abs() < 1e-12);
    }

    #[test]
    fn compose_and_inverse_round_trip() {
        let a = Pose2::new(1.0, -2.0, 0.7);
        let b = Pose2::new(-0.3, 4.0, -2.9);
        assert!(close(&a.compose(&a.inverse()), &Pose2::IDENTITY));
        assert!(close(&a.inverse().compose(&a.compose(&b)), &b));
        // A point 1 m ahead of a robot at (1, 0) facing +y lies at (1, 1).
        let p = Pose2::new(1.0, 0.0, PI / 2.0).transform([1.0, 0.0]);
        assert!((p[0] - 1.0).abs() < 1e-12 && (p[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn yaw_from_quaternion() {
        let half = 0.4_f64;
        let pose = Pose2::from_translation_quaternion(1.0, 2.0, 0.0, 0.0, half.sin(), half.cos());
        assert!((pose.yaw - 0.8).abs() < 1e-12);
    }

    #[test]
    fn interpolation_takes_the_short_way_round() {
        let a = Pose2::new(0.0, 0.0, PI - 0.1);
        let b = Pose2::new(2.0, 0.0, -PI + 0.1);
        let mid = a.interpolate(&b, 0.5);
        assert!((mid.x - 1.0).abs() < 1e-12);
        assert!((mid.yaw.abs() - PI).abs() < 1e-9);
    }
}
