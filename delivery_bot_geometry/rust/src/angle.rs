//! Angle helpers. All angles in rad, + = counter-clockwise.

pub const PI: f64 = std::f64::consts::PI;
pub const TWO_PI: f64 = std::f64::consts::TAU;

/// Wraps an angle into (-pi, pi]. -pi maps to +pi.
pub fn normalize_angle(angle: f64) -> f64 {
    // Same as C++ std::remainder(angle, 2 pi): round the quotient to the nearest integer, ties to even.
    let mut wrapped = angle - (angle / TWO_PI).round_ties_even() * TWO_PI;
    if wrapped <= -PI {
        wrapped += TWO_PI;
    } else if wrapped > PI {
        wrapped -= TWO_PI;
    }
    wrapped
}

/// Signed shortest rotation from `from` to `to`, in (-pi, pi].
pub fn angle_diff(to: f64, from: f64) -> f64 {
    normalize_angle(to - from)
}

/// Interpolates along the shortest rotation: t = 0 gives `from`, t = 1 gives `to` (normalised).
pub fn interpolate_angle(from: f64, to: f64, t: f64) -> f64 {
    normalize_angle(from + t * angle_diff(to, from))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize_range_and_boundaries() {
        assert_eq!(normalize_angle(PI), PI);
        assert_eq!(normalize_angle(-PI), PI);
        assert_eq!(normalize_angle(0.0), 0.0);
        assert!((normalize_angle(TWO_PI + 0.5) - 0.5).abs() < 1e-12);
        assert!((normalize_angle(-TWO_PI - 0.5) + 0.5).abs() < 1e-12);
        let mut a = -100.0;
        while a < 100.0 {
            let n = normalize_angle(a);
            assert!(n > -PI && n <= PI, "{a} -> {n}");
            assert!((n.cos() - a.cos()).abs() < 1e-12 && (n.sin() - a.sin()).abs() < 1e-12);
            a += 0.377;
        }
    }

    #[test]
    fn diff_and_interpolation_across_the_wrap() {
        assert!((angle_diff(PI - 0.1, -PI + 0.1) + 0.2).abs() < 1e-12);
        assert!((interpolate_angle(PI - 0.2, -PI + 0.2, 0.5) - PI).abs() < 1e-12);
    }
}
