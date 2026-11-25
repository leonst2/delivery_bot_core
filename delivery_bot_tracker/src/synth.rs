//! Synthetic test world (the card's mock): box-shaped objects seen by a simulated 2D LiDAR like the Unity one
//! (360°, 1° resolution, 0.1-12 m, 10 Hz), with seeded noise. Cyclists are 1.8 x 0.6 m boxes like `CyclistAgent`.

use std::f64::consts::PI;

use crate::geometry::Pose2;
use crate::static_map::StaticMap;

pub const CYCLIST_LENGTH: f64 = 1.8;
pub const CYCLIST_WIDTH: f64 = 0.6;
pub const LIDAR_HEIGHT: f64 = 0.3;

/// splitmix64: small, seedable, the same on every platform.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn gaussian(&mut self) -> f64 {
        let u1 = 1.0 - self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    pub fn shuffle<T>(&mut self, items: &mut [T]) {
        for i in (1..items.len()).rev() {
            let j = (self.next_u64() % (i as u64 + 1)) as usize;
            items.swap(i, j);
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Rect {
    pub center: [f64; 2],
    pub yaw: f64,
    pub length: f64,
    pub width: f64,
}

impl Rect {
    pub fn new(center: [f64; 2], yaw: f64, length: f64, width: f64) -> Self {
        Self {
            center,
            yaw,
            length,
            width,
        }
    }

    pub fn cyclist(center: [f64; 2], heading: f64) -> Self {
        Self::new(center, heading, CYCLIST_LENGTH, CYCLIST_WIDTH)
    }

    fn pose(&self) -> Pose2 {
        Pose2::new(self.center[0], self.center[1], self.yaw)
    }

    fn corners(&self) -> [[f64; 2]; 4] {
        let (l, w) = (self.length / 2.0, self.width / 2.0);
        let pose = self.pose();
        [[l, w], [-l, w], [-l, -w], [l, -w]].map(|c| pose.transform(c))
    }

    pub fn contains(&self, p: [f64; 2], grow: f64) -> bool {
        let local = self.pose().inverse().transform(p);
        local[0].abs() <= self.length / 2.0 + grow && local[1].abs() <= self.width / 2.0 + grow
    }

    /// Distance along the ray to the nearest edge, if it hits.
    fn ray_hit(&self, origin: [f64; 2], dir: [f64; 2]) -> Option<f64> {
        let c = self.corners();
        let mut best: Option<f64> = None;
        for k in 0..4 {
            let (a, b) = (c[k], c[(k + 1) % 4]);
            let e = [b[0] - a[0], b[1] - a[1]];
            let denom = dir[0] * e[1] - dir[1] * e[0];
            if denom.abs() < 1e-12 {
                continue;
            }
            let w = [a[0] - origin[0], a[1] - origin[1]];
            let t = (w[0] * e[1] - w[1] * e[0]) / denom;
            let s = (w[0] * dir[1] - w[1] * dir[0]) / denom;
            if t > 0.0 && (0.0..=1.0).contains(&s) && best.is_none_or(|b| t < b) {
                best = Some(t);
            }
        }
        best
    }
}

/// One LiDAR scan in `base_link` (points at the LiDAR height), from the true robot pose.
pub fn scan(robot: &Pose2, objects: &[Rect], noise_std: f64, rng: &mut Rng) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for k in 0..360 {
        let a = (k as f64).to_radians();
        let world = robot.yaw + a;
        let dir = [world.cos(), world.sin()];
        let hit = objects
            .iter()
            .filter_map(|o| o.ray_hit([robot.x, robot.y], dir))
            .fold(f64::INFINITY, f64::min);
        let noise = rng.gaussian() * noise_std;
        if (0.1..=12.0).contains(&hit) {
            let r = hit + noise;
            points.push([r * a.cos(), r * a.sin(), LIDAR_HEIGHT]);
        }
    }
    points
}

/// The exporter's view of the static world: cells whose centre lies within half a cell of an object are occupied.
pub fn rasterize(
    objects: &[Rect],
    lower_left: [f64; 2],
    size: [f64; 2],
    resolution: f64,
    margin: f64,
) -> StaticMap {
    let (w, h) = (
        (size[0] / resolution) as usize,
        (size[1] / resolution) as usize,
    );
    let mut data = vec![0i8; w * h];
    for row in 0..h {
        for col in 0..w {
            let p = [
                lower_left[0] + (col as f64 + 0.5) * resolution,
                lower_left[1] + (row as f64 + 0.5) * resolution,
            ];
            if objects.iter().any(|o| o.contains(p, resolution / 2.0)) {
                data[row * w + col] = 100;
            }
        }
    }
    StaticMap::from_occupancy(
        w,
        h,
        resolution,
        Pose2::new(lower_left[0], lower_left[1], 0.0),
        &data,
        margin,
    )
    .unwrap()
}

#[derive(Clone, Copy, Debug)]
pub enum Path {
    Line {
        start: [f64; 2],
        heading: f64,
        speed: f64,
    },
    /// Positive speed = counter-clockwise.
    Circle {
        center: [f64; 2],
        radius: f64,
        phase: f64,
        speed: f64,
    },
}

impl Path {
    /// (position, heading, velocity) at time `t`.
    pub fn state(&self, t: f64) -> ([f64; 2], f64, [f64; 2]) {
        match *self {
            Path::Line {
                start,
                heading,
                speed,
            } => {
                let (s, c) = heading.sin_cos();
                (
                    [start[0] + c * speed * t, start[1] + s * speed * t],
                    heading,
                    [c * speed, s * speed],
                )
            }
            Path::Circle {
                center,
                radius,
                phase,
                speed,
            } => {
                let angle = phase + speed * t / radius;
                let (s, c) = angle.sin_cos();
                let v = [-s * speed, c * speed];
                (
                    [center[0] + radius * c, center[1] + radius * s],
                    v[1].atan2(v[0]),
                    v,
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_scan_sees_the_near_face_of_a_box() {
        let wall = Rect::new([5.0, 0.0], 0.0, 1.0, 4.0);
        let points = scan(&Pose2::IDENTITY, &[wall], 0.0, &mut Rng::new(1));
        let ahead = points.iter().find(|p| p[1].abs() < 1e-9).unwrap();
        assert!((ahead[0] - 4.5).abs() < 1e-9);
        assert!(points.len() > 10 && points.len() < 60);
    }

    #[test]
    fn rng_is_reproducible_and_roughly_normal() {
        let (mut a, mut b) = (Rng::new(7), Rng::new(7));
        let xs: Vec<f64> = (0..10_000).map(|_| a.gaussian()).collect();
        assert!(xs
            .iter()
            .zip((0..10_000).map(|_| b.gaussian()))
            .all(|(x, y)| *x == y));
        let mean = xs.iter().sum::<f64>() / xs.len() as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / xs.len() as f64;
        assert!(mean.abs() < 0.05 && (var - 1.0).abs() < 0.05);
    }

    #[test]
    fn circle_paths_move_tangentially() {
        let path = Path::Circle {
            center: [0.0, 0.0],
            radius: 2.0,
            phase: 0.0,
            speed: 3.0,
        };
        let (p, heading, v) = path.state(0.0);
        assert_eq!(p, [2.0, 0.0]);
        assert!((heading - PI / 2.0).abs() < 1e-12 && (v[1] - 3.0).abs() < 1e-12);
    }
}
