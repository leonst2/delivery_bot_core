//! The static map as a lookup "is this point explained by the map?". Points within `margin` of an occupied cell are
//! static. Free, unknown and out-of-map cells are not, so objects in unmapped space are still tracked.

use crate::geometry::Pose2;

/// `nav_msgs/OccupancyGrid` values from this threshold up count as occupied (map_server writes 100, free is 0).
pub const OCCUPIED_THRESHOLD: i8 = 50;

pub struct StaticMap {
    width: usize,
    height: usize,
    resolution: f64,
    /// T_map_grid: the pose of the lower-left corner of cell (0, 0).
    origin_inverse: Pose2,
    /// Occupied cells dilated by the margin, row-major with rows along +y (like OccupancyGrid).
    mask: Vec<bool>,
}

impl StaticMap {
    pub fn from_occupancy(
        width: usize,
        height: usize,
        resolution: f64,
        origin: Pose2,
        data: &[i8],
        margin: f64,
    ) -> Result<Self, String> {
        if resolution.is_nan() || resolution <= 0.0 {
            return Err(format!("map resolution must be positive, got {resolution}"));
        }
        if data.len() != width * height {
            return Err(format!(
                "map data has {} cells, expected {width}x{height}",
                data.len()
            ));
        }
        let reach = (margin.max(0.0) / resolution + 1e-9).floor() as i64;
        let disk: Vec<(i64, i64)> = (-reach..=reach)
            .flat_map(|dy| (-reach..=reach).map(move |dx| (dx, dy)))
            .filter(|&(dx, dy)| ((dx * dx + dy * dy) as f64).sqrt() * resolution <= margin + 1e-9)
            .collect();
        let mut mask = vec![false; data.len()];
        let (w, h) = (width as i64, height as i64);
        for (index, &value) in data.iter().enumerate() {
            if value < OCCUPIED_THRESHOLD {
                continue;
            }
            let (cx, cy) = ((index % width) as i64, (index / width) as i64);
            for &(dx, dy) in &disk {
                let (x, y) = (cx + dx, cy + dy);
                if x >= 0 && y >= 0 && x < w && y < h {
                    mask[(y * w + x) as usize] = true;
                }
            }
        }
        Ok(Self {
            width,
            height,
            resolution,
            origin_inverse: origin.inverse(),
            mask,
        })
    }

    /// Grid cell (column, row) of a point in `map`, if it lies on the map.
    pub fn cell(&self, p: [f64; 2]) -> Option<(usize, usize)> {
        let local = self.origin_inverse.transform(p);
        let (col, row) = (
            (local[0] / self.resolution).floor(),
            (local[1] / self.resolution).floor(),
        );
        if col < 0.0 || row < 0.0 || col >= self.width as f64 || row >= self.height as f64 {
            return None;
        }
        Some((col as usize, row as usize))
    }

    pub fn is_static(&self, p: [f64; 2]) -> bool {
        self.cell(p)
            .is_some_and(|(col, row)| self.mask[row * self.width + col])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 10 x 10 m at 0.05 m, origin (-5, -5): one occupied block, a "building", with its lower-left corner at (1, 2).
    fn map(margin: f64) -> StaticMap {
        let (w, h, res) = (200, 200, 0.05);
        let mut data = vec![0i8; w * h];
        for row in 140..160 {
            for col in 120..140 {
                data[row * w + col] = 100;
            }
        }
        data[0] = -1;
        StaticMap::from_occupancy(w, h, res, Pose2::new(-5.0, -5.0, 0.0), &data, margin).unwrap()
    }

    #[test]
    fn a_building_corner_lands_on_the_expected_cell() {
        let m = map(0.0);
        // Corner at map (1, 2) → cell (1 + 5) / 0.05 = 120, (2 + 5) / 0.05 = 140.
        assert_eq!(m.cell([1.001, 2.001]), Some((120, 140)));
        assert!(m.is_static([1.001, 2.001]));
        assert!(!m.is_static([0.99, 2.001]));
        assert!(!m.is_static([-4.99, -4.99])); // unknown
        assert!(!m.is_static([50.0, 0.0])); // off the map
    }

    #[test]
    fn the_margin_grows_obstacles() {
        let m = map(0.15);
        assert!(m.is_static([0.87, 2.5]));
        assert!(!m.is_static([0.80, 2.5]));
    }

    #[test]
    fn a_rotated_origin_is_honoured() {
        let data = vec![100i8; 1];
        let m = StaticMap::from_occupancy(
            1,
            1,
            1.0,
            Pose2::new(2.0, 0.0, std::f64::consts::FRAC_PI_2),
            &data,
            0.0,
        )
        .unwrap();
        // The cell spans x in [1, 2], y in [0, 1] of the map after a 90° rotation about (2, 0).
        assert!(m.is_static([1.5, 0.5]));
        assert!(!m.is_static([2.5, 0.5]));
    }

    #[test]
    fn rejects_inconsistent_grids() {
        assert!(StaticMap::from_occupancy(2, 2, 0.05, Pose2::IDENTITY, &[0; 3], 0.0).is_err());
        assert!(StaticMap::from_occupancy(1, 1, 0.0, Pose2::IDENTITY, &[0; 1], 0.0).is_err());
    }
}
