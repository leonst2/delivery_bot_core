//! Euclidean clustering on a grid index. Two points belong to the same cluster when a chain of points connects them
//! with small enough gaps. The allowed gap grows with the distance from the sensor, because neighbouring LiDAR rays
//! spread apart (at a grazing angle, points on a cyclist 11 m away are 0.6 m apart). The result depends only on the
//! set of points, not on their order.

use std::collections::{BTreeMap, HashMap};

#[derive(Clone, Debug, PartialEq)]
pub struct Cluster {
    pub points: Vec<[f64; 2]>,
    pub centroid: [f64; 2],
    /// Largest distance of a point from the centroid.
    pub radius: f64,
    /// Diagonal of the axis-aligned bounding box (the length of a straight wall at any angle).
    pub extent: f64,
}

/// Total order on points, so clustering and every sum over points run in a fixed order.
pub fn sort_points(points: &mut [[f64; 2]]) {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

/// Allowed gaps: `base` near the sensor, `per_meter` times the distance of the farther point further out.
#[derive(Clone, Copy, Debug)]
pub struct Tolerance {
    pub sensor: [f64; 2],
    pub base: f64,
    pub per_meter: f64,
}

impl Tolerance {
    /// The same gap everywhere.
    pub fn fixed(base: f64) -> Self {
        Self {
            sensor: [0.0, 0.0],
            base,
            per_meter: 0.0,
        }
    }

    fn at(&self, range: f64) -> f64 {
        self.base.max(self.per_meter * range)
    }
}

/// Clusters of `points` in the order of their first point. Sorts the points first.
pub fn euclidean_clusters(points: &[[f64; 2]], tolerance: Tolerance) -> Vec<Cluster> {
    let mut points = points.to_vec();
    sort_points(&mut points);
    let ranges: Vec<f64> = points
        .iter()
        .map(|p| (p[0] - tolerance.sensor[0]).hypot(p[1] - tolerance.sensor[1]))
        .collect();
    // Grid cells as large as the largest allowed gap, so every neighbour lies in the 3 x 3 cells around a point.
    let cell_size = tolerance.at(ranges.iter().copied().fold(0.0, f64::max));
    let cell = |p: &[f64; 2]| {
        (
            (p[0] / cell_size).floor() as i64,
            (p[1] / cell_size).floor() as i64,
        )
    };
    // HashMap only for lookups: its iteration order never reaches the result.
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    let mut parent: Vec<usize> = (0..points.len()).collect();
    for (i, p) in points.iter().enumerate() {
        let (cx, cy) = cell(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(neighbours) = grid.get(&(cx + dx, cy + dy)) else {
                    continue;
                };
                for &j in neighbours {
                    let q = points[j];
                    let gap = tolerance.at(ranges[i].max(ranges[j]));
                    if (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) <= gap * gap {
                        // The smaller index becomes the root, so roots are the first point of each cluster.
                        let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                        let (root, child) = (ri.min(rj), ri.max(rj));
                        parent[child] = root;
                    }
                }
            }
        }
        grid.entry((cx, cy)).or_default().push(i);
    }
    let mut groups: BTreeMap<usize, Vec<[f64; 2]>> = BTreeMap::new();
    for (i, point) in points.iter().enumerate() {
        let root = find(&mut parent, i);
        groups.entry(root).or_default().push(*point);
    }
    groups.into_values().map(cluster_stats).collect()
}

fn cluster_stats(points: Vec<[f64; 2]>) -> Cluster {
    let n = points.len() as f64;
    let centroid = [
        points.iter().map(|p| p[0]).sum::<f64>() / n,
        points.iter().map(|p| p[1]).sum::<f64>() / n,
    ];
    let radius = points
        .iter()
        .map(|p| (p[0] - centroid[0]).hypot(p[1] - centroid[1]))
        .fold(0.0, f64::max);
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &points {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    Cluster {
        extent: (hi[0] - lo[0]).hypot(hi[1] - lo[1]),
        points,
        centroid,
        radius,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn separates_groups_by_the_tolerance() {
        let points = [
            [0.0, 0.0],
            [0.3, 0.0],
            [0.6, 0.0],
            [5.0, 5.0],
            [5.0, 5.2],
            [1.0, 0.0],
        ];
        let clusters = euclidean_clusters(&points, Tolerance::fixed(0.35));
        assert_eq!(clusters.len(), 3);
        assert_eq!(clusters[0].points.len(), 3);
        assert!((clusters[0].centroid[0] - 0.3).abs() < 1e-12);
        assert!((clusters[0].extent - 0.6).abs() < 1e-12);
        assert_eq!(clusters[1].points, vec![[1.0, 0.0]]);
        assert!((clusters[2].centroid[1] - 5.1).abs() < 1e-12);
    }

    #[test]
    fn chains_across_grid_cells() {
        // A diagonal line of points 0.3 apart: one cluster although it spans many cells.
        let points: Vec<[f64; 2]> = (0..20).map(|k| [k as f64 * 0.2, k as f64 * 0.2]).collect();
        let clusters = euclidean_clusters(&points, Tolerance::fixed(0.3));
        assert_eq!(clusters.len(), 1);
        assert!((clusters[0].extent - (19.0 * 0.2) * 2f64.sqrt()).abs() < 1e-9);
    }

    #[test]
    fn result_does_not_depend_on_the_input_order() {
        let mut points: Vec<[f64; 2]> = (0..60)
            .map(|k| [((k * 37) % 23) as f64 * 0.21, ((k * 11) % 7) as f64 * 0.4])
            .collect();
        let a = euclidean_clusters(&points, Tolerance::fixed(0.35));
        points.reverse();
        points.rotate_left(17);
        assert_eq!(a, euclidean_clusters(&points, Tolerance::fixed(0.35)));
    }

    #[test]
    fn the_gap_grows_with_range() {
        let tolerance = Tolerance {
            sensor: [0.0, 0.0],
            base: 0.35,
            per_meter: 0.05,
        };
        // 0.5 m apart: separate at 2 m, one cluster at 11 m (allowed gap 0.55 m).
        assert_eq!(
            euclidean_clusters(&[[2.0, 0.0], [2.0, 0.5]], tolerance).len(),
            2
        );
        assert_eq!(
            euclidean_clusters(&[[11.0, 0.0], [11.0, 0.5]], tolerance).len(),
            1
        );
    }

    #[test]
    fn handles_no_points() {
        assert!(euclidean_clusters(&[], Tolerance::fixed(0.35)).is_empty());
    }
}
