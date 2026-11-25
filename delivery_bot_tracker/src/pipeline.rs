//! Turns a point cloud in `base_link` into detections in `map` and feeds them to the tracker.

use crate::cluster::{euclidean_clusters, Tolerance};
use crate::geometry::Pose2;
use crate::params::Params;
use crate::static_map::StaticMap;
use crate::tracker::{Detection, TrackOutput, Tracker};

/// Input topics. Only the LiDAR drives perception cycles.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Source {
    /// /perception/points: scan points (#9).
    Lidar,
    /// /stereo/points: dense 3D stereo points (#10), clustered after a height filter.
    Stereo,
    /// /perception/side_points: one ground point per detected object (#33).
    Side,
}

pub struct Pipeline {
    tracker: Tracker,
    map: Option<StaticMap>,
}

impl Pipeline {
    pub fn new(params: Params) -> Self {
        Self {
            tracker: Tracker::new(params),
            map: None,
        }
    }

    pub fn params(&self) -> &Params {
        self.tracker.params()
    }

    pub fn set_map(&mut self, map: StaticMap) {
        self.map = Some(map);
    }

    pub fn has_map(&self) -> bool {
        self.map.is_some()
    }

    pub fn reset(&mut self) {
        self.tracker.reset();
    }

    pub fn last_stamp(&self) -> Option<i64> {
        self.tracker.last_stamp()
    }

    /// Detections in `map` from points in `base_link`, with `robot` = T_map_base_link at the cloud's stamp.
    pub fn detections(&self, source: Source, robot: &Pose2, points: &[[f64; 3]]) -> Vec<Detection> {
        let p = self.params();
        let candidates: Vec<[f64; 2]> = points
            .iter()
            .filter(|q| q[0].hypot(q[1]) <= p.max_range)
            .filter(|q| source != Source::Stereo || (q[2] >= p.min_z && q[2] <= p.max_z))
            .map(|q| robot.transform([q[0], q[1]]))
            .filter(|q| !self.map.as_ref().is_some_and(|m| m.is_static(*q)))
            .collect();
        if source == Source::Side {
            return candidates.into_iter().map(Detection::Point).collect();
        }
        let tolerance = Tolerance {
            sensor: [robot.x, robot.y],
            base: p.cluster_tolerance,
            per_meter: p.cluster_tolerance_per_meter,
        };
        euclidean_clusters(&candidates, tolerance)
            .into_iter()
            .filter(|c| c.points.len() >= p.min_cluster_points && c.extent <= p.max_cluster_extent)
            .map(Detection::Cluster)
            .collect()
    }

    /// Processes one cloud. Returns false when it was stale and ignored.
    pub fn process(
        &mut self,
        source: Source,
        stamp: i64,
        robot: &Pose2,
        points: &[[f64; 3]],
    ) -> bool {
        let detections = self.detections(source, robot, points);
        self.tracker.step(
            stamp,
            [robot.x, robot.y],
            &detections,
            source == Source::Lidar,
        )
    }

    pub fn outputs(&self) -> Vec<TrackOutput> {
        self.tracker.outputs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filters_range_height_and_static_points() {
        let mut pipeline = Pipeline::new(Params::default());
        // A wall cell at map x in [5, 5.05) along every y in the map.
        let (w, h) = (200, 200);
        let mut data = vec![0i8; w * h];
        for row in 0..h {
            data[row * w + 150] = 100;
        }
        pipeline.set_map(
            StaticMap::from_occupancy(w, h, 0.05, Pose2::new(-2.5, -5.0, 0.0), &data, 0.1).unwrap(),
        );
        let robot = Pose2::new(0.0, 0.0, 0.0);
        let object: Vec<[f64; 3]> = (0..4).map(|k| [2.0, 0.1 * k as f64, 0.5]).collect();
        let wall: Vec<[f64; 3]> = (0..4).map(|k| [5.02, 0.1 * k as f64, 0.5]).collect();
        let far: Vec<[f64; 3]> = (0..4).map(|k| [20.0, 0.1 * k as f64, 0.5]).collect();
        let points: Vec<[f64; 3]> = [object.clone(), wall, far].concat();
        let lidar = pipeline.detections(Source::Lidar, &robot, &points);
        assert_eq!(lidar.len(), 1);
        let ground: Vec<[f64; 3]> = object.iter().map(|q| [q[0], q[1], 0.02]).collect();
        assert!(pipeline
            .detections(Source::Stereo, &robot, &ground)
            .is_empty());
        assert_eq!(pipeline.detections(Source::Side, &robot, &object).len(), 4);
        // Rotating the robot by 180° puts the same points behind it, at map x = -2.
        let behind = pipeline.detections(
            Source::Lidar,
            &Pose2::new(0.0, 0.0, std::f64::consts::PI),
            &object,
        );
        match &behind[0] {
            Detection::Cluster(c) => assert!((c.centroid[0] + 2.0).abs() < 1e-9),
            Detection::Point(_) => panic!("expected a cluster"),
        }
    }
}
