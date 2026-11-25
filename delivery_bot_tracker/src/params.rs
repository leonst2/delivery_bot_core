//! Tracker parameters. The defaults are the ROS parameter defaults of the node; the README documents each one.

#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    /// s, how far ahead `predicted_position` looks (one grid step).
    pub prediction_horizon: f64,
    /// m, points farther from the robot are ignored (LiDAR range).
    pub max_range: f64,
    /// m, largest gap between neighbouring points of one object near the robot.
    pub cluster_tolerance: f64,
    /// m per m of range: farther out the allowed gap is this times the range (LiDAR rays spread apart).
    pub cluster_tolerance_per_meter: f64,
    pub min_cluster_points: usize,
    /// m, clusters longer than this are walls or other large structure, not objects to track.
    pub max_cluster_extent: f64,
    /// m, points this close to an occupied map cell are static.
    pub static_margin: f64,
    /// m, while an object's heading is unknown, its centre is taken this far behind its nearest surface, on the
    /// bearing of its centroid (the LiDAR sees only the near surface).
    pub surface_offset: f64,
    /// m, footprint of a tracked object, used to find its centre from the visible side once its heading is known.
    /// Defaults: a cyclist (bike plus rider).
    pub object_length: f64,
    pub object_width: f64,
    /// The footprint is fitted only to moving tracks at least this fast (m/s) whose velocity standard deviation is at
    /// most `heading_max_velocity_std` (m/s). A wrong heading would pull the measurements its own way.
    pub heading_min_speed: f64,
    pub heading_max_velocity_std: f64,
    /// m, reported radius: at least this (half the cyclist footprint diagonal).
    pub default_radius: f64,
    /// m/s^2, white-acceleration noise of the two constant-velocity models of the IMM filter: riding straight and
    /// turning. The measurements decide which one dominates.
    pub accel_noise_smooth: f64,
    pub accel_noise_turning: f64,
    /// Probability of switching between the two models from one update to the next.
    pub model_switch_probability: f64,
    /// m, standard deviation of a cluster position measurement (includes the scan-to-scan error of the robot pose).
    pub measurement_noise: f64,
    /// m, standard deviation of a single-point detection (/perception/side_points).
    pub point_measurement_noise: f64,
    /// m/s, initial velocity uncertainty of a new track.
    pub initial_speed_std: f64,
    /// Gate on the squared Mahalanobis distance (13.8 = 99.9 % for 2 DOF).
    pub gate_chi2: f64,
    /// m, a measurement this close to a track's prediction may always belong to it. The Mahalanobis gate gets tight
    /// once a track has settled, but a partly occluded object can be measured up to half its length off.
    pub min_gate_distance: f64,
    /// m, a measurement farther than this from a track's prediction never belongs to it.
    pub max_gate_distance: f64,
    /// m, no new track from a detection with a point closer than this to an existing track (pieces of an object that
    /// is partly occluded). Defaults to the cyclist radius.
    pub birth_suppression: f64,
    /// A track is confirmed after `confirm_hits` hits within its first `confirm_window` cycles, else dropped.
    pub confirm_hits: u32,
    pub confirm_window: u32,
    /// A confirmed track is dropped after this many cycles without a hit.
    pub max_misses: u32,
    /// m, a track is dropped when the standard deviation of its position grows beyond this.
    pub max_position_std: f64,
    /// m/s, slower tracks are reported with zero velocity.
    pub min_reported_speed: f64,
    /// m, a track is reported as moving only after it has moved this far from where it started, measured over at
    /// least three updates.
    pub min_displacement: f64,
    /// m, height band of /stereo/points (in base_link) that can belong to an obstacle.
    pub min_z: f64,
    pub max_z: f64,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            prediction_horizon: 1.5,
            max_range: 12.0,
            cluster_tolerance: 0.35,
            cluster_tolerance_per_meter: 0.05,
            min_cluster_points: 3,
            max_cluster_extent: 3.0,
            static_margin: 0.15,
            surface_offset: 0.4,
            object_length: 1.8,
            object_width: 0.6,
            heading_min_speed: 1.0,
            heading_max_velocity_std: 0.8,
            default_radius: 0.95,
            accel_noise_smooth: 1.0,
            accel_noise_turning: 4.0,
            model_switch_probability: 0.05,
            measurement_noise: 0.15,
            point_measurement_noise: 0.3,
            initial_speed_std: 4.0,
            gate_chi2: 13.8,
            min_gate_distance: 1.0,
            max_gate_distance: 2.5,
            birth_suppression: 0.95,
            confirm_hits: 2,
            confirm_window: 3,
            max_misses: 10,
            max_position_std: 2.0,
            min_reported_speed: 0.3,
            min_displacement: 0.75,
            min_z: 0.15,
            max_z: 2.0,
        }
    }
}

impl Params {
    pub fn validate(&self) -> Result<(), String> {
        let positive = [
            ("max_range", self.max_range),
            ("cluster_tolerance", self.cluster_tolerance),
            ("object_length", self.object_length),
            ("object_width", self.object_width),
            ("accel_noise_smooth", self.accel_noise_smooth),
            ("accel_noise_turning", self.accel_noise_turning),
            ("measurement_noise", self.measurement_noise),
            ("point_measurement_noise", self.point_measurement_noise),
            ("initial_speed_std", self.initial_speed_std),
            ("gate_chi2", self.gate_chi2),
            ("max_gate_distance", self.max_gate_distance),
            ("max_position_std", self.max_position_std),
        ];
        for (name, value) in positive {
            if value.is_nan() || value <= 0.0 {
                return Err(format!("{name} must be positive, got {value}"));
            }
        }
        if !(0.0..=0.5).contains(&self.model_switch_probability) {
            return Err("model_switch_probability must lie in [0, 0.5]".into());
        }
        if self.min_gate_distance > self.max_gate_distance {
            return Err("min_gate_distance must not exceed max_gate_distance".into());
        }
        if self.cluster_tolerance_per_meter < 0.0 {
            return Err("cluster_tolerance_per_meter must not be negative".into());
        }
        if self.prediction_horizon < 0.0 {
            return Err("prediction_horizon must not be negative".into());
        }
        if self.confirm_hits == 0
            || self.confirm_window < self.confirm_hits
            || self.confirm_window > 32
        {
            return Err("need 1 <= confirm_hits <= confirm_window <= 32".into());
        }
        if self.min_z > self.max_z {
            return Err("min_z must not exceed max_z".into());
        }
        Ok(())
    }
}
