//! Dynamic obstacle detection and tracking with prediction (delivery_bot_core#11).
//!
//! The rclrs node around the `delivery_bot_tracker` library: it turns messages into library calls and tracks into
//! `/tracked_obstacles`. `/perception/points` drives the perception cycles. `/stereo/points` and
//! `/perception/side_points` are optional; they are buffered and processed in stamp order before the next LiDAR
//! cloud, so the order in which callbacks run never changes the result.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::delivery_bot_interfaces::msg::{Heartbeat, TrackedObstacle, TrackedObstacleArray};
use ros_env::nav_msgs::msg::OccupancyGrid;
use ros_env::sensor_msgs::msg::PointCloud2;
use ros_env::tf2_msgs::msg::TFMessage;

use delivery_bot_tracker::cloud::{read_xyz, CloudView, Field};
use delivery_bot_tracker::static_map::StaticMap;
use delivery_bot_tracker::tf::TfBuffer;
use delivery_bot_tracker::{Params, Pipeline, Pose2, Source, TrackOutput};

/// How much TF history to keep, and how far a lookup may lie past the newest transform (localization publishes
/// map -> odom at the scan rate, so it can lag behind a cloud).
const TF_HISTORY: Duration = Duration::from_secs(2);
const TF_TOLERANCE: Duration = Duration::from_millis(200);
/// Buffered optional clouds more than this away from the LiDAR cloud's stamp are dropped.
const MAX_PENDING_AGE_NS: i64 = 1_000_000_000;
const MAX_PENDING: usize = 64;
/// A dropped message turns the heartbeat to WARN for this long.
const WARN_HOLD: Duration = Duration::from_secs(2);

fn stamp_ns(stamp: &Time) -> i64 {
    stamp.sec as i64 * 1_000_000_000 + stamp.nanosec as i64
}

fn to_stamp(ns: i64) -> Time {
    Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

fn points_of(msg: &PointCloud2) -> Result<Vec<[f64; 3]>, String> {
    read_xyz(&CloudView {
        fields: msg
            .fields
            .iter()
            .map(|f| Field {
                name: f.name.clone(),
                offset: f.offset as usize,
                datatype: f.datatype,
            })
            .collect(),
        data: &msg.data,
        width: msg.width as usize,
        height: msg.height as usize,
        point_step: msg.point_step as usize,
        row_step: msg.row_step as usize,
        is_bigendian: msg.is_bigendian,
    })
}

fn static_map_of(msg: &OccupancyGrid, margin: f64) -> Result<StaticMap, String> {
    let origin = &msg.info.origin;
    let q = &origin.orientation;
    StaticMap::from_occupancy(
        msg.info.width as usize,
        msg.info.height as usize,
        msg.info.resolution as f64,
        Pose2::from_translation_quaternion(
            origin.position.x,
            origin.position.y,
            q.x,
            q.y,
            q.z,
            q.w,
        ),
        &msg.data,
        margin,
    )
}

fn obstacle_msg(track: &TrackOutput) -> TrackedObstacle {
    let mut msg = TrackedObstacle {
        id: track.id,
        radius: track.radius as f32,
        position_covariance: track.position_covariance.map(|c| c as f32),
        prediction_horizon: track.prediction_horizon as f32,
        age: track.age as f32,
        ..Default::default()
    };
    msg.position.x = track.position[0];
    msg.position.y = track.position[1];
    msg.velocity.x = track.velocity[0];
    msg.velocity.y = track.velocity[1];
    msg.predicted_position.x = track.predicted_position[0];
    msg.predicted_position.y = track.predicted_position[1];
    msg
}

fn insert_transforms(tf: &mut TfBuffer, msg: &TFMessage, is_static: bool) {
    for t in &msg.transforms {
        let (p, q) = (&t.transform.translation, &t.transform.rotation);
        tf.insert(
            &t.header.frame_id,
            &t.child_frame_id,
            stamp_ns(&t.header.stamp),
            Pose2::from_translation_quaternion(p.x, p.y, q.x, q.y, q.z, q.w),
            is_static,
        );
    }
}

/// An optional-source cloud waiting for the next LiDAR cloud, already paired with its frame's pose at its stamp.
struct Pending {
    stamp: i64,
    source: Source,
    pose: Pose2,
    points: Vec<[f64; 3]>,
}

struct State {
    pipeline: Pipeline,
    tf: TfBuffer,
    map_frame: String,
    pending: Vec<Pending>,
    cycles: u64,
    tracks: usize,
    last_problem: Option<(Instant, String)>,
}

impl State {
    fn new(params: Params, map_frame: &str) -> Self {
        Self {
            pipeline: Pipeline::new(params),
            tf: TfBuffer::new(TF_HISTORY.as_nanos() as i64, TF_TOLERANCE.as_nanos() as i64),
            map_frame: map_frame.to_owned(),
            pending: Vec::new(),
            cycles: 0,
            tracks: 0,
            last_problem: None,
        }
    }

    fn problem(&mut self, what: String) {
        self.last_problem = Some((Instant::now(), what));
    }

    /// The cloud's stamp, the pose of its frame in `map` at that stamp, and its points.
    fn prepare(&mut self, msg: &PointCloud2, topic: &str) -> Option<(i64, Pose2, Vec<[f64; 3]>)> {
        let stamp = stamp_ns(&msg.header.stamp);
        let pose = match self.tf.lookup(&self.map_frame, &msg.header.frame_id, stamp) {
            Ok(pose) => pose,
            Err(err) => {
                self.problem(format!("dropped {topic}: {err}"));
                return None;
            }
        };
        match points_of(msg) {
            Ok(points) => Some((stamp, pose, points)),
            Err(err) => {
                self.problem(format!("dropped {topic}: {err}"));
                None
            }
        }
    }

    fn on_optional(&mut self, msg: &PointCloud2, source: Source, topic: &str) {
        let Some((stamp, pose, points)) = self.prepare(msg, topic) else {
            return;
        };
        self.pending.push(Pending {
            stamp,
            source,
            pose,
            points,
        });
        if self.pending.len() > MAX_PENDING {
            self.pending.sort_by_key(|p| (p.stamp, p.source));
            self.pending.remove(0);
        }
    }

    /// Processes the buffered optional clouds up to the LiDAR stamp, then the LiDAR cloud. Returns the stamp and
    /// the tracks to publish.
    fn on_lidar(&mut self, msg: &PointCloud2) -> Option<(i64, Vec<TrackOutput>)> {
        let (stamp, pose, points) = self.prepare(msg, "/perception/points")?;
        // Far in the future means from before a clock reset (/sim/reset); far in the past is useless.
        self.pending
            .retain(|p| (p.stamp - stamp).abs() <= MAX_PENDING_AGE_NS);
        self.pending.sort_by_key(|p| (p.stamp, p.source));
        let due = self.pending.partition_point(|p| p.stamp <= stamp);
        for p in self.pending.drain(..due).collect::<Vec<_>>() {
            self.pipeline.process(p.source, p.stamp, &p.pose, &p.points);
        }
        if !self.pipeline.process(Source::Lidar, stamp, &pose, &points) {
            self.problem("dropped /perception/points: older than the last update".into());
            return None;
        }
        self.cycles += 1;
        let outputs = self.pipeline.outputs();
        self.tracks = outputs.len();
        Some((stamp, outputs))
    }

    fn heartbeat(&self) -> (u8, String) {
        if let Some((when, what)) = &self.last_problem {
            if when.elapsed() < WARN_HOLD {
                return (Heartbeat::WARN, what.clone());
            }
        }
        // No input is not a fault: the stack runs without sensors in CI and before the simulation starts.
        let detail = if self.cycles == 0 {
            "waiting for /perception/points".to_owned()
        } else if !self.pipeline.has_map() {
            format!(
                "{} tracks, no /map yet (nothing is filtered as static)",
                self.tracks
            )
        } else {
            format!("{} tracks", self.tracks)
        };
        (Heartbeat::OK, detail)
    }
}

fn lock(state: &Mutex<State>) -> MutexGuard<'_, State> {
    state
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn declare_params(node: &Node) -> Result<Params> {
    let d = Params::default();
    let f = |name: &str, default: f64| -> Result<f64> {
        Ok(node
            .declare_parameter(name)
            .default(default)
            .mandatory()?
            .get())
    };
    let u = |name: &str, default: u32| -> Result<u32> {
        let value: i64 = node
            .declare_parameter(name)
            .default(default as i64)
            .mandatory()?
            .get();
        u32::try_from(value)
            .map_err(|_| anyhow!("parameter {name} must be a non-negative integer, got {value}"))
    };
    let params = Params {
        prediction_horizon: f("prediction_horizon", d.prediction_horizon)?,
        max_range: f("max_range", d.max_range)?,
        cluster_tolerance: f("cluster_tolerance", d.cluster_tolerance)?,
        cluster_tolerance_per_meter: f(
            "cluster_tolerance_per_meter",
            d.cluster_tolerance_per_meter,
        )?,
        min_cluster_points: u("min_cluster_points", d.min_cluster_points as u32)? as usize,
        max_cluster_extent: f("max_cluster_extent", d.max_cluster_extent)?,
        static_margin: f("static_margin", d.static_margin)?,
        surface_offset: f("surface_offset", d.surface_offset)?,
        object_length: f("object_length", d.object_length)?,
        object_width: f("object_width", d.object_width)?,
        heading_min_speed: f("heading_min_speed", d.heading_min_speed)?,
        heading_max_velocity_std: f("heading_max_velocity_std", d.heading_max_velocity_std)?,
        default_radius: f("default_radius", d.default_radius)?,
        accel_noise_smooth: f("accel_noise_smooth", d.accel_noise_smooth)?,
        accel_noise_turning: f("accel_noise_turning", d.accel_noise_turning)?,
        model_switch_probability: f("model_switch_probability", d.model_switch_probability)?,
        measurement_noise: f("measurement_noise", d.measurement_noise)?,
        point_measurement_noise: f("point_measurement_noise", d.point_measurement_noise)?,
        initial_speed_std: f("initial_speed_std", d.initial_speed_std)?,
        gate_chi2: f("gate_chi2", d.gate_chi2)?,
        min_gate_distance: f("min_gate_distance", d.min_gate_distance)?,
        max_gate_distance: f("max_gate_distance", d.max_gate_distance)?,
        birth_suppression: f("birth_suppression", d.birth_suppression)?,
        confirm_hits: u("confirm_hits", d.confirm_hits)?,
        confirm_window: u("confirm_window", d.confirm_window)?,
        max_misses: u("max_misses", d.max_misses)?,
        max_position_std: f("max_position_std", d.max_position_std)?,
        min_reported_speed: f("min_reported_speed", d.min_reported_speed)?,
        min_displacement: f("min_displacement", d.min_displacement)?,
        min_z: f("min_z", d.min_z)?,
        max_z: f("max_z", d.max_z)?,
    };
    params
        .validate()
        .map_err(|err| anyhow!("invalid parameters: {err}"))?;
    Ok(params)
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("tracker")?;

    let params = declare_params(&node)?;
    let heartbeat_period: f64 = node
        .declare_parameter("heartbeat_period")
        .default(0.25)
        .mandatory()?
        .get();
    let map_frame: Arc<str> = node
        .declare_parameter("map_frame")
        .default(Arc::from("map"))
        .mandatory()?
        .get();
    let margin = params.static_margin;
    let state = Arc::new(Mutex::new(State::new(params, &map_frame)));

    let publisher = node
        .create_publisher::<TrackedObstacleArray>("/tracked_obstacles".keep_last(10).reliable())?;

    let s = Arc::clone(&state);
    let _map = node.create_subscription::<OccupancyGrid, _>(
        "/map".keep_last(1).reliable().transient_local(),
        move |msg: OccupancyGrid| {
            let mut state = lock(&s);
            match static_map_of(&msg, margin) {
                Ok(map) => state.pipeline.set_map(map),
                Err(err) => state.problem(format!("ignored /map: {err}")),
            }
        },
    )?;

    let s = Arc::clone(&state);
    let _tf = node
        .create_subscription::<TFMessage, _>("/tf".keep_last(100), move |msg: TFMessage| {
            insert_transforms(&mut lock(&s).tf, &msg, false)
        })?;
    let s = Arc::clone(&state);
    let _tf_static = node.create_subscription::<TFMessage, _>(
        "/tf_static".keep_last(100).reliable().transient_local(),
        move |msg: TFMessage| insert_transforms(&mut lock(&s).tf, &msg, true),
    )?;

    let s = Arc::clone(&state);
    let _stereo = node.create_subscription::<PointCloud2, _>(
        "/stereo/points".sensor_data_qos().keep_last(5),
        move |msg: PointCloud2| lock(&s).on_optional(&msg, Source::Stereo, "/stereo/points"),
    )?;
    let s = Arc::clone(&state);
    let _side = node.create_subscription::<PointCloud2, _>(
        "/perception/side_points".sensor_data_qos().keep_last(5),
        move |msg: PointCloud2| lock(&s).on_optional(&msg, Source::Side, "/perception/side_points"),
    )?;

    let s = Arc::clone(&state);
    let frame_id = map_frame.to_string();
    let _lidar = node.create_subscription::<PointCloud2, _>(
        "/perception/points".sensor_data_qos().keep_last(5),
        move |msg: PointCloud2| {
            let result = lock(&s).on_lidar(&msg);
            let Some((stamp, tracks)) = result else {
                return;
            };
            let mut out = TrackedObstacleArray {
                obstacles: tracks.iter().map(obstacle_msg).collect(),
                ..Default::default()
            };
            out.header.stamp = to_stamp(stamp);
            out.header.frame_id = frame_id.clone();
            if let Err(err) = publisher.publish(&out) {
                eprintln!("failed to publish /tracked_obstacles: {err}");
            }
        },
    )?;

    let heartbeat_publisher = node.create_publisher::<Heartbeat>("/heartbeat")?;
    let heartbeat_node = node.clone();
    let s = Arc::clone(&state);
    // Steady clock on purpose: with use_sim_time, a node-clock timer stops without /clock.
    let _timer = node.create_timer_repeating(
        Duration::from_secs_f64(heartbeat_period).steady_time(),
        move || {
            let (status, detail) = lock(&s).heartbeat();
            let mut msg = Heartbeat {
                node: heartbeat_node.name(),
                status,
                detail,
                ..Default::default()
            };
            if let Ok((sec, nanosec)) = heartbeat_node.get_clock().now().to_sec_nanosec() {
                msg.header.stamp.sec = sec;
                msg.header.stamp.nanosec = nanosec;
            }
            if let Err(err) = heartbeat_publisher.publish(&msg) {
                eprintln!("failed to publish heartbeat: {err}");
            }
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::sensor_msgs::msg::PointField;

    fn cloud(frame: &str, stamp: i64, points: &[[f32; 3]]) -> PointCloud2 {
        let mut msg = PointCloud2 {
            height: 1,
            width: points.len() as u32,
            fields: ["x", "y", "z"]
                .iter()
                .enumerate()
                .map(|(i, name)| PointField {
                    name: (*name).into(),
                    offset: 4 * i as u32,
                    datatype: 7,
                    count: 1,
                })
                .collect(),
            is_bigendian: false,
            point_step: 12,
            row_step: 12 * points.len() as u32,
            data: points
                .iter()
                .flatten()
                .flat_map(|v| v.to_le_bytes())
                .collect(),
            is_dense: true,
            ..Default::default()
        };
        msg.header.frame_id = frame.into();
        msg.header.stamp = to_stamp(stamp);
        msg
    }

    fn state() -> State {
        State::new(Params::default(), "map")
    }

    #[test]
    fn stamps_round_trip() {
        for ns in [0, 1, 999_999_999, 1_000_000_000, 12_345_678_901] {
            assert_eq!(stamp_ns(&to_stamp(ns)), ns);
        }
    }

    #[test]
    fn a_moving_cluster_becomes_a_track_in_map() {
        let mut s = state();
        assert_eq!(s.heartbeat().0, Heartbeat::OK);
        // Without TF the cloud is dropped and the heartbeat warns.
        assert!(s
            .on_lidar(&cloud("base_link", 0, &[[3.0, 0.0, 0.3]]))
            .is_none());
        assert_eq!(s.heartbeat().0, Heartbeat::WARN);
        s.last_problem = None;
        let mut last = None;
        for k in 0..10i64 {
            let stamp = 1_000_000_000 + k * 100_000_000;
            // Robot at map (10, 0) facing map +y, through map -> odom -> base_link.
            s.tf.insert("map", "odom", stamp, Pose2::new(10.0, 0.0, 0.0), false);
            let facing_y = Pose2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2);
            s.tf.insert("odom", "base_link", stamp, facing_y, false);
            // The long side of a cyclist 4 m ahead, riding to the robot's right (base_link -y = map +x) at 3 m/s.
            let y = -(k as f32) * 0.3;
            let points: Vec<[f32; 3]> = (0..7)
                .map(|i| [4.0, y - 0.9 + 0.3 * i as f32, 0.3])
                .collect();
            last = s.on_lidar(&cloud("base_link", stamp, &points));
        }
        let (stamp, tracks) = last.unwrap();
        assert_eq!(stamp, 1_900_000_000);
        assert_eq!(tracks.len(), 1);
        let msg = obstacle_msg(&tracks[0]);
        // Its centre lies half the cyclist width behind the visible side: map y = 4.3.
        assert!((msg.position.y - 4.3).abs() < 0.2, "{:?}", msg.position);
        assert!(
            (msg.velocity.x - 3.0).abs() < 0.5 && msg.velocity.y.abs() < 0.5,
            "{:?}",
            msg.velocity
        );
        assert_eq!(s.heartbeat().0, Heartbeat::OK);
    }

    #[test]
    fn optional_clouds_wait_for_the_lidar_cloud() {
        let mut s = state();
        for k in 0..30i64 {
            s.tf.insert("map", "base_link", k * 50_000_000, Pose2::IDENTITY, false);
        }
        for stamp in [300_000_000, 200_000_000, 900_000_000] {
            s.on_optional(
                &cloud("base_link", stamp, &[[5.0, 1.0, 0.0]]),
                Source::Side,
                "side",
            );
        }
        assert_eq!(s.pending.len(), 3);
        s.on_lidar(&cloud("base_link", 400_000_000, &[])).unwrap();
        assert_eq!(s.pending.len(), 1);
        assert_eq!(s.pending[0].stamp, 900_000_000);
    }

    #[test]
    fn the_map_message_is_converted_with_its_origin() {
        let mut msg = OccupancyGrid::default();
        msg.info.width = 2;
        msg.info.height = 1;
        msg.info.resolution = 0.5;
        msg.info.origin.position.x = -1.0;
        msg.info.origin.orientation.w = 1.0;
        msg.data = vec![0, 100];
        let map = static_map_of(&msg, 0.0).unwrap();
        // Cells span map x in [-1, -0.5) (free) and [-0.5, 0) (occupied).
        assert!(map.is_static([-0.3, 0.2]));
        assert!(!map.is_static([-0.8, 0.2]));
        assert!(!map.is_static([0.2, 0.2]));
    }
}
