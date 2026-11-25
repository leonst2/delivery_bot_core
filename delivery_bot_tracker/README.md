# delivery_bot_tracker

Dynamic obstacle detection and tracking with prediction: finds moving objects (cyclists) in the points that the static
map doesn't explain, keeps a stable ID per object, and predicts where it will be one grid step ahead. The executive
(#21) uses the predictions for its yield check.

| | |
| --- | --- |
| Card | [delivery_bot_core#11](https://github.com/leonst2/delivery_bot_core/issues/11) |
| Language | Rust (rclrs) |
| Executable | `tracker` |

The algorithms are a ROS-free library (`src/lib.rs`), so `cargo test` covers them without a running graph.
`src/main.rs` is the node around it. Keep `build.rs`: every Rust node needs it (see the top-level README).

## Topics

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/perception/points` | `sensor_msgs/PointCloud2` (`base_link`) | in | Scan points from the scan filter (#9). Each cloud is one perception cycle. |
| `/stereo/points` | `sensor_msgs/PointCloud2` | in, optional | Stereo points (#10). Only points with `min_z` ≤ z ≤ `max_z` are used. |
| `/perception/side_points` | `sensor_msgs/PointCloud2` (`base_link`) | in, optional | One ground point per object seen by the side cameras (#33). |
| `/map` | `nav_msgs/OccupancyGrid` | in, transient-local | Static map (#12). Points near occupied cells are static. |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | in | `map → odom` (#8), `odom → base_link` (#7). Any cloud frame with a TF chain to `map` works. |
| `/tracked_obstacles` | `delivery_bot_interfaces/TrackedObstacleArray` (`map`) | out | Confirmed tracks sorted by id, once per `/perception/points` cloud, stamped with that cloud's stamp. |
| `/heartbeat` | `delivery_bot_interfaces/Heartbeat` | out | 4 Hz on the steady clock. |

The optional clouds are buffered and processed in stamp order before the next LiDAR cloud, so the order in which
callbacks run never changes the output.

## Algorithm

1. **Points to map.** Look up the cloud frame's pose in `map` at the cloud's stamp (interpolated, up to 0.2 s past
   the newest transform). Without it the cloud is dropped and the heartbeat warns.
2. **Filter.** Drop points beyond `max_range` and points within `static_margin` of an occupied map cell. Free,
   unknown and off-map space count as not static.
3. **Cluster.** Euclidean clustering on a grid index. The allowed gap is `cluster_tolerance`, growing to
   `cluster_tolerance_per_meter` × range farther out, because neighbouring rays spread apart (a cyclist 11 m away at
   a grazing angle has points 0.6 m apart). Clusters with fewer than `min_cluster_points` or longer than
   `max_cluster_extent` (walls) are dropped. Side-camera points are single detections.
4. **Measure.** The LiDAR sees only an object's near side. Until a track's heading is known, the centre is taken
   `surface_offset` behind the nearest surface on the centroid's bearing. Once the track moves and its velocity is
   certain enough (`heading_min_speed`, `heading_max_velocity_std`), an `object_length` × `object_width` footprint
   along the heading is fitted to the visible side. When a track switches between the two, its position is shifted
   by the difference so the switch doesn't look like motion.
5. **Filter per track.** IMM filter over two constant-velocity Kalman filters: smooth riding
   (`accel_noise_smooth`) and turning (`accel_noise_turning`). The measurements decide which dominates, so straight
   riding gets smooth velocities and turns get little lag.
6. **Associate.** Hungarian assignment on squared Mahalanobis distances, gated by `gate_chi2`, always allowed within
   `min_gate_distance`, never beyond `max_gate_distance`. Ties go to the lowest index.
7. **Manage tracks.** An unassigned detection starts a tentative track unless it overlaps an existing one
   (`birth_suppression`). Confirmed after `confirm_hits` hits in its first `confirm_window` cycles, else dropped. A
   confirmed track coasts through up to `max_misses` cycles without a hit. Only LiDAR clouds count as cycles; the
   optional sources add hits but never misses, because they don't see all around. IDs count up from 1 and restart
   after a clock reset (`/sim/reset`).
8. **Report.** A track reports its velocity only after it has moved `min_displacement` from where it started
   (measured over at least three updates) and while it is faster than `min_reported_speed`. Otherwise velocity is 0
   and `predicted_position` = `position`, so parked or unmapped static objects are never reported as moving.
   `predicted_position` = `position` + `velocity` × `prediction_horizon`. `radius` is at least `default_radius`.

Everything is deterministic: no randomness, no wall-clock time in the algorithm, points sorted before clustering,
fixed tie-breaking.

## Parameters

All are ROS parameters with these defaults (`src/params.rs`). None differ between simulation and robot.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `prediction_horizon` | 1.5 | s, how far `predicted_position` looks ahead (one grid step) |
| `max_range` | 12.0 | m, points farther from the robot are ignored |
| `cluster_tolerance` | 0.35 | m, largest gap between points of one object near the robot |
| `cluster_tolerance_per_meter` | 0.05 | m/m, the allowed gap farther out, times the range |
| `min_cluster_points` | 3 | smaller clusters are noise |
| `max_cluster_extent` | 3.0 | m, longer clusters are structure, not objects |
| `static_margin` | 0.15 | m, points this close to an occupied map cell are static |
| `surface_offset` | 0.4 | m, centre behind the visible surface while the heading is unknown |
| `object_length`, `object_width` | 1.8, 0.6 | m, footprint fitted once the heading is known (a cyclist) |
| `heading_min_speed` | 1.0 | m/s, slower tracks don't get the footprint fit |
| `heading_max_velocity_std` | 0.8 | m/s, less certain velocities don't either |
| `default_radius` | 0.95 | m, minimum reported radius (half the cyclist footprint diagonal) |
| `accel_noise_smooth`, `accel_noise_turning` | 1.0, 4.0 | m/s², process noise of the two IMM models |
| `model_switch_probability` | 0.05 | IMM model switch probability per update |
| `measurement_noise` | 0.15 | m, cluster measurement noise, including the robot pose error |
| `point_measurement_noise` | 0.3 | m, side-camera point noise |
| `initial_speed_std` | 4.0 | m/s, velocity uncertainty of a new track |
| `gate_chi2` | 13.8 | Mahalanobis gate (99.9 %, 2 DOF) |
| `min_gate_distance`, `max_gate_distance` | 1.0, 2.5 | m, always / never associate within / beyond |
| `birth_suppression` | 0.95 | m, no new track from points this close to a track |
| `confirm_hits`, `confirm_window` | 2, 3 | N-of-M confirmation |
| `max_misses` | 10 | cycles a confirmed track may go without a hit |
| `max_position_std` | 2.0 | m, tracks less certain than this are dropped |
| `min_reported_speed` | 0.3 | m/s, slower tracks report zero velocity |
| `min_displacement` | 0.75 | m, a track must move this far before it reports velocity |
| `min_z`, `max_z` | 0.15, 2.0 | m, height band of `/stereo/points` in `base_link` |
| `map_frame` | `map` | output frame and TF target |
| `heartbeat_period` | 0.25 | s |

## Heartbeat

OK while running, with the number of tracks (or "waiting for /perception/points" before the first cloud, "no /map
yet" without a map). No input is not a fault, since CI starts the stack without sensors. WARN for 2 s after a message
was dropped, with the reason: no transform to `map`, an unreadable cloud, an invalid map, or a stale cloud.

## Tests

`cargo test` runs the unit tests of every module plus `src/scenarios.rs`, one test per acceptance criterion, on a
synthetic world (`src/synth.rs`, the card's mock). It has cyclist-sized boxes (1.8 × 0.6 m, like `CyclistAgent`)
on straight and curved paths, a simulated LiDAR like the Unity one (360°, 1°, 0.1–12 m, 10 Hz, 1 cm noise), and
2 cm / 0.17° scan-to-scan pose error. Accuracy is measured from 1 s after confirmation while the cyclist is within
12 m. Curves and circles keep the lateral acceleration at or below 2.5 m/s², about what cyclists ride. Current
results (worst case over 2, 5 and 8 m/s):

| Criterion | Limit | Synthetic result |
| --- | --- | --- |
| Position RMSE | < 0.3 m | 0.05 m straight, 0.22 m in curves |
| Speed error | < 0.5 m/s | 0.08 m/s |
| Confirmation after entering 12 m | ≤ 3 cycles | passes |
| ID switches | ≤ 1 per 100 tracked s | 0 in 300 tracked s (two cyclists occluding each other) and in a crossing |
| Static scene, 5 min | no moving tracks | none; one unmapped bin tracked with zero velocity |
| Constant-velocity prediction after 1.5 s | < 0.5 m | 0.34 m |
| Identical input | identical output | passes, also with the points of each scan shuffled |

## Limitations

- The footprint fit assumes cyclist-sized objects. A much shorter object (a pedestrian) seen by the LiDAR is
  measured with up to half `object_length` of error along its heading. Side-camera points are unaffected.
- Velocity needs about a second after confirmation to settle. Before that, predictions are less accurate.
- The criteria are checked on synthetic data. The check against `/ground_truth/dynamic_objects` in the Unity city
  waits for the scan filter (#9) and a TF source for `map → base_link` (#7, #8; localization's `ground_truth` mode).
