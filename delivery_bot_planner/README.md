# delivery_bot_planner

Hybrid-A* path planner for the Ackermann robot (PlanPath action). Path smoothing, resampling and step segmentation
(#18) will live here too and are not implemented yet.

| | |
| --- | --- |
| Cards | [delivery_bot_core#17](https://github.com/leonst2/delivery_bot_core/issues/17) (planner, done), [delivery_bot_core#18](https://github.com/leonst2/delivery_bot_core/issues/18) (post-processing, open) |
| Language | C++ (rclcpp, rclcpp_action) |
| Executable | `planner` |
| Libraries | `planner_core` (the search, no ROS), `planner_node` (the node, for in-process tests) |

## Interface

| Topic / action | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/plan_path` | action `delivery_bot_interfaces/PlanPath` | server | One search at a time, on its own thread. Feedback: expanded nodes. |
| `map_topic` (default `/costmap/global`) | `nav_msgs/OccupancyGrid`, transient-local | in | The costmap (#14) once it exists; set `map_topic: /map` to plan on the static map. Origin yaw must be 0. |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | in | Only for starts with an empty `frame_id`: `map → … → base_link`, latest transforms. |
| `/plan` | `nav_msgs/Path` (`map`), transient-local | out | Every successful result, sampled every `path_resolution`. |
| `/heartbeat` | `delivery_bot_interfaces/Heartbeat` | out | 4 Hz on the steady clock. OK, also without a map (detail says so); WARN if a map was rejected. |

**Goals:** `start` with an empty `frame_id` means the current pose; otherwise `start` and `goal` must be in `map`
(else `ERROR_INVALID_START` / `ERROR_INVALID_GOAL`). `time_limit` ≤ 0 uses `default_time_limit`. Errors:
`ERROR_INVALID_START` / `ERROR_INVALID_GOAL` (in collision, off the map, wrong frame, no TF), `ERROR_NO_PATH` (no
path, or no map received yet), `ERROR_TIMEOUT`. A cancelled goal ends as canceled.

**Paths:** `nav_msgs/Path` has no driving direction. The poses carry the robot heading, so a reverse segment is
recognisable by poses moving against their heading. A path ends exactly on the goal when a Reeds-Shepp shot reaches it,
otherwise within `goal_xy_tolerance` / `goal_yaw_tolerance` (see below).

## Algorithm

Hybrid-A* over (x, y, heading bin, driving direction):

- **Motions:** the Ackermann primitives of `delivery_bot_geometry` (0.5 m arcs, 5 steering angles up to R_min =
  1.25 m, forward and reverse). Every motion is checked with the conservative swept check of `delivery_bot_collision`;
  the first motions from the robot's own pose and the end at the goal use its refinement, so the robot can leave a
  pose next to a wall.
- **Cost:** length, reverse length × `reverse_penalty`, `direction_change_penalty` per cusp, small penalties for
  steering and steering changes, plus the costmap cost along the motion (inflation keeps paths away from walls).
- **Heuristic:** `heuristic_weight` × max(8-connected Dijkstra distance from the goal, Reeds-Shepp length ignoring
  obstacles). The Dijkstra runs at map resolution for the footprint centroid (Dial's algorithm, a few ms for
  100 × 100 m at 0.1 m), with cells closer than the inscribed radius blocked, so it never squeezes through gaps
  narrower than the robot. Reeds-Shepp only within `rs_heuristic_radius` of the goal. Cached per goal cell.
- **Reaching the goal:** obstacle-checked Reeds-Shepp shots every `analytic_every` expansions and at every expansion
  within `analytic_radius`; lattice nodes within the goal tolerance. Both go into the open list as goal candidates with
  their full cost, so the search ends when the cheapest candidate is cheaper than everything still open.
- **Determinism:** the open list is ordered by (f, h, insertion order), no hash-table order reaches the result, and
  everything is built with `-ffp-contract=off`: the same inputs give a byte-identical path, on ARM64 and x86_64.
- **Re-plans:** if the goal is unchanged and the robot is within 0.3 m and 15° of the previous path, the node joins
  the robot's pose to that path 1.5–5 m ahead (short cusp-free Reeds-Shepp curve) and keeps the rest, if it is still
  collision-free on the current map. Otherwise it plans from scratch.

## Parameters

| Parameter | Default | |
| --- | --- | --- |
| `map_topic` | `/costmap/global` | |
| `map_frame`, `base_frame` | `map`, `base_link` | |
| `default_time_limit` | 1.0 s | when the goal's `time_limit` is ≤ 0 |
| `path_resolution` | 0.05 m | spacing of the result poses |
| `occupied_threshold`, `unknown_is_lethal` | 65, true | which cells are obstacles (`delivery_bot_collision`) |
| `footprint.{x_min,x_max,y_min,y_max,padding}` | −0.085, 0.575, −0.27, 0.27, 0.05 m | URDF chassis box |
| `sweep_step` | 0.05 m | swept-check spacing |
| `xy_resolution`, `yaw_bins` | 0.25 m, 72 | search lattice |
| `wheelbase`, `min_turning_radius`, `primitive_length`, `steering_count`, `allow_reverse` | 0.49 m, 1.25 m, 0.5 m, 5, true | primitives |
| `reverse_penalty`, `direction_change_penalty` | 2.0, 4.0 m | |
| `steering_penalty`, `steering_change_penalty`, `cost_weight` | 0.05, 0.2, 1.0 | |
| `heuristic_weight` | 1.5 | |
| `goal_xy_tolerance`, `goal_yaw_tolerance` | 0.2 m, 0.1 rad | |

## Tests and tuning

`colcon test --packages-select delivery_bot_planner`:

- `test_planner_core`: heuristic; the card's mock maps (open field, L-corner corridor, U-turn dead end, parking bay,
  narrow gap, enclosed goal) with every path checked for its endpoints, collisions (refined swept check) and
  curvature ≤ 1/R_min; errors, time limit, cancellation; byte-identical paths across runs and instances; re-plan reuse
  and its fallbacks; the planar TF chain; and the benchmark below.
- `test_planner_node`: the node in-process (no map, plan and `/plan`, start from TF, wrong goal frame).

**Benchmark** (`test_benchmark.cpp`, standing in for the #26 corpus): four city-like 100 × 100 m maps at 0.1 m (blocks,
8 m streets, parked cars, clutter), 100 street-to-street queries with headings along the street (either direction,
±25°) and at least 0.1 m clearance. Release on arm64: median 36 ms, p95 0.61 s, no failures; the test asserts
p95 < 1 s in optimised builds, scaled up (at most 2×) on machines that check poses more slowly than a reference about
20 % faster than the x86_64 CI runner, so a slow shared runner doesn't fail the test (CI: p95 0.94–1.04 s against a
limit of 1.2–1.5 s). With uniformly random headings (goals across the street next to a parked car) p95 is
about 0.66 s.

How the defaults were chosen: at weight 1.0 the search explored tens of thousands of nodes per query (1–2 s and more),
because the heuristic doesn't know about manoeuvres in tight streets; traces showed 60–70 % of the expansions within
5 m of the goal, where exact Reeds-Shepp shots next to parked cars nearly always collide. The goal tolerance, the
centroid heuristic, weight 1.5 and the coarser 0.25 m lattice fixed the tail; inlining the geometry and grid helpers
and Dial's algorithm made it fast. Weight 1.2 gives ~5 % cheaper paths but times out on ~10 % of the queries; a
cusp penalty of 4 m halves the cusps of 3 m at little cost in time.

## Known limitations

- Paths are not smoothed yet (#18): they follow the 0.5 m primitives and Reeds-Shepp curves.
- Weighted A* returns a path within 1.5× of the cheapest, not the cheapest.
- Planning on `/costmap/global` needs #14; until then set `map_topic: /map` (needs #12) or the node waits for a map.
