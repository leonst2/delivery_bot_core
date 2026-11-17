# Contributing

## The self-programmed rule

The stack is written by us. Only ROS 2 core may be used:

- **Allowed:** rclcpp, rclpy, rclrs, the standard message packages, launch, colcon, rosbag2, the Unity ROS-TCP bridge,
  RViz as an optional debug viewer, and general-purpose libraries such as Eigen, tinyxml2, numpy, serde and nalgebra.
- **Not allowed:** Nav2, slam_toolbox, AMCL, map_server, robot_state_publisher, kdl_parser or OpenCV algorithms.
  The dev image build fails if one of these packages is present (`docker/check_forbidden_packages.sh`).

Add dependencies to `docker/Dockerfile` and the package's `package.xml`, never through `rosdep install`.

## Package layout

One package per component. The layout is fixed, so cards can be worked on in parallel. The
[README](README.md#packages) lists every package with its language and card. Topics, messages and actions are the
contract between packages: code against them (`delivery_bot_interfaces`) and mock the other side in your tests
instead of waiting for another card.

| Language | Used for | Packages |
| --- | --- | --- |
| **C++** (rclcpp) | Compute-heavy work: URDF/TF, scan and stereo processing, mapping, costmaps, collision checks, geometry, planning, safety | description_publisher, scan_filter, stereo, occupancy, costmap, collision, geometry, planner, safety |
| **Rust** (rclrs) | Estimation and control loops: odometry/EKF, localization, tracking, step control | odometry, localization, tracker, step_controller |
| **Python** (rclpy) | Orchestration and evaluation: launch, health, map I/O, executive, visualization, benchmarks | bringup, map_server, executive, viz, evaluation |

`delivery_bot` holds the URDF and meshes. Keep it byte-identical with the copy in delivery_bot_sim.

## Working on a card

Every package starts as a stub. A node stub only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat` under
its stack name, so `stack.launch.py` and the health monitor already work. To implement a card:

1. Branch `feat/<issue>-<short-name>` off `main`.
2. Replace the stub in the card's package. Keep the executable name from
   `delivery_bot_bringup/delivery_bot_bringup/stack.py`, or change it in the same PR.
3. Keep publishing heartbeats at >= 2 Hz on the **steady clock** (`create_wall_timer` in C++, `HeartbeatPublisher` from
   `delivery_bot_bringup` in Python, `.steady_time()` timers in Rust). With `use_sim_time`, ROS-time timers stop
   without `/clock`, and the node would look stalled.
4. Keep the node deterministic: no randomness without a fixed seed, and fixed tie-breaking.
5. Add unit tests, and update the package README (purpose, topics, parameters).
6. Run the build, tests and lint (below) before opening the PR.

**Rust packages** need the `build.rs` from the stubs. rclrs's build script doesn't rerun when `AMENT_PREFIX_PATH`
changes, so without it a new interface package fails to link in a cached workspace. Commit `Cargo.lock`.

## Build, test and lint

Run everything in the dev image, from the workspace root (`<ws>`, with this repo in `<ws>/src/delivery_bot_core`):

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test && colcon test-result --verbose
source install/setup.bash
src/delivery_bot_core/scripts/lint.sh
```

| Language | Tests | Format and static analysis |
| --- | --- | --- |
| C++ | gtest via `ament_cmake_gtest` | `clang-format` (`.clang-format`, ROS 2 style, 120 columns), `clang-tidy` (`.clang-tidy`, findings are errors) |
| Python | pytest (`test/` of the package) | `ruff check` (`ruff.toml`, 120 columns, single quotes) |
| Rust | `cargo test` | `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings` |

To fix formatting: `clang-format -i <files>`, `ruff check --fix .`, `cargo fmt`. CI runs the same steps on every push
and pull request (`.github/workflows/dev-image.yml`), and also starts the whole stack once (`sim:=true`) and checks
that every node reports OK within 30 s.
