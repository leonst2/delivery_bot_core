# delivery_bot_core

Shared ROS 2 (Jazzy) packages for the **delivery_bot**: the robot description and a fully self-programmed,
deterministic navigation stack in C++, Rust and Python. It only uses ROS 2 core (see [CONTRIBUTING.md](CONTRIBUTING.md)
for the rules and how to work on a package).

This repository is the single source of the navigation logic. The same commit runs in two places:

| Consumer | How it uses this repo |
| --- | --- |
| [delivery_bot_sim](https://github.com/leonst2/delivery_bot_sim) (Unity simulation) | Git submodule at `ros2_ws/src/delivery_bot_core`, launched with `sim:=true` |
| [delivery_bot_os](https://github.com/leonst2/delivery_bot_os) (physical robot, Raspberry Pi 5) | Git submodule in its `ros2_ws/src/` *(to be added)*, launched with `sim:=false` |

## Packages

Every package exists and builds. Packages marked *stub* only publish heartbeats (or hold an empty library) until
their card lands; each package README names its card.

| Package | Lang | Contents | Card |
| --- | --- | --- | --- |
| `delivery_bot` | — | URDF (V-0.2) and STL meshes | |
| `delivery_bot_interfaces` | rosidl | Every custom message, service and action, see its [README](delivery_bot_interfaces/README.md) | #3 |
| `delivery_bot_interfaces_tests` | C++/Python | Serialisation checks for `delivery_bot_interfaces` | #3 |
| `delivery_bot_bringup` | Python | `stack.launch.py`, parameters, sim overlay, health monitor, see its [README](delivery_bot_bringup/README.md) | #4 |
| `delivery_bot_description_publisher` | C++ | Own URDF parser: `/robot_description`, `/tf_static`, joint-driven `/tf` *(stub)* | #5 |
| `delivery_bot_viz` | Python | Marker topics and an RViz configuration *(stub)* | #6 |
| `delivery_bot_odometry` | Rust | Wheel + IMU odometry with own EKF *(stub)* | #7 |
| `delivery_bot_localization` | Rust | Particle-filter localization (MCL) *(stub)* | #8 |
| `delivery_bot_scan_filter` | C++ | Cleaned scans and obstacle points *(stub)* | #9 |
| `delivery_bot_stereo` | C++ | Stereo disparity and 3D points *(stub)* | #10 |
| `delivery_bot_tracker` | Rust | Dynamic obstacle tracking with prediction, see its [README](delivery_bot_tracker/README.md) | #11 |
| `delivery_bot_map_server` | Python | Own map format, latched `/map`, `LoadMap` *(stub)* | #12 |
| `delivery_bot_occupancy` | C++ | Occupancy-grid mapping *(stub)* | #13 |
| `delivery_bot_costmap` | C++ | Layered global and local costmaps *(stub)* | #14 |
| `delivery_bot_collision` | C++ | Footprint, exact distance fields, inflation and collision-checking library | #15 |
| `delivery_bot_geometry` | C++, Rust | SE(2), Reeds-Shepp/Dubins, Ackermann primitives and arc utilities library | #16 |
| `delivery_bot_planner` | C++ | Hybrid-A* planner (PlanPath); path post-processing *(stub)* | #17, #18 |
| `delivery_bot_step_controller` | Rust | Follows one `step_size` arc and stops *(stub)* | #19 |
| `delivery_bot_safety` | C++ | Emergency stop, command watchdog and gating *(stub)* | #20 |
| `delivery_bot_executive` | Python | Grid-step executive and mission manager *(stub)* | #21, #22 |
| `delivery_bot_evaluation` | Python | Metrics, scenarios, benchmarks, reports, record/replay *(stub)* | #23–#27 |

The description package is called `delivery_bot`, not `delivery_bot_description`, because the URDF refers to its
meshes as `package://delivery_bot/meshes/...`. The Unity simulation imports the same URDF file. Its copy in
`Assets/URDF/delivery_bot/` must stay byte-identical (the sim repo checks this in CI).

## Sim vs. robot

```bash
ros2 launch delivery_bot_bringup stack.launch.py sim:=true    # Unity simulation
ros2 launch delivery_bot_bringup stack.launch.py              # physical robot
```

`sim:=true` only layers `delivery_bot_bringup/params/sim.yaml` on top of the shared parameters. Simulation differences
are limited to that overlay. Never fork the navigation logic for the simulation.

## Build, test and lint

Everything runs in the development image (below). With the repo checked out in `<ws>/src/delivery_bot_core`:

```bash
docker build -t delivery_bot_core_dev:jazzy docker/
docker run --rm -it -v "$PWD/../..:/ws" -w /ws delivery_bot_core_dev:jazzy
# inside:
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test && colcon test-result --verbose
source install/setup.bash
src/delivery_bot_core/scripts/lint.sh     # ruff, clang-format, clang-tidy, cargo fmt, clippy
```

CI (`.github/workflows/dev-image.yml`) does the same on every push and pull request: it builds the image, builds and
tests every package (gtest, pytest, `cargo test`), starts the whole stack with `sim:=true` and waits until every node
reports OK, then runs `scripts/lint.sh`. Any failure fails the build.

## Development image

`docker/Dockerfile` is the build environment for this repo and its consumers: ROS 2 Jazzy **ros-core** plus the
C++, Python and **Rust** toolchains. It's based on ros-core instead of ros-base on purpose: ros-base ships
`robot_state_publisher` and `kdl_parser`, and the image build fails if any forbidden package is present
(`docker/check_forbidden_packages.sh`). Add dependencies to the Dockerfile; don't run `rosdep install` in it.

| Toolchain | Contents |
| --- | --- |
| C++ | build-essential, cmake, gtest (`ament_cmake_gtest`), Eigen, tinyxml2, clang-format, clang-tidy |
| Python | colcon, pytest (`ament_cmake_pytest`), numpy, ruff |
| Rust | Rust 1.98.0 (rustfmt, clippy), `cargo-ament-build` 0.1.11, `colcon-cargo` 0.2.0, `colcon-ros-cargo` (pinned commit), libclang |
| ROS extras | `tf2_msgs`, `rosbag2`, ros2 CLI |

**Rust nodes** use [rclrs](https://github.com/ros2-rust/ros2_rust) 0.8 from crates.io and get message types through
the `ros-env` crate. On Jazzy, message packages only get Rust bindings if they're built while
`rosidl_generator_rs` is installed. So the image builds an overlay in `/opt/ros2_rust` with the generator and the
standard interfaces rebuilt from `docker/ros2_rust_jazzy.repos` (pinned commits), plus `tf2_msgs` so Rust nodes can
use `/tf` (the rest of geometry2 is removed before the build). Interface packages in this
workspace (e.g. `delivery_bot_interfaces`) get Rust bindings automatically. `scripts/setup_rust.sh` sets up the same
on a native Ubuntu 24.04 machine.

Every Rust package needs a `build.rs` like the one in `delivery_bot_odometry`. rclrs's build script doesn't rerun when
`AMENT_PREFIX_PATH` changes, so without it a new interface package fails to link in a cached workspace.

**Fallback if native Rust nodes ever break** (e.g. an rclrs/Jazzy incompatibility): keep the algorithms in a plain
Rust crate with a C ABI (`cbindgen` header), and wrap it in a thin rclcpp node. The Rust cards (#7, #8, #11, #19)
would then implement crate + wrapper instead of an rclrs node.

## Updating a consumer

```bash
# in delivery_bot_sim (or delivery_bot_os)
git submodule update --remote ros2_ws/src/delivery_bot_core
git add ros2_ws/src/delivery_bot_core && git commit -m "Bump delivery_bot_core"
```

## History

Until November 2026 this repo held a Nav2 configuration with Python stubs for the grid-step controller and the
obstacle tracker. #1 replaced it with the self-programmed stack above.

## License

Proprietary (see `package.xml`). The repository is public for reference, but no open-source licence is granted.
