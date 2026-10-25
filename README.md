# delivery_bot_core

Shared ROS 2 (Jazzy) packages for the **delivery_bot**: the robot description and the deterministic navigation
stack (Nav2 configuration, grid-step controller, dynamic obstacle tracking).

This repository is the single source of the navigation logic. The same commit runs in two places:

| Consumer | How it uses this repo |
| --- | --- |
| [delivery_bot_sim](https://github.com/leonst2/delivery_bot_sim) (Unity simulation) | Git submodule at `ros2_ws/src/delivery_bot_core`, launched with `sim:=true` |
| [delivery_bot_os](https://github.com/leonst2/delivery_bot_os) (physical robot, Raspberry Pi 5) | Git submodule in its `ros2_ws/src/` *(to be added)*, launched with `sim:=false` |

## Packages

| Package | Type | Contents |
| --- | --- | --- |
| `delivery_bot` | ament_cmake | URDF (V-0.2), STL meshes, `description.launch.py` (robot_state_publisher → `/tf_static`) |
| `delivery_bot_navigation` | ament_cmake | `nav2_params.yaml`, `sim.yaml` overlay, `bringup.launch.py` |
| `delivery_bot_grid_step` | ament_python | `grid_step_controller` node: halt → perceive → map → plan → move one cell → repeat *(stub)* |
| `delivery_bot_perception` | ament_python | `obstacle_tracker` node: dynamic obstacle tracking from `/scan` *(stub)* |
| `delivery_bot_rust_hello` | ament_cargo | Rust toolchain spike (#2): rclrs node, `/clock` → `/heartbeat` |
| `delivery_bot_spike_msgs` | ament_cmake (rosidl) | Temporary `Heartbeat` message for the spike, removed once `delivery_bot_interfaces` (#3) lands |

The description package is called `delivery_bot`, not `delivery_bot_description`, because the URDF refers to its
meshes as `package://delivery_bot/meshes/...`. The Unity simulation imports the same URDF file. Its copy in
`Assets/URDF/delivery_bot/` must stay byte-identical (the sim repo checks this in CI).

## Sim vs. robot

```bash
ros2 launch delivery_bot_navigation bringup.launch.py sim:=true    # Unity simulation
ros2 launch delivery_bot_navigation bringup.launch.py              # physical robot
```

`sim:=true` only layers `delivery_bot_navigation/config/sim.yaml` on top of the shared parameters and sets
`use_sim_time` on robot_state_publisher. Simulation differences are limited to that overlay: sim time and topic
remaps. Never fork the navigation logic for the simulation.

## Build

```bash
mkdir -p ws/src && cd ws/src && git clone https://github.com/leonst2/delivery_bot_core.git && cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build && colcon test && colcon test-result --verbose
```

CI (`.github/workflows/build.yml`) runs the same steps in the `ros:jazzy` container on every push. That job is
legacy: it still installs `robot_state_publisher` for the old launch smoke test, until #5 replaces it.

## Development image

`docker/Dockerfile` is the build environment for this repo and its consumers: ROS 2 Jazzy **ros-core** plus the
C++, Python and **Rust** toolchains. It's based on ros-core instead of ros-base on purpose: ros-base ships
`robot_state_publisher` and `kdl_parser`, and the image build fails if any forbidden package (Nav2,
slam_toolbox, AMCL, map_server, robot_state_publisher, kdl_parser) is present
(`docker/check_forbidden_packages.sh`). Add dependencies to the Dockerfile; don't run `rosdep install` in it.

```bash
docker build -t delivery_bot_core_dev:jazzy docker/
docker run --rm -it -v "$PWD/..:/ws/src" delivery_bot_core_dev:jazzy   # repo checked out in <ws>/src/
# inside: colcon build && colcon test && colcon test-result --verbose
```

| Toolchain | Contents |
| --- | --- |
| C++ | build-essential, cmake, gtest (`ament_cmake_gtest`), Eigen, tinyxml2, clang-format, clang-tidy |
| Python | colcon, pytest (`ament_cmake_pytest`), numpy |
| Rust | Rust 1.98.0 (rustfmt, clippy), `cargo-ament-build` 0.1.11, `colcon-cargo` 0.2.0, `colcon-ros-cargo` (pinned commit), libclang |
| ROS extras | `tf2_msgs`, `rosbag2`, ros2 CLI |

**Rust nodes** use [rclrs](https://github.com/ros2-rust/ros2_rust) 0.8 from crates.io and get message types through
the `ros-env` crate. On Jazzy, message packages only get Rust bindings if they're built while
`rosidl_generator_rs` is installed. So the image builds an overlay in `/opt/ros2_rust` with the generator and the
standard interfaces rebuilt from `docker/ros2_rust_jazzy.repos` (pinned commits). Interface packages in this
workspace (e.g. `delivery_bot_spike_msgs`) get Rust bindings automatically. `scripts/setup_rust.sh` sets up the same
on a native Ubuntu 24.04 machine. CI: `.github/workflows/dev-image.yml` builds the image, builds and tests the
workspace in it, and runs `cargo fmt --check`, `clippy -D warnings` and `cargo test` for every Rust package.

**Fallback if native Rust nodes ever break** (e.g. an rclrs/Jazzy incompatibility): keep the algorithms in a plain
Rust crate with a C ABI (`cbindgen` header), and wrap it in a thin rclcpp node. The Rust cards (#7, #8, #11, #19)
would then implement crate + wrapper instead of an rclrs node.

## Updating a consumer

```bash
# in delivery_bot_sim (or delivery_bot_os)
git submodule update --remote ros2_ws/src/delivery_bot_core
git add ros2_ws/src/delivery_bot_core && git commit -m "Bump delivery_bot_core"
```

## License

Proprietary (see `package.xml`). The repository is public for reference, but no open-source licence is granted.
