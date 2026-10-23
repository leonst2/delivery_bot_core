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

CI (`.github/workflows/build.yml`) runs the same steps in the `ros:jazzy` container on every push.

## Updating a consumer

```bash
# in delivery_bot_sim (or delivery_bot_os)
git submodule update --remote ros2_ws/src/delivery_bot_core
git add ros2_ws/src/delivery_bot_core && git commit -m "Bump delivery_bot_core"
```

## License

Proprietary (see `package.xml`). The repository is public for reference, but no open-source licence is granted.
