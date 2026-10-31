# delivery_bot_interfaces

Every custom message, service and action of the stack. C++, Rust and Python bindings are generated at build time,
and the simulation generates C# from these files. Changing a definition means updating the cards that use it.

Units are SI. Frames are stated where a message has its own frame rule.

## Messages

| Message | Fields | Units and frames |
| --- | --- | --- |
| `AckermannCommand` | `header`, `speed`, `steering_angle` | m/s (+ forward, at the rear-axle centre), rad (+ left, bicycle model) |
| `GridStepState` | `header`, `state`, `state_name`, `step_index`, `replans`, `yields`, `goal`, `detail` | `state` is one of `IDLE`, `HALT`, `PERCEIVE`, `UPDATE_MAP`, `PLAN`, `YIELD`, `MOVE_ONE_STEP`, `DONE`, `FAILED` |
| `TrackedObstacle` | `id`, `position`, `velocity`, `radius`, `position_covariance`, `predicted_position`, `prediction_horizon`, `age` | m, m/s, m, m² (row-major 2×2 for x, y), m, s, s. Frame from the array header |
| `TrackedObstacleArray` | `header`, `obstacles` | frame `map` |
| `CollisionEvent` | `header`, `robot_link`, `other_object`, `other_tag`, `impulse` | N·s |
| `Heartbeat` | `header`, `node`, `status`, `detail` | `status` is one of `OK`, `WARN`, `ERROR` |

## Services

| Service | Request | Response |
| --- | --- | --- |
| `LoadMap` | `map_path` | `success`, `message` |
| `ResetSim` | `robot_pose` (frame `map`), `seed` | `success`, `message` |
| `LoadScenario` | `scenario_name`, `seed` | `success`, `message` |

## Actions

| Action | Goal | Result | Feedback |
| --- | --- | --- | --- |
| `NavigateToGoal` | `goal`, `goal_tolerance` (m) | `success`, `error_code`, `message`, `steps`, `replans`, `yields` | `state`, `distance_remaining` (m) |
| `PlanPath` | `start` (empty frame = current pose), `goal`, `time_limit` (s) | `path`, `cost`, `planning_time` (s), `error_code` | `expanded_nodes` |
| `ExecuteStep` | `path`, `step_size` (m), `max_speed` (m/s) | `success`, `distance_travelled` (m), `final_cross_track_error` (m), `error_code` | `distance_travelled` (m) |

Each action's result defines its `ERROR_` constants, with `ERROR_NONE=0` for success.

## Tests

Each language has a check that constructs and converts a message: C++ and Python in `delivery_bot_interfaces_tests`,
Rust in `delivery_bot_rust_hello`.
