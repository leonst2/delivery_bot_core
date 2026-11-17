# delivery_bot_planner

Hybrid-A* path planner for an Ackermann robot (PlanPath action), plus path smoothing, resampling, curvature validation and step segmentation.

| | |
| --- | --- |
| Card | [delivery_bot_core#17](https://github.com/leonst2/delivery_bot_core/issues/17), [delivery_bot_core#18](https://github.com/leonst2/delivery_bot_core/issues/18) |
| Language | C++ (rclcpp) |
| Executable | `planner` |

**Status: stub.** `planner` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node.
