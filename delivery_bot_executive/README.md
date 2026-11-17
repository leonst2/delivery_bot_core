# delivery_bot_executive

Grid-step executive (NavigateToGoal: halt, perceive, map, plan, yield check, move one step) and the mission manager (multi-stop routes, goal queue, cancel and abort).

| | |
| --- | --- |
| Card | [delivery_bot_core#21](https://github.com/leonst2/delivery_bot_core/issues/21), [delivery_bot_core#22](https://github.com/leonst2/delivery_bot_core/issues/22) |
| Language | Python (rclpy) |
| Executable | `executive` |

**Status: stub.** `executive` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat` (through
`HeartbeatPublisher` from `delivery_bot_bringup`), so the stack and its health monitor run. The card replaces it
with the real node.
