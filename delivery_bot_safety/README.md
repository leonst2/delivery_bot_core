# delivery_bot_safety

Safety supervisor: emergency stop from the scan, command watchdog and gating.

| | |
| --- | --- |
| Card | [delivery_bot_core#20](https://github.com/leonst2/delivery_bot_core/issues/20) |
| Language | C++ (rclcpp) |
| Executable | `safety` |

**Status: stub.** `safety` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node.
