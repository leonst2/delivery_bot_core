# delivery_bot_costmap

Layered costmap: global and rolling local costmaps (static, obstacle and dynamic-prediction layers).

| | |
| --- | --- |
| Card | [delivery_bot_core#14](https://github.com/leonst2/delivery_bot_core/issues/14) |
| Language | C++ (rclcpp) |
| Executable | `costmap` |

**Status: stub.** `costmap` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node.
