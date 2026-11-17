# delivery_bot_scan_filter

Cleans the LiDAR scans and publishes obstacle points in base_link.

| | |
| --- | --- |
| Card | [delivery_bot_core#9](https://github.com/leonst2/delivery_bot_core/issues/9) |
| Language | C++ (rclcpp) |
| Executable | `scan_filter` |

**Status: stub.** `scan_filter` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node.
