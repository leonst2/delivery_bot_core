# delivery_bot_stereo

Stereo depth from scratch: disparity and 3D points (/stereo/points).

| | |
| --- | --- |
| Card | [delivery_bot_core#10](https://github.com/leonst2/delivery_bot_core/issues/10) |
| Language | C++ (rclcpp) |
| Executable | `stereo` |

**Status: stub.** `stereo` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node.
