# delivery_bot_map_server

Map server and map tools: own map format, latched /map and the LoadMap service.

| | |
| --- | --- |
| Card | [delivery_bot_core#12](https://github.com/leonst2/delivery_bot_core/issues/12) |
| Language | Python (rclpy) |
| Executable | `map_server` |

**Status: stub.** `map_server` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat` (through
`HeartbeatPublisher` from `delivery_bot_bringup`), so the stack and its health monitor run. The card replaces it
with the real node.
