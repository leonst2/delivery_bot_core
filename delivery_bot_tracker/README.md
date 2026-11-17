# delivery_bot_tracker

Dynamic obstacle detection and tracking with prediction (/tracked_obstacles).

| | |
| --- | --- |
| Card | [delivery_bot_core#11](https://github.com/leonst2/delivery_bot_core/issues/11) |
| Language | Rust (rclrs) |
| Executable | `tracker` |

**Status: stub.** `tracker` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node. Keep `build.rs`: every Rust node needs it (see the
top-level README).
