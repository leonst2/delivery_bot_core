# delivery_bot_step_controller

Step controller (ExecuteStep action): follows one step_size arc and stops.

| | |
| --- | --- |
| Card | [delivery_bot_core#19](https://github.com/leonst2/delivery_bot_core/issues/19) |
| Language | Rust (rclrs) |
| Executable | `step_controller` |

**Status: stub.** `step_controller` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node. Keep `build.rs`: every Rust node needs it (see the
top-level README).
