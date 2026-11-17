# delivery_bot_localization

Particle-filter localization (MCL) against the static map (map → odom). The stack also starts it as ground_truth_localization for simulation runs.

| | |
| --- | --- |
| Card | [delivery_bot_core#8](https://github.com/leonst2/delivery_bot_core/issues/8) |
| Language | Rust (rclrs) |
| Executable | `localization` |

**Status: stub.** `localization` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node. Keep `build.rs`: every Rust node needs it (see the
top-level README).
