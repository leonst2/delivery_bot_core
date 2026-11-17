# delivery_bot_odometry

Wheel and IMU odometry with its own EKF (/odom and odom → base_link).

| | |
| --- | --- |
| Card | [delivery_bot_core#7](https://github.com/leonst2/delivery_bot_core/issues/7) |
| Language | Rust (rclrs) |
| Executable | `odometry` |

**Status: stub.** `odometry` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node. Keep `build.rs`: every Rust node needs it (see the
top-level README).
