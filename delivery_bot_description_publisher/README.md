# delivery_bot_description_publisher

Own URDF parser that publishes /robot_description, /tf_static and the joint-driven /tf.

| | |
| --- | --- |
| Card | [delivery_bot_core#5](https://github.com/leonst2/delivery_bot_core/issues/5) |
| Language | C++ (rclcpp) |
| Executable | `description_publisher` |

**Status: stub.** `description_publisher` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat`, so the stack and its
health monitor run. The card replaces it with the real node.
