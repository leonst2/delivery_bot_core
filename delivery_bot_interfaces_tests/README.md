# delivery_bot_interfaces_tests

Checks that the C++ and Python bindings of `delivery_bot_interfaces` construct, serialise and deserialise messages
(gtest and pytest). It's a separate package so that `delivery_bot_interfaces` itself stays a pure rosidl package. The
Rust check is a unit test in `delivery_bot_step_controller`.

Card: [delivery_bot_core#3](https://github.com/leonst2/delivery_bot_core/issues/3).
