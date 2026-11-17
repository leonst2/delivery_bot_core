# delivery_bot_collision

Footprint, inflation and collision-checking library, shared by the planner, the controller and the safety supervisor.

| | |
| --- | --- |
| Card | [delivery_bot_core#15](https://github.com/leonst2/delivery_bot_core/issues/15) |
| Language | C++ library (no node) |
| Used by | the planner, the step controller and the safety supervisor |

```cmake
find_package(delivery_bot_collision REQUIRED)
target_link_libraries(my_target delivery_bot_collision::delivery_bot_collision)
```

**Status: stub.** `include/delivery_bot_collision/collision.hpp` is empty until the card lands.
