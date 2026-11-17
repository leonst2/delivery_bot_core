# delivery_bot_geometry

Geometry and motion-primitive library: SE(2), Reeds-Shepp and Dubins curves, Ackermann motion primitives.

| | |
| --- | --- |
| Card | [delivery_bot_core#16](https://github.com/leonst2/delivery_bot_core/issues/16) |
| Language | C++ library (no node) |
| Used by | the planner, the step controller and the collision checker |

```cmake
find_package(delivery_bot_geometry REQUIRED)
target_link_libraries(my_target delivery_bot_geometry::delivery_bot_geometry)
```

**Status: stub.** `include/delivery_bot_geometry/geometry.hpp` is empty until the card lands.
