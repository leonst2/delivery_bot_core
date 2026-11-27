# delivery_bot_collision

Footprint, inflation and collision-checking library: the one implementation of "does the robot collide here?",
used by the costmap (#14), the planner (#17), path smoothing (#18) and the safety supervisor (#20).

| | |
| --- | --- |
| Card | [delivery_bot_core#15](https://github.com/leonst2/delivery_bot_core/issues/15) |
| Language | C++ library (no node, no ROS; Eigen and `delivery_bot_geometry`) |
| Used by | the costmap, the planner, the step controller and the safety supervisor |

```cmake
find_package(delivery_bot_collision REQUIRED)
target_link_libraries(my_target delivery_bot_collision::delivery_bot_collision)
```

```cpp
#include "delivery_bot_collision/collision.hpp"  // or the single headers below
```

## What counts as a collision

A pose (base_link in the map frame) is **in collision** if the footprint polygon overlaps the square of any
obstacle cell (touching doesn't count), or, when unknown is lethal, if the footprint reaches outside the map.

- A cell is an **obstacle** if its value is ≥ `occupied_threshold` (default 65), or if it is unknown (−1) and the
  rule says `UnknownPolicy::kLethal`. Outside the map counts as unknown.
- Grids use the `nav_msgs/OccupancyGrid` layout (row-major, cell (0, 0) at the lower-left, origin at its lower-left
  corner, yaw 0), so a node copies the message data straight into `Grid`.

## API

| Header | Contents |
| --- | --- |
| `grid.hpp` | `Grid` (size, resolution, origin, `int8` data, `world_to_cell`, `cell_center`), `OccupancyRule`, `UnknownPolicy` |
| `distance_field.hpp` | `compute_distance_field`: exact Euclidean distance from every cell centre to the nearest obstacle cell centre (Felzenszwalb–Huttenlocher, O(cells)). With unknown lethal, the outside of the map counts as obstacle too. |
| `footprint.hpp` | `Footprint` (convex polygon in base_link): `rectangle`, `padded` (mitred), `inscribed_radius`, `circumscribed_radius`, `circle_cover`; `FootprintParams` |
| `inflation.hpp` | `inflation_cost(distance)`, `inflate(grid, field)` → costs 0 … 100, unknown −1 (the `/costmap/*` contract) |
| `collision_checker.hpp` | `CollisionChecker`: `in_collision(pose)`, `clearance_lower_bound(pose)`, `segment_in_collision(start, segment)`, `path_in_collision(path)` |

### Footprint

| `FootprintParams` | Default | |
| --- | --- | --- |
| `x_min`, `x_max` | −0.085, 0.575 m | URDF chassis collision box around base_link (rear-axle origin) |
| `y_min`, `y_max` | −0.27, 0.27 m | the box also contains all four wheels |
| `padding` | 0.05 m | added on every side |

The card named 0.70 × 0.50 m; the URDF box is 0.66 × 0.54 m, so the card's width would be narrower than the robot.
The defaults follow the URDF.

### Inflation

| `InflationParams` | Default | |
| --- | --- | --- |
| `inscribed_radius` | 0.32 m | lethal (100) up to here: the inscribed radius of the padded default footprint |
| `inflation_radius` | 1.0 m | cost reaches 0 here |
| `decay` | 3.0 1/m | exponential decay in between, rescaled to end at exactly 0, rounded up |

### Pose checks

`in_collision` is exact for every yaw (no yaw-binned masks):

1. **Fast path:** the footprint is covered by `cover_circles` circles (default 3). If every circle centre is farther
   than its radius + resolution · √2 from any obstacle on the distance field, the pose is free.
2. **Exact:** otherwise every obstacle cell in the footprint's bounding box is tested against the rotated polygon with
   the separating-axis theorem.

Throughput on a 100 × 100 m city-like map at 0.1 m with poses on the streets (37 % in collision): about 2.6 M
checks/s (Release, arm64). The package builds as Release when no build type is given.

### Swept checks

`segment_in_collision` samples the segment so that no footprint point moves more than `sweep_step` (default 0.05 m)
between two samples, and checks each sample with the footprint padded by `sweep_step / 2`. Every point of the swept
area is within `sweep_step / 2` of a sample, so the padded samples cover it: **no obstacle is ever missed**, also thin
ones. The price is up to 2.5 cm of extra margin in tight squeezes. Without that padding, the adversarial tests below
fail (a footprint corner clips a cell between samples).

## Tests

`colcon test --packages-select delivery_bot_collision`. All tests use fixed seeds.

- **Distance field:** every cell against a brute force over all cell pairs on 60 random grids under three rules,
  plus sparse, empty and full grids.
- **Footprint:** URDF defaults, radii, padding contains every point within the padding distance, the circle cover
  contains the polygon.
- **Inflation:** lethal up to the inscribed radius, monotone, exactly 0 at the inflation radius, unknown kept.
- **Pose checks:** against an independent reference (Sutherland–Hodgman clipping of every obstacle square) on random
  grids and footprints, 72 evenly spaced plus random yaws, map borders, unknown cells, touching, padding monotonicity,
  and the clearance bound against true polygon distances.
- **Swept checks:** cells placed exactly between two samples, 5 cm walls across the path at any angle, Reeds-Shepp
  paths with cusps, all against dense exact checks every millimetre.
- **Throughput:** ≥ 1 M checks/s, asserted in optimised builds.
