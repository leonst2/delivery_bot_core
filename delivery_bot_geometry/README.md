# delivery_bot_geometry

Geometry and motion-primitive library: SE(2) poses, constant-curvature paths, Dubins and Reeds-Shepp curves,
Ackermann motion primitives and arc utilities for path tracking. The robot can't turn on the spot (minimum turning
radius ≈ 1.25 m), so planning and control work on these curves.

| | |
| --- | --- |
| Card | [delivery_bot_core#16](https://github.com/leonst2/delivery_bot_core/issues/16) |
| Language | C++ library (no node, Eigen only, no ROS dependency), plus a Rust port of the arc utilities |
| Used by | the planner (#17, #18), the step controller (#19) and the collision checker (#15) |

```cmake
find_package(delivery_bot_geometry REQUIRED)
target_link_libraries(my_target delivery_bot_geometry::delivery_bot_geometry)
```

```cpp
#include "delivery_bot_geometry/geometry.hpp"  // or the single headers below
```

## Conventions

- Units m and rad. Yaw + = counter-clockwise (ROS FLU). Every operation that produces a pose normalises the yaw to
  (-π, π].
- A **segment** has a constant **steering curvature** (1/m, + = steering left) and a **signed length** (< 0 =
  reverse; the robot keeps facing the same way). Driving a length `l` changes the yaw by `curvature · l`, so reversing
  with the wheels turned left turns the robot clockwise.
- `s` is the **travelled arc length** along a path: always ≥ 0 and growing in both directions. A point on a segment
  boundary belongs to the following segment (the end point to the last segment).
- **Deterministic:** candidates are generated in a fixed order and ties go to the earliest one, so the same inputs
  always give bitwise identical results (needed for the planner's byte-identical paths).

## API

| Header | Contents |
| --- | --- |
| `angle.hpp` | `normalize_angle` (-π, π], `normalize_angle_positive` [0, 2π), `angle_diff`, `interpolate_angle` |
| `se2.hpp` | `Pose2`: composition `a * b`, `inverse`, `transform` / `inverse_transform` of points; `between`, `interpolate`, `distance`, `approx_equal` |
| `path.hpp` | `Segment`, `advance(pose, curvature, length)`, `Path`: `length`, `end`, `pose_at(s)`, `curvature_at(s)`, `direction_at(s)`, `cusp_count`, `sample(resolution)`; `CurveCandidate` (word + path) |
| `dubins.hpp` | `dubins_all`, `dubins_shortest`: forward-only, the six words LSL RSR LSR RSL RLR LRL |
| `reeds_shepp.hpp` | `reeds_shepp_all`, `reeds_shepp_shortest`: forward and reverse, word families CSC, CCC, CCCC, CCSC, CCSCC |
| `primitives.hpp` | `AckermannParams`, `make_primitives`: forward and reverse arcs of a fixed length at evenly spaced steering angles up to `R_min`; steering ↔ curvature conversions |
| `arc.hpp` | `point_at`, `closest_point` (exact, optionally limited to an arc-length window), `cross_track_error`, `heading_error`, `tracking_error` |

`advance` uses the chord form `l · sinc(κl/2)`, so it needs no special case for straight lines and stays exact for
curvature → 0. `Path::sample` keeps every segment boundary as a sample, so cusps are never skipped.

### Ackermann parameters

| Field | Default | |
| --- | --- | --- |
| `wheelbase` | 0.49 m | URDF |
| `min_turning_radius` | 1.25 m | at the rear-axle centre; with the wheelbase this is the 21.4° steering limit |
| `arc_length` | 0.5 m | travelled length of every primitive |
| `steering_count` | 5 | odd: −max … 0 … +max |
| `allow_reverse` | true | adds the same arcs driven in reverse |

The nodes read their own ROS parameters and fill these structs; the library has no parameters of its own.

## Rust

`rust/` is a dependency-free crate with the parts the step controller needs: `Pose2`, `Segment`, `Path`, `advance`,
`point_at`, `closest_point`, `cross_track_error`, `heading_error`, `tracking_error`, `normalize_angle`. It isn't a
colcon package (`COLCON_IGNORE`); a Rust node uses it as a path dependency:

```toml
[dependencies]
delivery_bot_geometry = { path = "../delivery_bot_geometry/rust" }
```

Points are `(f64, f64)` tuples, and "no upper window limit" is `f64::INFINITY`.

## Tests

`colcon test --packages-select delivery_bot_geometry` runs the gtest suite. `cargo test` in `rust/` runs the Rust
tests (CI does both). The property tests use fixed seeds and check:

- SE(2): inverse, identity, associativity, `between`, point transforms, interpolation across ±π.
- Dubins and Reeds-Shepp: every candidate ends on the goal (< 1e-6), length ≥ the straight-line distance, the
  shortest is the minimum of all candidates, Reeds-Shepp ≤ forward- and reverse-only Dubins, invariance under rigid
  motion, mirroring, swapping start and goal (Reeds-Shepp) and turning both poses around, the **triangle
  inequality** (it fails if a word family is removed), and bitwise determinism.
- Hand-derived reference cases with closed-form lengths: straight ahead and back, the four quarter circles, the half
  circle, quarter circle plus straight, goal behind (Dubins 2πR + d, Reeds-Shepp d in reverse), parallel parking
  (Reeds-Shepp needs a cusp). These replace the literature tables named in the card.
- Primitives: count, fixed arc length, curvature ≤ 1/R_min, left/right mirror pairs, reverse = inverse of forward.
- Arc utilities: the exact closest point against a dense brute-force search (segment boundaries included), windows,
  ties, laps of long arcs, cusps, and the sign of the cross-track error.

### Shared test vectors

`test/vectors/arc_vectors.txt` holds angle, SE(2), `advance`, `pose_at`, closest-point and tracking-error records
with their expected values. Both the gtest (`test_vectors.cpp`) and the Rust test (`rust/tests/vectors.rs`) recompute
every record and must agree to 1e-9, so the two implementations can't drift apart. The file is generated by the C++
library; regenerate it only when the formulas change on purpose:

```bash
build/delivery_bot_geometry/generate_arc_vectors > src/delivery_bot_core/delivery_bot_geometry/test/vectors/arc_vectors.txt
```

Tests have their own `.clang-tidy` (`test/`), which only turns off the cognitive-complexity check that gtest's
assertion macros trip.
