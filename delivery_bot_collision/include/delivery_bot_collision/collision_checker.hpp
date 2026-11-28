// Collision checks of the robot footprint against an occupancy grid (delivery_bot_core#15).
//
// A pose is in collision if the footprint polygon overlaps the square of any obstacle cell (touching doesn't count),
// or, when the rule treats unknown as lethal, if the footprint reaches outside the map. The check is exact for every
// yaw: a fast path on the distance field clears poses far from obstacles, everything else is tested cell by cell with
// the separating-axis theorem.

#ifndef DELIVERY_BOT_COLLISION_COLLISION_CHECKER_HPP_
#define DELIVERY_BOT_COLLISION_COLLISION_CHECKER_HPP_

#include <cstddef>
#include <vector>

#include "delivery_bot_collision/distance_field.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_collision/grid.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_collision
{

/// Most vertices a footprint may have (the padded sweep footprint of a rectangle has four).
constexpr std::size_t kMaxFootprintVertices = 16;

/// Swept checks: refine the sampling towards the start and/or the end of a segment (or path). Use it where the robot
/// may be closer to an obstacle than sweep_step / 2, e.g. the pose it is standing at or a goal next to a wall.
struct SweepRefine
{
  bool start{false};
  bool end{false};
};

/// Refinement levels: level j pads the footprint by sweep_step / 2 / 2^j. Level 0 is the regular sweep padding.
constexpr int kSweepLevels = 6;

/// A pose a swept check tests, and the padding level it is tested with.
struct SweepSample
{
  delivery_bot_geometry::Pose2 pose;
  int level{0};
};

struct CheckerParams
{
  OccupancyRule rule;
  int cover_circles{3};     // circles of the fast path
  double sweep_step{0.05};  // m, swept checks: max movement of any footprint point between two sampled poses
};

class CollisionChecker
{
public:
  /// `footprint` is used as given (pad it first, e.g. FootprintParams::make()). Computes the distance field once.
  /// Throws std::invalid_argument for more than kMaxFootprintVertices vertices, cover_circles < 1 or
  /// sweep_step <= 0.
  CollisionChecker(Grid grid, Footprint footprint, CheckerParams params = {});

  [[nodiscard]] const Grid & grid() const { return grid_; }
  [[nodiscard]] const Footprint & footprint() const { return body_.polygon; }
  [[nodiscard]] const DistanceField & distance_field() const { return field_; }
  [[nodiscard]] const CheckerParams & params() const { return params_; }

  /// Is the footprint at `pose` (base_link in the map frame) in collision?
  [[nodiscard]] bool in_collision(const delivery_bot_geometry::Pose2 & pose) const;
  /// The same answer without the fast path (reference and tests).
  [[nodiscard]] bool in_collision_exact(const delivery_bot_geometry::Pose2 & pose) const;

  /// A lower bound (m, >= 0) of the distance between the footprint at `pose` and the nearest obstacle (or the map
  /// border when unknown is lethal), from the circle cover and the distance field. 0 if it may be in collision.
  [[nodiscard]] double clearance_lower_bound(const delivery_bot_geometry::Pose2 & pose) const;

  /// Does driving `segment` from `start` ever collide? Conservative: the poses along the segment are sampled so that
  /// no footprint point moves more than sweep_step between two samples, and each sample is checked with the footprint
  /// padded by half the movement to its neighbours (sweep_step / 2 between regular samples). The padded samples cover
  /// the whole swept area, so no obstacle is ever missed; the price is up to sweep_step / 2 of extra margin.
  ///
  /// With `refine`, extra samples halve the gap step by step towards the start and/or end, down to kSweepLevels
  /// levels: the pose there is then checked with only sweep_step / 2^(kSweepLevels + 1) of padding (0.4 mm by
  /// default), still conservatively. Without it, a robot standing within sweep_step / 2 of a wall can't move at all.
  [[nodiscard]] bool segment_in_collision(
    const delivery_bot_geometry::Pose2 & start, const delivery_bot_geometry::Segment & segment,
    SweepRefine refine = {}) const;
  /// segment_in_collision for every segment of the path, refining the path's start and end as asked (an empty path
  /// checks its start pose).
  [[nodiscard]] bool path_in_collision(const delivery_bot_geometry::Path & path, SweepRefine refine = {}) const;

  /// The poses a swept check of `segment` from `start` tests, with their padding levels (for tests and debugging).
  [[nodiscard]] std::vector<SweepSample> sweep_samples(
    const delivery_bot_geometry::Pose2 & start, const delivery_bot_geometry::Segment & segment,
    SweepRefine refine = {}) const;

private:
  struct Shape
  {
    Footprint polygon;
    std::vector<Circle> circles;
  };

  [[nodiscard]] bool check(const Shape & shape, const delivery_bot_geometry::Pose2 & pose) const;
  [[nodiscard]] bool fast_free(const Shape & shape, const delivery_bot_geometry::Pose2 & pose) const;
  [[nodiscard]] bool exact(const Footprint & polygon, const delivery_bot_geometry::Pose2 & pose) const;

  Grid grid_;
  CheckerParams params_;
  DistanceField field_;
  Shape body_;
  std::vector<Shape> swept_;  // level j: body padded by sweep_step / 2 / 2^j
};

}  // namespace delivery_bot_collision

#endif  // DELIVERY_BOT_COLLISION_COLLISION_CHECKER_HPP_
