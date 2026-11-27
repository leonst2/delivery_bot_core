// Hybrid-A* path planner for the Ackermann robot (delivery_bot_core#17).
//
// Searches over (x, y, heading bin, direction) with the motion primitives of delivery_bot_geometry, checks every
// motion with the conservative swept check of delivery_bot_collision and closes the gap to the goal with an
// obstacle-checked Reeds-Shepp shot, so the path ends exactly on the goal pose. Deterministic: the open list is
// ordered by (f, h, insertion order) and no hash-table iteration order reaches the result.

#ifndef DELIVERY_BOT_PLANNER_HYBRID_ASTAR_HPP_
#define DELIVERY_BOT_PLANNER_HYBRID_ASTAR_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/primitives.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "delivery_bot_planner/heuristic.hpp"
#include "delivery_bot_planner/planner_params.hpp"

namespace delivery_bot_planner
{

/// Same values as the ERROR_ constants of delivery_bot_interfaces/action/PlanPath, plus kCancelled.
enum class PlanError : std::uint8_t {
  kNone = 0,
  kNoPath = 1,
  kTimeout = 2,
  kInvalidStart = 3,
  kInvalidGoal = 4,
  kCancelled = 255,
};

struct PlanResult
{
  PlanError error{PlanError::kNoPath};
  delivery_bot_geometry::Path path;
  double cost{0.0};         // in the planner's cost units (metres of driving plus penalties)
  std::size_t expanded{0};  // search nodes expanded
};

/// How a search may be stopped early. Both are optional.
struct PlanLimits
{
  std::optional<std::chrono::steady_clock::time_point> deadline;
  std::function<bool()> cancelled;
  std::function<void(std::size_t expanded)> progress;  // called now and then with the expansion count
};

class HybridAStar
{
public:
  /// Throws std::invalid_argument for invalid parameters.
  HybridAStar(std::shared_ptr<const delivery_bot_collision::CollisionChecker> checker, PlannerParams params = {});

  [[nodiscard]] PlanResult plan(
    const delivery_bot_geometry::Pose2 & start, const delivery_bot_geometry::Pose2 & goal,
    const PlanLimits & limits = {});

  /// Cost of driving `segment` from `start` after a motion with direction `previous_direction` (0 = none yet) and
  /// curvature `previous_curvature`, as the search counts it. Exposed for tests and for re-plans.
  [[nodiscard]] double segment_cost(
    const delivery_bot_geometry::Pose2 & start, const delivery_bot_geometry::Segment & segment, int previous_direction,
    double previous_curvature) const;

  /// Total cost of a path, as segment_cost summed over its segments.
  [[nodiscard]] double path_cost(const delivery_bot_geometry::Path & path) const;

  [[nodiscard]] const PlannerParams & params() const { return params_; }
  [[nodiscard]] const delivery_bot_collision::CollisionChecker & checker() const { return *checker_; }
  [[nodiscard]] const std::vector<delivery_bot_geometry::MotionPrimitive> & primitives() const { return primitives_; }

private:
  [[nodiscard]] const DistanceHeuristic & heuristic_for(const delivery_bot_geometry::Pose2 & goal);
  [[nodiscard]] double cost_at(const delivery_bot_geometry::Pose2 & pose) const;

  std::shared_ptr<const delivery_bot_collision::CollisionChecker> checker_;
  PlannerParams params_;
  std::vector<delivery_bot_geometry::MotionPrimitive> primitives_;
  double max_curvature_{0.0};
  double base_clearance_{0.0};
  Eigen::Vector2d footprint_centre_{0.0, 0.0};

  // Heuristic cache: re-plans to the same goal cell reuse the Dijkstra grid.
  std::unique_ptr<DistanceHeuristic> heuristic_;
  int heuristic_goal_x_{0};
  int heuristic_goal_y_{0};
};

}  // namespace delivery_bot_planner

#endif  // DELIVERY_BOT_PLANNER_HYBRID_ASTAR_HPP_
