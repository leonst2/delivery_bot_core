// Fast re-plans (delivery_bot_core#17): reuse the previous path while it is still valid.

#ifndef DELIVERY_BOT_PLANNER_REPLAN_HPP_
#define DELIVERY_BOT_PLANNER_REPLAN_HPP_

#include <array>
#include <optional>

#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "delivery_bot_planner/hybrid_astar.hpp"

namespace delivery_bot_planner
{

struct ReuseParams
{
  double max_offset{0.3};                           // m, robot to the previous path
  double max_heading_error{0.26};                   // rad (15 deg)
  std::array<double, 3> join_ahead{1.5, 3.0, 5.0};  // m along the previous path where the robot may rejoin it
  double max_stretch{1.3};                          // the joining curve may be at most this much longer than join_ahead
};

/// The part of `path` from arc length s on (s clamped to the path).
delivery_bot_geometry::Path suffix(const delivery_bot_geometry::Path & path, double s);

/// The previous path from `start`, if it can be reused: the goal is unchanged, `start` lies within the offset and
/// heading limits of the path, a short cusp-free Reeds-Shepp curve joins `start` to the path a little further on, and
/// the joined path is still collision-free on the current map. nullopt otherwise (plan from scratch).
std::optional<delivery_bot_geometry::Path> reuse_previous(
  const HybridAStar & planner, const delivery_bot_geometry::Path & previous,
  const delivery_bot_geometry::Pose2 & previous_goal, const delivery_bot_geometry::Pose2 & start,
  const delivery_bot_geometry::Pose2 & goal, const ReuseParams & params = {});

}  // namespace delivery_bot_planner

#endif  // DELIVERY_BOT_PLANNER_REPLAN_HPP_
