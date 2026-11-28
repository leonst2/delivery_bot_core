#include "delivery_bot_planner/replan.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/arc.hpp"
#include "delivery_bot_geometry/reeds_shepp.hpp"

namespace delivery_bot_planner
{

namespace dbg = delivery_bot_geometry;

dbg::Path suffix(const dbg::Path & path, double s)
{
  if (path.empty() || s >= path.length()) {
    return {path.end(), {}};
  }
  const double clamped = std::max(s, 0.0);
  const std::size_t i = path.segment_index_at(clamped);
  const dbg::Segment & first = path.segments()[i];
  const double remaining = path.segment_start_s(i + 1) - clamped;
  std::vector<dbg::Segment> segments{{first.curvature, first.direction() * remaining}};
  segments.insert(segments.end(), path.segments().begin() + static_cast<std::ptrdiff_t>(i) + 1, path.segments().end());
  return {path.pose_at(clamped), std::move(segments)};
}

std::optional<dbg::Path> reuse_previous(
  const HybridAStar & planner, const dbg::Path & previous, const dbg::Pose2 & previous_goal, const dbg::Pose2 & start,
  const dbg::Pose2 & goal, const ReuseParams & params)
{
  if (previous.empty() || !dbg::approx_equal(previous_goal, goal, 1e-6)) {
    return std::nullopt;
  }
  const dbg::TrackingError error = dbg::tracking_error(previous, start);
  if (
    std::abs(error.cross_track) > params.max_offset || dbg::distance(error.reference, start) > params.max_offset ||
    std::abs(error.heading) > params.max_heading_error) {
    return std::nullopt;
  }
  const double radius = planner.params().motion.min_turning_radius;
  const int direction = previous.direction_at(error.s);
  for (const double ahead : params.join_ahead) {
    const double s_join = error.s + ahead;
    if (s_join >= previous.length()) {
      break;  // too close to the end to rejoin: a fresh plan is cheap there anyway
    }
    // Rejoin within the same driving direction, before any cusp of the old path.
    bool cusp_between = false;
    for (std::size_t i = previous.segment_index_at(error.s); i < previous.segments().size(); ++i) {
      if (previous.segment_start_s(i) > s_join) {
        break;
      }
      cusp_between = cusp_between || previous.segments()[i].direction() != direction;
    }
    if (cusp_between) {
      break;
    }
    const dbg::CurveCandidate join = dbg::reeds_shepp_shortest(start, previous.pose_at(s_join), radius);
    if (
      join.path.cusp_count() > 0 || join.path.length() > params.max_stretch * ahead ||
      (!join.path.empty() && join.path.direction_at(0.0) != direction)) {
      continue;
    }
    const dbg::Path rest = suffix(previous, s_join);
    std::vector<dbg::Segment> segments = join.path.segments();
    segments.insert(segments.end(), rest.segments().begin(), rest.segments().end());
    dbg::Path joined(start, std::move(segments));
    // The map may have changed since the previous plan.
    if (!planner.checker().path_in_collision(joined, {true, true})) {
      return joined;
    }
  }
  return std::nullopt;
}

}  // namespace delivery_bot_planner
