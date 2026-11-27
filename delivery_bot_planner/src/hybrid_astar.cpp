#include "delivery_bot_planner/hybrid_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/reeds_shepp.hpp"

namespace delivery_bot_planner
{

namespace dbg = delivery_bot_geometry;
namespace dbc = delivery_bot_collision;

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::size_t kCheckEvery = 64;  // expansions between deadline / cancel checks

struct Node
{
  dbg::Pose2 pose;
  double g{0.0};
  std::int32_t parent{-1};
  dbg::Segment segment;   // motion from the parent
  int direction{0};       // of that motion, 0 for the start
  std::int32_t shot{-1};  // >= 0: a goal candidate, reached by shots[shot] from `parent`
};

struct OpenEntry
{
  double f;
  double h;
  std::uint64_t order;  // insertion counter: the last tie-breaker
  std::int32_t node;
};

struct OpenOrder
{
  bool operator()(const OpenEntry & a, const OpenEntry & b) const
  {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    if (a.h != b.h) {
      return a.h > b.h;
    }
    return a.order > b.order;
  }
};

struct Visit
{
  double g;
  std::int32_t node;
  bool closed;
};

// One search from start to goal. Uses only the public interface of the planner.
class Search
{
public:
  Search(
    const HybridAStar & planner, const DistanceHeuristic & distance, const dbg::Pose2 & start, const dbg::Pose2 & goal)
  : planner_(planner),
    params_(planner.params()),
    distance_(distance),
    start_(start),
    goal_(goal),
    radius_(planner.params().motion.min_turning_radius),
    yaw_bin_(dbg::kTwoPi / planner.params().yaw_bins)
  {
  }

  PlanResult run(const PlanLimits & limits)
  {
    PlanResult result;
    const double h0 = heuristic(start_);
    if (!std::isfinite(h0)) {
      result.error = PlanError::kNoPath;
      return result;
    }
    nodes_.push_back({start_, 0.0, -1, {}, 0, -1});
    visits_[key(start_, 0)] = {0.0, 0, false};
    open_.push({h0, h0, order_++, 0});

    while (!open_.empty()) {
      const OpenEntry entry = open_.top();
      open_.pop();
      const Node & node = nodes_[static_cast<std::size_t>(entry.node)];
      if (node.shot >= 0) {
        return finish(entry.node);  // the cheapest goal candidate is cheaper than anything still open
      }
      Visit & visit = visits_[key(node.pose, node.direction)];
      if (visit.closed || visit.node != entry.node) {
        continue;  // stale entry: the cell was reached more cheaply later
      }
      visit.closed = true;
      ++expanded_;
      if (const auto stop = check_limits(limits)) {
        result.error = *stop;
        result.expanded = expanded_;
        return result;
      }
      if (expanded_ > params_.max_expansions) {
        break;
      }
      try_shot(entry.node);
      expand(entry.node);
    }
    result.error = PlanError::kNoPath;
    result.expanded = expanded_;
    return result;
  }

private:
  double heuristic(const dbg::Pose2 & pose) const
  {
    const double d = distance_.at(pose.x, pose.y);
    if (!std::isfinite(d)) {
      return kInf;
    }
    return params_.heuristic_weight * std::max(d, dbg::reeds_shepp_length(pose, goal_, radius_));
  }

  // Lattice cell of a pose: (x, y) cell, heading bin, direction.
  std::uint64_t key(const dbg::Pose2 & pose, int direction) const
  {
    const auto ix = static_cast<std::int64_t>(std::floor(pose.x / params_.xy_resolution));
    const auto iy = static_cast<std::int64_t>(std::floor(pose.y / params_.xy_resolution));
    const auto iyaw =
      static_cast<std::int64_t>(std::lround(dbg::normalize_angle_positive(pose.yaw) / yaw_bin_)) % params_.yaw_bins;
    constexpr std::int64_t kOffset = std::int64_t{1} << 20;
    return (static_cast<std::uint64_t>(ix + kOffset) << 40U) | (static_cast<std::uint64_t>(iy + kOffset) << 19U) |
           (static_cast<std::uint64_t>(iyaw) << 1U) | (direction < 0 ? 1U : 0U);
  }

  std::optional<PlanError> check_limits(const PlanLimits & limits) const
  {
    if (expanded_ % kCheckEvery != 0) {
      return std::nullopt;
    }
    if (limits.cancelled && limits.cancelled()) {
      return PlanError::kCancelled;
    }
    if (limits.deadline && std::chrono::steady_clock::now() > *limits.deadline) {
      return PlanError::kTimeout;
    }
    if (limits.progress) {
      limits.progress(expanded_);
    }
    return std::nullopt;
  }

  // Analytic expansion: an obstacle-free Reeds-Shepp shot reaches the exact goal pose. It becomes a goal candidate in
  // the open list with its full cost, so a cheaper way (fewer cusps, less reversing) still wins.
  void try_shot(std::int32_t index)
  {
    const Node node = nodes_[static_cast<std::size_t>(index)];
    const bool near = dbg::distance(node.pose, goal_) <= params_.analytic_radius;
    if (!near && (expanded_ - 1) % static_cast<std::size_t>(params_.analytic_every) != 0) {
      return;
    }
    dbg::CurveCandidate shot = dbg::reeds_shepp_shortest(node.pose, goal_, radius_);
    // Far from the goal, a shortest Reeds-Shepp curve with a cusp is rarely what a driver would do; only near it
    // (parking, turning around) are cusps fair game.
    if ((!near && shot.path.cusp_count() > 0) || planner_.checker().path_in_collision(shot.path)) {
      return;
    }
    double g = node.g;
    int direction = node.direction;
    double curvature = node.parent >= 0 ? node.segment.curvature : 0.0;
    for (std::size_t i = 0; i < shot.path.segments().size(); ++i) {
      const dbg::Segment & seg = shot.path.segments()[i];
      g += planner_.segment_cost(shot.path.segment_start_pose(i), seg, direction, curvature);
      direction = seg.direction();
      curvature = seg.curvature;
    }
    const auto goal_index = static_cast<std::int32_t>(nodes_.size());
    nodes_.push_back({goal_, g, index, {}, direction, static_cast<std::int32_t>(shots_.size())});
    shots_.push_back(std::move(shot.path));
    open_.push({g, 0.0, order_++, goal_index});
  }

  void expand(std::int32_t index)
  {
    const Node node = nodes_[static_cast<std::size_t>(index)];
    const double curvature = node.parent >= 0 ? node.segment.curvature : 0.0;
    for (const dbg::MotionPrimitive & primitive : planner_.primitives()) {
      const dbg::Pose2 next = primitive.apply(node.pose);
      const int direction = primitive.direction();
      const std::uint64_t next_key = key(next, direction);
      const auto found = visits_.find(next_key);
      if (found != visits_.end() && found->second.closed) {
        continue;
      }
      const double g = node.g + planner_.segment_cost(node.pose, primitive.segment, node.direction, curvature);
      if (found != visits_.end() && g >= found->second.g) {
        continue;
      }
      if (planner_.checker().segment_in_collision(node.pose, primitive.segment)) {
        continue;
      }
      const double h = heuristic(next);
      if (!std::isfinite(h)) {
        continue;
      }
      const auto next_index = static_cast<std::int32_t>(nodes_.size());
      nodes_.push_back({next, g, index, primitive.segment, direction, -1});
      visits_[next_key] = {g, next_index, false};
      open_.push({g + h, h, order_++, next_index});
    }
  }

  PlanResult finish(std::int32_t goal_index)
  {
    const Node & goal_node = nodes_[static_cast<std::size_t>(goal_index)];
    std::vector<dbg::Segment> segments;
    for (std::int32_t n = goal_node.parent; nodes_[static_cast<std::size_t>(n)].parent >= 0;
         n = nodes_[static_cast<std::size_t>(n)].parent) {
      segments.push_back(nodes_[static_cast<std::size_t>(n)].segment);
    }
    std::reverse(segments.begin(), segments.end());
    const dbg::Path & shot = shots_[static_cast<std::size_t>(goal_node.shot)];
    segments.insert(segments.end(), shot.segments().begin(), shot.segments().end());
    PlanResult result;
    result.path = dbg::Path(start_, std::move(segments));
    result.cost = goal_node.g;
    result.error = PlanError::kNone;
    result.expanded = expanded_;
    return result;
  }

  const HybridAStar & planner_;
  const PlannerParams & params_;
  const DistanceHeuristic & distance_;
  dbg::Pose2 start_;
  dbg::Pose2 goal_;
  double radius_;
  double yaw_bin_;

  std::vector<Node> nodes_;
  std::vector<dbg::Path> shots_;  // Reeds-Shepp shots of the goal candidates
  std::unordered_map<std::uint64_t, Visit> visits_;
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenOrder> open_;
  std::uint64_t order_{0};
  std::size_t expanded_{0};
};

}  // namespace

HybridAStar::HybridAStar(std::shared_ptr<const dbc::CollisionChecker> checker, PlannerParams params)
: checker_(std::move(checker)),
  params_(params)
{
  if (!checker_) {
    throw std::invalid_argument("HybridAStar: no collision checker");
  }
  if (!(params_.xy_resolution > 0.0) || params_.yaw_bins < 4 || params_.analytic_every < 1) {
    throw std::invalid_argument("HybridAStar: need xy_resolution > 0, yaw_bins >= 4, analytic_every >= 1");
  }
  if (params_.motion.arc_length < params_.xy_resolution * std::sqrt(2.0)) {
    throw std::invalid_argument("HybridAStar: primitives must be longer than a lattice cell diagonal");
  }
  if (!(params_.reverse_penalty >= 1.0) || !(params_.heuristic_weight >= 1.0) || !(params_.cost_sample_step > 0.0)) {
    throw std::invalid_argument("HybridAStar: need reverse_penalty >= 1, heuristic_weight >= 1, cost_sample_step > 0");
  }
  primitives_ = dbg::make_primitives(params_.motion);
  max_curvature_ = 1.0 / params_.motion.min_turning_radius;
  base_clearance_ = base_clearance(checker_->footprint());
  footprint_centre_ = checker_->footprint().centroid();
}

double HybridAStar::cost_at(const dbg::Pose2 & pose) const
{
  const dbc::Grid & grid = checker_->grid();
  const Eigen::Vector2d p = pose.transform(footprint_centre_);
  const dbc::CellIndex cell = grid.world_to_cell(p.x(), p.y());
  if (!grid.contains(cell.x, cell.y)) {
    return 0.0;
  }
  return std::max(0, static_cast<int>(grid.at(cell.x, cell.y))) / 100.0;
}

double HybridAStar::segment_cost(
  const dbg::Pose2 & start, const dbg::Segment & segment, int previous_direction, double previous_curvature) const
{
  const double length = std::abs(segment.length);
  const int direction = segment.direction();
  double cost = length * (direction < 0 ? params_.reverse_penalty : 1.0);
  cost += params_.steering_penalty * length * std::abs(segment.curvature) / max_curvature_;
  if (previous_direction != 0) {
    if (direction != previous_direction) {
      cost += params_.direction_change_penalty;
    }
    cost += params_.steering_change_penalty * std::abs(segment.curvature - previous_curvature) / max_curvature_;
  }
  if (params_.cost_weight > 0.0 && length > 0.0) {
    // Mean costmap cost at evenly spaced points after the start.
    const auto samples = static_cast<int>(std::max(1.0, std::ceil(length / params_.cost_sample_step)));
    double sum = 0.0;
    for (int k = 1; k <= samples; ++k) {
      sum += cost_at(dbg::advance(start, segment.curvature, direction * length * k / samples));
    }
    cost += params_.cost_weight * length * sum / samples;
  }
  return cost;
}

double HybridAStar::path_cost(const dbg::Path & path) const
{
  double cost = 0.0;
  int direction = 0;
  double curvature = 0.0;
  for (std::size_t i = 0; i < path.segments().size(); ++i) {
    const dbg::Segment & seg = path.segments()[i];
    cost += segment_cost(path.segment_start_pose(i), seg, direction, curvature);
    direction = seg.direction();
    curvature = seg.curvature;
  }
  return cost;
}

const DistanceHeuristic & HybridAStar::heuristic_for(const dbg::Pose2 & goal)
{
  const int gx = static_cast<int>(std::floor(goal.x / params_.xy_resolution));
  const int gy = static_cast<int>(std::floor(goal.y / params_.xy_resolution));
  if (!heuristic_ || gx != heuristic_goal_x_ || gy != heuristic_goal_y_) {
    heuristic_ = std::make_unique<DistanceHeuristic>(*checker_, goal.position(), base_clearance_);
    heuristic_goal_x_ = gx;
    heuristic_goal_y_ = gy;
  }
  return *heuristic_;
}

PlanResult HybridAStar::plan(const dbg::Pose2 & start_in, const dbg::Pose2 & goal_in, const PlanLimits & limits)
{
  const dbg::Pose2 start{start_in.x, start_in.y, dbg::normalize_angle(start_in.yaw)};
  const dbg::Pose2 goal{goal_in.x, goal_in.y, dbg::normalize_angle(goal_in.yaw)};
  PlanResult result;
  if (checker_->in_collision(start)) {
    result.error = PlanError::kInvalidStart;
    return result;
  }
  if (checker_->in_collision(goal)) {
    result.error = PlanError::kInvalidGoal;
    return result;
  }
  if (dbg::approx_equal(start, goal, 1e-9)) {
    result.error = PlanError::kNone;
    result.path = dbg::Path(start, {});
    return result;
  }
  Search search(*this, heuristic_for(goal), start, goal);
  return search.run(limits);
}

}  // namespace delivery_bot_planner
