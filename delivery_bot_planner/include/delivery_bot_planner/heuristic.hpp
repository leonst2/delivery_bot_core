// Obstacle-aware distance heuristic for the planner (delivery_bot_core#17): 2D Dijkstra from the goal.

#ifndef DELIVERY_BOT_PLANNER_HEURISTIC_HPP_
#define DELIVERY_BOT_PLANNER_HEURISTIC_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "delivery_bot_collision/collision_checker.hpp"

namespace delivery_bot_planner
{

/// Shortest 8-connected distance (m) from every map cell to the goal's cell, on the map's own resolution. A cell is
/// blocked only if base_link can't be anywhere in it: it is closer to an obstacle than `base_clearance` (the largest
/// disk around base_link inside the footprint) minus half a cell diagonal. So the heuristic never declares a
/// reachable position unreachable, and thin walls still count.
class DistanceHeuristic
{
public:
  DistanceHeuristic(
    const delivery_bot_collision::CollisionChecker & checker, const Eigen::Vector2d & goal, double base_clearance);

  /// Distance to the goal from the cell containing (x, y); +infinity if unreachable, 0 outside the map (no
  /// information there).
  [[nodiscard]] double at(double x, double y) const;

private:
  void run_dijkstra(const std::vector<std::uint8_t> & open, std::size_t goal_index);

  int width_{0};
  int height_{0};
  double resolution_{1.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<double> distance_;
};

/// Radius of the largest disk around the base_link origin that lies inside the footprint (0 if the origin is not
/// inside).
double base_clearance(const delivery_bot_collision::Footprint & footprint);

}  // namespace delivery_bot_planner

#endif  // DELIVERY_BOT_PLANNER_HEURISTIC_HPP_
