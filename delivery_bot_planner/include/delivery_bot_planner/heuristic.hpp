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

/// Shortest 8-connected distance (m) from every map cell to the goal's cell, on the map's own resolution, for a
/// reference point of the robot that always has `clearance` to every obstacle in a collision-free pose (the planner
/// uses the footprint centroid and the inscribed radius). A cell is blocked only if the point can't be anywhere in it:
/// it is closer to an obstacle than `clearance` minus half a cell diagonal. So the heuristic never declares a
/// reachable position unreachable, and thin walls and gaps narrower than the robot still count.
class DistanceHeuristic
{
public:
  DistanceHeuristic(
    const delivery_bot_collision::CollisionChecker & checker, const Eigen::Vector2d & goal, double clearance);

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
