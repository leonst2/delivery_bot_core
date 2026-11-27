// Parameters of the Hybrid-A* planner (delivery_bot_core#17).

#ifndef DELIVERY_BOT_PLANNER_PLANNER_PARAMS_HPP_
#define DELIVERY_BOT_PLANNER_PLANNER_PARAMS_HPP_

#include <cstddef>

#include "delivery_bot_geometry/primitives.hpp"

namespace delivery_bot_planner
{

struct PlannerParams
{
  double xy_resolution{0.2};                      // m, cell size of the search lattice and of the 2D heuristic
  int yaw_bins{72};                               // 5 degree heading bins
  delivery_bot_geometry::AckermannParams motion;  // primitives: 0.5 m arcs, 5 steering angles, forward + reverse

  // Cost terms (all in metres of driving).
  double reverse_penalty{2.0};           // reverse length counts this many times
  double direction_change_penalty{3.0};  // per cusp
  double steering_penalty{0.05};         // per metre, times |curvature| / max curvature: prefer straight
  double steering_change_penalty{0.2};   // per change of curvature by max curvature: prefer steady steering
  double cost_weight{1.0};               // per metre, times costmap cost / 100 (inflation keeps away from walls)
  double cost_sample_step{0.25};         // m, costmap samples along a segment

  // Weighted A*: 1.2 is ~10-20x faster than 1.0 on city maps at a few percent of path cost (see the README).
  double heuristic_weight{1.2};
  int analytic_every{10};       // Reeds-Shepp shot to the goal every N expansions (shots with cusps rejected) ...
  double analytic_radius{4.0};  // ... and at every expansion this close (m) to the goal (cusps allowed)
  std::size_t max_expansions{2000000};
};

}  // namespace delivery_bot_planner

#endif  // DELIVERY_BOT_PLANNER_PLANNER_PARAMS_HPP_
