// Parameters of the Hybrid-A* planner (delivery_bot_core#17).

#ifndef DELIVERY_BOT_PLANNER_PLANNER_PARAMS_HPP_
#define DELIVERY_BOT_PLANNER_PLANNER_PARAMS_HPP_

#include <cstddef>

#include "delivery_bot_geometry/primitives.hpp"

namespace delivery_bot_planner
{

// Defaults chosen on the city benchmark (test/test_benchmark.cpp, README "Tuning").
struct PlannerParams
{
  double xy_resolution{0.25};                     // m, cell size of the search lattice
  int yaw_bins{72};                               // 5 degree heading bins
  delivery_bot_geometry::AckermannParams motion;  // primitives: 0.5 m arcs, 5 steering angles, forward + reverse

  // Cost terms (all in metres of driving).
  double reverse_penalty{2.0};           // reverse length counts this many times
  double direction_change_penalty{4.0};  // per cusp
  double steering_penalty{0.05};         // per metre, times |curvature| / max curvature: prefer straight
  double steering_change_penalty{0.2};   // per change of curvature by max curvature: prefer steady steering
  double cost_weight{1.0};               // per metre, times costmap cost / 100 (inflation keeps away from walls)
  double cost_sample_step{0.25};         // m, costmap samples along a segment

  // Weighted A*: the heuristic doesn't know about manoeuvres in tight streets, so 1.0 explores far too much.
  double heuristic_weight{1.5};
  // h = max(Dijkstra distance, Reeds-Shepp length) within this distance (m) of the goal, the Dijkstra distance alone
  // farther out: Reeds-Shepp only adds a bounded heading correction there and costs ~5 us per node.
  double rs_heuristic_radius{10.0};
  int analytic_every{5};            // Reeds-Shepp shot to the goal every N expansions ...
  double analytic_radius{2.0};      // ... and at every expansion this close (m) to the goal
  bool far_shots_with_cusps{true};  // shots with cusps also count far from the goal (their cost includes the cusps)
  // A lattice node this close to the goal may end the path too (Reeds-Shepp shots reach the goal exactly, but next
  // to obstacles they mostly collide). The path then ends within the tolerance instead of on the goal.
  double goal_xy_tolerance{0.2};   // m
  double goal_yaw_tolerance{0.1};  // rad
  std::size_t max_expansions{2000000};
};

}  // namespace delivery_bot_planner

#endif  // DELIVERY_BOT_PLANNER_PLANNER_PARAMS_HPP_
