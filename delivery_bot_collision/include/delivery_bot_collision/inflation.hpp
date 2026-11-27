// Inflation costs around obstacles (delivery_bot_core#15), used by the costmap (delivery_bot_core#14).
//
// Costs follow the costmap contract: 0 = free ... 100 = lethal, -1 = unknown.

#ifndef DELIVERY_BOT_COLLISION_INFLATION_HPP_
#define DELIVERY_BOT_COLLISION_INFLATION_HPP_

#include <cstdint>
#include <vector>

#include "delivery_bot_collision/distance_field.hpp"
#include "delivery_bot_collision/grid.hpp"

namespace delivery_bot_collision
{

constexpr std::int8_t kLethalCost = 100;
constexpr std::int8_t kMaxInflatedCost = 99;

struct InflationParams
{
  double inscribed_radius{0.32};  // m, lethal within (FootprintParams{}.make().inscribed_radius())
  double inflation_radius{1.0};   // m, cost reaches 0 here
  double decay{3.0};              // 1/m, exponential decay rate between the two

  /// Throws std::invalid_argument unless 0 <= inscribed_radius < inflation_radius and decay > 0.
  void validate() const;
};

/// Cost of a cell at `distance` (m) from the nearest obstacle: 100 up to the inscribed radius, then from 99 down
/// along exp(-decay * (distance - inscribed_radius)), rescaled so it reaches exactly 0 at the inflation radius and
/// rounded up (every cell closer than the inflation radius costs at least 1). Monotonically non-increasing.
std::int8_t inflation_cost(double distance, const InflationParams & params);

/// The inflation cost of every cell of `field`; cells that are unknown (-1) in `grid` stay -1. `field` must have
/// been computed from `grid`.
std::vector<std::int8_t> inflate(const Grid & grid, const DistanceField & field, const InflationParams & params);

}  // namespace delivery_bot_collision

#endif  // DELIVERY_BOT_COLLISION_INFLATION_HPP_
