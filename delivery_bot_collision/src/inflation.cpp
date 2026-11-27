#include "delivery_bot_collision/inflation.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace delivery_bot_collision
{

void InflationParams::validate() const
{
  if (!(inscribed_radius >= 0.0) || !(inflation_radius > inscribed_radius) || !std::isfinite(inflation_radius)) {
    throw std::invalid_argument("InflationParams: need 0 <= inscribed_radius < inflation_radius");
  }
  if (!(decay > 0.0) || !std::isfinite(decay)) {
    throw std::invalid_argument("InflationParams: decay must be finite and > 0");
  }
}

std::int8_t inflation_cost(double distance, const InflationParams & params)
{
  if (distance <= params.inscribed_radius) {
    return kLethalCost;
  }
  if (distance >= params.inflation_radius) {
    return 0;
  }
  const double at_end = std::exp(-params.decay * (params.inflation_radius - params.inscribed_radius));
  const double raw = std::exp(-params.decay * (distance - params.inscribed_radius));
  const double scaled = kMaxInflatedCost * (raw - at_end) / (1.0 - at_end);  // 99 ... 0, monotone
  return static_cast<std::int8_t>(std::ceil(scaled));
}

std::vector<std::int8_t> inflate(const Grid & grid, const DistanceField & field, const InflationParams & params)
{
  params.validate();
  if (field.width() != grid.width() || field.height() != grid.height()) {
    throw std::invalid_argument("inflate: field and grid sizes differ");
  }
  std::vector<std::int8_t> costs(grid.data().size());
  for (std::size_t i = 0; i < costs.size(); ++i) {
    costs[i] = grid.data()[i] < 0 ? std::int8_t{-1} : inflation_cost(field.data()[i], params);
  }
  return costs;
}

}  // namespace delivery_bot_collision
