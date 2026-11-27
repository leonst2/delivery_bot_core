#include "delivery_bot_geometry/primitives.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace delivery_bot_geometry
{

namespace
{
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
}  // namespace

double steering_angle_for_curvature(double wheelbase, double curvature) { return std::atan(wheelbase * curvature); }

double curvature_for_steering_angle(double wheelbase, double steering_angle)
{
  return std::tan(steering_angle) / wheelbase;
}

double max_steering_angle(const AckermannParams & params)
{
  return std::atan(params.wheelbase / params.min_turning_radius);
}

std::vector<MotionPrimitive> make_primitives(const AckermannParams & params)
{
  if (!positive(params.wheelbase) || !positive(params.min_turning_radius) || !positive(params.arc_length)) {
    throw std::invalid_argument("make_primitives: wheelbase, min_turning_radius and arc_length must be > 0");
  }
  if (params.steering_count < 1 || params.steering_count % 2 == 0) {
    throw std::invalid_argument("make_primitives: steering_count must be odd and >= 1");
  }

  const auto count = static_cast<std::size_t>(params.steering_count);
  const double max_angle = max_steering_angle(params);
  const double max_curvature = 1.0 / params.min_turning_radius;
  std::vector<MotionPrimitive> primitives;
  primitives.reserve(params.allow_reverse ? 2 * count : count);

  for (const double direction : {1.0, -1.0}) {
    if (direction < 0.0 && !params.allow_reverse) {
      break;
    }
    for (std::size_t i = 0; i < count; ++i) {
      double steering = 0.0;
      double curvature = 0.0;
      if (count > 1) {
        // Evenly spaced from -max to +max. The outermost ones get the exact limit curvature (no tan/atan rounding).
        const auto half = static_cast<double>(count - 1) / 2.0;
        const double fraction = (static_cast<double>(i) - half) / half;
        steering = fraction * max_angle;
        if (i == 0) {
          curvature = -max_curvature;
        } else if (i == count - 1) {
          curvature = max_curvature;
        } else {
          curvature = curvature_for_steering_angle(params.wheelbase, steering);
        }
      }
      const Segment segment{curvature, direction * params.arc_length};
      primitives.push_back({segment, steering, advance(Pose2{}, segment.curvature, segment.length)});
    }
  }
  return primitives;
}

}  // namespace delivery_bot_geometry
