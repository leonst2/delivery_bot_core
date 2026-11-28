#include "delivery_bot_geometry/angle.hpp"

#include <cmath>

namespace delivery_bot_geometry
{

double normalize_angle_positive(double angle)
{
  constexpr double kSnap = 1e-10;
  double wrapped = std::fmod(angle, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  if (wrapped >= kTwoPi - kSnap) {
    wrapped = 0.0;
  }
  return wrapped;
}

double interpolate_angle(double from, double to, double t) { return normalize_angle(from + t * angle_diff(to, from)); }

}  // namespace delivery_bot_geometry
