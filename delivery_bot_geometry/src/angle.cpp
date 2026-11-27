#include "delivery_bot_geometry/angle.hpp"

#include <cmath>

namespace delivery_bot_geometry
{

double normalize_angle(double angle)
{
  // std::remainder gives [-pi, pi]; move the lower bound to the other side.
  double wrapped = std::remainder(angle, kTwoPi);
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

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

double angle_diff(double to, double from) { return normalize_angle(to - from); }

double interpolate_angle(double from, double to, double t) { return normalize_angle(from + t * angle_diff(to, from)); }

}  // namespace delivery_bot_geometry
