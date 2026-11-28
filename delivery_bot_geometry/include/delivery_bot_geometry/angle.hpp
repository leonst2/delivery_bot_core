// Angle helpers (delivery_bot_core#16). All angles in rad, + = counter-clockwise (ROS FLU).

#ifndef DELIVERY_BOT_GEOMETRY_ANGLE_HPP_
#define DELIVERY_BOT_GEOMETRY_ANGLE_HPP_

#include <cmath>

namespace delivery_bot_geometry
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

/// Wraps an angle into (-pi, pi]. -pi maps to +pi, so every direction has
/// exactly one representation.
inline double normalize_angle(double angle)
{
  // std::remainder gives [-pi, pi]; move the lower bound to the other side.
  double wrapped = std::remainder(angle, kTwoPi);
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

/// Wraps an angle into [0, 2 pi). Values within 1e-10 below 2 pi snap to 0, so
/// rounding noise never turns a zero turn into a full circle (Dubins words rely
/// on this).
double normalize_angle_positive(double angle);

/// Signed shortest rotation from `from` to `to`, in (-pi, pi].
inline double angle_diff(double to, double from) { return normalize_angle(to - from); }

/// Interpolates along the shortest rotation: t = 0 gives `from`, t = 1 gives
/// `to` (normalised).
double interpolate_angle(double from, double to, double t);

}  // namespace delivery_bot_geometry

#endif  // DELIVERY_BOT_GEOMETRY_ANGLE_HPP_
