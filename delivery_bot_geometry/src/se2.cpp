#include "delivery_bot_geometry/se2.hpp"

#include <cmath>

#include "delivery_bot_geometry/angle.hpp"

namespace delivery_bot_geometry
{

Pose2::Pose2(double x_in, double y_in, double yaw_in)
: x(x_in),
  y(y_in),
  yaw(yaw_in)
{
}

Pose2 Pose2::operator*(const Pose2 & other) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {x + c * other.x - s * other.y, y + s * other.x + c * other.y, normalize_angle(yaw + other.yaw)};
}

Pose2 Pose2::inverse() const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {-c * x - s * y, s * x - c * y, normalize_angle(-yaw)};
}

Eigen::Vector2d Pose2::transform(const Eigen::Vector2d & point) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {x + c * point.x() - s * point.y(), y + s * point.x() + c * point.y()};
}

Eigen::Vector2d Pose2::inverse_transform(const Eigen::Vector2d & point) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double dx = point.x() - x;
  const double dy = point.y() - y;
  return {c * dx + s * dy, -s * dx + c * dy};
}

Pose2 between(const Pose2 & from, const Pose2 & to) { return from.inverse() * to; }

Pose2 interpolate(const Pose2 & from, const Pose2 & to, double t)
{
  return {from.x + t * (to.x - from.x), from.y + t * (to.y - from.y), interpolate_angle(from.yaw, to.yaw, t)};
}

double distance(const Pose2 & a, const Pose2 & b) { return std::hypot(b.x - a.x, b.y - a.y); }

bool approx_equal(const Pose2 & a, const Pose2 & b, double tolerance)
{
  return std::abs(a.x - b.x) <= tolerance && std::abs(a.y - b.y) <= tolerance &&
         std::abs(angle_diff(a.yaw, b.yaw)) <= tolerance;
}

}  // namespace delivery_bot_geometry
