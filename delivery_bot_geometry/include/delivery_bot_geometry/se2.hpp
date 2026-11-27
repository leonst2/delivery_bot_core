// SE(2) poses (delivery_bot_core#16). Units m / rad, yaw + = counter-clockwise (ROS FLU).

#ifndef DELIVERY_BOT_GEOMETRY_SE2_HPP_
#define DELIVERY_BOT_GEOMETRY_SE2_HPP_

#include <Eigen/Core>

namespace delivery_bot_geometry
{

/// A planar pose: position (x, y) and heading yaw. Yaw is kept normalised to
/// (-pi, pi] by every operation that produces a pose; the constructor stores it
/// as given.
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  Pose2() = default;
  Pose2(double x_in, double y_in, double yaw_in);

  /// Composition: `a * b` is pose b (expressed in frame a) expressed in a's
  /// parent frame.
  Pose2 operator*(const Pose2 & other) const;

  /// The inverse transform: `p * p.inverse()` is the identity.
  [[nodiscard]] Pose2 inverse() const;

  /// Maps a point given in this pose's frame into the parent frame.
  [[nodiscard]] Eigen::Vector2d transform(const Eigen::Vector2d & point) const;

  /// Maps a point given in the parent frame into this pose's frame.
  [[nodiscard]] Eigen::Vector2d inverse_transform(const Eigen::Vector2d & point) const;

  [[nodiscard]] Eigen::Vector2d position() const { return {x, y}; }
};

/// The pose of `to` expressed in the frame of `from`: from.inverse() * to.
Pose2 between(const Pose2 & from, const Pose2 & to);

/// Linear interpolation of the position and shortest-rotation interpolation of
/// the yaw. t = 0 gives `from`, t = 1 gives `to`.
Pose2 interpolate(const Pose2 & from, const Pose2 & to, double t);

/// Euclidean distance between the positions.
double distance(const Pose2 & a, const Pose2 & b);

/// True if the positions differ by at most `tolerance` (m) and the yaws by at
/// most `tolerance` (rad).
bool approx_equal(const Pose2 & a, const Pose2 & b, double tolerance);

}  // namespace delivery_bot_geometry

#endif  // DELIVERY_BOT_GEOMETRY_SE2_HPP_
