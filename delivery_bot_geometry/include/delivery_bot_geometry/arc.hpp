// Arc utilities for path tracking (delivery_bot_core#16), used by the step controller (delivery_bot_core#19).
//
// The Rust port in delivery_bot_geometry/rust implements the same functions with the same formulas; both are
// checked against test/vectors/arc_vectors.txt.

#ifndef DELIVERY_BOT_GEOMETRY_ARC_HPP_
#define DELIVERY_BOT_GEOMETRY_ARC_HPP_

#include <limits>

#include <Eigen/Core>

#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_geometry
{

struct ClosestPoint
{
  double s{0.0};         // arc length of the closest point
  Pose2 pose;            // path pose there
  double distance{0.0};  // m
};

struct TrackingError
{
  double s{0.0};            // arc length of the reference (closest) point
  Pose2 reference;          // path pose there
  double cross_track{0.0};  // m, + = robot left of the reference heading
  double heading{0.0};      // rad, robot yaw - reference yaw, in (-pi, pi]
};

/// The pose at arc length s (clamped to the path), same as path.pose_at(s).
Pose2 point_at(const Path & path, double s);

/// The exact closest point of the path to `point`, searched only within the arc-length window [s_min, s_max]
/// (clamped to the path). The window lets a controller stay on its part of a path that crosses itself. Ties go to
/// the smallest s. An empty path gives its start pose at s = 0.
ClosestPoint closest_point(
  const Path & path, const Eigen::Vector2d & point, double s_min = 0.0,
  double s_max = std::numeric_limits<double>::infinity());

/// Signed lateral offset of `point` from the line through `reference` along its heading, + = left.
/// On reverse segments the heading is still the robot heading, so + stays the robot's left.
double cross_track_error(const Pose2 & reference, const Eigen::Vector2d & point);

/// yaw - reference.yaw, normalised to (-pi, pi].
double heading_error(const Pose2 & reference, double yaw);

/// closest_point of the robot position, with the cross-track and heading errors against it.
TrackingError tracking_error(
  const Path & path, const Pose2 & robot, double s_min = 0.0, double s_max = std::numeric_limits<double>::infinity());

}  // namespace delivery_bot_geometry

#endif  // DELIVERY_BOT_GEOMETRY_ARC_HPP_
