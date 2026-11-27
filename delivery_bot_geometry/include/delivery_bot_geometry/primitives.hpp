// Ackermann motion primitives (delivery_bot_core#16): constant-steering arcs of a fixed length, forward and reverse,
// for the search of the planner (delivery_bot_core#17).
//
// Bicycle model around the rear-axle centre (base_link): curvature = tan(steering angle) / wheelbase.

#ifndef DELIVERY_BOT_GEOMETRY_PRIMITIVES_HPP_
#define DELIVERY_BOT_GEOMETRY_PRIMITIVES_HPP_

#include <vector>

#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_geometry
{

struct AckermannParams
{
  double wheelbase{0.49};           // m (URDF)
  double min_turning_radius{1.25};  // m, at the rear-axle centre
  double arc_length{0.5};           // m, travelled length of every primitive
  int steering_count{5};            // odd, >= 1: steering angles spread evenly over [-max, +max], 0 included
  bool allow_reverse{true};
};

/// Steering angle (rad, + left) that drives the given curvature.
double steering_angle_for_curvature(double wheelbase, double curvature);
/// Curvature (1/m, + left) of the given steering angle.
double curvature_for_steering_angle(double wheelbase, double steering_angle);
/// The steering angle of the minimum turning radius.
double max_steering_angle(const AckermannParams & params);

struct MotionPrimitive
{
  Segment segment;             // curvature and signed length (< 0 = reverse)
  double steering_angle{0.0};  // rad, + left
  Pose2 delta;                 // end pose in the frame of the start pose

  [[nodiscard]] int direction() const { return segment.direction(); }
  /// End pose when the primitive starts at `from`.
  [[nodiscard]] Pose2 apply(const Pose2 & from) const { return from * delta; }
  /// The primitive as a path from `from`.
  [[nodiscard]] Path path_from(const Pose2 & from) const { return Path(from, {segment}); }
};

/// The primitive set: forward first, then reverse (if allowed); within a direction from full right to full left
/// steering. Throws std::invalid_argument for non-positive lengths or an even / non-positive steering_count.
std::vector<MotionPrimitive> make_primitives(const AckermannParams & params);

}  // namespace delivery_bot_geometry

#endif  // DELIVERY_BOT_GEOMETRY_PRIMITIVES_HPP_
