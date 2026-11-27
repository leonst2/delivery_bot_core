// Internal helpers shared by the Dubins and Reeds-Shepp solvers. Not installed.

#ifndef DELIVERY_BOT_GEOMETRY_CURVE_UTIL_HPP_
#define DELIVERY_BOT_GEOMETRY_CURVE_UTIL_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_geometry::detail
{

/// Throws std::invalid_argument unless radius is a finite value > 0.
void check_radius(double radius);

/// Builds a candidate from a word (one letter L/R/S per value) and its normalised signed lengths: arcs in rad,
/// straights in units of the radius, < 0 = reverse.
CurveCandidate make_candidate(
  const Pose2 & start, double radius, const std::string & letters, const double * values, std::size_t count);

/// The shortest candidate; ties go to the earliest one. Requires a non-empty list.
CurveCandidate shortest(std::vector<CurveCandidate> candidates);

}  // namespace delivery_bot_geometry::detail

#endif  // DELIVERY_BOT_GEOMETRY_CURVE_UTIL_HPP_
