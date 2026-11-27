// Reeds-Shepp curves (delivery_bot_core#16): shortest paths between two poses with a minimum turning radius when
// the robot may also reverse.
//
// Reeds & Shepp, "Optimal paths for a car that goes both forwards and backwards" (1990): the shortest path is one of
// the word families CSC, CCC, CCCC, CCSC and CCSCC. Each base formula is applied in its time-flipped, reflected and
// (where it adds new words) backwards variant, which gives every word of the paper.

#ifndef DELIVERY_BOT_GEOMETRY_REEDS_SHEPP_HPP_
#define DELIVERY_BOT_GEOMETRY_REEDS_SHEPP_HPP_

#include <vector>

#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_geometry
{

/// Every feasible Reeds-Shepp candidate from `start` to `goal`, in a fixed order (families CSC, CCC, CCCC, CCSC,
/// CCSCC; per formula: plain, time-flipped, reflected, both; then the backwards variants). `radius` > 0 (m); throws
/// std::invalid_argument otherwise.
std::vector<CurveCandidate> reeds_shepp_all(const Pose2 & start, const Pose2 & goal, double radius);

/// The shortest Reeds-Shepp path. Ties go to the earlier candidate in the order above, so the result is
/// deterministic.
CurveCandidate reeds_shepp_shortest(const Pose2 & start, const Pose2 & goal, double radius);

}  // namespace delivery_bot_geometry

#endif  // DELIVERY_BOT_GEOMETRY_REEDS_SHEPP_HPP_
