// Dubins curves (delivery_bot_core#16): shortest forward-only paths between two poses with a minimum turning radius.
//
// The shortest path is one of the six words LSL, RSR, LSR, RSL, RLR, LRL (Dubins 1957). The formulas follow
// Shkel & Lumelsky, "Classification of the Dubins set" (2001), in the frame where the start-goal line is the x axis.

#ifndef DELIVERY_BOT_GEOMETRY_DUBINS_HPP_
#define DELIVERY_BOT_GEOMETRY_DUBINS_HPP_

#include <vector>

#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_geometry
{

/// Every feasible Dubins word from `start` to `goal`, in the fixed order LSL, RSR, LSR, RSL, RLR, LRL.
/// `radius` > 0 (m); throws std::invalid_argument otherwise.
std::vector<CurveCandidate> dubins_all(const Pose2 & start, const Pose2 & goal, double radius);

/// The shortest Dubins path. Ties go to the earlier word in the order above, so the result is deterministic.
/// A Dubins path always exists, so this never fails for radius > 0.
CurveCandidate dubins_shortest(const Pose2 & start, const Pose2 & goal, double radius);

}  // namespace delivery_bot_geometry

#endif  // DELIVERY_BOT_GEOMETRY_DUBINS_HPP_
