// A minimal planar transform tree (delivery_bot_core#17): the latest transform of every frame, ground-plane part
// only (x, y, yaw). The planner needs the current map -> base_link pose for "plan from where I am"; tf2_ros is not
// part of the allowed ROS 2 core, so the node feeds /tf and /tf_static into this.

#ifndef DELIVERY_BOT_PLANNER_PLANAR_TF_HPP_
#define DELIVERY_BOT_PLANNER_PLANAR_TF_HPP_

#include <map>
#include <optional>
#include <string>

#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_planner
{

class PlanarTf
{
public:
  /// Stores `child`'s pose in `parent` (replaces the previous transform of `child`). Leading '/' are ignored.
  void set(const std::string & parent, const std::string & child, const delivery_bot_geometry::Pose2 & child_in_parent);

  /// The pose of frame `source` expressed in frame `target`, through their common ancestor; nullopt if the frames
  /// aren't connected (or the tree has a loop).
  [[nodiscard]] std::optional<delivery_bot_geometry::Pose2> lookup(
    const std::string & target, const std::string & source) const;

  /// Yaw (rad) of a quaternion, for planar use: rotation about z.
  static double yaw_from_quaternion(double x, double y, double z, double w);

private:
  struct Link
  {
    std::string parent;
    delivery_bot_geometry::Pose2 pose;  // child in parent
  };

  // Every ancestor of `frame` (and `frame` itself, as the identity) with `frame`'s pose expressed in it; empty on a loop.
  [[nodiscard]] std::map<std::string, delivery_bot_geometry::Pose2> ancestors(const std::string & frame) const;

  std::map<std::string, Link> links_;  // keyed by child frame
};

}  // namespace delivery_bot_planner

#endif  // DELIVERY_BOT_PLANNER_PLANAR_TF_HPP_
