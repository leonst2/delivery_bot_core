#include "delivery_bot_planner/planar_tf.hpp"

#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <string>

namespace delivery_bot_planner
{

namespace dbg = delivery_bot_geometry;

namespace
{

constexpr std::size_t kMaxDepth = 64;  // more links than this means a loop

std::string clean(const std::string & frame)
{
  const std::size_t first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string{} : frame.substr(first);
}

}  // namespace

void PlanarTf::set(const std::string & parent, const std::string & child, const dbg::Pose2 & child_in_parent)
{
  links_[clean(child)] = {clean(parent), child_in_parent};
}

std::map<std::string, dbg::Pose2> PlanarTf::ancestors(const std::string & frame) const
{
  std::map<std::string, dbg::Pose2> result;
  std::string current = frame;
  dbg::Pose2 pose_in_current{};  // `frame` expressed in `current`
  result[current] = pose_in_current;
  for (std::size_t depth = 0; depth < kMaxDepth; ++depth) {
    const auto link = links_.find(current);
    if (link == links_.end()) {
      return result;
    }
    pose_in_current = link->second.pose * pose_in_current;
    current = link->second.parent;
    if (result.count(current) != 0) {
      return {};  // loop
    }
    result[current] = pose_in_current;
  }
  return {};
}

std::optional<dbg::Pose2> PlanarTf::lookup(const std::string & target_in, const std::string & source_in) const
{
  const std::string target = clean(target_in);
  const std::string source = clean(source_in);
  const auto from_source = ancestors(source);
  // Walk up from the target until a frame on the source's chain: source_in_target = target_in_x^-1 * source_in_x.
  std::string current = target;
  dbg::Pose2 target_in_current{};
  for (std::size_t depth = 0; depth < kMaxDepth; ++depth) {
    const auto common = from_source.find(current);
    if (common != from_source.end()) {
      return target_in_current.inverse() * common->second;
    }
    const auto link = links_.find(current);
    if (link == links_.end()) {
      return std::nullopt;
    }
    target_in_current = link->second.pose * target_in_current;
    current = link->second.parent;
  }
  return std::nullopt;
}

double PlanarTf::yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace delivery_bot_planner
