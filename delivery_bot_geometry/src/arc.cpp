#include "delivery_bot_geometry/arc.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "delivery_bot_geometry/angle.hpp"

namespace delivery_bot_geometry
{

namespace
{

constexpr double kStraightCurvature = 1e-9;  // below this a segment is treated as straight
constexpr double kCentreDistance = 1e-12;    // query point on the arc centre: every arc point is equally far

// Candidate local arc lengths (from the segment start, >= 0) of the closest point on one segment, within
// [lo, hi]. Always contains lo and hi; the result is sorted ascending.
std::vector<double> segment_candidates(
  const Pose2 & start, const Segment & seg, const Eigen::Vector2d & point, double lo, double hi)
{
  std::vector<double> local{lo, hi};
  const double dir = seg.direction();
  if (std::abs(seg.curvature) < kStraightCurvature) {
    // Projection onto the line; travelled length grows along dir * heading.
    const double along = std::cos(start.yaw) * (point.x() - start.x) + std::sin(start.yaw) * (point.y() - start.y);
    local.push_back(std::clamp(dir * along, lo, hi));
  } else {
    const double k = seg.curvature;
    const double cx = start.x - std::sin(start.yaw) / k;
    const double cy = start.y + std::cos(start.yaw) / k;
    if (std::hypot(point.x() - cx, point.y() - cy) >= kCentreDistance) {
      // The nearest arc point lies in the direction of the query point from the centre. Its heading is that
      // direction + 90 deg for a left arc (- 90 deg for a right one); sigma is the signed length to get there.
      const double sign = k > 0.0 ? 1.0 : -1.0;
      const double yaw_q = std::atan2(point.y() - cy, point.x() - cx) + sign * kPi / 2.0;
      const double sigma0 = normalize_angle(yaw_q - start.yaw) / k;
      const double period = kTwoPi / std::abs(k);
      // All sigma = sigma0 + n * period with local = dir * sigma in [lo, hi].
      const double sigma_lo = dir > 0.0 ? lo : -hi;
      const double sigma_hi = dir > 0.0 ? hi : -lo;
      const auto n_min = static_cast<long long>(std::ceil((sigma_lo - sigma0) / period));
      const auto n_max = static_cast<long long>(std::floor((sigma_hi - sigma0) / period));
      for (long long n = n_min; n <= n_max; ++n) {
        local.push_back(std::clamp(dir * (sigma0 + static_cast<double>(n) * period), lo, hi));
      }
    }
  }
  std::sort(local.begin(), local.end());
  return local;
}

}  // namespace

Pose2 point_at(const Path & path, double s) { return path.pose_at(s); }

ClosestPoint closest_point(const Path & path, const Eigen::Vector2d & point, double s_min, double s_max)
{
  const auto distance_to = [&point](const Pose2 & pose) { return std::hypot(point.x() - pose.x, point.y() - pose.y); };
  if (path.empty()) {
    return {0.0, path.start(), distance_to(path.start())};
  }
  const double total = path.length();
  const double window_lo = std::clamp(s_min, 0.0, total);
  const double window_hi = std::clamp(s_max, window_lo, total);

  ClosestPoint best{0.0, Pose2{}, std::numeric_limits<double>::infinity()};
  for (std::size_t i = 0; i < path.segments().size(); ++i) {
    const double seg_start = path.segment_start_s(i);
    const double seg_end = path.segment_start_s(i + 1);
    const double lo = std::max(window_lo, seg_start);
    const double hi = std::min(window_hi, seg_end);
    if (lo > hi) {
      continue;
    }
    const Segment & seg = path.segments()[i];
    const Pose2 & start = path.segment_start_pose(i);
    for (const double local : segment_candidates(start, seg, point, lo - seg_start, hi - seg_start)) {
      const Pose2 pose = advance(start, seg.curvature, seg.direction() * local);
      const double d = distance_to(pose);
      if (d < best.distance) {
        best = {seg_start + local, pose, d};
      }
    }
  }
  return best;
}

double cross_track_error(const Pose2 & reference, const Eigen::Vector2d & point)
{
  return -std::sin(reference.yaw) * (point.x() - reference.x) + std::cos(reference.yaw) * (point.y() - reference.y);
}

double heading_error(const Pose2 & reference, double yaw) { return angle_diff(yaw, reference.yaw); }

TrackingError tracking_error(const Path & path, const Pose2 & robot, double s_min, double s_max)
{
  const ClosestPoint closest = closest_point(path, robot.position(), s_min, s_max);
  return {
    closest.s, closest.pose, cross_track_error(closest.pose, robot.position()), heading_error(closest.pose, robot.yaw)};
}

}  // namespace delivery_bot_geometry
