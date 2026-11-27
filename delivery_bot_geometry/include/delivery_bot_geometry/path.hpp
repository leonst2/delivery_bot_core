// Paths made of constant-curvature segments (delivery_bot_core#16).
//
// Conventions:
// - A segment has a signed length: > 0 drives forward, < 0 drives in reverse (the robot keeps facing the same way).
// - Curvature is the steering curvature, + = steering left: driving a length l changes the yaw by curvature * l, so
//   reversing with the wheels turned left turns the robot clockwise.
// - s is the travelled arc length along the path, always >= 0 and growing in both directions.

#ifndef DELIVERY_BOT_GEOMETRY_PATH_HPP_
#define DELIVERY_BOT_GEOMETRY_PATH_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_geometry
{

struct Segment
{
  double curvature{0.0};  // 1/m, + = left
  double length{0.0};     // m, signed: < 0 = reverse

  static Segment straight(double length) { return {0.0, length}; }
  static Segment left(double length, double radius) { return {1.0 / radius, length}; }
  static Segment right(double length, double radius) { return {-1.0 / radius, length}; }

  /// +1 forward, -1 reverse. A zero-length segment counts as forward.
  [[nodiscard]] int direction() const { return length < 0.0 ? -1 : 1; }
};

/// The pose after driving the signed `length` with constant `curvature` from `start`. Numerically stable for
/// curvature -> 0 (it becomes a straight line without a special case).
Pose2 advance(const Pose2 & start, double curvature, double length);

struct PathSample
{
  double s{0.0};          // travelled arc length from the path start (m)
  Pose2 pose;             // robot pose
  double curvature{0.0};  // steering curvature of the segment the sample belongs to
  int direction{1};       // +1 forward, -1 reverse
};

/// A path from `start` along a list of segments. Segments shorter than 1e-12 m are dropped on construction.
/// A point exactly on a segment boundary belongs to the following segment (the last point to the last segment).
class Path
{
public:
  Path() = default;
  Path(const Pose2 & start, std::vector<Segment> segments);

  [[nodiscard]] const Pose2 & start() const { return start_; }
  [[nodiscard]] const std::vector<Segment> & segments() const { return segments_; }
  [[nodiscard]] bool empty() const { return segments_.empty(); }

  /// Total travelled arc length (sum of |segment length|).
  [[nodiscard]] double length() const { return cumulative_s_.back(); }
  [[nodiscard]] Pose2 end() const { return segment_start_.back(); }

  /// Index of the segment that contains s (clamped to [0, length]). Requires a non-empty path.
  [[nodiscard]] std::size_t segment_index_at(double s) const;
  /// Arc length at which segment i starts; i == segments().size() gives length().
  [[nodiscard]] double segment_start_s(std::size_t i) const { return cumulative_s_.at(i); }
  /// Pose at which segment i starts; i == segments().size() gives end().
  [[nodiscard]] const Pose2 & segment_start_pose(std::size_t i) const { return segment_start_.at(i); }

  /// Pose at arc length s, clamped to [0, length].
  [[nodiscard]] Pose2 pose_at(double s) const;
  /// Steering curvature at s (0 for an empty path).
  [[nodiscard]] double curvature_at(double s) const;
  /// +1 forward, -1 reverse at s (+1 for an empty path).
  [[nodiscard]] int direction_at(double s) const;
  /// Number of direction changes (cusps).
  [[nodiscard]] std::size_t cusp_count() const;

  /// Samples every segment at a spacing of at most `resolution` (m). Segment boundaries, including cusps, are always
  /// samples, so the first and last samples are the start and the end. An empty path gives one sample at the start.
  [[nodiscard]] std::vector<PathSample> sample(double resolution) const;

private:
  Pose2 start_;
  std::vector<Segment> segments_;
  std::vector<double> cumulative_s_{0.0};      // size segments + 1
  std::vector<Pose2> segment_start_{Pose2{}};  // size segments + 1, the last entry is the end pose
};

/// One solution of a curve family (Dubins, Reeds-Shepp): its word (L = left, R = right, S = straight, one letter per
/// segment of the base word, before zero-length segments are dropped) and the path.
struct CurveCandidate
{
  std::string word;
  Path path;
};

}  // namespace delivery_bot_geometry

#endif  // DELIVERY_BOT_GEOMETRY_PATH_HPP_
