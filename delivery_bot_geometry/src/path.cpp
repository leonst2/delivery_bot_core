#include "delivery_bot_geometry/path.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "delivery_bot_geometry/angle.hpp"

namespace delivery_bot_geometry
{

namespace
{
constexpr double kMinSegmentLength = 1e-12;

}  // namespace

Path::Path(const Pose2 & start, std::vector<Segment> segments)
: start_(start.x, start.y, normalize_angle(start.yaw))
{
  segments.erase(
    std::remove_if(
      segments.begin(), segments.end(), [](const Segment & seg) { return std::abs(seg.length) < kMinSegmentLength; }),
    segments.end());
  segments_ = std::move(segments);

  cumulative_s_.assign(1, 0.0);
  segment_start_.assign(1, start_);
  cumulative_s_.reserve(segments_.size() + 1);
  segment_start_.reserve(segments_.size() + 1);
  for (const Segment & seg : segments_) {
    cumulative_s_.push_back(cumulative_s_.back() + std::abs(seg.length));
    segment_start_.push_back(advance(segment_start_.back(), seg.curvature, seg.length));
  }
}

std::size_t Path::segment_index_at(double s) const
{
  if (segments_.empty()) {
    throw std::out_of_range("Path::segment_index_at on an empty path");
  }
  // First boundary strictly greater than s, minus one: a boundary belongs to the following segment.
  const auto it = std::upper_bound(cumulative_s_.begin(), cumulative_s_.end(), s);
  const auto index =
    static_cast<std::size_t>(std::max<std::ptrdiff_t>(std::distance(cumulative_s_.begin(), it) - 1, 0));
  return std::min(index, segments_.size() - 1);
}

Pose2 Path::pose_at(double s) const
{
  if (segments_.empty()) {
    return start_;
  }
  const double clamped = std::clamp(s, 0.0, length());
  const std::size_t i = segment_index_at(clamped);
  const Segment & seg = segments_[i];
  const double local = clamped - cumulative_s_[i];
  return advance(segment_start_[i], seg.curvature, seg.direction() * local);
}

double Path::curvature_at(double s) const { return segments_.empty() ? 0.0 : segments_[segment_index_at(s)].curvature; }

int Path::direction_at(double s) const { return segments_.empty() ? 1 : segments_[segment_index_at(s)].direction(); }

std::size_t Path::cusp_count() const
{
  std::size_t cusps = 0;
  for (std::size_t i = 1; i < segments_.size(); ++i) {
    if (segments_[i].direction() != segments_[i - 1].direction()) {
      ++cusps;
    }
  }
  return cusps;
}

std::vector<PathSample> Path::sample(double resolution) const
{
  if (!(resolution > 0.0)) {
    throw std::invalid_argument("Path::sample: resolution must be > 0");
  }
  std::vector<PathSample> samples;
  if (segments_.empty()) {
    samples.push_back({0.0, start_, 0.0, 1});
    return samples;
  }
  for (std::size_t i = 0; i < segments_.size(); ++i) {
    const Segment & seg = segments_[i];
    const double seg_length = std::abs(seg.length);
    const auto pieces = static_cast<std::size_t>(std::max(1.0, std::ceil(seg_length / resolution - 1e-9)));
    for (std::size_t k = 0; k < pieces; ++k) {
      const double local = seg_length * static_cast<double>(k) / static_cast<double>(pieces);
      samples.push_back(
        {cumulative_s_[i] + local, advance(segment_start_[i], seg.curvature, seg.direction() * local), seg.curvature,
         seg.direction()});
    }
  }
  const Segment & last = segments_.back();
  samples.push_back({length(), end(), last.curvature, last.direction()});
  return samples;
}

}  // namespace delivery_bot_geometry
