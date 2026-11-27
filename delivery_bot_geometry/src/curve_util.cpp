#include "curve_util.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace delivery_bot_geometry::detail
{

void check_radius(double radius)
{
  if (!std::isfinite(radius) || !(radius > 0.0)) {
    throw std::invalid_argument("turning radius must be finite and > 0");
  }
}

CurveCandidate make_candidate(
  const Pose2 & start, double radius, const std::string & letters, const double * values, std::size_t count)
{
  if (letters.size() != count) {
    throw std::logic_error("make_candidate: one letter per value expected");
  }
  std::vector<Segment> segments;
  segments.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double length = values[i] * radius;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    switch (letters[i]) {
      case 'L':
        segments.push_back(Segment::left(length, radius));
        break;
      case 'R':
        segments.push_back(Segment::right(length, radius));
        break;
      case 'S':
        segments.push_back(Segment::straight(length));
        break;
      default:
        throw std::logic_error("make_candidate: unknown letter in word " + letters);
    }
  }
  return {letters, Path(start, std::move(segments))};
}

CurveCandidate shortest(std::vector<CurveCandidate> candidates)
{
  if (candidates.empty()) {
    throw std::logic_error("shortest: no candidates");
  }
  std::size_t best = 0;
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    if (candidates[i].path.length() < candidates[best].path.length()) {
      best = i;
    }
  }
  return std::move(candidates[best]);
}

}  // namespace delivery_bot_geometry::detail
