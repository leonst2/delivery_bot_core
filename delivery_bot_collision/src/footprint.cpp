#include "delivery_bot_collision/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace delivery_bot_collision
{

namespace
{

double cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b) { return a.x() * b.y() - a.y() * b.x(); }

// Outward unit normal of the edge from a to b of a counter-clockwise polygon.
Eigen::Vector2d outward_normal(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d t = (b - a).normalized();
  return {t.y(), -t.x()};
}

}  // namespace

Footprint::Footprint(std::vector<Eigen::Vector2d> vertices)
: vertices_(std::move(vertices))
{
  const std::size_t n = vertices_.size();
  if (n < 3) {
    throw std::invalid_argument("Footprint: at least three vertices required");
  }
  for (const auto & v : vertices_) {
    if (!v.allFinite()) {
      throw std::invalid_argument("Footprint: vertices must be finite");
    }
  }
  double area2 = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    area2 += cross(vertices_[i], vertices_[(i + 1) % n]);
  }
  if (std::abs(area2) < 1e-12) {
    throw std::invalid_argument("Footprint: zero area");
  }
  if (area2 < 0.0) {
    std::reverse(vertices_.begin(), vertices_.end());
  }
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d & a = vertices_[i];
    const Eigen::Vector2d & b = vertices_[(i + 1) % n];
    const Eigen::Vector2d & c = vertices_[(i + 2) % n];
    if (cross(b - a, c - b) <= 1e-12) {
      throw std::invalid_argument("Footprint: polygon must be strictly convex");
    }
  }
}

Footprint Footprint::rectangle(double x_min, double x_max, double y_min, double y_max)
{
  if (!(x_max > x_min) || !(y_max > y_min)) {
    throw std::invalid_argument("Footprint::rectangle: empty rectangle");
  }
  return Footprint({{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}});
}

Footprint Footprint::padded(double distance) const
{
  if (!(distance >= 0.0)) {
    throw std::invalid_argument("Footprint::padded: distance must be >= 0");
  }
  const std::size_t n = vertices_.size();
  std::vector<Eigen::Vector2d> out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d & prev = vertices_[(i + n - 1) % n];
    const Eigen::Vector2d & v = vertices_[i];
    const Eigen::Vector2d & next = vertices_[(i + 1) % n];
    const Eigen::Vector2d n1 = outward_normal(prev, v);
    const Eigen::Vector2d n2 = outward_normal(v, next);
    // Mitre: the intersection of both edges moved out by `distance`.
    out.emplace_back(v + distance * (n1 + n2) / (1.0 + n1.dot(n2)));
  }
  return Footprint(std::move(out));
}

Eigen::Vector2d Footprint::centroid() const
{
  Eigen::Vector2d sum{0.0, 0.0};
  for (const auto & v : vertices_) {
    sum += v;
  }
  return sum / static_cast<double>(vertices_.size());
}

double Footprint::inscribed_radius() const
{
  const Eigen::Vector2d c = centroid();
  const std::size_t n = vertices_.size();
  double r = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d & a = vertices_[i];
    const Eigen::Vector2d & b = vertices_[(i + 1) % n];
    r = std::min(r, -outward_normal(a, b).dot(c - a));  // distance of c inside the edge's line
  }
  return r;
}

double Footprint::circumscribed_radius() const
{
  double r = 0.0;
  for (const auto & v : vertices_) {
    r = std::max(r, v.norm());
  }
  return r;
}

Eigen::Vector2d Footprint::min_corner() const
{
  Eigen::Vector2d m = vertices_.front();
  for (const auto & v : vertices_) {
    m = m.cwiseMin(v);
  }
  return m;
}

Eigen::Vector2d Footprint::max_corner() const
{
  Eigen::Vector2d m = vertices_.front();
  for (const auto & v : vertices_) {
    m = m.cwiseMax(v);
  }
  return m;
}

std::vector<Circle> Footprint::circle_cover(int count) const
{
  if (count < 1) {
    throw std::invalid_argument("Footprint::circle_cover: count must be >= 1");
  }
  const Eigen::Vector2d lo = min_corner();
  const Eigen::Vector2d hi = max_corner();
  const Eigen::Vector2d size = hi - lo;
  const bool along_x = size.x() >= size.y();
  const double length = along_x ? size.x() : size.y();
  const double width = along_x ? size.y() : size.x();
  const double piece = length / count;
  // Each circle circumscribes one piece of the bounding box, so together they cover the box and the footprint.
  const double radius = std::hypot(piece / 2.0, width / 2.0);
  std::vector<Circle> circles;
  circles.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double along = (along_x ? lo.x() : lo.y()) + (i + 0.5) * piece;
    const double across = along_x ? (lo.y() + hi.y()) / 2.0 : (lo.x() + hi.x()) / 2.0;
    circles.push_back({along_x ? Eigen::Vector2d{along, across} : Eigen::Vector2d{across, along}, radius});
  }
  return circles;
}

Footprint FootprintParams::make() const { return Footprint::rectangle(x_min, x_max, y_min, y_max).padded(padding); }

}  // namespace delivery_bot_collision
