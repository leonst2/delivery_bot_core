#include "delivery_bot_collision/collision_checker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace delivery_bot_collision
{

namespace dbg = delivery_bot_geometry;

namespace
{

// The footprint at a pose in world coordinates, with what the separating-axis test needs per edge. Fixed-size
// arrays: the exact check runs millions of times per second and must not allocate.
struct WorldPolygon
{
  std::size_t count{0};
  std::array<double, kMaxFootprintVertices> vx{};
  std::array<double, kMaxFootprintVertices> vy{};
  std::array<double, kMaxFootprintVertices> nx{};  // outward unit normal of edge i (vertex i -> i + 1)
  std::array<double, kMaxFootprintVertices> ny{};
  std::array<double, kMaxFootprintVertices> lo{};  // min over vertices of normal . v
  std::array<double, kMaxFootprintVertices> hi{};  // normal . v on the edge (the max)
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

WorldPolygon place(const Footprint & footprint, const dbg::Pose2 & pose)
{
  WorldPolygon w;
  const auto & local = footprint.vertices();
  w.count = local.size();
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (std::size_t i = 0; i < w.count; ++i) {
    w.vx[i] = pose.x + c * local[i].x() - s * local[i].y();
    w.vy[i] = pose.y + s * local[i].x() + c * local[i].y();
  }
  w.min_x = w.max_x = w.vx[0];
  w.min_y = w.max_y = w.vy[0];
  for (std::size_t i = 1; i < w.count; ++i) {
    w.min_x = std::min(w.min_x, w.vx[i]);
    w.max_x = std::max(w.max_x, w.vx[i]);
    w.min_y = std::min(w.min_y, w.vy[i]);
    w.max_y = std::max(w.max_y, w.vy[i]);
  }
  for (std::size_t i = 0; i < w.count; ++i) {
    const std::size_t j = i + 1 == w.count ? 0 : i + 1;
    const double ex = w.vx[j] - w.vx[i];
    const double ey = w.vy[j] - w.vy[i];
    const double len = std::hypot(ex, ey);
    w.nx[i] = ey / len;
    w.ny[i] = -ex / len;
    double lo = w.nx[i] * w.vx[0] + w.ny[i] * w.vy[0];
    for (std::size_t k = 1; k < w.count; ++k) {
      lo = std::min(lo, w.nx[i] * w.vx[k] + w.ny[i] * w.vy[k]);
    }
    w.lo[i] = lo;
    w.hi[i] = w.nx[i] * w.vx[i] + w.ny[i] * w.vy[i];
  }
  return w;
}

// Separating-axis test of a convex polygon against the axis-aligned square [x0, x1] x [y0, y1]. Touching counts as
// separated. The square's own axes are covered by the bounding-box test.
bool overlaps(const WorldPolygon & w, double x0, double y0, double x1, double y1)
{
  if (w.max_x <= x0 || w.min_x >= x1 || w.max_y <= y0 || w.min_y >= y1) {
    return false;
  }
  for (std::size_t i = 0; i < w.count; ++i) {
    // Projection range of the square onto the edge normal: centre +- half extent.
    const double centre = w.nx[i] * (x0 + x1) / 2.0 + w.ny[i] * (y0 + y1) / 2.0;
    const double half = (std::abs(w.nx[i]) * (x1 - x0) + std::abs(w.ny[i]) * (y1 - y0)) / 2.0;
    if (centre - half >= w.hi[i] || centre + half <= w.lo[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

CollisionChecker::CollisionChecker(Grid grid, Footprint footprint, CheckerParams params)
: grid_(std::move(grid)),
  params_(params),
  field_(compute_distance_field(grid_, params_.rule)),
  body_{std::move(footprint), {}},
  swept_{body_.polygon, {}}
{
  if (body_.polygon.vertices().size() > kMaxFootprintVertices) {
    throw std::invalid_argument("CollisionChecker: footprints are limited to kMaxFootprintVertices vertices");
  }
  if (params_.cover_circles < 1) {
    throw std::invalid_argument("CollisionChecker: cover_circles must be >= 1");
  }
  if (!(params_.sweep_step > 0.0) || !std::isfinite(params_.sweep_step)) {
    throw std::invalid_argument("CollisionChecker: sweep_step must be finite and > 0");
  }
  body_.circles = body_.polygon.circle_cover(params_.cover_circles);
  swept_.polygon = body_.polygon.padded(params_.sweep_step / 2.0);
  swept_.circles = swept_.polygon.circle_cover(params_.cover_circles);
}

bool CollisionChecker::in_collision(const dbg::Pose2 & pose) const { return check(body_, pose); }

bool CollisionChecker::in_collision_exact(const dbg::Pose2 & pose) const { return exact(body_.polygon, pose); }

bool CollisionChecker::check(const Shape & shape, const dbg::Pose2 & pose) const
{
  return !fast_free(shape, pose) && exact(shape.polygon, pose);
}

bool CollisionChecker::fast_free(const Shape & shape, const dbg::Pose2 & pose) const
{
  // A point p is at least (field at p's cell) - resolution * sqrt(2) from every obstacle square: half a diagonal
  // from p to its cell centre, half a diagonal from the obstacle centre to its square. If every circle of the cover
  // clears that bound, the footprint is free. The outside ring of the field makes this cover the map border too.
  const double slack = grid_.resolution() * std::sqrt(2.0);
  return std::all_of(shape.circles.begin(), shape.circles.end(), [&](const Circle & circle) {
    const Eigen::Vector2d c = pose.transform(circle.center);
    const CellIndex cell = grid_.world_to_cell(c.x(), c.y());
    return grid_.contains(cell.x, cell.y) && field_.at(cell.x, cell.y) - slack > circle.radius;
  });
}

bool CollisionChecker::exact(const Footprint & polygon, const dbg::Pose2 & pose) const
{
  const WorldPolygon w = place(polygon, pose);
  if (params_.rule.outside_is_obstacle()) {
    // A convex polygon leaves the (convex) map rectangle iff one of its vertices does.
    for (std::size_t i = 0; i < w.count; ++i) {
      if (!grid_.contains_world(w.vx[i], w.vy[i])) {
        return true;
      }
    }
  }
  const double res = grid_.resolution();
  const CellIndex lo = grid_.world_to_cell(w.min_x, w.min_y);
  const CellIndex hi = grid_.world_to_cell(w.max_x, w.max_y);
  const int x0 = std::max(lo.x, 0);
  const int y0 = std::max(lo.y, 0);
  const int x1 = std::min(hi.x, grid_.width() - 1);
  const int y1 = std::min(hi.y, grid_.height() - 1);
  for (int y = y0; y <= y1; ++y) {
    const double cy0 = grid_.origin_y() + y * res;
    for (int x = x0; x <= x1; ++x) {
      if (!params_.rule.is_obstacle(grid_.at(x, y))) {
        continue;
      }
      const double cx0 = grid_.origin_x() + x * res;
      if (overlaps(w, cx0, cy0, cx0 + res, cy0 + res)) {
        return true;
      }
    }
  }
  return false;
}

double CollisionChecker::clearance_lower_bound(const dbg::Pose2 & pose) const
{
  const double slack = grid_.resolution() * std::sqrt(2.0);
  double clearance = std::numeric_limits<double>::infinity();
  for (const Circle & circle : body_.circles) {
    const Eigen::Vector2d c = pose.transform(circle.center);
    const CellIndex cell = grid_.world_to_cell(c.x(), c.y());
    if (!grid_.contains(cell.x, cell.y)) {
      return 0.0;
    }
    clearance = std::min(clearance, field_.at(cell.x, cell.y) - slack - circle.radius);
  }
  return std::max(clearance, 0.0);
}

std::vector<dbg::Pose2> CollisionChecker::sweep_samples(const dbg::Pose2 & start, const dbg::Segment & segment) const
{
  // A footprint point at distance rho from base_link moves at most (1 + |curvature| * rho) per metre of travel.
  const double length = std::abs(segment.length);
  const double factor = 1.0 + std::abs(segment.curvature) * body_.polygon.circumscribed_radius();
  const auto count = static_cast<std::size_t>(std::max(1.0, std::ceil(length * factor / params_.sweep_step)));
  std::vector<dbg::Pose2> poses;
  poses.reserve(count + 1);
  for (std::size_t k = 0; k <= count; ++k) {
    const double s = length * static_cast<double>(k) / static_cast<double>(count);
    poses.push_back(dbg::advance(start, segment.curvature, segment.direction() * s));
  }
  return poses;
}

bool CollisionChecker::segment_in_collision(const dbg::Pose2 & start, const dbg::Segment & segment) const
{
  const std::vector<dbg::Pose2> poses = sweep_samples(start, segment);
  return std::any_of(poses.begin(), poses.end(), [this](const dbg::Pose2 & pose) { return check(swept_, pose); });
}

bool CollisionChecker::path_in_collision(const dbg::Path & path) const
{
  if (path.empty()) {
    return in_collision(path.start());
  }
  for (std::size_t i = 0; i < path.segments().size(); ++i) {
    if (segment_in_collision(path.segment_start_pose(i), path.segments()[i])) {
      return true;
    }
  }
  return false;
}

}  // namespace delivery_bot_collision
