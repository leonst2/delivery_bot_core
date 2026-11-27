#include "delivery_bot_collision/distance_field.hpp"

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
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

DistanceField::DistanceField(
  int width, int height, double resolution, double origin_x, double origin_y, std::vector<double> m)
: width_(width),
  height_(height),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  distance_(std::move(m))
{
  if (distance_.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
    throw std::invalid_argument("DistanceField: data size must be width * height");
  }
}

void distance_transform_1d(const std::vector<double> & f, std::vector<double> & d)
{
  // Lower envelope of the parabolas (q - p)^2 + f[p] over the finite samples p only: with an "infinity" of 1e20
  // as in the paper, the intersection formula would lose the q^2 terms to rounding.
  const std::size_t n = f.size();
  d.assign(n, kInf);
  std::vector<std::size_t> v(n);  // parabola apexes of the envelope
  std::vector<double> z(n + 1);   // boundaries between them
  std::size_t k = 0;
  bool any = false;
  const auto intersection = [&f](std::size_t q, std::size_t p) {
    const auto qd = static_cast<double>(q);
    const auto pd = static_cast<double>(p);
    return ((f[q] + qd * qd) - (f[p] + pd * pd)) / (2.0 * qd - 2.0 * pd);
  };
  for (std::size_t q = 0; q < n; ++q) {
    if (!std::isfinite(f[q])) {
      continue;
    }
    if (!any) {
      any = true;
      k = 0;
      v[0] = q;
      z[0] = -kInf;
      z[1] = kInf;
      continue;
    }
    double s = intersection(q, v[k]);
    while (s <= z[k]) {
      --k;  // z[0] = -inf, so k never goes below 0
      s = intersection(q, v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  if (!any) {
    return;
  }
  k = 0;
  for (std::size_t q = 0; q < n; ++q) {
    const auto qd = static_cast<double>(q);
    while (z[k + 1] < qd) {
      ++k;
    }
    const auto offset = qd - static_cast<double>(v[k]);
    d[q] = offset * offset + f[v[k]];
  }
}

DistanceField compute_distance_field(const Grid & grid, const OccupancyRule & rule)
{
  // With an obstacle outside the map, work on a grid with a one-cell obstacle ring around it.
  const int pad = rule.outside_is_obstacle() ? 1 : 0;
  const int w = grid.width() + 2 * pad;
  const int h = grid.height() + 2 * pad;
  const auto uw = static_cast<std::size_t>(w);
  const auto uh = static_cast<std::size_t>(h);

  std::vector<double> squared(uw * uh, kInf);  // in cells^2
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int gx = x - pad;
      const int gy = y - pad;
      const bool obstacle = grid.contains(gx, gy) ? rule.is_obstacle(grid.at(gx, gy)) : true;
      if (obstacle) {
        squared[static_cast<std::size_t>(y) * uw + static_cast<std::size_t>(x)] = 0.0;
      }
    }
  }

  // Columns, then rows.
  std::vector<double> f(uh);
  std::vector<double> d;
  for (std::size_t x = 0; x < uw; ++x) {
    for (std::size_t y = 0; y < uh; ++y) {
      f[y] = squared[y * uw + x];
    }
    distance_transform_1d(f, d);
    for (std::size_t y = 0; y < uh; ++y) {
      squared[y * uw + x] = d[y];
    }
  }
  f.resize(uw);
  for (std::size_t y = 0; y < uh; ++y) {
    for (std::size_t x = 0; x < uw; ++x) {
      f[x] = squared[y * uw + x];
    }
    distance_transform_1d(f, d);
    for (std::size_t x = 0; x < uw; ++x) {
      squared[y * uw + x] = d[x];
    }
  }

  std::vector<double> metres(static_cast<std::size_t>(grid.width()) * static_cast<std::size_t>(grid.height()));
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const double sq = squared[static_cast<std::size_t>(y + pad) * uw + static_cast<std::size_t>(x + pad)];
      metres[grid.index(x, y)] = std::sqrt(sq) * grid.resolution();
    }
  }
  return {grid.width(), grid.height(), grid.resolution(), grid.origin_x(), grid.origin_y(), std::move(metres)};
}

}  // namespace delivery_bot_collision
