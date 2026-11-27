#include "delivery_bot_planner/heuristic.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace delivery_bot_planner
{

namespace dbc = delivery_bot_collision;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

double base_clearance(const dbc::Footprint & footprint)
{
  const auto & v = footprint.vertices();
  double clearance = kInf;
  for (std::size_t i = 0; i < v.size(); ++i) {
    const Eigen::Vector2d & a = v[i];
    const Eigen::Vector2d & b = v[(i + 1) % v.size()];
    const Eigen::Vector2d t = (b - a).normalized();
    const Eigen::Vector2d outward{t.y(), -t.x()};
    clearance = std::min(clearance, outward.dot(a));  // distance of the origin inside this edge
  }
  return std::max(clearance, 0.0);
}

DistanceHeuristic::DistanceHeuristic(
  const dbc::CollisionChecker & checker, const Eigen::Vector2d & goal, double base_clearance)
{
  const dbc::Grid & grid = checker.grid();
  const dbc::DistanceField & field = checker.distance_field();
  resolution_ = grid.resolution();
  origin_x_ = grid.origin_x();
  origin_y_ = grid.origin_y();
  width_ = grid.width();
  height_ = grid.height();
  const auto cells = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  const double resolution = resolution_;

  // A cell can hold base_link if its distance (centre to centre) is at least this.
  const double min_free = base_clearance - resolution * std::sqrt(2.0) / 2.0;
  std::vector<std::uint8_t> open(cells, 0);
  for (std::size_t i = 0; i < cells; ++i) {
    open[i] = field.data()[i] >= min_free ? 1 : 0;
  }

  distance_.assign(cells, kInf);
  const int gx = static_cast<int>(std::floor((goal.x() - origin_x_) / resolution));
  const int gy = static_cast<int>(std::floor((goal.y() - origin_y_) / resolution));
  if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
    return;
  }
  run_dijkstra(open, static_cast<std::size_t>(gy) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(gx));
}

void DistanceHeuristic::run_dijkstra(const std::vector<std::uint8_t> & open, std::size_t goal_index)
{
  using Entry = std::pair<double, std::size_t>;  // distance, cell index: ties broken by index (deterministic)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;
  distance_[goal_index] = 0.0;
  queue.emplace(0.0, goal_index);
  const double diagonal = resolution_ * std::sqrt(2.0);
  const auto w = static_cast<std::size_t>(width_);
  while (!queue.empty()) {
    const auto [d, index] = queue.top();
    queue.pop();
    if (d > distance_[index]) {
      continue;
    }
    const auto x = static_cast<int>(index % w);
    const auto y = static_cast<int>(index / w);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
          continue;
        }
        const std::size_t n = static_cast<std::size_t>(ny) * w + static_cast<std::size_t>(nx);
        const double nd = d + ((dx != 0 && dy != 0) ? diagonal : resolution_);
        if (open[n] != 0 && nd < distance_[n]) {
          distance_[n] = nd;
          queue.emplace(nd, n);
        }
      }
    }
  }
}

double DistanceHeuristic::at(double x, double y) const
{
  const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {
    return 0.0;
  }
  return distance_[static_cast<std::size_t>(cy) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(cx)];
}

}  // namespace delivery_bot_planner
