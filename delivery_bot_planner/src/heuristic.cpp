#include "delivery_bot_planner/heuristic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
  const dbc::CollisionChecker & checker, const Eigen::Vector2d & goal, double clearance)
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

  // A cell can hold the reference point if its distance (centre to centre) is at least this.
  const double min_free = clearance - resolution * std::sqrt(2.0) / 2.0;
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
  // Dial's algorithm: integer octile costs (10 straight, 14 diagonal, i.e. sqrt(2) ~ 1.4) and a circular array of
  // buckets, O(cells) instead of a binary heap. The 1 % under sqrt(2) doesn't matter for a heuristic. Buckets are
  // processed in a fixed order, so the result is deterministic.
  constexpr std::uint32_t kStraight = 10;
  constexpr std::uint32_t kDiagonal = 14;
  constexpr std::size_t kBuckets = kDiagonal + 1;
  constexpr std::uint32_t kUnreached = std::numeric_limits<std::uint32_t>::max();
  struct Neighbour
  {
    int dx;
    int dy;
    std::uint32_t cost;
  };
  static constexpr std::array<Neighbour, 8> kNeighbours{{
    {1, 0, kStraight},
    {-1, 0, kStraight},
    {0, 1, kStraight},
    {0, -1, kStraight},
    {1, 1, kDiagonal},
    {1, -1, kDiagonal},
    {-1, 1, kDiagonal},
    {-1, -1, kDiagonal},
  }};
  const auto w = static_cast<std::size_t>(width_);
  std::vector<std::uint32_t> cost(distance_.size(), kUnreached);
  std::array<std::vector<std::size_t>, kBuckets> buckets;
  cost[goal_index] = 0;
  buckets[0].push_back(goal_index);
  std::size_t pending = 1;
  for (std::uint32_t d = 0; pending > 0; ++d) {
    std::vector<std::size_t> & bucket = buckets[d % kBuckets];
    while (!bucket.empty()) {
      const std::size_t index = bucket.back();
      bucket.pop_back();
      --pending;
      if (cost[index] != d) {
        continue;  // reached more cheaply after it was queued here
      }
      const auto x = static_cast<int>(index % w);
      const auto y = static_cast<int>(index / w);
      for (const Neighbour & nb : kNeighbours) {
        const int nx = x + nb.dx;
        const int ny = y + nb.dy;
        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
          continue;
        }
        const std::size_t n = static_cast<std::size_t>(ny) * w + static_cast<std::size_t>(nx);
        const std::uint32_t nd = d + nb.cost;
        if (open[n] != 0 && nd < cost[n]) {
          cost[n] = nd;
          buckets[nd % kBuckets].push_back(n);
          ++pending;
        }
      }
    }
  }
  const double unit = resolution_ / static_cast<double>(kStraight);
  for (std::size_t i = 0; i < cost.size(); ++i) {
    distance_[i] = cost[i] == kUnreached ? kInf : cost[i] * unit;
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
