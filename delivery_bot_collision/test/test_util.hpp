// Helpers for the collision tests: fixed-seed random grids and poses.

#ifndef DELIVERY_BOT_COLLISION_TEST_UTIL_HPP_
#define DELIVERY_BOT_COLLISION_TEST_UTIL_HPP_

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "delivery_bot_collision/grid.hpp"
#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/se2.hpp"

namespace delivery_bot_collision::test
{

class Random
{
public:
  explicit Random(std::uint32_t seed)
  : engine_(seed)
  {
  }

  double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(engine_); }
  int integer(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(engine_); }

  /// A grid where each cell is occupied (100) with probability `occupied`, unknown (-1) with probability `unknown`,
  /// else free with a random value below the occupied threshold.
  Grid grid(int width, int height, double resolution, double occupied, double unknown)
  {
    std::vector<std::int8_t> data(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    for (auto & cell : data) {
      const double r = uniform(0.0, 1.0);
      if (r < occupied) {
        cell = 100;
      } else if (r < occupied + unknown) {
        cell = -1;
      } else {
        cell = static_cast<std::int8_t>(integer(0, 64));
      }
    }
    return {width, height, resolution, uniform(-3.0, 3.0), uniform(-3.0, 3.0), std::move(data)};
  }

  delivery_bot_geometry::Pose2 pose_in(const Grid & grid, double margin)
  {
    return {
      uniform(grid.origin_x() - margin, grid.origin_x() + grid.width() * grid.resolution() + margin),
      uniform(grid.origin_y() - margin, grid.origin_y() + grid.height() * grid.resolution() + margin),
      uniform(-delivery_bot_geometry::kPi, delivery_bot_geometry::kPi)};
  }

private:
  std::mt19937 engine_;
};

}  // namespace delivery_bot_collision::test

#endif  // DELIVERY_BOT_COLLISION_TEST_UTIL_HPP_
