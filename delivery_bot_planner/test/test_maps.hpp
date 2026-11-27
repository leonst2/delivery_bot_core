// Synthetic maps for the planner tests (the card's mocks): open field, corridor with an L-corner, U-turn dead end,
// parking bay, narrow gap, enclosed goal. 0.1 m cells, origin (0, 0).

#ifndef DELIVERY_BOT_PLANNER_TEST_MAPS_HPP_
#define DELIVERY_BOT_PLANNER_TEST_MAPS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_collision/grid.hpp"

namespace delivery_bot_planner::test
{

namespace dbc = delivery_bot_collision;

class MapBuilder
{
public:
  MapBuilder(double width_m, double height_m)
  : grid_(
      static_cast<int>(std::lround(width_m / kRes)), static_cast<int>(std::lround(height_m / kRes)), kRes, 0.0, 0.0,
      std::int8_t{0})
  {
  }

  /// Fills the rectangle [x0, x1] x [y0, y1] (m) with obstacle cells.
  MapBuilder & box(double x0, double y0, double x1, double y1)
  {
    const auto from_x = std::max(0, static_cast<int>(std::floor(x0 / kRes + 1e-9)));
    const auto from_y = std::max(0, static_cast<int>(std::floor(y0 / kRes + 1e-9)));
    const auto to_x = std::min(grid_.width(), static_cast<int>(std::ceil(x1 / kRes - 1e-9)));
    const auto to_y = std::min(grid_.height(), static_cast<int>(std::ceil(y1 / kRes - 1e-9)));
    for (int y = from_y; y < to_y; ++y) {
      for (int x = from_x; x < to_x; ++x) {
        grid_.set(x, y, 100);
      }
    }
    return *this;
  }

  /// A one-cell (0.1 m) wall around the whole map.
  MapBuilder & border()
  {
    const double w = grid_.width() * kRes;
    const double h = grid_.height() * kRes;
    return box(0.0, 0.0, w, kRes).box(0.0, h - kRes, w, h).box(0.0, 0.0, kRes, h).box(w - kRes, 0.0, w, h);
  }

  [[nodiscard]] std::shared_ptr<const dbc::CollisionChecker> checker() const
  {
    return std::make_shared<const dbc::CollisionChecker>(grid_, dbc::FootprintParams{}.make(), dbc::CheckerParams{});
  }

  [[nodiscard]] const dbc::Grid & grid() const { return grid_; }

  static constexpr double kRes = 0.1;

private:
  dbc::Grid grid_;
};

constexpr int kCitySize = 1000;  // cells per side

/// 100 x 100 m city-like map at 0.1 m: blocks between 8 m streets, parked cars along the kerbs, clutter.
inline dbc::Grid city(std::uint32_t seed)
{
  std::mt19937 rng(seed);
  const auto uniform_int = [&rng](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
  dbc::Grid grid(kCitySize, kCitySize, 0.1, 0.0, 0.0, std::int8_t{0});
  const auto fill = [&grid](int x0, int y0, int x1, int y1) {
    for (int y = std::max(y0, 0); y < std::min(y1, kCitySize); ++y) {
      for (int x = std::max(x0, 0); x < std::min(x1, kCitySize); ++x) {
        grid.set(x, y, 100);
      }
    }
  };
  for (int bx = 0; bx < kCitySize; bx += 200) {
    for (int by = 0; by < kCitySize; by += 200) {
      fill(bx + 80, by + 80, bx + 200, by + 200);  // block, 8 m street on two sides
      for (int car = 0; car < 4; ++car) {
        const int cx = bx + uniform_int(85, 180);
        fill(cx, by + 70, cx + 45, by + 78);  // 4.5 x 0.8 m along the kerb
      }
    }
  }
  for (int i = 0; i < 300; ++i) {
    const int x = uniform_int(0, kCitySize - 1);
    const int y = uniform_int(0, kCitySize - 1);
    fill(x, y, x + 2, y + 2);
  }
  return grid;
}

}  // namespace delivery_bot_planner::test

#endif  // DELIVERY_BOT_PLANNER_TEST_MAPS_HPP_
