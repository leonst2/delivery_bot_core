#include "delivery_bot_collision/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace delivery_bot_collision
{

Grid::Grid(int width, int height, double resolution, double origin_x, double origin_y, std::vector<std::int8_t> data)
: width_(width),
  height_(height),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  data_(std::move(data))
{
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("Grid: width and height must be > 0");
  }
  if (!std::isfinite(resolution) || !(resolution > 0.0)) {
    throw std::invalid_argument("Grid: resolution must be finite and > 0");
  }
  if (data_.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
    throw std::invalid_argument("Grid: data size must be width * height");
  }
}

Grid::Grid(int width, int height, double resolution, double origin_x, double origin_y, std::int8_t fill)
: Grid(
    width, height, resolution, origin_x, origin_y,
    std::vector<std::int8_t>(
      static_cast<std::size_t>(std::max(width, 0)) * static_cast<std::size_t>(std::max(height, 0)), fill))
{
}

}  // namespace delivery_bot_collision
