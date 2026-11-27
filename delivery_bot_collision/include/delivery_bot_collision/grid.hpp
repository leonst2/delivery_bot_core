// Occupancy grids for collision checking (delivery_bot_core#15).
//
// Same layout as nav_msgs/OccupancyGrid, so nodes can copy the message data in directly: row-major, cell (0, 0) is
// the lower-left cell, its lower-left corner is the origin, values -1 = unknown and 0 ... 100 = occupancy (costmaps
// use 100 = lethal). The grid is aligned with the map frame (origin yaw 0, like the maps exported from Unity).

#ifndef DELIVERY_BOT_COLLISION_GRID_HPP_
#define DELIVERY_BOT_COLLISION_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace delivery_bot_collision
{

struct CellIndex
{
  int x{0};  // column
  int y{0};  // row
};

class Grid
{
public:
  Grid() = default;
  /// Throws std::invalid_argument for non-positive sizes or resolution, or data of the wrong size.
  Grid(int width, int height, double resolution, double origin_x, double origin_y, std::vector<std::int8_t> data);
  /// A grid with every cell set to `fill`.
  Grid(int width, int height, double resolution, double origin_x, double origin_y, std::int8_t fill);

  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] double resolution() const { return resolution_; }
  [[nodiscard]] double origin_x() const { return origin_x_; }
  [[nodiscard]] double origin_y() const { return origin_y_; }
  [[nodiscard]] const std::vector<std::int8_t> & data() const { return data_; }

  [[nodiscard]] bool contains(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }
  [[nodiscard]] std::size_t index(int x, int y) const
  {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  }
  /// Value of an in-map cell (unchecked).
  [[nodiscard]] std::int8_t at(int x, int y) const { return data_[index(x, y)]; }
  void set(int x, int y, std::int8_t value) { data_[index(x, y)] = value; }

  /// The cell that contains a world point (may lie outside the map: check with contains()).
  [[nodiscard]] CellIndex world_to_cell(double wx, double wy) const;
  /// World coordinates of the centre of cell (x, y).
  [[nodiscard]] Eigen::Vector2d cell_center(int x, int y) const;
  /// True if the world point lies inside the map rectangle.
  [[nodiscard]] bool contains_world(double wx, double wy) const;

private:
  int width_{0};
  int height_{0};
  double resolution_{1.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<std::int8_t> data_;
};

enum class UnknownPolicy : std::uint8_t {
  kLethal,  // unknown cells, and everything outside the map, are obstacles
  kFree,    // unknown cells, and everything outside the map, are free
};

/// Which cells are obstacles. Outside the map counts as unknown.
struct OccupancyRule
{
  int occupied_threshold{65};  // value >= threshold is an obstacle (ROS map_server's occupied_thresh 0.65)
  UnknownPolicy unknown{UnknownPolicy::kLethal};

  [[nodiscard]] bool is_obstacle(std::int8_t value) const
  {
    return value < 0 ? unknown == UnknownPolicy::kLethal : value >= occupied_threshold;
  }
  [[nodiscard]] bool outside_is_obstacle() const { return unknown == UnknownPolicy::kLethal; }
};

}  // namespace delivery_bot_collision

#endif  // DELIVERY_BOT_COLLISION_GRID_HPP_
