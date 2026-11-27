// Exact Euclidean distance fields (delivery_bot_core#15).

#ifndef DELIVERY_BOT_COLLISION_DISTANCE_FIELD_HPP_
#define DELIVERY_BOT_COLLISION_DISTANCE_FIELD_HPP_

#include <vector>

#include "delivery_bot_collision/grid.hpp"

namespace delivery_bot_collision
{

/// For every cell, the exact Euclidean distance (m) from its centre to the centre of the nearest obstacle cell.
/// 0 on obstacle cells, +infinity if there is no obstacle at all. Same geometry as the grid it was computed from.
class DistanceField
{
public:
  DistanceField() = default;
  DistanceField(int width, int height, double resolution, double origin_x, double origin_y, std::vector<double> m);

  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] double resolution() const { return resolution_; }
  [[nodiscard]] double origin_x() const { return origin_x_; }
  [[nodiscard]] double origin_y() const { return origin_y_; }
  [[nodiscard]] const std::vector<double> & data() const { return distance_; }

  /// Distance of an in-map cell (unchecked).
  [[nodiscard]] double at(int x, int y) const
  {
    return distance_[static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x)];
  }

private:
  int width_{0};
  int height_{0};
  double resolution_{1.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<double> distance_;
};

/// Computes the distance field with the two-pass lower-envelope transform of Felzenszwalb & Huttenlocher,
/// "Distance Transforms of Sampled Functions" (2012): O(cells), exact. When the rule treats unknown as lethal, the
/// outside of the map is an obstacle too (as a ring of obstacle cells around the map), so distances also measure
/// how far a cell is from leaving the map.
DistanceField compute_distance_field(const Grid & grid, const OccupancyRule & rule);

/// Exact squared distance transform of one sampled function (exposed for tests): d[q] = min_p (q - p)^2 + f[p],
/// where f[p] = +infinity marks "no obstacle". Entries that are all infinite give infinity.
void distance_transform_1d(const std::vector<double> & f, std::vector<double> & d);

}  // namespace delivery_bot_collision

#endif  // DELIVERY_BOT_COLLISION_DISTANCE_FIELD_HPP_
