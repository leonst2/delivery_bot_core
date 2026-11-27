// Robot footprint (delivery_bot_core#15): a convex polygon in base_link (x forward, y left, origin on the ground
// under the rear-axle centre).

#ifndef DELIVERY_BOT_COLLISION_FOOTPRINT_HPP_
#define DELIVERY_BOT_COLLISION_FOOTPRINT_HPP_

#include <vector>

#include <Eigen/Core>

namespace delivery_bot_collision
{

struct Circle
{
  Eigen::Vector2d center{0.0, 0.0};  // base_link
  double radius{0.0};
};

class Footprint
{
public:
  /// A convex polygon with at least three vertices, in either orientation (stored counter-clockwise). Throws
  /// std::invalid_argument for fewer vertices, zero area or a non-convex polygon.
  explicit Footprint(std::vector<Eigen::Vector2d> vertices);

  /// The axis-aligned rectangle [x_min, x_max] x [y_min, y_max].
  static Footprint rectangle(double x_min, double x_max, double y_min, double y_max);

  /// Counter-clockwise vertices in base_link.
  [[nodiscard]] const std::vector<Eigen::Vector2d> & vertices() const { return vertices_; }

  /// Every edge moved outward by `distance` >= 0 (mitred corners). Contains every point within `distance` of the
  /// footprint, so it is a safe (slightly larger) stand-in for the footprint grown by a disk.
  [[nodiscard]] Footprint padded(double distance) const;

  /// Mean of the vertices.
  [[nodiscard]] Eigen::Vector2d centroid() const;
  /// Radius of the largest circle around the centroid that fits inside (exact for rectangles, a lower bound of the
  /// inscribed radius for other shapes).
  [[nodiscard]] double inscribed_radius() const;
  /// Largest distance of a vertex from the base_link origin: no footprint point is farther away.
  [[nodiscard]] double circumscribed_radius() const;

  /// Lower-left and upper-right corner of the axis-aligned bounding box in base_link.
  [[nodiscard]] Eigen::Vector2d min_corner() const;
  [[nodiscard]] Eigen::Vector2d max_corner() const;

  /// `count` >= 1 equal circles along the longer side of the bounding box that together cover the footprint.
  [[nodiscard]] std::vector<Circle> circle_cover(int count) const;

private:
  std::vector<Eigen::Vector2d> vertices_;
};

/// Footprint parameters as the nodes read them. The defaults are the URDF chassis collision box, which also contains
/// all four wheels: x -0.085 ... 0.575 m, y -0.27 ... 0.27 m (0.66 x 0.54 m) around base_link.
struct FootprintParams
{
  double x_min{-0.085};
  double x_max{0.575};
  double y_min{-0.27};
  double y_max{0.27};
  double padding{0.05};  // m, safety margin on every side

  /// The padded rectangle. Throws std::invalid_argument for an empty rectangle or negative padding.
  [[nodiscard]] Footprint make() const;
};

}  // namespace delivery_bot_collision

#endif  // DELIVERY_BOT_COLLISION_FOOTPRINT_HPP_
