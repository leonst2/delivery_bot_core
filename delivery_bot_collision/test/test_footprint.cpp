#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "delivery_bot_collision/footprint.hpp"
#include "test_util.hpp"

namespace dbc = delivery_bot_collision;

namespace
{
constexpr double kTol = 1e-12;

// Signed distance of a point to a convex CCW polygon (< 0 inside).
double signed_distance(const dbc::Footprint & fp, const Eigen::Vector2d & p)
{
  const auto & v = fp.vertices();
  const std::size_t n = v.size();
  bool inside = true;
  double outside_dist = 1e300;
  double inside_dist = 1e300;
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d & a = v[i];
    const Eigen::Vector2d & b = v[(i + 1) % n];
    const Eigen::Vector2d ab = b - a;
    const double side = ab.x() * (p - a).y() - ab.y() * (p - a).x();
    inside = inside && side >= 0.0;
    const double t = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
    const double d = (a + t * ab - p).norm();
    outside_dist = std::min(outside_dist, d);
    inside_dist = std::min(inside_dist, side / ab.norm());
  }
  return inside ? -inside_dist : outside_dist;
}
}  // namespace

TEST(Footprint, RobotDefaultIsTheUrdfChassisBox)
{
  const dbc::FootprintParams params;
  EXPECT_DOUBLE_EQ(params.x_min, -0.085);
  EXPECT_DOUBLE_EQ(params.x_max, 0.575);
  EXPECT_DOUBLE_EQ(params.y_min, -0.27);
  EXPECT_DOUBLE_EQ(params.y_max, 0.27);
  const dbc::Footprint fp = params.make();
  // Padded by 0.05 m on every side: x -0.135 ... 0.625, y -0.32 ... 0.32.
  EXPECT_NEAR(fp.min_corner().x(), -0.135, kTol);
  EXPECT_NEAR(fp.max_corner().x(), 0.625, kTol);
  EXPECT_NEAR(fp.min_corner().y(), -0.32, kTol);
  EXPECT_NEAR(fp.max_corner().y(), 0.32, kTol);
  EXPECT_NEAR(fp.inscribed_radius(), 0.32, kTol);
  EXPECT_NEAR(fp.circumscribed_radius(), std::hypot(0.625, 0.32), kTol);
}

TEST(Footprint, RectangleRadii)
{
  const auto fp = dbc::Footprint::rectangle(-1.0, 3.0, -0.5, 0.5);
  EXPECT_NEAR(fp.centroid().x(), 1.0, kTol);
  EXPECT_NEAR(fp.centroid().y(), 0.0, kTol);
  EXPECT_NEAR(fp.inscribed_radius(), 0.5, kTol);
  EXPECT_NEAR(fp.circumscribed_radius(), std::hypot(3.0, 0.5), kTol);
}

TEST(Footprint, OrientationAndValidation)
{
  // Clockwise input is stored counter-clockwise.
  const dbc::Footprint cw({{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}});
  const auto & v = cw.vertices();
  double area2 = 0.0;
  for (std::size_t i = 0; i < v.size(); ++i) {
    const auto & a = v[i];
    const auto & b = v[(i + 1) % v.size()];
    area2 += a.x() * b.y() - a.y() * b.x();
  }
  EXPECT_GT(area2, 0.0);
  EXPECT_THROW(dbc::Footprint({{0.0, 0.0}, {1.0, 0.0}}), std::invalid_argument);
  EXPECT_THROW(dbc::Footprint({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}), std::invalid_argument);
  // A dent: not convex.
  EXPECT_THROW(dbc::Footprint({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {1.0, 0.5}, {0.0, 2.0}}), std::invalid_argument);
  EXPECT_THROW(dbc::Footprint::rectangle(1.0, 1.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(dbc::Footprint::rectangle(0.0, 1.0, 0.0, 1.0).padded(-0.1), std::invalid_argument);
}

TEST(Footprint, PaddingGrowsEveryEdge)
{
  const auto rect = dbc::Footprint::rectangle(-0.2, 0.6, -0.3, 0.3).padded(0.1);
  EXPECT_NEAR(rect.min_corner().x(), -0.3, kTol);
  EXPECT_NEAR(rect.max_corner().x(), 0.7, kTol);
  EXPECT_NEAR(rect.min_corner().y(), -0.4, kTol);
  EXPECT_NEAR(rect.max_corner().y(), 0.4, kTol);

  // A hexagon: every point within d of the footprint is inside the padded one, and the padded edges are exactly d
  // further out.
  const dbc::Footprint hexagon({{1.0, 0.0}, {0.5, 0.8}, {-0.5, 0.8}, {-1.0, 0.0}, {-0.5, -0.8}, {0.5, -0.8}});
  const double d = 0.15;
  const auto grown = hexagon.padded(d);
  dbc::test::Random rng(200);
  for (int i = 0; i < 20000; ++i) {
    const Eigen::Vector2d p{rng.uniform(-1.5, 1.5), rng.uniform(-1.2, 1.2)};
    if (signed_distance(hexagon, p) <= d) {
      EXPECT_LE(signed_distance(grown, p), 1e-12) << p.transpose();
    }
  }
  EXPECT_NEAR(grown.inscribed_radius(), hexagon.inscribed_radius() + d, 1e-9);
}

TEST(Footprint, CircleCoverContainsTheFootprint)
{
  dbc::test::Random rng(201);
  for (const auto & fp :
       {dbc::FootprintParams{}.make(), dbc::Footprint::rectangle(-0.3, 0.3, -1.0, 1.0),
        dbc::Footprint({{1.0, 0.0}, {0.5, 0.8}, {-0.5, 0.8}, {-1.0, 0.0}, {-0.5, -0.8}, {0.5, -0.8}})}) {
    for (const int count : {1, 2, 3, 5}) {
      const auto circles = fp.circle_cover(count);
      ASSERT_EQ(circles.size(), static_cast<std::size_t>(count));
      const Eigen::Vector2d lo = fp.min_corner();
      const Eigen::Vector2d hi = fp.max_corner();
      for (int i = 0; i < 5000; ++i) {
        const Eigen::Vector2d p{rng.uniform(lo.x(), hi.x()), rng.uniform(lo.y(), hi.y())};
        if (signed_distance(fp, p) > 0.0) {
          continue;
        }
        bool covered = false;
        for (const auto & c : circles) {
          covered = covered || (p - c.center).norm() <= c.radius + 1e-12;
        }
        EXPECT_TRUE(covered) << count << ": " << p.transpose();
      }
    }
  }
  EXPECT_THROW(dbc::FootprintParams{}.make().circle_cover(0), std::invalid_argument);
}
