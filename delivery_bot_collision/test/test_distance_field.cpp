#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "delivery_bot_collision/distance_field.hpp"
#include "delivery_bot_collision/grid.hpp"
#include "test_util.hpp"

namespace dbc = delivery_bot_collision;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

// Brute force over all cell pairs, with the outside ring of obstacle cells when the rule says so.
double brute_force(const dbc::Grid & grid, const dbc::OccupancyRule & rule, int x, int y)
{
  const int pad = rule.outside_is_obstacle() ? 1 : 0;
  double best_sq = kInf;
  for (int oy = -pad; oy < grid.height() + pad; ++oy) {
    for (int ox = -pad; ox < grid.width() + pad; ++ox) {
      const bool obstacle = grid.contains(ox, oy) ? rule.is_obstacle(grid.at(ox, oy)) : true;
      if (obstacle) {
        const double dx = ox - x;
        const double dy = oy - y;
        best_sq = std::min(best_sq, dx * dx + dy * dy);
      }
    }
  }
  return std::sqrt(best_sq) * grid.resolution();
}

void expect_matches_brute_force(const dbc::Grid & grid, const dbc::OccupancyRule & rule)
{
  const dbc::DistanceField field = dbc::compute_distance_field(grid, rule);
  ASSERT_EQ(field.width(), grid.width());
  ASSERT_EQ(field.height(), grid.height());
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const double expected = brute_force(grid, rule, x, y);
      if (std::isinf(expected)) {
        EXPECT_TRUE(std::isinf(field.at(x, y))) << x << ", " << y;
      } else {
        EXPECT_NEAR(field.at(x, y), expected, 1e-12) << x << ", " << y;
      }
    }
  }
}
}  // namespace

TEST(DistanceTransform1d, KnownValues)
{
  std::vector<double> d;
  dbc::distance_transform_1d({kInf, 0.0, kInf, kInf, kInf, 0.0}, d);
  EXPECT_EQ(d, (std::vector<double>{1.0, 0.0, 1.0, 4.0, 1.0, 0.0}));
  // Non-zero samples (as in the second pass).
  dbc::distance_transform_1d({4.0, kInf, 0.0}, d);
  EXPECT_EQ(d, (std::vector<double>{4.0, 1.0, 0.0}));
  // Nothing finite.
  dbc::distance_transform_1d({kInf, kInf}, d);
  EXPECT_TRUE(std::isinf(d[0]) && std::isinf(d[1]));
  dbc::distance_transform_1d({}, d);
  EXPECT_TRUE(d.empty());
}

TEST(DistanceField, MatchesBruteForceOnRandomGrids)
{
  dbc::test::Random rng(100);
  for (int i = 0; i < 60; ++i) {
    const int w = rng.integer(1, 40);
    const int h = rng.integer(1, 30);
    const double occupied = rng.uniform(0.0, 0.3);
    const double unknown = i % 3 == 0 ? 0.0 : rng.uniform(0.0, 0.1);
    const dbc::Grid grid = rng.grid(w, h, rng.uniform(0.05, 0.5), occupied, unknown);
    expect_matches_brute_force(grid, dbc::OccupancyRule{65, dbc::UnknownPolicy::kLethal});
    expect_matches_brute_force(grid, dbc::OccupancyRule{65, dbc::UnknownPolicy::kFree});
    expect_matches_brute_force(grid, dbc::OccupancyRule{100, dbc::UnknownPolicy::kFree});
  }
}

TEST(DistanceField, SparseObstacles)
{
  // Long envelopes: very few obstacles on a larger grid.
  dbc::test::Random rng(101);
  for (int i = 0; i < 5; ++i) {
    const dbc::Grid grid = rng.grid(80, 60, 0.1, 0.002, 0.0);
    expect_matches_brute_force(grid, dbc::OccupancyRule{65, dbc::UnknownPolicy::kFree});
  }
}

TEST(DistanceField, EmptyAndFullGrids)
{
  const dbc::Grid empty(5, 4, 0.1, 0.0, 0.0, std::int8_t{0});
  const auto free_field = dbc::compute_distance_field(empty, {65, dbc::UnknownPolicy::kFree});
  for (const double d : free_field.data()) {
    EXPECT_TRUE(std::isinf(d));
  }
  // With the outside lethal, the distance is to the ring around the map.
  const auto ring = dbc::compute_distance_field(empty, {65, dbc::UnknownPolicy::kLethal});
  EXPECT_NEAR(ring.at(0, 0), 0.1, 1e-12);
  EXPECT_NEAR(ring.at(2, 1), 0.2, 1e-12);
  const dbc::Grid full(3, 3, 0.2, 0.0, 0.0, std::int8_t{100});
  const auto full_field = dbc::compute_distance_field(full, {});
  for (const double d : full_field.data()) {
    EXPECT_DOUBLE_EQ(d, 0.0);
  }
}

TEST(DistanceField, Deterministic)
{
  dbc::test::Random rng(102);
  const dbc::Grid grid = rng.grid(50, 40, 0.1, 0.1, 0.05);
  EXPECT_EQ(dbc::compute_distance_field(grid, {}).data(), dbc::compute_distance_field(grid, {}).data());
}
