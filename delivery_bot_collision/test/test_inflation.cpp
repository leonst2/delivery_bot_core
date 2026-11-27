#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "delivery_bot_collision/distance_field.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_collision/grid.hpp"
#include "delivery_bot_collision/inflation.hpp"
#include "test_util.hpp"

namespace dbc = delivery_bot_collision;

TEST(Inflation, CostCurve)
{
  for (const double decay : {0.5, 3.0, 10.0}) {
    const dbc::InflationParams p{0.32, 1.0, decay};
    EXPECT_EQ(dbc::inflation_cost(0.0, p), dbc::kLethalCost);
    EXPECT_EQ(dbc::inflation_cost(0.32, p), dbc::kLethalCost);
    EXPECT_EQ(dbc::inflation_cost(0.320001, p), dbc::kMaxInflatedCost);
    EXPECT_EQ(dbc::inflation_cost(1.0, p), 0);
    EXPECT_EQ(dbc::inflation_cost(5.0, p), 0);
    EXPECT_EQ(dbc::inflation_cost(std::numeric_limits<double>::infinity(), p), 0);
    EXPECT_GE(dbc::inflation_cost(0.999999, p), 1);  // non-zero right up to the radius
    // Monotonically non-increasing, within 0 ... 100.
    std::int8_t previous = dbc::kLethalCost;
    for (int i = 0; i <= 12000; ++i) {
      const double d = i * 1e-4;
      const std::int8_t c = dbc::inflation_cost(d, p);
      EXPECT_LE(c, previous) << d;
      EXPECT_GE(c, 0);
      EXPECT_LE(c, dbc::kLethalCost);
      previous = c;
    }
  }
}

TEST(Inflation, DefaultInscribedRadiusMatchesTheFootprint)
{
  EXPECT_DOUBLE_EQ(dbc::InflationParams{}.inscribed_radius, dbc::FootprintParams{}.make().inscribed_radius());
}

TEST(Inflation, InflateGrid)
{
  dbc::test::Random rng(300);
  const dbc::Grid grid = rng.grid(40, 30, 0.1, 0.05, 0.05);
  const dbc::OccupancyRule rule{65, dbc::UnknownPolicy::kFree};
  const auto field = dbc::compute_distance_field(grid, rule);
  const dbc::InflationParams params{0.3, 0.8, 4.0};
  const auto costs = dbc::inflate(grid, field, params);
  ASSERT_EQ(costs.size(), grid.data().size());
  for (std::size_t i = 0; i < costs.size(); ++i) {
    if (grid.data()[i] < 0) {
      EXPECT_EQ(costs[i], -1);  // unknown stays unknown
    } else if (rule.is_obstacle(grid.data()[i])) {
      EXPECT_EQ(costs[i], dbc::kLethalCost);
    } else {
      EXPECT_EQ(costs[i], dbc::inflation_cost(field.data()[i], params));
    }
  }
}

TEST(Inflation, RejectsBadParams)
{
  const dbc::Grid grid(2, 2, 0.1, 0.0, 0.0, std::int8_t{0});
  const auto field = dbc::compute_distance_field(grid, {});
  EXPECT_THROW(dbc::inflate(grid, field, {0.5, 0.5, 1.0}), std::invalid_argument);
  EXPECT_THROW(dbc::inflate(grid, field, {-0.1, 0.5, 1.0}), std::invalid_argument);
  EXPECT_THROW(dbc::inflate(grid, field, {0.1, 0.5, 0.0}), std::invalid_argument);
  const dbc::Grid other(3, 2, 0.1, 0.0, 0.0, std::int8_t{0});
  EXPECT_THROW(dbc::inflate(other, field, {}), std::invalid_argument);
}
