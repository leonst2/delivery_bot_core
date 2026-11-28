#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_planner/heuristic.hpp"
#include "test_maps.hpp"

namespace dbc = delivery_bot_collision;
namespace dbp = delivery_bot_planner;

TEST(Heuristic, BaseClearanceOfTheDefaultFootprint)
{
  // Padded default footprint: x -0.135 ... 0.625, y -0.32 ... 0.32; the rear edge is closest to base_link.
  EXPECT_NEAR(dbp::base_clearance(dbc::FootprintParams{}.make()), 0.135, 1e-12);
  EXPECT_DOUBLE_EQ(dbp::base_clearance(dbc::Footprint::rectangle(0.5, 1.0, -0.2, 0.2)), 0.0);  // origin outside
}

TEST(Heuristic, OpenFieldIsOctileDistance)
{
  const auto checker = dbp::test::MapBuilder(20.0, 20.0).checker();
  const dbp::DistanceHeuristic h(*checker, {10.05, 10.05}, 0.135);
  EXPECT_DOUBLE_EQ(h.at(10.05, 10.05), 0.0);
  EXPECT_NEAR(h.at(14.05, 10.05), 4.0, 1e-9);
  EXPECT_NEAR(h.at(14.05, 14.05), 40 * 0.1 * 1.4, 1e-9);  // integer octile costs: diagonal 1.4
  // Close to the straight line in open space (octile metric with a 1.4 diagonal: between -1 % and +8 %).
  for (const double x : {1.0, 5.3, 17.9}) {
    for (const double y : {1.1, 9.7, 18.2}) {
      const double straight = std::hypot(x - 10.05, y - 10.05);
      EXPECT_LE(h.at(x, y), straight * 1.083 + 0.3);
      EXPECT_GE(h.at(x, y), straight * 0.98 - 0.3);
    }
  }
  EXPECT_DOUBLE_EQ(h.at(-5.0, 3.0), 0.0);  // outside: no information
}

TEST(Heuristic, WallsMakeDetoursAndEnclosuresUnreachable)
{
  // Wall across the map at x = 10 with a 2 m gap at the top.
  const auto checker = dbp::test::MapBuilder(20.0, 20.0).box(9.9, 0.0, 10.1, 18.0).checker();
  const dbp::DistanceHeuristic h(*checker, {15.0, 2.0}, 0.135);
  EXPECT_GT(h.at(5.0, 2.0), 2.0 * 16.0 - 1.0);  // up to the gap and down again
  const auto boxed = dbp::test::MapBuilder(20.0, 20.0)
                       .box(4.0, 4.0, 4.2, 8.0)
                       .box(4.0, 4.0, 8.0, 4.2)
                       .box(7.8, 4.0, 8.0, 8.0)
                       .box(4.0, 7.8, 8.0, 8.0)
                       .checker();
  const dbp::DistanceHeuristic enclosed(*boxed, {6.0, 6.0}, 0.135);
  EXPECT_TRUE(std::isinf(enclosed.at(15.0, 15.0)));
  EXPECT_TRUE(std::isfinite(enclosed.at(6.5, 6.5)));
}
