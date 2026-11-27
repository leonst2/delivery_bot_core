#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "delivery_bot_collision/grid.hpp"

namespace dbc = delivery_bot_collision;

TEST(Grid, LayoutLikeOccupancyGrid)
{
  // 3 x 2 cells of 0.5 m, origin (1, 2): cell (x, y) has index y * width + x.
  const dbc::Grid grid(3, 2, 0.5, 1.0, 2.0, std::vector<std::int8_t>{0, 1, 2, 3, 4, 5});
  EXPECT_EQ(grid.at(0, 0), 0);
  EXPECT_EQ(grid.at(2, 0), 2);
  EXPECT_EQ(grid.at(0, 1), 3);
  EXPECT_EQ(grid.at(2, 1), 5);
  EXPECT_DOUBLE_EQ(grid.cell_center(0, 0).x(), 1.25);
  EXPECT_DOUBLE_EQ(grid.cell_center(0, 0).y(), 2.25);
  EXPECT_DOUBLE_EQ(grid.cell_center(2, 1).x(), 2.25);
  EXPECT_DOUBLE_EQ(grid.cell_center(2, 1).y(), 2.75);
}

TEST(Grid, WorldToCell)
{
  const dbc::Grid grid(4, 4, 0.1, -0.2, -0.2, std::int8_t{0});
  const auto c = grid.world_to_cell(0.05, -0.15);
  EXPECT_EQ(c.x, 2);
  EXPECT_EQ(c.y, 0);
  // Points outside give cells outside.
  const auto out = grid.world_to_cell(-0.25, 0.3);
  EXPECT_EQ(out.x, -1);
  EXPECT_EQ(out.y, 5);
  EXPECT_FALSE(grid.contains(out.x, out.y));
  // Every cell centre maps back to its own cell.
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      const auto centre = grid.cell_center(x, y);
      const auto back = grid.world_to_cell(centre.x(), centre.y());
      EXPECT_EQ(back.x, x);
      EXPECT_EQ(back.y, y);
    }
  }
  EXPECT_TRUE(grid.contains_world(-0.2, 0.2));
  EXPECT_FALSE(grid.contains_world(-0.21, 0.0));
}

TEST(Grid, RejectsBadInput)
{
  EXPECT_THROW(dbc::Grid(0, 2, 0.1, 0.0, 0.0, std::int8_t{0}), std::invalid_argument);
  EXPECT_THROW(dbc::Grid(2, 2, 0.0, 0.0, 0.0, std::int8_t{0}), std::invalid_argument);
  EXPECT_THROW(dbc::Grid(2, 2, 0.1, 0.0, 0.0, std::vector<std::int8_t>(3, 0)), std::invalid_argument);
}

TEST(OccupancyRule, Classification)
{
  const dbc::OccupancyRule lethal;
  EXPECT_FALSE(lethal.is_obstacle(0));
  EXPECT_FALSE(lethal.is_obstacle(64));
  EXPECT_TRUE(lethal.is_obstacle(65));
  EXPECT_TRUE(lethal.is_obstacle(100));
  EXPECT_TRUE(lethal.is_obstacle(-1));
  EXPECT_TRUE(lethal.outside_is_obstacle());

  const dbc::OccupancyRule free{100, dbc::UnknownPolicy::kFree};
  EXPECT_FALSE(free.is_obstacle(-1));
  EXPECT_FALSE(free.is_obstacle(99));
  EXPECT_TRUE(free.is_obstacle(100));
  EXPECT_FALSE(free.outside_is_obstacle());
}
