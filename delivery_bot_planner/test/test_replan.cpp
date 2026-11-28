#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "delivery_bot_planner/hybrid_astar.hpp"
#include "delivery_bot_planner/replan.hpp"
#include "test_maps.hpp"

namespace dbc = delivery_bot_collision;
namespace dbg = delivery_bot_geometry;
namespace dbp = delivery_bot_planner;

TEST(Replan, Suffix)
{
  const dbg::Path path(
    {1.0, 2.0, 0.3}, {dbg::Segment::left(1.0, 1.25), dbg::Segment::straight(-2.0), dbg::Segment::right(0.7, 1.25)});
  for (const double s : {0.0, 0.4, 1.0, 1.7, 3.0, 3.69}) {
    const dbg::Path rest = dbp::suffix(path, s);
    EXPECT_TRUE(dbg::approx_equal(rest.start(), path.pose_at(s), 1e-12)) << s;
    EXPECT_TRUE(dbg::approx_equal(rest.end(), path.end(), 1e-9)) << s;
    EXPECT_NEAR(rest.length(), path.length() - s, 1e-12) << s;
  }
  EXPECT_TRUE(dbp::suffix(path, 10.0).empty());
}

TEST(Replan, ReusesThePreviousPathAfterDrivingOn)
{
  const auto checker = std::make_shared<const dbc::CollisionChecker>(dbp::test::city(7), dbc::FootprintParams{}.make());
  dbp::HybridAStar planner(checker);
  const dbg::Pose2 start{4.0, 4.0, 0.0};
  const dbg::Pose2 goal{64.0, 44.0, dbg::kPi / 2.0};
  const auto first = planner.plan(start, goal);
  ASSERT_EQ(first.error, dbp::PlanError::kNone);

  // The robot drove 1 m and is a little off the path.
  const dbg::Pose2 now = first.path.pose_at(1.0) * dbg::Pose2(0.02, 0.08, 0.05);
  const auto t0 = std::chrono::steady_clock::now();
  const auto reused = dbp::reuse_previous(planner, first.path, goal, now, goal);
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - t0;
  ASSERT_TRUE(reused.has_value());
  const dbg::Path path = reused.value_or(dbg::Path{});
  EXPECT_LT(elapsed.count(), 0.2);  // the card: re-planning while the previous path is valid < 200 ms
  EXPECT_TRUE(dbg::approx_equal(path.start(), now, 1e-12));
  EXPECT_TRUE(dbg::approx_equal(path.end(), first.path.end(), 1e-9));
  EXPECT_FALSE(checker->path_in_collision(path, {true, true}));
  EXPECT_EQ(path.cusp_count(), first.path.cusp_count());
  EXPECT_LT(path.length(), first.path.length());
}

TEST(Replan, FallsBackWhenThePreviousPathDoesNotFit)
{
  auto builder = dbp::test::MapBuilder(30.0, 12.0).border();
  const auto open = builder.checker();
  dbp::HybridAStar planner(open);
  const dbg::Pose2 start{2.0, 6.0, 0.0};
  const dbg::Pose2 goal{27.0, 6.0, 0.0};
  const auto first = planner.plan(start, goal);
  ASSERT_EQ(first.error, dbp::PlanError::kNone);
  const dbg::Pose2 now = first.path.pose_at(2.0);
  ASSERT_TRUE(dbp::reuse_previous(planner, first.path, goal, now, goal).has_value());
  // A different goal, or too far off the path.
  EXPECT_FALSE(dbp::reuse_previous(planner, first.path, goal, now, {27.0, 5.0, 0.0}).has_value());
  EXPECT_FALSE(dbp::reuse_previous(planner, first.path, goal, now * dbg::Pose2(0.0, 0.5, 0.0), goal).has_value());
  EXPECT_FALSE(dbp::reuse_previous(planner, first.path, goal, now * dbg::Pose2(0.0, 0.0, 0.5), goal).has_value());
  // An obstacle appeared on the rest of the old path.
  const auto blocked = builder.box(15.0, 5.0, 15.5, 7.0).checker();
  const dbp::HybridAStar replanner(blocked);
  EXPECT_FALSE(dbp::reuse_previous(replanner, first.path, goal, now, goal).has_value());
}
