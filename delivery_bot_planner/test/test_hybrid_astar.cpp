#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "delivery_bot_planner/hybrid_astar.hpp"
#include "test_maps.hpp"

namespace dbg = delivery_bot_geometry;
namespace dbp = delivery_bot_planner;

namespace
{

constexpr double kRMin = 1.25;

// What every returned path must satisfy.
void expect_valid(
  const dbp::HybridAStar & planner, const dbp::PlanResult & r, const dbg::Pose2 & a, const dbg::Pose2 & b)
{
  ASSERT_EQ(r.error, dbp::PlanError::kNone);
  EXPECT_TRUE(dbg::approx_equal(r.path.start(), a, 1e-12));
  EXPECT_TRUE(dbg::approx_equal(r.path.end(), b, 1e-6)) << r.path.end().x << " " << r.path.end().y;
  EXPECT_FALSE(planner.checker().path_in_collision(r.path));
  for (const auto & seg : r.path.segments()) {
    EXPECT_LE(std::abs(seg.curvature), 1.0 / kRMin + 1e-9);
  }
  EXPECT_GE(r.path.length(), dbg::distance(a, b) - 1e-9);
}

}  // namespace

TEST(HybridAStar, OpenFieldDrivesForward)
{
  dbp::HybridAStar planner(dbp::test::MapBuilder(30.0, 30.0).border().checker());
  const dbg::Pose2 a{5.0, 15.0, 0.0};
  const dbg::Pose2 b{25.0, 17.0, 0.3};
  const auto r = planner.plan(a, b);
  expect_valid(planner, r, a, b);
  EXPECT_EQ(r.path.cusp_count(), 0U);
  for (const auto & seg : r.path.segments()) {
    EXPECT_GT(seg.length, 0.0);  // no reversing when it isn't needed
  }
  EXPECT_LT(r.path.length(), 21.0);
}

TEST(HybridAStar, CorridorWithLCorner)
{
  // 4 m wide corridor east along y = 2..6, then north along x = 20..24.
  const auto checker =
    dbp::test::MapBuilder(30.0, 30.0).border().box(0.0, 6.0, 20.0, 30.0).box(24.0, 0.0, 30.0, 30.0).checker();
  dbp::HybridAStar planner(checker);
  const dbg::Pose2 a{2.0, 4.0, 0.0};
  const dbg::Pose2 b{22.0, 25.0, dbg::kPi / 2.0};
  const auto r = planner.plan(a, b);
  expect_valid(planner, r, a, b);
  EXPECT_EQ(r.path.cusp_count(), 0U);
}

TEST(HybridAStar, UTurnInADeadEnd)
{
  // A dead-end street 3 m wide: too narrow to turn around in one go (R_min 1.25 m + footprint).
  const auto checker = dbp::test::MapBuilder(20.0, 10.0)
                         .border()
                         .box(0.0, 0.0, 20.0, 3.5)
                         .box(0.0, 6.5, 20.0, 10.0)
                         .box(15.0, 0.0, 20.0, 10.0)
                         .checker();
  dbp::HybridAStar planner(checker);
  const dbg::Pose2 a{4.0, 5.0, 0.0};
  const dbg::Pose2 b{4.0, 5.0, dbg::kPi};
  const auto r = planner.plan(a, b);
  expect_valid(planner, r, a, b);
  EXPECT_GE(r.path.cusp_count(), 1U);  // needs reversing
}

TEST(HybridAStar, ParkingBay)
{
  // Cars on both sides of a 1.1 m wide bay, the robot has to enter it from the 6 m aisle.
  const auto checker =
    dbp::test::MapBuilder(20.0, 12.0).border().box(0.0, 0.0, 9.45, 2.5).box(10.55, 0.0, 20.0, 2.5).checker();
  dbp::HybridAStar planner(checker);
  const dbg::Pose2 a{3.0, 7.0, 0.0};
  const dbg::Pose2 b{10.0, 1.2, -dbg::kPi / 2.0};  // nose into the bay
  const auto r = planner.plan(a, b);
  expect_valid(planner, r, a, b);
}

TEST(HybridAStar, NarrowGap)
{
  // A wall with a 1.0 m gap: the padded footprint (0.64 m) plus the sweep margin fits through.
  const auto checker =
    dbp::test::MapBuilder(20.0, 12.0).border().box(9.9, 0.0, 10.1, 5.5).box(9.9, 6.5, 10.1, 12.0).checker();
  dbp::HybridAStar planner(checker);
  const dbg::Pose2 a{3.0, 3.0, 0.0};
  const dbg::Pose2 b{17.0, 9.0, 0.0};
  expect_valid(planner, planner.plan(a, b), a, b);
}

TEST(HybridAStar, Errors)
{
  const auto checker = dbp::test::MapBuilder(20.0, 20.0)
                         .border()
                         .box(8.0, 8.0, 8.2, 12.0)
                         .box(8.0, 8.0, 12.0, 8.2)
                         .box(11.8, 8.0, 12.0, 12.0)
                         .box(8.0, 11.8, 12.0, 12.0)
                         .checker();
  dbp::HybridAStar planner(checker);
  // Enclosed goal: no path, found quickly.
  const auto enclosed = planner.plan({3.0, 3.0, 0.0}, {10.0, 10.0, 0.0});
  EXPECT_EQ(enclosed.error, dbp::PlanError::kNoPath);
  EXPECT_LT(enclosed.expanded, 10U);
  // Start or goal in collision, or off the map (unknown is lethal by default).
  EXPECT_EQ(planner.plan({8.1, 10.0, 0.0}, {3.0, 3.0, 0.0}).error, dbp::PlanError::kInvalidStart);
  EXPECT_EQ(planner.plan({3.0, 3.0, 0.0}, {30.0, 3.0, 0.0}).error, dbp::PlanError::kInvalidGoal);
  // Start == goal: an empty path.
  const auto same = planner.plan({3.0, 3.0, 0.5}, {3.0, 3.0, 0.5});
  EXPECT_EQ(same.error, dbp::PlanError::kNone);
  EXPECT_TRUE(same.path.empty());
}

TEST(HybridAStar, TimeLimitAndCancel)
{
  const auto grid = dbp::test::city(1);
  const auto checker = std::make_shared<const delivery_bot_collision::CollisionChecker>(
    grid, delivery_bot_collision::FootprintParams{}.make());
  dbp::PlannerParams params;
  params.analytic_every = 1000000;  // no lucky early shot
  params.analytic_radius = 0.0;
  dbp::HybridAStar planner(checker, params);
  const dbg::Pose2 a{4.0, 4.0, 0.0};
  const dbg::Pose2 b{84.0, 84.0, 0.0};  // on a street
  dbp::PlanLimits past;
  past.deadline = std::chrono::steady_clock::now();
  EXPECT_EQ(planner.plan(a, b, past).error, dbp::PlanError::kTimeout);
  dbp::PlanLimits cancel;
  cancel.cancelled = [] { return true; };
  EXPECT_EQ(planner.plan(a, b, cancel).error, dbp::PlanError::kCancelled);
}

TEST(HybridAStar, Deterministic)
{
  const auto grid = dbp::test::city(2);
  const auto checker = std::make_shared<const delivery_bot_collision::CollisionChecker>(
    grid, delivery_bot_collision::FootprintParams{}.make());
  const dbg::Pose2 a{4.0, 4.0, 0.0};
  const dbg::Pose2 b{44.0, 24.0, dbg::kPi / 2.0};
  dbp::HybridAStar first(checker);
  const auto r1 = first.plan(a, b);
  const auto r2 = first.plan(a, b);  // with the cached heuristic
  dbp::HybridAStar fresh(checker);
  const auto r3 = fresh.plan(a, b);
  ASSERT_EQ(r1.error, dbp::PlanError::kNone);
  for (const auto * other : {&r2, &r3}) {
    ASSERT_EQ(other->error, dbp::PlanError::kNone);
    ASSERT_EQ(other->path.segments().size(), r1.path.segments().size());
    for (std::size_t k = 0; k < r1.path.segments().size(); ++k) {
      EXPECT_EQ(other->path.segments()[k].length, r1.path.segments()[k].length);  // bitwise
      EXPECT_EQ(other->path.segments()[k].curvature, r1.path.segments()[k].curvature);
    }
    EXPECT_EQ(other->cost, r1.cost);
    EXPECT_EQ(other->expanded, r1.expanded);
  }
}

TEST(HybridAStar, RejectsBadParams)
{
  const auto checker = dbp::test::MapBuilder(5.0, 5.0).checker();
  dbp::PlannerParams short_arcs;
  short_arcs.motion.arc_length = 0.2;  // shorter than a 0.2 m cell diagonal
  EXPECT_THROW(dbp::HybridAStar(checker, short_arcs), std::invalid_argument);
  EXPECT_THROW(dbp::HybridAStar(nullptr), std::invalid_argument);
}
