// Planning time on city-like maps: the card asks for < 1 s in 95 % of the queries on 100 x 100 m maps at 0.1 m.
// Stands in for the benchmark corpus of delivery_bot_core#26 until it exists.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "delivery_bot_planner/hybrid_astar.hpp"
#include "test_maps.hpp"

namespace dbc = delivery_bot_collision;
namespace dbg = delivery_bot_geometry;
namespace dbp = delivery_bot_planner;

namespace
{

// A random pose on a street (the streets are the first 8 m of every 20 m in x or y), heading along the street in
// either direction +- 25 deg, with at least 0.1 m clearance: where a delivery robot stands or is sent to.
dbg::Pose2 street_pose(std::mt19937 & rng, const dbc::CollisionChecker & checker)
{
  std::uniform_real_distribution<double> along(1.0, 99.0);
  std::uniform_real_distribution<double> across(1.0, 7.0);
  std::uniform_int_distribution<int> block(0, 4);
  std::uniform_real_distribution<double> noise(-0.44, 0.44);
  std::bernoulli_distribution coin(0.5);
  for (;;) {
    const double a = along(rng);
    const double b = 20.0 * block(rng) + across(rng);
    const bool horizontal = coin(rng);
    const double yaw =
      dbg::normalize_angle((horizontal ? 0.0 : dbg::kPi / 2.0) + (coin(rng) ? dbg::kPi : 0.0) + noise(rng));
    const dbg::Pose2 pose = horizontal ? dbg::Pose2{a, b, yaw} : dbg::Pose2{b, a, yaw};
    if (checker.clearance_lower_bound(pose) >= 0.1) {  // on the street, not wedged between a car and clutter
      return pose;
    }
  }
}

}  // namespace

TEST(Benchmark, NinetyFivePercentUnderOneSecond)
{
  constexpr int kMaps = 4;
  constexpr int kQueriesPerMap = 25;
  std::vector<double> seconds;
  std::vector<std::size_t> expansions;
  int failures = 0;
  for (int m = 0; m < kMaps; ++m) {
    const auto checker = std::make_shared<const dbc::CollisionChecker>(
      dbp::test::city(static_cast<std::uint32_t>(100 + m)), dbc::FootprintParams{}.make());
    dbp::HybridAStar planner(checker);
    std::mt19937 rng(static_cast<std::uint32_t>(200 + m));
    for (int q = 0; q < kQueriesPerMap; ++q) {
      const dbg::Pose2 start = street_pose(rng, *checker);
      const dbg::Pose2 goal = street_pose(rng, *checker);
      dbp::PlanLimits limits;
      limits.deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      const auto t0 = std::chrono::steady_clock::now();
      const auto result = planner.plan(start, goal, limits);
      const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - t0;
      seconds.push_back(elapsed.count());
      expansions.push_back(result.expanded);
      if (result.error != dbp::PlanError::kNone) {
        ++failures;
        std::printf(
          "  failed: (%.1f, %.1f, %.2f) -> (%.1f, %.1f, %.2f): error %d after %.2f s\n", start.x, start.y, start.yaw,
          goal.x, goal.y, goal.yaw, static_cast<int>(result.error), elapsed.count());
      } else {
        EXPECT_FALSE(checker->path_in_collision(result.path, {true, true}));
        EXPECT_LE(dbg::distance(result.path.end(), goal), planner.params().goal_xy_tolerance + 1e-9);
      }
    }
  }
  std::vector<double> sorted = seconds;
  std::sort(sorted.begin(), sorted.end());
  const double median = sorted[sorted.size() / 2];
  const double p95 = sorted[sorted.size() * 95 / 100];
  const double worst = sorted.back();
  std::sort(expansions.begin(), expansions.end());
  std::printf(
    "planning: %zu queries, median %.3f s, p95 %.3f s, max %.3f s, median expansions %zu, failures %d\n", sorted.size(),
    median, p95, worst, expansions[expansions.size() / 2], failures);
  EXPECT_EQ(failures, 0);
#ifdef __OPTIMIZE__
  EXPECT_LT(p95, 1.0);
#else
  GTEST_SKIP() << "unoptimised build: planning time not asserted";
#endif
}
