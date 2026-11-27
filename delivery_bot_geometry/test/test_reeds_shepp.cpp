#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/dubins.hpp"
#include "delivery_bot_geometry/reeds_shepp.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbg = delivery_bot_geometry;

namespace
{
constexpr double kRadius = 1.25;
constexpr double kEndTol = 1e-6;  // acceptance criterion: endpoint error < 1e-6
constexpr double kLenTol = 1e-9;

double rs_length(const dbg::Pose2 & a, const dbg::Pose2 & b)
{
  return dbg::reeds_shepp_shortest(a, b, kRadius).path.length();
}

dbg::Pose2 flipped(const dbg::Pose2 & p) { return {p.x, p.y, dbg::normalize_angle(p.yaw + dbg::kPi)}; }
}  // namespace

TEST(ReedsShepp, EveryCandidateReachesTheGoal)
{
  dbg::test::RandomPoses rng(40);
  for (int i = 0; i < 5000; ++i) {
    // Mostly close goals, where the CCCC / CCSC / CCSCC words matter.
    const double extent = i % 2 == 0 ? 3.0 : 10.0;
    const dbg::Pose2 start = rng.pose(extent);
    const dbg::Pose2 goal = rng.pose(extent);
    const auto candidates = dbg::reeds_shepp_all(start, goal, kRadius);
    ASSERT_FALSE(candidates.empty());
    for (const auto & c : candidates) {
      EXPECT_TRUE(dbg::approx_equal(c.path.end(), goal, kEndTol)) << c.word;
      for (const auto & seg : c.path.segments()) {
        EXPECT_LE(std::abs(seg.curvature), 1.0 / kRadius + 1e-12);
      }
    }
  }
}

TEST(ReedsShepp, EveryFamilyIsUsed)
{
  // Each base word (up to reflection and time flip) is the shortest for some goal. A family that is never
  // feasible would point at a broken formula.
  dbg::test::RandomPoses rng(41);
  bool csc = false;
  bool ccc = false;
  bool cccc = false;
  bool ccsc = false;
  bool ccscc = false;
  for (int i = 0; i < 20000; ++i) {
    for (const auto & c : dbg::reeds_shepp_all(dbg::Pose2{}, rng.pose(4.0), kRadius)) {
      const std::string & w = c.word;
      csc = csc || (w.size() == 3 && w[1] == 'S');
      ccc = ccc || (w.size() == 3 && w[1] != 'S');
      cccc = cccc || (w.size() == 4 && w.find('S') == std::string::npos);
      ccsc = ccsc || (w.size() == 4 && w.find('S') != std::string::npos);
      ccscc = ccscc || w.size() == 5;
    }
  }
  EXPECT_TRUE(csc);
  EXPECT_TRUE(ccc);
  EXPECT_TRUE(cccc);
  EXPECT_TRUE(ccsc);
  EXPECT_TRUE(ccscc);
}

TEST(ReedsShepp, ShortestBounds)
{
  dbg::test::RandomPoses rng(42);
  for (int i = 0; i < 2000; ++i) {
    const dbg::Pose2 a = rng.pose(8.0);
    const dbg::Pose2 b = rng.pose(8.0);
    const auto best = dbg::reeds_shepp_shortest(a, b, kRadius);
    EXPECT_TRUE(dbg::approx_equal(best.path.end(), b, kEndTol));
    EXPECT_GE(best.path.length(), dbg::distance(a, b) - kLenTol);
    for (const auto & c : dbg::reeds_shepp_all(a, b, kRadius)) {
      EXPECT_LE(best.path.length(), c.path.length());
    }
    // Reversing is allowed, so never longer than the forward-only or the reverse-only Dubins path.
    EXPECT_LE(best.path.length(), dbg::dubins_shortest(a, b, kRadius).path.length() + kLenTol);
    EXPECT_LE(best.path.length(), dbg::dubins_shortest(flipped(a), flipped(b), kRadius).path.length() + kLenTol);
  }
}

TEST(ReedsShepp, Symmetries)
{
  dbg::test::RandomPoses rng(43);
  for (int i = 0; i < 2000; ++i) {
    const dbg::Pose2 a = rng.pose(6.0);
    const dbg::Pose2 b = rng.pose(6.0);
    const double len = rs_length(a, b);
    // Swapping start and goal: drive the same path in reverse.
    EXPECT_NEAR(rs_length(b, a), len, 1e-6);
    // Rigid motion of both poses.
    const dbg::Pose2 t = rng.pose(20.0);
    EXPECT_NEAR(rs_length(t * a, t * b), len, 1e-6);
    // Mirror in the x axis.
    EXPECT_NEAR(rs_length({a.x, -a.y, -a.yaw}, {b.x, -b.y, -b.yaw}), len, 1e-6);
    // Turning both poses around: forward and reverse swap.
    EXPECT_NEAR(rs_length(flipped(a), flipped(b)), len, 1e-6);
  }
}

TEST(ReedsShepp, TriangleInequality)
{
  // Reeds-Shepp length is a metric: no detour via a third pose is shorter. Catches a missing or wrong word.
  dbg::test::RandomPoses rng(44);
  for (int i = 0; i < 3000; ++i) {
    const dbg::Pose2 a = rng.pose(4.0);
    const dbg::Pose2 b = rng.pose(4.0);
    const dbg::Pose2 c = rng.pose(4.0);
    EXPECT_LE(rs_length(a, c), rs_length(a, b) + rs_length(b, c) + 1e-9);
  }
}

TEST(ReedsShepp, HandDerivedCases)
{
  const dbg::Pose2 o{};
  const double quarter = dbg::kPi / 2.0 * kRadius;
  // Straight ahead and straight back.
  EXPECT_NEAR(rs_length(o, {3.0, 0.0, 0.0}), 3.0, kLenTol);
  EXPECT_NEAR(rs_length(o, {-3.0, 0.0, 0.0}), 3.0, kLenTol);
  // Quarter circles: forward left, forward right, reverse with the wheels left, reverse with the wheels right.
  EXPECT_NEAR(rs_length(o, {kRadius, kRadius, dbg::kPi / 2.0}), quarter, kLenTol);
  EXPECT_NEAR(rs_length(o, {kRadius, -kRadius, -dbg::kPi / 2.0}), quarter, kLenTol);
  EXPECT_NEAR(rs_length(o, {-kRadius, kRadius, -dbg::kPi / 2.0}), quarter, kLenTol);
  EXPECT_NEAR(rs_length(o, {-kRadius, -kRadius, dbg::kPi / 2.0}), quarter, kLenTol);
  // Half circle.
  EXPECT_NEAR(rs_length(o, {0.0, 2.0 * kRadius, dbg::kPi}), dbg::kPi * kRadius, kLenTol);
  // Quarter circle, then straight on.
  EXPECT_NEAR(rs_length(o, {kRadius, kRadius + 2.0, dbg::kPi / 2.0}), quarter + 2.0, kLenTol);
  // Start == goal.
  EXPECT_NEAR(rs_length({1.0, 2.0, 0.3}, {1.0, 2.0, 0.3}), 0.0, kLenTol);
}

TEST(ReedsShepp, GoalBehindIsReachedInReverse)
{
  // Dubins needs a loop of 2 pi R + d; Reeds-Shepp just backs up.
  const auto path = dbg::reeds_shepp_shortest(dbg::Pose2{}, {-10.0, 0.0, 0.0}, kRadius).path;
  ASSERT_EQ(path.segments().size(), 1U);
  EXPECT_NEAR(path.segments()[0].length, -10.0, kLenTol);
  EXPECT_NEAR(path.segments()[0].curvature, 0.0, kLenTol);
}

TEST(ReedsShepp, ParallelParkingUsesCusps)
{
  // A pure sideways shift can't be driven in one direction without a long loop.
  for (const double offset : {0.3, 0.6, 1.0}) {
    const dbg::Pose2 goal{0.0, offset, 0.0};
    const auto rs = dbg::reeds_shepp_shortest(dbg::Pose2{}, goal, kRadius);
    EXPECT_GE(rs.path.cusp_count(), 1U) << offset;
    EXPECT_LT(rs.path.length(), dbg::dubins_shortest(dbg::Pose2{}, goal, kRadius).path.length()) << offset;
    // Lower bound: the path must at least cover the offset.
    EXPECT_GE(rs.path.length(), offset);
  }
}

TEST(ReedsShepp, Deterministic)
{
  dbg::test::RandomPoses rng(45);
  for (int i = 0; i < 200; ++i) {
    const dbg::Pose2 a = rng.pose(5.0);
    const dbg::Pose2 b = rng.pose(5.0);
    const auto first = dbg::reeds_shepp_shortest(a, b, kRadius);
    const auto second = dbg::reeds_shepp_shortest(a, b, kRadius);
    EXPECT_EQ(first.word, second.word);
    ASSERT_EQ(first.path.segments().size(), second.path.segments().size());
    for (std::size_t k = 0; k < first.path.segments().size(); ++k) {
      EXPECT_EQ(first.path.segments()[k].length, second.path.segments()[k].length);  // bitwise equal
      EXPECT_EQ(first.path.segments()[k].curvature, second.path.segments()[k].curvature);
    }
  }
}

TEST(ReedsShepp, RejectsBadRadius)
{
  EXPECT_THROW(dbg::reeds_shepp_all({}, {1.0, 0.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_THROW(dbg::reeds_shepp_shortest({}, {1.0, 0.0, 0.0}, -2.0), std::invalid_argument);
}
