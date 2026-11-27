#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/dubins.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbg = delivery_bot_geometry;

namespace
{
constexpr double kRadius = 1.25;
constexpr double kEndTol = 1e-6;  // acceptance criterion: endpoint error < 1e-6
constexpr double kLenTol = 1e-9;

double shortest_length(const dbg::Pose2 & a, const dbg::Pose2 & b)
{
  return dbg::dubins_shortest(a, b, kRadius).path.length();
}

dbg::Pose2 flipped(const dbg::Pose2 & p) { return {p.x, p.y, dbg::normalize_angle(p.yaw + dbg::kPi)}; }
}  // namespace

TEST(Dubins, EveryCandidateReachesTheGoal)
{
  dbg::test::RandomPoses rng(30);
  for (int i = 0; i < 5000; ++i) {
    const dbg::Pose2 start = rng.pose(10.0);
    const dbg::Pose2 goal = rng.pose(10.0);
    const auto candidates = dbg::dubins_all(start, goal, kRadius);
    ASSERT_FALSE(candidates.empty());
    for (const auto & c : candidates) {
      EXPECT_TRUE(dbg::approx_equal(c.path.end(), goal, kEndTol)) << c.word;
      EXPECT_EQ(c.path.cusp_count(), 0U) << c.word;
      for (const auto & seg : c.path.segments()) {
        EXPECT_GT(seg.length, 0.0) << c.word;  // forward only
        EXPECT_LE(std::abs(seg.curvature), 1.0 / kRadius + 1e-12);
      }
    }
  }
}

TEST(Dubins, ShortestIsTheMinimumAndAtLeastTheStraightLine)
{
  dbg::test::RandomPoses rng(31);
  for (int i = 0; i < 2000; ++i) {
    const dbg::Pose2 start = rng.pose(10.0);
    const dbg::Pose2 goal = rng.pose(10.0);
    const auto best = dbg::dubins_shortest(start, goal, kRadius);
    EXPECT_TRUE(dbg::approx_equal(best.path.end(), goal, kEndTol));
    EXPECT_GE(best.path.length(), dbg::distance(start, goal) - kLenTol);
    for (const auto & c : dbg::dubins_all(start, goal, kRadius)) {
      EXPECT_LE(best.path.length(), c.path.length());
    }
  }
}

TEST(Dubins, Symmetries)
{
  dbg::test::RandomPoses rng(32);
  for (int i = 0; i < 2000; ++i) {
    const dbg::Pose2 a = rng.pose(8.0);
    const dbg::Pose2 b = rng.pose(8.0);
    const double len = shortest_length(a, b);
    // Rigid motion of both poses.
    const dbg::Pose2 t = rng.pose(20.0);
    EXPECT_NEAR(shortest_length(t * a, t * b), len, 1e-6);
    // Mirror in the x axis.
    EXPECT_NEAR(shortest_length({a.x, -a.y, -a.yaw}, {b.x, -b.y, -b.yaw}), len, 1e-6);
    // Driving the path backwards in time: from b facing back to a facing back.
    EXPECT_NEAR(shortest_length(flipped(b), flipped(a)), len, 1e-6);
  }
}

TEST(Dubins, TriangleInequality)
{
  // No concatenation of two Dubins paths is shorter than the direct one: catches a missing or wrong word.
  dbg::test::RandomPoses rng(33);
  for (int i = 0; i < 3000; ++i) {
    const dbg::Pose2 a = rng.pose(6.0);
    const dbg::Pose2 b = rng.pose(6.0);
    const dbg::Pose2 c = rng.pose(6.0);
    EXPECT_LE(shortest_length(a, c), shortest_length(a, b) + shortest_length(b, c) + 1e-9);
  }
}

TEST(Dubins, HandDerivedCases)
{
  const dbg::Pose2 o{};
  // Straight ahead.
  EXPECT_NEAR(shortest_length(o, {3.0, 0.0, 0.0}), 3.0, kLenTol);
  // Quarter circle left / right.
  EXPECT_NEAR(shortest_length(o, {kRadius, kRadius, dbg::kPi / 2.0}), dbg::kPi / 2.0 * kRadius, kLenTol);
  EXPECT_NEAR(shortest_length(o, {kRadius, -kRadius, -dbg::kPi / 2.0}), dbg::kPi / 2.0 * kRadius, kLenTol);
  // Half circle.
  EXPECT_NEAR(shortest_length(o, {0.0, 2.0 * kRadius, dbg::kPi}), dbg::kPi * kRadius, kLenTol);
  // Quarter circle, then straight on.
  EXPECT_NEAR(shortest_length(o, {kRadius, kRadius + 2.0, dbg::kPi / 2.0}), dbg::kPi / 2.0 * kRadius + 2.0, kLenTol);
  // Goal far behind, facing the same way: half circle, straight back, half circle (no reversing allowed).
  EXPECT_NEAR(shortest_length(o, {-10.0, 0.0, 0.0}), 2.0 * dbg::kPi * kRadius + 10.0, kLenTol);
  // Start == goal: the empty path.
  EXPECT_NEAR(shortest_length({1.0, 2.0, 0.3}, {1.0, 2.0, 0.3}), 0.0, kLenTol);
}

TEST(Dubins, WordsOfKnownCases)
{
  const dbg::Pose2 o{};
  EXPECT_EQ(dbg::dubins_shortest(o, {kRadius, kRadius, dbg::kPi / 2.0}, kRadius).path.segments().size(), 1U);
  // Close goal facing back the other way needs three arcs (a CCC word).
  const auto ccc = dbg::dubins_shortest(o, {0.5, 0.0, dbg::kPi}, kRadius);
  EXPECT_TRUE(ccc.word == "LRL" || ccc.word == "RLR") << ccc.word;
}

TEST(Dubins, Deterministic)
{
  dbg::test::RandomPoses rng(34);
  for (int i = 0; i < 200; ++i) {
    const dbg::Pose2 a = rng.pose(8.0);
    const dbg::Pose2 b = rng.pose(8.0);
    const auto first = dbg::dubins_shortest(a, b, kRadius);
    const auto second = dbg::dubins_shortest(a, b, kRadius);
    EXPECT_EQ(first.word, second.word);
    ASSERT_EQ(first.path.segments().size(), second.path.segments().size());
    for (std::size_t k = 0; k < first.path.segments().size(); ++k) {
      EXPECT_EQ(first.path.segments()[k].length, second.path.segments()[k].length);  // bitwise equal
      EXPECT_EQ(first.path.segments()[k].curvature, second.path.segments()[k].curvature);
    }
  }
}

TEST(Dubins, RejectsBadRadius)
{
  EXPECT_THROW(dbg::dubins_all({}, {1.0, 0.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_THROW(dbg::dubins_all({}, {1.0, 0.0, 0.0}, -1.0), std::invalid_argument);
  EXPECT_THROW(dbg::dubins_all({}, {1.0, 0.0, 0.0}, std::nan("")), std::invalid_argument);
}
