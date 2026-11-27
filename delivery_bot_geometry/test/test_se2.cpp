#include <gtest/gtest.h>

#include <cmath>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbg = delivery_bot_geometry;

namespace
{
constexpr double kTol = 1e-9;
const dbg::Pose2 kIdentity{};
}  // namespace

TEST(Pose2, ComposeKnownValues)
{
  const dbg::Pose2 a{1.0, 2.0, dbg::kPi / 2.0};
  const dbg::Pose2 b{1.0, 0.0, 0.0};
  // One metre forward from a, which faces +y.
  EXPECT_TRUE(dbg::approx_equal(a * b, dbg::Pose2(1.0, 3.0, dbg::kPi / 2.0), kTol));
  const dbg::Pose2 c{0.0, 0.0, dbg::kPi};
  EXPECT_TRUE(dbg::approx_equal(c * c, kIdentity, kTol));
}

TEST(Pose2, InverseAndIdentity)
{
  dbg::test::RandomPoses rng(10);
  for (int i = 0; i < 1000; ++i) {
    const dbg::Pose2 p = rng.pose(50.0);
    EXPECT_TRUE(dbg::approx_equal(p * p.inverse(), kIdentity, kTol));
    EXPECT_TRUE(dbg::approx_equal(p.inverse() * p, kIdentity, kTol));
    EXPECT_TRUE(dbg::approx_equal(p * kIdentity, p, kTol));
    EXPECT_TRUE(dbg::approx_equal(kIdentity * p, p, kTol));
    EXPECT_TRUE(dbg::approx_equal(p.inverse().inverse(), p, kTol));
  }
}

TEST(Pose2, Associativity)
{
  dbg::test::RandomPoses rng(11);
  for (int i = 0; i < 1000; ++i) {
    const dbg::Pose2 a = rng.pose(20.0);
    const dbg::Pose2 b = rng.pose(20.0);
    const dbg::Pose2 c = rng.pose(20.0);
    EXPECT_TRUE(dbg::approx_equal((a * b) * c, a * (b * c), kTol));
    // (ab)^-1 = b^-1 a^-1
    EXPECT_TRUE(dbg::approx_equal((a * b).inverse(), b.inverse() * a.inverse(), kTol));
  }
}

TEST(Pose2, BetweenRoundTrip)
{
  dbg::test::RandomPoses rng(12);
  for (int i = 0; i < 1000; ++i) {
    const dbg::Pose2 a = rng.pose(20.0);
    const dbg::Pose2 b = rng.pose(20.0);
    EXPECT_TRUE(dbg::approx_equal(a * dbg::between(a, b), b, kTol));
  }
}

TEST(Pose2, TransformPoints)
{
  dbg::test::RandomPoses rng(13);
  for (int i = 0; i < 1000; ++i) {
    const dbg::Pose2 p = rng.pose(20.0);
    const Eigen::Vector2d local{rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)};
    const Eigen::Vector2d world = p.transform(local);
    EXPECT_NEAR((p.inverse_transform(world) - local).norm(), 0.0, kTol);
    // Same as composing with a pose that has the point as its position.
    const dbg::Pose2 composed = p * dbg::Pose2(local.x(), local.y(), 0.0);
    EXPECT_NEAR(composed.x, world.x(), kTol);
    EXPECT_NEAR(composed.y, world.y(), kTol);
    // Rigid: distances are preserved.
    EXPECT_NEAR((world - p.position()).norm(), local.norm(), kTol);
  }
}

TEST(Pose2, Interpolate)
{
  const dbg::Pose2 a{0.0, 0.0, dbg::kPi - 0.1};
  const dbg::Pose2 b{2.0, -4.0, -dbg::kPi + 0.1};
  EXPECT_TRUE(dbg::approx_equal(dbg::interpolate(a, b, 0.0), a, kTol));
  EXPECT_TRUE(dbg::approx_equal(dbg::interpolate(a, b, 1.0), b, kTol));
  // The yaw takes the short way across +-pi.
  EXPECT_TRUE(dbg::approx_equal(dbg::interpolate(a, b, 0.5), dbg::Pose2(1.0, -2.0, dbg::kPi), kTol));
}

TEST(Pose2, YawStaysNormalised)
{
  dbg::test::RandomPoses rng(14);
  for (int i = 0; i < 1000; ++i) {
    const dbg::Pose2 r = rng.pose(5.0) * rng.pose(5.0);
    EXPECT_GT(r.yaw, -dbg::kPi);
    EXPECT_LE(r.yaw, dbg::kPi);
  }
}
