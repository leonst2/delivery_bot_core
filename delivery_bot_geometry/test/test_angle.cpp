#include <gtest/gtest.h>

#include <cmath>

#include "delivery_bot_geometry/angle.hpp"
#include "test_util.hpp"

namespace dbg = delivery_bot_geometry;

TEST(Angle, NormalizeRange)
{
  dbg::test::RandomPoses rng(1);
  for (int i = 0; i < 10000; ++i) {
    const double a = rng.uniform(-100.0, 100.0);
    const double n = dbg::normalize_angle(a);
    EXPECT_GT(n, -dbg::kPi);
    EXPECT_LE(n, dbg::kPi);
    // Same direction as the input.
    EXPECT_NEAR(std::cos(n), std::cos(a), 1e-12);
    EXPECT_NEAR(std::sin(n), std::sin(a), 1e-12);
  }
}

TEST(Angle, NormalizeBoundaries)
{
  EXPECT_DOUBLE_EQ(dbg::normalize_angle(dbg::kPi), dbg::kPi);
  EXPECT_DOUBLE_EQ(dbg::normalize_angle(-dbg::kPi), dbg::kPi);
  EXPECT_DOUBLE_EQ(dbg::normalize_angle(0.0), 0.0);
  EXPECT_NEAR(dbg::normalize_angle(3.0 * dbg::kPi), dbg::kPi, 1e-12);
  EXPECT_NEAR(dbg::normalize_angle(dbg::kTwoPi + 0.5), 0.5, 1e-12);
  EXPECT_NEAR(dbg::normalize_angle(-dbg::kTwoPi - 0.5), -0.5, 1e-12);
}

TEST(Angle, NormalizePositive)
{
  dbg::test::RandomPoses rng(2);
  for (int i = 0; i < 10000; ++i) {
    const double a = rng.uniform(-100.0, 100.0);
    const double n = dbg::normalize_angle_positive(a);
    EXPECT_GE(n, 0.0);
    EXPECT_LT(n, dbg::kTwoPi);
    EXPECT_NEAR(std::cos(n), std::cos(a), 1e-9);
    EXPECT_NEAR(std::sin(n), std::sin(a), 1e-9);
  }
  EXPECT_DOUBLE_EQ(dbg::normalize_angle_positive(dbg::kTwoPi), 0.0);
  EXPECT_DOUBLE_EQ(dbg::normalize_angle_positive(-1e-12), 0.0);
  EXPECT_NEAR(dbg::normalize_angle_positive(-0.5), dbg::kTwoPi - 0.5, 1e-12);
}

TEST(Angle, DiffIsShortestRotation)
{
  EXPECT_NEAR(dbg::angle_diff(0.1, -0.1), 0.2, 1e-12);
  EXPECT_NEAR(dbg::angle_diff(dbg::kPi - 0.1, -dbg::kPi + 0.1), -0.2, 1e-12);
  EXPECT_NEAR(dbg::angle_diff(-dbg::kPi + 0.1, dbg::kPi - 0.1), 0.2, 1e-12);
  dbg::test::RandomPoses rng(3);
  for (int i = 0; i < 1000; ++i) {
    const double a = rng.uniform(-10.0, 10.0);
    const double b = rng.uniform(-10.0, 10.0);
    // Antisymmetric except at exactly pi, which can't come out of random
    // doubles.
    EXPECT_NEAR(dbg::angle_diff(a, b), -dbg::angle_diff(b, a), 1e-12);
    EXPECT_NEAR(dbg::normalize_angle(b + dbg::angle_diff(a, b)), dbg::normalize_angle(a), 1e-12);
  }
}

TEST(Angle, InterpolateAcrossWrap)
{
  const double from = dbg::kPi - 0.2;
  const double to = -dbg::kPi + 0.2;
  EXPECT_NEAR(dbg::interpolate_angle(from, to, 0.0), from, 1e-12);
  EXPECT_NEAR(dbg::interpolate_angle(from, to, 1.0), to, 1e-12);
  EXPECT_NEAR(dbg::interpolate_angle(from, to, 0.5), dbg::kPi, 1e-12);
}
