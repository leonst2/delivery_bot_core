#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/primitives.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbg = delivery_bot_geometry;

namespace
{
constexpr double kTol = 1e-9;
}  // namespace

TEST(Primitives, DefaultSetMatchesTheRobot)
{
  const dbg::AckermannParams params;
  // 0.49 m wheelbase and R_min 1.25 m: the 21.4 deg steering limit of the robot.
  EXPECT_NEAR(dbg::max_steering_angle(params), 21.4 * dbg::kPi / 180.0, 1e-3);
  const auto primitives = dbg::make_primitives(params);
  ASSERT_EQ(primitives.size(), 10U);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(primitives[i].direction(), 1);
    EXPECT_EQ(primitives[i + 5].direction(), -1);
  }
  EXPECT_DOUBLE_EQ(primitives[0].segment.curvature, -1.0 / 1.25);
  EXPECT_DOUBLE_EQ(primitives[2].segment.curvature, 0.0);
  EXPECT_DOUBLE_EQ(primitives[4].segment.curvature, 1.0 / 1.25);
}

TEST(Primitives, Properties)
{
  for (const int count : {1, 3, 5, 7, 11}) {
    for (const bool reverse : {true, false}) {
      dbg::AckermannParams params;
      params.steering_count = count;
      params.allow_reverse = reverse;
      params.arc_length = 0.7;
      const auto primitives = dbg::make_primitives(params);
      const auto n = static_cast<std::size_t>(count);
      ASSERT_EQ(primitives.size(), reverse ? 2 * n : n);
      for (const auto & p : primitives) {
        // Fixed arc length, curvature within the limit, consistent with the steering angle.
        EXPECT_DOUBLE_EQ(std::abs(p.segment.length), params.arc_length);
        EXPECT_LE(std::abs(p.segment.curvature), 1.0 / params.min_turning_radius + kTol);
        EXPECT_NEAR(dbg::steering_angle_for_curvature(params.wheelbase, p.segment.curvature), p.steering_angle, kTol);
        EXPECT_TRUE(dbg::approx_equal(p.delta, dbg::advance({}, p.segment.curvature, p.segment.length), kTol));
        // The end pose of the path is the same as apply().
        const dbg::Pose2 from{1.0, -2.0, 0.7};
        EXPECT_TRUE(dbg::approx_equal(p.path_from(from).end(), p.apply(from), kTol));
        // Never moves further than its arc.
        EXPECT_LE(p.delta.position().norm(), params.arc_length + kTol);
      }
      for (std::size_t i = 0; i < n; ++i) {
        // Left/right mirror pairs within a direction.
        const auto & a = primitives[i];
        const auto & b = primitives[n - 1 - i];
        EXPECT_NEAR(a.segment.curvature, -b.segment.curvature, kTol);
        EXPECT_TRUE(dbg::approx_equal(a.delta, dbg::Pose2(b.delta.x, -b.delta.y, -b.delta.yaw), kTol));
        if (reverse) {
          // The reverse primitive undoes the forward one with the same steering.
          const auto & back = primitives[n + i];
          EXPECT_DOUBLE_EQ(back.segment.curvature, a.segment.curvature);
          EXPECT_TRUE(dbg::approx_equal(back.delta, a.delta.inverse(), kTol));
        }
      }
      // The straight one is always there (odd count).
      EXPECT_DOUBLE_EQ(primitives[n / 2].segment.curvature, 0.0);
    }
  }
}

TEST(Primitives, SteeringConversionsRoundTrip)
{
  dbg::test::RandomPoses rng(50);
  for (int i = 0; i < 1000; ++i) {
    const double angle = rng.uniform(-0.6, 0.6);
    const double k = dbg::curvature_for_steering_angle(0.49, angle);
    EXPECT_NEAR(dbg::steering_angle_for_curvature(0.49, k), angle, 1e-12);
  }
}

TEST(Primitives, RejectsBadParams)
{
  dbg::AckermannParams even;
  even.steering_count = 4;
  EXPECT_THROW(dbg::make_primitives(even), std::invalid_argument);
  dbg::AckermannParams none;
  none.steering_count = 0;
  EXPECT_THROW(dbg::make_primitives(none), std::invalid_argument);
  dbg::AckermannParams length;
  length.arc_length = 0.0;
  EXPECT_THROW(dbg::make_primitives(length), std::invalid_argument);
  dbg::AckermannParams radius;
  radius.min_turning_radius = -1.0;
  EXPECT_THROW(dbg::make_primitives(radius), std::invalid_argument);
}
