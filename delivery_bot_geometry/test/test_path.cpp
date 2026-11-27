#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbg = delivery_bot_geometry;

namespace
{
constexpr double kTol = 1e-9;
constexpr double kRadius = 1.25;
}  // namespace

TEST(Advance, StraightAndArcKnownValues)
{
  const dbg::Pose2 origin{};
  EXPECT_TRUE(dbg::approx_equal(dbg::advance(origin, 0.0, 2.0), dbg::Pose2(2.0, 0.0, 0.0), kTol));
  EXPECT_TRUE(dbg::approx_equal(dbg::advance(origin, 0.0, -2.0), dbg::Pose2(-2.0, 0.0, 0.0), kTol));
  // Quarter circle to the left: ends at (R, R) facing +y.
  const double quarter = dbg::kPi / 2.0 * kRadius;
  EXPECT_TRUE(dbg::approx_equal(
    dbg::advance(origin, 1.0 / kRadius, quarter), dbg::Pose2(kRadius, kRadius, dbg::kPi / 2.0), kTol));
  // Quarter circle to the right: ends at (R, -R) facing -y.
  EXPECT_TRUE(dbg::approx_equal(
    dbg::advance(origin, -1.0 / kRadius, quarter), dbg::Pose2(kRadius, -kRadius, -dbg::kPi / 2.0), kTol));
  // Reverse with the wheels left: backwards and clockwise, ending at (-R, R) facing -y.
  EXPECT_TRUE(dbg::approx_equal(
    dbg::advance(origin, 1.0 / kRadius, -quarter), dbg::Pose2(-kRadius, kRadius, -dbg::kPi / 2.0), kTol));
  // Full circle returns to the start.
  EXPECT_TRUE(dbg::approx_equal(dbg::advance(origin, 1.0 / kRadius, dbg::kTwoPi * kRadius), origin, kTol));
}

TEST(Advance, Properties)
{
  dbg::test::RandomPoses rng(20);
  for (int i = 0; i < 2000; ++i) {
    const dbg::Pose2 p = rng.pose(10.0);
    const double k = rng.uniform(-1.0 / kRadius, 1.0 / kRadius);
    const double a = rng.uniform(-3.0, 3.0);
    const double b = rng.uniform(-3.0, 3.0);
    // Driving back the same length undoes the motion.
    EXPECT_TRUE(dbg::approx_equal(dbg::advance(dbg::advance(p, k, a), k, -a), p, kTol));
    // Splitting a segment doesn't change where it ends.
    EXPECT_TRUE(dbg::approx_equal(dbg::advance(dbg::advance(p, k, a), k, b), dbg::advance(p, k, a + b), kTol));
    // Left/right mirror symmetry: mirroring the start and the curvature mirrors the end.
    const dbg::Pose2 mirrored_start{p.x, -p.y, -p.yaw};
    const dbg::Pose2 end = dbg::advance(p, k, a);
    EXPECT_TRUE(dbg::approx_equal(dbg::advance(mirrored_start, -k, a), dbg::Pose2(end.x, -end.y, -end.yaw), kTol));
    // The travelled chord is never longer than the arc.
    EXPECT_LE(dbg::distance(p, end), std::abs(a) + kTol);
  }
}

TEST(Advance, ContinuousAtZeroCurvature)
{
  const dbg::Pose2 p{1.0, 2.0, 0.3};
  const dbg::Pose2 straight = dbg::advance(p, 0.0, 1.5);
  for (const double k : {1e-3, 1e-5, 1e-7, 1e-9, -1e-9, -1e-5}) {
    EXPECT_TRUE(dbg::approx_equal(dbg::advance(p, k, 1.5), straight, std::abs(k) * 2.0 + 1e-12)) << k;
  }
}

TEST(Path, ComposesSegments)
{
  const dbg::Pose2 start{1.0, -1.0, 0.5};
  const std::vector<dbg::Segment> segments{
    dbg::Segment::left(1.0, kRadius), dbg::Segment::straight(-2.0), dbg::Segment::right(0.7, kRadius)};
  const dbg::Path path(start, segments);
  dbg::Pose2 expected = start;
  for (const auto & seg : segments) {
    expected = dbg::advance(expected, seg.curvature, seg.length);
  }
  EXPECT_TRUE(dbg::approx_equal(path.end(), expected, kTol));
  EXPECT_NEAR(path.length(), 3.7, kTol);
  EXPECT_EQ(path.cusp_count(), 2U);
  EXPECT_TRUE(dbg::approx_equal(path.pose_at(0.0), start, kTol));
  EXPECT_TRUE(dbg::approx_equal(path.pose_at(path.length()), expected, kTol));
  // Clamped outside [0, length].
  EXPECT_TRUE(dbg::approx_equal(path.pose_at(-1.0), start, kTol));
  EXPECT_TRUE(dbg::approx_equal(path.pose_at(100.0), expected, kTol));
}

TEST(Path, BoundariesBelongToTheNextSegment)
{
  const dbg::Path path(
    dbg::Pose2{}, {dbg::Segment::left(1.0, kRadius), dbg::Segment::straight(-2.0), dbg::Segment::right(0.5, kRadius)});
  EXPECT_EQ(path.segment_index_at(0.0), 0U);
  EXPECT_EQ(path.segment_index_at(0.999), 0U);
  EXPECT_EQ(path.segment_index_at(1.0), 1U);
  EXPECT_EQ(path.segment_index_at(3.0), 2U);
  EXPECT_EQ(path.segment_index_at(3.5), 2U);
  EXPECT_EQ(path.segment_index_at(10.0), 2U);
  EXPECT_EQ(path.direction_at(0.5), 1);
  EXPECT_EQ(path.direction_at(1.0), -1);
  EXPECT_EQ(path.direction_at(3.0), 1);
  EXPECT_NEAR(path.curvature_at(0.5), 1.0 / kRadius, kTol);
  EXPECT_NEAR(path.curvature_at(2.0), 0.0, kTol);
  EXPECT_NEAR(path.curvature_at(3.2), -1.0 / kRadius, kTol);
}

TEST(Path, DropsZeroLengthSegments)
{
  const dbg::Path path(dbg::Pose2{}, {dbg::Segment::straight(0.0), dbg::Segment::left(1.0, kRadius), {0.3, 1e-15}});
  ASSERT_EQ(path.segments().size(), 1U);
  EXPECT_EQ(path.cusp_count(), 0U);
}

TEST(Path, EmptyPath)
{
  const dbg::Pose2 start{3.0, 4.0, 1.0};
  const dbg::Path path(start, {});
  EXPECT_TRUE(path.empty());
  EXPECT_DOUBLE_EQ(path.length(), 0.0);
  EXPECT_TRUE(dbg::approx_equal(path.end(), start, kTol));
  EXPECT_TRUE(dbg::approx_equal(path.pose_at(1.0), start, kTol));
  const auto samples = path.sample(0.1);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_TRUE(dbg::approx_equal(samples[0].pose, start, kTol));
}

TEST(Path, PoseIsContinuous)
{
  const dbg::Path path(
    dbg::Pose2{0.0, 0.0, 0.2}, {dbg::Segment::left(2.0, kRadius), dbg::Segment::straight(1.0),
                                dbg::Segment::right(-1.5, kRadius), dbg::Segment::left(-0.5, kRadius)});
  constexpr double kStep = 1e-3;
  for (double s = 0.0; s + kStep <= path.length(); s += kStep) {
    const dbg::Pose2 a = path.pose_at(s);
    const dbg::Pose2 b = path.pose_at(s + kStep);
    EXPECT_LE(dbg::distance(a, b), kStep + 1e-12);
    EXPECT_LE(std::abs(dbg::angle_diff(b.yaw, a.yaw)), kStep / kRadius + 1e-12);
  }
}

TEST(Path, Sampling)
{
  const dbg::Path path(
    dbg::Pose2{1.0, 1.0, -0.4},
    {dbg::Segment::left(1.03, kRadius), dbg::Segment::straight(0.5), dbg::Segment::right(-0.77, kRadius)});
  for (const double resolution : {0.05, 0.1, 0.3, 2.0}) {
    const auto samples = path.sample(resolution);
    ASSERT_GE(samples.size(), 2U);
    EXPECT_TRUE(dbg::approx_equal(samples.front().pose, path.start(), kTol));
    EXPECT_TRUE(dbg::approx_equal(samples.back().pose, path.end(), kTol));
    EXPECT_NEAR(samples.back().s, path.length(), kTol);
    for (std::size_t i = 1; i < samples.size(); ++i) {
      const double ds = samples[i].s - samples[i - 1].s;
      EXPECT_GT(ds, 0.0);
      EXPECT_LE(ds, resolution + kTol);
      EXPECT_LE(dbg::distance(samples[i - 1].pose, samples[i].pose), ds + kTol);
      EXPECT_LE(std::abs(samples[i].curvature), 1.0 / kRadius + kTol);
    }
    for (const auto & sample : samples) {
      EXPECT_TRUE(dbg::approx_equal(sample.pose, path.pose_at(sample.s), kTol));
      EXPECT_EQ(sample.direction, path.direction_at(sample.s));
    }
    // Every segment boundary (here: 1.03 and 1.53, the cusp) is a sample.
    for (std::size_t i = 1; i < path.segments().size(); ++i) {
      bool found = false;
      for (const auto & sample : samples) {
        found = found || std::abs(sample.s - path.segment_start_s(i)) < kTol;
      }
      EXPECT_TRUE(found) << "boundary " << i;
    }
  }
}
