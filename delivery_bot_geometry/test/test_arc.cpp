#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/arc.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbg = delivery_bot_geometry;

namespace
{
constexpr double kTol = 1e-9;
constexpr double kRadius = 1.25;

std::vector<dbg::Path> test_paths()
{
  using S = dbg::Segment;
  return {
    dbg::Path({0.0, 0.0, 0.0}, {S::straight(5.0)}),
    dbg::Path({1.0, 2.0, 0.4}, {S::left(2.0, kRadius), S::right(2.0, kRadius)}),               // S-curve
    dbg::Path({0.0, 0.0, 0.0}, {S::straight(2.0), S::left(-1.5, kRadius), S::straight(1.0)}),  // with a cusp
    dbg::Path({-1.0, 0.0, 1.0}, {S::left(dbg::kTwoPi * kRadius * 1.3, kRadius)}),  // more than a full circle
    dbg::Path({0.0, 0.0, -2.0}, {S::right(-2.5, kRadius), S::straight(-1.0), S::left(0.8, 2.0 * kRadius)}),
  };
}

// Brute force: the minimum distance over a dense sampling of the window plus every segment boundary in it (at a
// cusp the distance has a kink, so plain sampling would only be first-order accurate there), refined by sampling
// densely again around the best coarse sample.
double brute_force_distance(const dbg::Path & path, const Eigen::Vector2d & point, double lo, double hi)
{
  const auto distance_at = [&](double s) {
    const dbg::Pose2 p = path.pose_at(s);
    return std::hypot(point.x() - p.x, point.y() - p.y);
  };
  constexpr int kSteps = 20000;
  const double h = (hi - lo) / kSteps;
  double best = std::numeric_limits<double>::infinity();
  double best_s = lo;
  const auto consider = [&](double s) {
    const double d = distance_at(s);
    if (d < best) {
      best = d;
      best_s = s;
    }
  };
  for (int i = 0; i <= kSteps; ++i) {
    consider(lo + h * static_cast<double>(i));
  }
  for (std::size_t i = 0; i <= path.segments().size(); ++i) {
    const double s = path.segment_start_s(i);
    if (s >= lo && s <= hi) {
      consider(s);
    }
  }
  const double centre = best_s;
  for (int i = -1000; i <= 1000; ++i) {
    consider(std::clamp(centre + h * static_cast<double>(i) / 1000.0, lo, hi));
  }
  return best;
}
}  // namespace

TEST(Arc, PointAtIsPoseAt)
{
  for (const auto & path : test_paths()) {
    for (int i = 0; i * 0.13 < path.length() + 1.0; ++i) {
      const double s = -0.5 + i * 0.13;
      EXPECT_TRUE(dbg::approx_equal(dbg::point_at(path, s), path.pose_at(s), kTol));
    }
  }
}

TEST(Arc, ClosestPointMatchesBruteForce)
{
  dbg::test::RandomPoses rng(60);
  for (const auto & path : test_paths()) {
    for (int i = 0; i < 300; ++i) {
      const Eigen::Vector2d p = rng.pose(6.0).position();
      const auto c = dbg::closest_point(path, p);
      // On the path, at the reported distance, and no sample of the path is closer.
      EXPECT_TRUE(dbg::approx_equal(c.pose, path.pose_at(c.s), kTol));
      EXPECT_NEAR(c.distance, std::hypot(p.x() - c.pose.x, p.y() - c.pose.y), kTol);
      const double brute = brute_force_distance(path, p, 0.0, path.length());
      EXPECT_LE(c.distance, brute + kTol);
      EXPECT_GE(c.distance, brute - 1e-6);  // sampling spacing ~2.5e-4 m bounds the brute-force error
    }
  }
}

TEST(Arc, ClosestPointInWindow)
{
  dbg::test::RandomPoses rng(61);
  for (const auto & path : test_paths()) {
    for (int i = 0; i < 200; ++i) {
      const Eigen::Vector2d p = rng.pose(6.0).position();
      double lo = rng.uniform(-1.0, path.length() + 1.0);
      double hi = rng.uniform(-1.0, path.length() + 1.0);
      if (lo > hi) {
        std::swap(lo, hi);
      }
      const auto c = dbg::closest_point(path, p, lo, hi);
      const double clo = std::clamp(lo, 0.0, path.length());
      const double chi = std::clamp(hi, clo, path.length());
      EXPECT_GE(c.s, clo - kTol);
      EXPECT_LE(c.s, chi + kTol);
      EXPECT_LE(c.distance, brute_force_distance(path, p, clo, chi) + kTol);
    }
  }
}

TEST(Arc, PointsOnThePathRoundTrip)
{
  // Away from self-intersections, the closest point of a path point is the point itself.
  const dbg::Path path = test_paths()[1];
  for (int i = 0; i * 0.05 <= path.length(); ++i) {
    const double s = i * 0.05;
    const dbg::Pose2 on = path.pose_at(s);
    const auto c = dbg::closest_point(path, on.position());
    EXPECT_NEAR(c.s, s, 1e-9);
    EXPECT_NEAR(c.distance, 0.0, 1e-9);
  }
}

TEST(Arc, EndsAndCusps)
{
  // Behind the start and past the end clamp to the ends.
  const dbg::Path straight = test_paths()[0];  // 5 m along +x
  EXPECT_NEAR(dbg::closest_point(straight, {-3.0, 1.0}).s, 0.0, kTol);
  EXPECT_NEAR(dbg::closest_point(straight, {7.0, -1.0}).s, 5.0, kTol);
  EXPECT_NEAR(dbg::closest_point(straight, {7.0, -1.0}).distance, std::hypot(2.0, 1.0), kTol);
  // At the cusp (s = 2) of straight -> reverse arc the yaw is the same on both sides, and the point belongs to the
  // reverse segment.
  const dbg::Path cusp = test_paths()[2];
  const auto at_cusp = dbg::closest_point(cusp, {2.0, 0.0});
  EXPECT_NEAR(at_cusp.distance, 0.0, kTol);
  EXPECT_NEAR(at_cusp.pose.yaw, 0.0, kTol);
  EXPECT_EQ(cusp.direction_at(at_cusp.s), at_cusp.s < 2.0 ? 1 : -1);
}

TEST(Arc, TiesGoToTheSmallestS)
{
  // The centre of a full circle is equally far from every point: the start wins.
  const dbg::Path circle({kRadius, 0.0, dbg::kPi / 2.0}, {dbg::Segment::left(dbg::kTwoPi * kRadius, kRadius)});
  EXPECT_NEAR(dbg::closest_point(circle, {0.0, 0.0}).s, 0.0, kTol);
  // A point on a circle driven 1.3 times is found on the first lap.
  const dbg::Path laps({kRadius, 0.0, dbg::kPi / 2.0}, {dbg::Segment::left(dbg::kTwoPi * kRadius * 1.3, kRadius)});
  const auto c = dbg::closest_point(laps, {0.0, kRadius});
  EXPECT_NEAR(c.s, dbg::kPi / 2.0 * kRadius, 1e-9);
  // The same point on two laps is an exact tie, decided only by rounding; it must still go to the first lap on every
  // CPU (this case picked the second lap on ARM64 with fused multiply-adds and the first one on x86_64).
  const dbg::Path overlap({-1.0, 0.0, 1.0}, {dbg::Segment::left(dbg::kTwoPi * kRadius * 1.3, kRadius)});
  EXPECT_LT(dbg::closest_point(overlap, {5.3478002895817873, 2.0045647293698532}).s, dbg::kTwoPi * kRadius);
  // ... unless the window starts after it.
  const auto later = dbg::closest_point(laps, {0.0, kRadius}, dbg::kPi * kRadius);
  EXPECT_NEAR(later.s, (dbg::kTwoPi + dbg::kPi / 2.0) * kRadius, 1e-9);
}

TEST(Arc, CrossTrackAndHeadingErrors)
{
  const dbg::Pose2 ref{1.0, 1.0, dbg::kPi / 2.0};                   // facing +y
  EXPECT_NEAR(dbg::cross_track_error(ref, {0.5, 3.0}), 0.5, kTol);  // left of a +y heading is -x
  EXPECT_NEAR(dbg::cross_track_error(ref, {1.2, 0.0}), -0.2, kTol);
  EXPECT_NEAR(dbg::cross_track_error(ref, {1.0, 7.0}), 0.0, kTol);
  EXPECT_NEAR(dbg::heading_error(ref, dbg::kPi / 2.0 + 0.1), 0.1, kTol);
  EXPECT_NEAR(dbg::heading_error({0.0, 0.0, dbg::kPi - 0.05}, -dbg::kPi + 0.05), 0.1, kTol);

  // On the path: both errors are 0. Offset sideways by d well inside one segment: cross-track = d, heading as set.
  // (Near a segment boundary a point offset towards the inside of one segment can be closer to the next one.)
  dbg::test::RandomPoses rng(62);
  constexpr double kMargin = 0.2;
  for (const auto & path : test_paths()) {
    for (int i = 0; i < 100; ++i) {
      const double s = rng.uniform(0.0, path.length());
      const dbg::Pose2 on = path.pose_at(s);
      const auto zero = dbg::tracking_error(path, on, std::max(0.0, s - 0.1), s + 0.1);
      EXPECT_NEAR(zero.cross_track, 0.0, 1e-9);
      EXPECT_NEAR(zero.heading, 0.0, 1e-9);

      const std::size_t seg = path.segment_index_at(s);
      if (s - path.segment_start_s(seg) < kMargin || path.segment_start_s(seg + 1) - s < kMargin) {
        continue;
      }
      const double d = rng.uniform(-0.1, 0.1);
      const dbg::Pose2 offset = on * dbg::Pose2(0.0, d, 0.05);
      const auto err = dbg::tracking_error(path, offset, s - 0.1, s + 0.1);
      EXPECT_NEAR(err.s, s, 1e-9);
      EXPECT_NEAR(err.cross_track, d, 1e-9);
      EXPECT_NEAR(err.heading, 0.05, 1e-9);
    }
  }
}

TEST(Arc, EmptyPath)
{
  const dbg::Path empty({1.0, 1.0, 0.0}, {});
  const auto c = dbg::closest_point(empty, {4.0, 5.0});
  EXPECT_DOUBLE_EQ(c.s, 0.0);
  EXPECT_NEAR(c.distance, 5.0, kTol);
}
