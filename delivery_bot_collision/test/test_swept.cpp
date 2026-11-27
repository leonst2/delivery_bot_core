#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_collision/grid.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/reeds_shepp.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbc = delivery_bot_collision;
namespace dbg = delivery_bot_geometry;

namespace
{

constexpr double kRes = 0.05;  // one cell = the thinnest obstacle the card names

// Dense ground truth: exact checks of the unpadded footprint every millimetre along the segment.
bool dense_collision(const dbc::CollisionChecker & checker, const dbg::Pose2 & start, const dbg::Segment & seg)
{
  const double length = std::abs(seg.length);
  const auto steps = static_cast<int>(std::ceil(length / 1e-3));
  for (int k = 0; k <= steps; ++k) {
    const double s = length * k / std::max(steps, 1);
    if (checker.in_collision_exact(dbg::advance(start, seg.curvature, seg.direction() * s))) {
      return true;
    }
  }
  return false;
}

dbg::Segment random_segment(dbc::test::Random & rng)
{
  const double k = rng.integer(0, 3) == 0 ? 0.0 : rng.uniform(-0.8, 0.8);
  return {k, rng.uniform(-2.0, 2.0)};
}

}  // namespace

TEST(Swept, SampleSpacing)
{
  dbc::test::Random rng(500);
  const dbc::Grid grid(10, 10, 0.1, 0.0, 0.0, std::int8_t{0});
  const dbc::CollisionChecker checker(grid, dbc::FootprintParams{}.make(), {});
  for (int i = 0; i < 500; ++i) {
    const dbg::Pose2 start{rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-3.0, 3.0)};
    const dbg::Segment seg = random_segment(rng);
    const auto poses = checker.sweep_samples(start, seg);
    ASSERT_GE(poses.size(), 2U);
    EXPECT_TRUE(dbg::approx_equal(poses.front(), start, 1e-12));
    EXPECT_TRUE(dbg::approx_equal(poses.back(), dbg::advance(start, seg.curvature, seg.length), 1e-9));
    // Measured along the true motion, no footprint vertex travels more than sweep_step between two samples.
    for (std::size_t k = 1; k < poses.size(); ++k) {
      for (const auto & v : checker.footprint().vertices()) {
        double travelled = 0.0;
        Eigen::Vector2d previous = poses[k - 1].transform(v);
        for (int j = 1; j <= 50; ++j) {
          const double ds = std::abs(seg.length) / static_cast<double>(poses.size() - 1) * j / 50.0;
          const Eigen::Vector2d now = dbg::advance(poses[k - 1], seg.curvature, seg.direction() * ds).transform(v);
          travelled += (now - previous).norm();
          previous = now;
        }
        EXPECT_LE(travelled, checker.params().sweep_step + 1e-9);
      }
    }
  }
}

TEST(Swept, NeverMissesACellBetweenSamples)
{
  // Adversarial: put a single 5 cm cell where a footprint corner passes halfway between two samples.
  dbc::test::Random rng(501);
  const dbc::Footprint fp = dbc::FootprintParams{}.make();
  int detected = 0;
  for (int i = 0; i < 600; ++i) {
    dbc::Grid grid(160, 160, kRes, -4.0, -4.0, std::int8_t{0});
    const dbg::Pose2 start{rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-3.0, 3.0)};
    const dbg::Segment seg = random_segment(rng);
    const dbc::CollisionChecker empty(grid, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
    const auto samples = empty.sweep_samples(start, seg);
    const auto k = static_cast<std::size_t>(rng.integer(0, static_cast<int>(samples.size()) - 2));
    const double step = std::abs(seg.length) / static_cast<double>(samples.size() - 1);
    const dbg::Pose2 between = dbg::advance(samples[k], seg.curvature, seg.direction() * step / 2.0);
    const auto & corners = fp.vertices();
    const Eigen::Vector2d p = between.transform(corners[static_cast<std::size_t>(rng.integer(0, 3))]);
    const dbc::CellIndex cell = grid.world_to_cell(p.x(), p.y());
    ASSERT_TRUE(grid.contains(cell.x, cell.y));
    grid.set(cell.x, cell.y, 100);
    const dbc::CollisionChecker checker(grid, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
    if (dense_collision(checker, start, seg)) {
      EXPECT_TRUE(checker.segment_in_collision(start, seg)) << i;
      ++detected;
    }
  }
  EXPECT_GT(detected, 300);
}

TEST(Swept, ThinWallAcrossTheSegment)
{
  // A wall one 5 cm cell thick across the path is always found, at any angle.
  dbc::test::Random rng(502);
  const dbc::Footprint fp = dbc::FootprintParams{}.make();
  for (int i = 0; i < 100; ++i) {
    dbc::Grid grid(200, 200, kRes, -5.0, -5.0, std::int8_t{0});
    const double angle = rng.uniform(-3.0, 3.0);
    const double offset = rng.uniform(1.0, 1.8);  // ahead of the start, within reach of a 2 m drive
    for (int j = -240; j <= 240; ++j) {
      const double t = j * kRes / 4.0;  // -3 ... 3 m along the wall
      const Eigen::Vector2d q{
        offset * std::cos(angle) - t * std::sin(angle), offset * std::sin(angle) + t * std::cos(angle)};
      const dbc::CellIndex cell = grid.world_to_cell(q.x(), q.y());
      if (grid.contains(cell.x, cell.y)) {
        grid.set(cell.x, cell.y, 100);
      }
    }
    const dbc::CollisionChecker checker(grid, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
    const dbg::Pose2 start{0.0, 0.0, angle};
    const dbg::Segment seg{rng.uniform(-0.3, 0.3), 2.0};
    EXPECT_EQ(dense_collision(checker, start, seg), true) << i;  // the setup really crosses the wall
    EXPECT_TRUE(checker.segment_in_collision(start, seg)) << i;
  }
}

TEST(Swept, PathsWithCuspsMatchDenseChecks)
{
  dbc::test::Random rng(503);
  const dbc::Footprint fp = dbc::FootprintParams{}.make();
  int collisions = 0;
  int frees = 0;
  for (int i = 0; i < 80; ++i) {
    const dbc::Grid grid = rng.grid(120, 120, kRes, 0.0012, 0.0);
    const dbc::CollisionChecker checker(grid, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
    const dbg::Pose2 a = rng.pose_in(grid, -1.5);
    const dbg::Pose2 b{a.x + rng.uniform(-1.5, 1.5), a.y + rng.uniform(-1.5, 1.5), rng.uniform(-3.0, 3.0)};
    const dbg::Path path = dbg::reeds_shepp_shortest(a, b, 1.25).path;
    bool dense = false;
    for (std::size_t s = 0; s < path.segments().size(); ++s) {
      dense = dense || dense_collision(checker, path.segment_start_pose(s), path.segments()[s]);
    }
    const bool swept = checker.path_in_collision(path);
    if (dense) {
      EXPECT_TRUE(swept) << i;  // never misses
      ++collisions;
    } else if (!swept) {
      ++frees;
    }
  }
  EXPECT_GT(collisions, 5);
  EXPECT_GT(frees, 5);
}

TEST(Swept, EmptyPathChecksTheStart)
{
  dbc::Grid grid(20, 20, 0.1, 0.0, 0.0, std::int8_t{0});
  grid.set(10, 10, 100);
  const dbc::CollisionChecker checker(grid, dbc::FootprintParams{}.make(), {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
  EXPECT_TRUE(checker.path_in_collision(dbg::Path({1.0, 1.0, 0.0}, {})));
  EXPECT_FALSE(checker.path_in_collision(dbg::Path({0.3, 0.3, 0.0}, {})));
}
