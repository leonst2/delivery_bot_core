// Throughput of the pose checks: the card asks for >= 1 M checks per second.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_collision/grid.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbc = delivery_bot_collision;
namespace dbg = delivery_bot_geometry;

namespace
{

constexpr int kSize = 1000;  // cells per side

// 100 x 100 m at 0.1 m: city blocks (buildings) between 8 m streets, parked cars along the kerbs, some clutter.
dbc::Grid city(dbc::test::Random & rng)
{
  dbc::Grid grid(kSize, kSize, 0.1, -50.0, -50.0, std::int8_t{0});
  const auto fill = [&grid](int x0, int y0, int x1, int y1) {
    for (int y = std::max(y0, 0); y < std::min(y1, kSize); ++y) {
      for (int x = std::max(x0, 0); x < std::min(x1, kSize); ++x) {
        grid.set(x, y, 100);
      }
    }
  };
  for (int bx = 0; bx < kSize; bx += 200) {
    for (int by = 0; by < kSize; by += 200) {
      fill(bx + 40, by + 40, bx + 200, by + 200);  // block, 8 m street on two sides
      for (int car = 0; car < 6; ++car) {
        const int cx = bx + rng.integer(45, 180);
        fill(cx, by + 30, cx + 45, by + 38);  // 4.5 x 0.8 m along the kerb
      }
    }
  }
  for (int i = 0; i < 2000; ++i) {
    const int x = rng.integer(0, kSize - 1);
    const int y = rng.integer(0, kSize - 1);
    fill(x, y, x + 2, y + 2);
  }
  return grid;
}

}  // namespace

TEST(Performance, MillionPoseChecksPerSecond)
{
  dbc::test::Random rng(600);
  const dbc::Grid grid = city(rng);
  const dbc::CollisionChecker checker(grid, dbc::FootprintParams{}.make(), {});
  std::vector<dbg::Pose2> poses;
  constexpr std::size_t kPoses = 1 << 20;
  poses.reserve(kPoses);
  // Poses on the streets (base_link on a free cell), as the planner queries them: close to kerbs and cars, where
  // the exact path runs.
  while (poses.size() < kPoses) {
    const dbg::Pose2 pose = rng.pose_in(grid, 0.0);
    const dbc::CellIndex cell = grid.world_to_cell(pose.x, pose.y);
    if (grid.contains(cell.x, cell.y) && grid.at(cell.x, cell.y) == 0) {
      poses.push_back(pose);
    }
  }

  double best_rate = 0.0;
  std::size_t collisions = 0;
  for (int run = 0; run < 3; ++run) {
    collisions = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (const auto & pose : poses) {
      collisions += checker.in_collision(pose) ? 1 : 0;
    }
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - t0;
    best_rate = std::max(best_rate, static_cast<double>(kPoses) / elapsed.count());
  }
  std::printf(
    "pose checks: %.2f M/s (%.0f %% in collision)\n", best_rate / 1e6,
    100.0 * static_cast<double>(collisions) / static_cast<double>(kPoses));
  // A realistic mix: neither almost everything free nor almost everything blocked.
  EXPECT_GT(collisions, kPoses / 10);
  EXPECT_LT(collisions, kPoses * 9 / 10);
#ifdef __OPTIMIZE__
  EXPECT_GE(best_rate, 1e6);
#else
  GTEST_SKIP() << "unoptimised build: throughput not asserted";
#endif
}
