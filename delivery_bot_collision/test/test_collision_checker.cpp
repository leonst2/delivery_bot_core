#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_collision/grid.hpp"
#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "test_util.hpp"

namespace dbc = delivery_bot_collision;
namespace dbg = delivery_bot_geometry;

namespace
{

enum class Truth : std::uint8_t {
  kFree,
  kCollision,
  kAmbiguous,  // overlap area within rounding of zero: SAT and clipping may legitimately disagree
};

// Area of the part of a convex polygon (CCW) that lies inside another convex polygon (Sutherland-Hodgman clipping).
double overlap_area(std::vector<Eigen::Vector2d> subject, const std::vector<Eigen::Vector2d> & clip)
{
  const std::size_t n = clip.size();
  for (std::size_t i = 0; i < n && !subject.empty(); ++i) {
    const Eigen::Vector2d & a = clip[i];
    const Eigen::Vector2d & b = clip[(i + 1) % n];
    const auto side = [&](const Eigen::Vector2d & p) {
      return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x());
    };
    std::vector<Eigen::Vector2d> out;
    for (std::size_t k = 0; k < subject.size(); ++k) {
      const Eigen::Vector2d & p = subject[k];
      const Eigen::Vector2d & q = subject[(k + 1) % subject.size()];
      const double sp = side(p);
      const double sq = side(q);
      if (sp >= 0.0) {
        out.push_back(p);
      }
      if ((sp >= 0.0) != (sq >= 0.0)) {
        out.emplace_back(p + (q - p) * (sp / (sp - sq)));
      }
    }
    subject = std::move(out);
  }
  double area2 = 0.0;
  for (std::size_t k = 0; k < subject.size(); ++k) {
    const Eigen::Vector2d & p = subject[k];
    const Eigen::Vector2d & q = subject[(k + 1) % subject.size()];
    area2 += p.x() * q.y() - p.y() * q.x();
  }
  return std::abs(area2) / 2.0;
}

// Independent reference: clip every obstacle square of the whole grid against the placed footprint.
Truth reference(
  const dbc::Grid & grid, const dbc::OccupancyRule & rule, const dbc::Footprint & footprint, const dbg::Pose2 & pose)
{
  std::vector<Eigen::Vector2d> world;
  for (const auto & v : footprint.vertices()) {
    world.push_back(pose.transform(v));
  }
  if (rule.outside_is_obstacle()) {
    for (const auto & v : world) {
      if (!grid.contains_world(v.x(), v.y())) {
        return Truth::kCollision;
      }
    }
  }
  const double res = grid.resolution();
  bool ambiguous = false;
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (!rule.is_obstacle(grid.at(x, y))) {
        continue;
      }
      const double x0 = grid.origin_x() + x * res;
      const double y0 = grid.origin_y() + y * res;
      const double area = overlap_area({{x0, y0}, {x0 + res, y0}, {x0 + res, y0 + res}, {x0, y0 + res}}, world);
      if (area > 1e-9) {
        return Truth::kCollision;
      }
      ambiguous = ambiguous || area > 0.0;
    }
  }
  return ambiguous ? Truth::kAmbiguous : Truth::kFree;
}

void expect_matches_reference(
  const dbc::CollisionChecker & checker, const dbg::Pose2 & pose, int & compared, int & collisions)
{
  const Truth truth = reference(checker.grid(), checker.params().rule, checker.footprint(), pose);
  if (truth == Truth::kAmbiguous) {
    return;
  }
  const bool expected = truth == Truth::kCollision;
  EXPECT_EQ(checker.in_collision(pose), expected) << pose.x << " " << pose.y << " " << pose.yaw;
  EXPECT_EQ(checker.in_collision_exact(pose), expected) << pose.x << " " << pose.y << " " << pose.yaw;
  ++compared;
  collisions += expected ? 1 : 0;
}

double point_segment_distance(const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double t = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
  return (a + t * ab - p).norm();
}

// Distance between two disjoint convex polygons: the closest pair always involves a vertex of one of them.
double polygon_distance(const std::vector<Eigen::Vector2d> & a, const std::vector<Eigen::Vector2d> & b)
{
  double best = std::numeric_limits<double>::infinity();
  for (int pass = 0; pass < 2; ++pass) {
    const auto & p = pass == 0 ? a : b;
    const auto & q = pass == 0 ? b : a;
    for (const auto & v : p) {
      for (std::size_t i = 0; i < q.size(); ++i) {
        best = std::min(best, point_segment_distance(v, q[i], q[(i + 1) % q.size()]));
      }
    }
  }
  return best;
}

// True distance from the placed footprint to the nearest obstacle square and, if unknown is lethal, to the map
// border. Only valid for collision-free poses.
double true_clearance(
  const dbc::Grid & grid, const dbc::OccupancyRule & rule, const dbc::Footprint & footprint, const dbg::Pose2 & pose)
{
  std::vector<Eigen::Vector2d> world;
  for (const auto & v : footprint.vertices()) {
    world.push_back(pose.transform(v));
  }
  double best = std::numeric_limits<double>::infinity();
  const double res = grid.resolution();
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (rule.is_obstacle(grid.at(x, y))) {
        const double x0 = grid.origin_x() + x * res;
        const double y0 = grid.origin_y() + y * res;
        best =
          std::min(best, polygon_distance(world, {{x0, y0}, {x0 + res, y0}, {x0 + res, y0 + res}, {x0, y0 + res}}));
      }
    }
  }
  if (rule.outside_is_obstacle()) {
    const double x_max = grid.origin_x() + grid.width() * res;
    const double y_max = grid.origin_y() + grid.height() * res;
    for (const auto & v : world) {
      best = std::min({best, v.x() - grid.origin_x(), x_max - v.x(), v.y() - grid.origin_y(), y_max - v.y()});
    }
  }
  return best;
}

const dbc::Footprint & hexagon()
{
  static const dbc::Footprint kHexagon({{0.6, 0.0}, {0.4, 0.3}, {-0.2, 0.3}, {-0.3, 0.0}, {-0.2, -0.3}, {0.4, -0.3}});
  return kHexagon;
}

}  // namespace

TEST(CollisionChecker, MatchesReferenceOnRandomGrids)
{
  dbc::test::Random rng(400);
  int compared = 0;
  int collisions = 0;
  for (int g = 0; g < 24; ++g) {
    const dbc::Grid grid = rng.grid(rng.integer(15, 45), rng.integer(15, 45), 0.1, rng.uniform(0.0, 0.04), 0.01);
    const dbc::OccupancyRule rule{65, g % 2 == 0 ? dbc::UnknownPolicy::kLethal : dbc::UnknownPolicy::kFree};
    const dbc::Footprint fp = g % 3 == 0 ? hexagon() : dbc::FootprintParams{}.make();
    const dbc::CollisionChecker checker(grid, fp, {rule, 3, 0.05});
    for (int i = 0; i < 150; ++i) {
      expect_matches_reference(checker, rng.pose_in(grid, 0.6), compared, collisions);
    }
  }
  // Both outcomes occur often enough for the comparison to mean something.
  EXPECT_GT(compared, 3000);
  EXPECT_GT(collisions, compared / 10);
  EXPECT_LT(collisions, compared * 9 / 10);
}

TEST(CollisionChecker, EveryYaw)
{
  // One obstacle cell next to the robot: whether it is hit depends on the yaw alone.
  dbc::Grid grid(40, 40, 0.1, -2.0, -2.0, std::int8_t{0});
  grid.set(26, 20, 100);  // cell [0.6, 0.7] x [0.0, 0.1]
  grid.set(17, 25, 100);
  const dbc::CollisionChecker checker(grid, dbc::FootprintParams{}.make(), {});
  dbc::test::Random rng(401);
  int compared = 0;
  int collisions = 0;
  for (const dbg::Pose2 base : {dbg::Pose2{0.0, 0.0, 0.0}, dbg::Pose2{0.05, -0.03, 0.0}, dbg::Pose2{-0.1, 0.1, 0.0}}) {
    for (int k = 0; k < 72; ++k) {
      expect_matches_reference(checker, {base.x, base.y, -dbg::kPi + k * dbg::kTwoPi / 72.0}, compared, collisions);
    }
    for (int k = 0; k < 500; ++k) {
      expect_matches_reference(checker, {base.x, base.y, rng.uniform(-dbg::kPi, dbg::kPi)}, compared, collisions);
    }
  }
  EXPECT_GT(collisions, 0);
  EXPECT_LT(collisions, compared);
}

TEST(CollisionChecker, MapBorders)
{
  const dbc::Grid empty(30, 30, 0.1, 0.0, 0.0, std::int8_t{0});
  const dbc::Footprint fp = dbc::FootprintParams{}.make();  // x -0.135 ... 0.625, y -0.32 ... 0.32
  const dbc::CollisionChecker lethal(empty, fp, {{65, dbc::UnknownPolicy::kLethal}, 3, 0.05});
  const dbc::CollisionChecker free(empty, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
  EXPECT_FALSE(lethal.in_collision({1.5, 1.5, 0.3}));
  // The nose pokes out of the right border (x = 3).
  EXPECT_TRUE(lethal.in_collision({2.5, 1.5, 0.0}));
  EXPECT_FALSE(free.in_collision({2.5, 1.5, 0.0}));
  // Exactly on the border still counts as inside.
  EXPECT_FALSE(lethal.in_collision({3.0 - 0.625, 1.5, 0.0}));
  EXPECT_TRUE(lethal.in_collision({3.0 - 0.624, 1.5, 0.0}));
  // Entirely outside.
  EXPECT_TRUE(lethal.in_collision({-5.0, -5.0, 1.0}));
  EXPECT_FALSE(free.in_collision({-5.0, -5.0, 1.0}));
}

TEST(CollisionChecker, UnknownCells)
{
  dbc::Grid grid(30, 30, 0.1, 0.0, 0.0, std::int8_t{0});
  grid.set(15, 15, -1);
  const dbc::Footprint fp = dbc::FootprintParams{}.make();
  const dbc::CollisionChecker lethal(grid, fp, {{65, dbc::UnknownPolicy::kLethal}, 3, 0.05});
  const dbc::CollisionChecker free(grid, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
  EXPECT_TRUE(lethal.in_collision({1.45, 1.5, 0.0}));
  EXPECT_FALSE(free.in_collision({1.45, 1.5, 0.0}));
  // The occupied threshold decides for known cells.
  grid.set(15, 15, 70);
  EXPECT_TRUE(dbc::CollisionChecker(grid, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05}).in_collision({1.45, 1.5, 0}));
  EXPECT_FALSE(
    dbc::CollisionChecker(grid, fp, {{80, dbc::UnknownPolicy::kFree}, 3, 0.05}).in_collision({1.45, 1.5, 0}));
}

TEST(CollisionChecker, TouchingIsNotACollision)
{
  dbc::Grid grid(20, 20, 0.1, 0.0, 0.0, std::int8_t{0});
  grid.set(10, 10, 100);  // square [1.0, 1.1]^2
  const auto fp = dbc::Footprint::rectangle(-0.2, 0.2, -0.1, 0.1);
  const dbc::CollisionChecker checker(grid, fp, {{65, dbc::UnknownPolicy::kFree}, 3, 0.05});
  EXPECT_FALSE(checker.in_collision({0.8, 1.05, 0.0}));  // right edge at x = 1.0
  EXPECT_TRUE(checker.in_collision({0.8 + 1e-9, 1.05, 0.0}));
  EXPECT_FALSE(checker.in_collision({1.05, 0.9, 0.0}));  // top edge at y = 1.0
}

TEST(CollisionChecker, PaddingIsMonotone)
{
  dbc::test::Random rng(402);
  const dbc::Grid grid = rng.grid(40, 40, 0.1, 0.03, 0.0);
  std::vector<dbc::CollisionChecker> checkers;
  for (const double padding : {0.0, 0.02, 0.05, 0.1, 0.2}) {
    dbc::FootprintParams params;
    params.padding = padding;
    checkers.emplace_back(grid, params.make(), dbc::CheckerParams{});
  }
  for (int i = 0; i < 2000; ++i) {
    const dbg::Pose2 pose = rng.pose_in(grid, 0.0);
    for (std::size_t k = 1; k < checkers.size(); ++k) {
      if (checkers[k - 1].in_collision(pose)) {
        EXPECT_TRUE(checkers[k].in_collision(pose));
      }
    }
  }
}

TEST(CollisionChecker, ClearanceIsALowerBound)
{
  dbc::test::Random rng(403);
  const dbc::Grid grid = rng.grid(50, 50, 0.1, 0.01, 0.0);
  const dbc::Footprint fp = dbc::FootprintParams{}.make();
  const dbc::CollisionChecker checker(grid, fp, {});
  int positive = 0;
  for (int i = 0; i < 1000; ++i) {
    const dbg::Pose2 pose = rng.pose_in(grid, -0.5);
    const double clearance = checker.clearance_lower_bound(pose);
    EXPECT_GE(clearance, 0.0);
    if (clearance > 1e-6) {
      ++positive;
      ASSERT_FALSE(checker.in_collision(pose));
      // Nothing is closer than the bound. (Padding the footprint by the clearance is no test: padding mitres the
      // corners, which then reach further than the padding distance.)
      EXPECT_GE(true_clearance(grid, checker.params().rule, fp, pose), clearance - 1e-12);
    }
  }
  EXPECT_GT(positive, 50);
}

TEST(CollisionChecker, RejectsBadParams)
{
  const dbc::Grid grid(4, 4, 0.1, 0.0, 0.0, std::int8_t{0});
  const dbc::Footprint fp = dbc::FootprintParams{}.make();
  EXPECT_THROW(dbc::CollisionChecker(grid, fp, {{}, 0, 0.05}), std::invalid_argument);
  EXPECT_THROW(dbc::CollisionChecker(grid, fp, {{}, 3, 0.0}), std::invalid_argument);
}
