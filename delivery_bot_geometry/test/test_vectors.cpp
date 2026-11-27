// Recomputes every record of the shared arc test vectors (test/vectors/arc_vectors.txt, also read by the Rust port)
// and compares it with the committed expected values.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/arc.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"

#ifndef ARC_VECTORS_FILE
#error "ARC_VECTORS_FILE must point at test/vectors/arc_vectors.txt"
#endif

namespace dbg = delivery_bot_geometry;

namespace
{

constexpr double kTol = 1e-9;

dbg::Pose2 read_pose(std::istringstream & in)
{
  dbg::Pose2 p;
  in >> p.x >> p.y >> p.yaw;
  return p;
}

double read(std::istringstream & in)
{
  double v = 0.0;
  in >> v;
  return v;
}

void expect_pose(const dbg::Pose2 & actual, const dbg::Pose2 & expected, const std::string & line)
{
  EXPECT_NEAR(actual.x, expected.x, kTol) << line;
  EXPECT_NEAR(actual.y, expected.y, kTol) << line;
  EXPECT_NEAR(dbg::angle_diff(actual.yaw, expected.yaw), 0.0, kTol) << line;
}

}  // namespace

TEST(SharedVectors, MatchTheCommittedFile)
{
  std::ifstream file(ARC_VECTORS_FILE);
  ASSERT_TRUE(file.is_open()) << ARC_VECTORS_FILE;
  dbg::Path path;
  std::size_t records = 0;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream in(line);
    std::string kind;
    in >> kind;
    ++records;
    if (kind == "normalize") {
      const double a = read(in);
      EXPECT_NEAR(dbg::angle_diff(dbg::normalize_angle(a), read(in)), 0.0, kTol) << line;
    } else if (kind == "compose") {
      const dbg::Pose2 a = read_pose(in);
      const dbg::Pose2 b = read_pose(in);
      expect_pose(a * b, read_pose(in), line);
    } else if (kind == "inverse") {
      const dbg::Pose2 a = read_pose(in);
      expect_pose(a.inverse(), read_pose(in), line);
    } else if (kind == "advance") {
      const dbg::Pose2 p = read_pose(in);
      const double k = read(in);
      const double l = read(in);
      expect_pose(dbg::advance(p, k, l), read_pose(in), line);
    } else if (kind == "path") {
      const dbg::Pose2 start = read_pose(in);
      std::size_t n = 0;
      in >> n;
      std::vector<dbg::Segment> segments(n);
      for (auto & seg : segments) {
        in >> seg.curvature >> seg.length;
      }
      path = dbg::Path(start, segments);
    } else if (kind == "length") {
      EXPECT_NEAR(path.length(), read(in), kTol) << line;
    } else if (kind == "pose_at") {
      const double s = read(in);
      expect_pose(dbg::point_at(path, s), read_pose(in), line);
    } else if (kind == "closest") {
      const double px = read(in);
      const double py = read(in);
      const double lo = read(in);
      const double hi = read(in);
      const auto c = dbg::closest_point(path, {px, py}, lo, hi);
      EXPECT_NEAR(c.s, read(in), kTol) << line;
      expect_pose(c.pose, read_pose(in), line);
      EXPECT_NEAR(c.distance, read(in), kTol) << line;
    } else if (kind == "track") {
      const dbg::Pose2 robot = read_pose(in);
      const double lo = read(in);
      const double hi = read(in);
      const auto e = dbg::tracking_error(path, robot, lo, hi);
      EXPECT_NEAR(e.s, read(in), kTol) << line;
      EXPECT_NEAR(e.cross_track, read(in), kTol) << line;
      EXPECT_NEAR(dbg::angle_diff(e.heading, read(in)), 0.0, kTol) << line;
    } else {
      ADD_FAILURE() << "unknown record: " << line;
    }
    EXPECT_FALSE(in.fail()) << "malformed record: " << line;
  }
  EXPECT_GT(records, 500U);
}
