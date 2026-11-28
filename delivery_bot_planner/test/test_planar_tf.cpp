#include <gtest/gtest.h>

#include <cmath>

#include "delivery_bot_geometry/angle.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "delivery_bot_planner/planar_tf.hpp"

namespace dbg = delivery_bot_geometry;
namespace dbp = delivery_bot_planner;

namespace
{
constexpr double kTol = 1e-12;
}  // namespace

TEST(PlanarTf, ChainMapOdomBaseLink)
{
  dbp::PlanarTf tf;
  const dbg::Pose2 odom_in_map{10.0, 5.0, 0.5};
  const dbg::Pose2 base_in_odom{2.0, -1.0, 1.2};
  tf.set("map", "odom", odom_in_map);
  tf.set("/odom", "base_link", base_in_odom);  // leading '/' ignored
  const auto base_in_map = tf.lookup("map", "base_link");
  ASSERT_TRUE(base_in_map.has_value());
  EXPECT_TRUE(dbg::approx_equal(base_in_map.value_or(dbg::Pose2{}), odom_in_map * base_in_odom, kTol));
  // Reverse direction and identity.
  const auto map_in_base = tf.lookup("base_link", "map");
  EXPECT_TRUE(dbg::approx_equal(map_in_base.value_or(dbg::Pose2{}), (odom_in_map * base_in_odom).inverse(), 1e-9));
  EXPECT_TRUE(dbg::approx_equal(tf.lookup("odom", "odom").value_or(dbg::Pose2{1, 1, 1}), dbg::Pose2{}, kTol));
}

TEST(PlanarTf, SiblingsAndUpdates)
{
  dbp::PlanarTf tf;
  tf.set("base_link", "lidar_link", {0.3, 0.0, 0.0});
  tf.set("base_link", "camera", {0.5, 0.1, 0.2});
  const auto camera_in_lidar = tf.lookup("lidar_link", "camera");
  EXPECT_TRUE(dbg::approx_equal(camera_in_lidar.value_or(dbg::Pose2{}), dbg::Pose2(0.2, 0.1, 0.2), kTol));
  // The latest transform wins.
  tf.set("base_link", "camera", {0.6, 0.0, 0.0});
  EXPECT_TRUE(dbg::approx_equal(tf.lookup("lidar_link", "camera").value_or(dbg::Pose2{}), {0.3, 0.0, 0.0}, kTol));
}

TEST(PlanarTf, UnconnectedAndLoops)
{
  dbp::PlanarTf tf;
  tf.set("map", "odom", {1.0, 0.0, 0.0});
  tf.set("world", "robot", {0.0, 1.0, 0.0});
  EXPECT_FALSE(tf.lookup("map", "robot").has_value());
  EXPECT_FALSE(tf.lookup("map", "base_link").has_value());
  tf.set("a", "b", {});
  tf.set("b", "a", {});
  EXPECT_FALSE(tf.lookup("map", "a").has_value());
}

TEST(PlanarTf, YawFromQuaternion)
{
  for (const double yaw : {0.0, 0.3, -1.2, 2.9, -dbg::kPi + 0.01}) {
    EXPECT_NEAR(dbp::PlanarTf::yaw_from_quaternion(0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)), yaw, 1e-12);
  }
}
