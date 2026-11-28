// The planner node in-process: map in, PlanPath goals out, /plan published, start from TF.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <delivery_bot_interfaces/action/plan_path.hpp>
#include <delivery_bot_interfaces/msg/heartbeat.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "delivery_bot_planner/planner_node.hpp"

using delivery_bot_interfaces::action::PlanPath;
using namespace std::chrono_literals;

namespace
{

geometry_msgs::msg::PoseStamped pose(const std::string & frame, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

// 20 x 20 m at 0.1 m with a wall around it.
nav_msgs::msg::OccupancyGrid map()
{
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id = "map";
  m.info.resolution = 0.1F;
  m.info.width = 200;
  m.info.height = 200;
  m.info.origin.orientation.w = 1.0;
  constexpr std::size_t kSize = 200;
  m.data.assign(kSize * kSize, 0);
  for (std::size_t i = 0; i < kSize; ++i) {
    m.data[i] = 100;
    m.data[(kSize - 1) * kSize + i] = 100;
    m.data[i * kSize] = 100;
    m.data[i * kSize + kSize - 1] = 100;
  }
  return m;
}

class PlannerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"map_topic", std::string("/test/map")}, {"default_time_limit", 5.0}});
    planner_ = std::make_shared<delivery_bot_planner::PlannerNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("planner_test_helper");
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    map_publisher_ = helper_->create_publisher<nav_msgs::msg::OccupancyGrid>("/test/map", latched);
    tf_publisher_ = helper_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    plan_subscription_ = helper_->create_subscription<nav_msgs::msg::Path>(
      "/plan", latched, [this](const nav_msgs::msg::Path & path) { plans_received_ += path.poses.empty() ? 0 : 1; });
    heartbeat_subscription_ = helper_->create_subscription<delivery_bot_interfaces::msg::Heartbeat>(
      "/heartbeat", 10, [this](const delivery_bot_interfaces::msg::Heartbeat & h) {
        const std::lock_guard<std::mutex> lock(detail_mutex_);
        last_detail_ = h.detail;
      });
    client_ = rclcpp_action::create_client<PlanPath>(helper_, "/plan_path");
    executor_.add_node(planner_);
    executor_.add_node(helper_);
    spinner_ = std::thread([this] { executor_.spin(); });
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
  }

  rclcpp_action::ClientGoalHandle<PlanPath>::WrappedResult plan(const PlanPath::Goal & goal)
  {
    auto handle = client_->async_send_goal(goal).get();
    EXPECT_TRUE(handle);
    auto result = client_->async_get_result(handle);
    EXPECT_EQ(result.wait_for(20s), std::future_status::ready);
    return result.get();
  }

  void publish_map_and_wait()
  {
    map_publisher_->publish(map());
    std::string detail;
    for (int i = 0; i < 100 && detail.rfind("map 200", 0) != 0; ++i) {
      std::this_thread::sleep_for(50ms);
      const std::lock_guard<std::mutex> lock(detail_mutex_);
      detail = last_detail_;
    }
    ASSERT_EQ(detail.rfind("map 200", 0), 0U) << detail;
  }

  std::shared_ptr<delivery_bot_planner::PlannerNode> planner_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  rclcpp::Subscription<delivery_bot_interfaces::msg::Heartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp_action::Client<PlanPath>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  std::atomic<int> plans_received_{0};
  std::mutex detail_mutex_;
  std::string last_detail_;  // heartbeat detail, written by the executor thread
};

}  // namespace

TEST_F(PlannerNodeTest, NoMapGivesAnError)
{
  PlanPath::Goal goal;
  goal.start = pose("map", 2.0, 2.0, 0.0);
  goal.goal = pose("map", 15.0, 15.0, 0.0);
  const auto result = plan(goal);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, PlanPath::Result::ERROR_NO_PATH);
}

TEST_F(PlannerNodeTest, PlansAndPublishes)
{
  publish_map_and_wait();
  PlanPath::Goal goal;
  goal.start = pose("map", 3.0, 3.0, 0.0);
  goal.goal = pose("map", 15.0, 14.0, 1.5);
  goal.time_limit = 2.0F;
  const auto result = plan(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->error_code, PlanPath::Result::ERROR_NONE);
  const auto & poses = result.result->path.poses;
  ASSERT_GE(poses.size(), 2U);
  EXPECT_EQ(result.result->path.header.frame_id, "map");
  EXPECT_NEAR(poses.front().pose.position.x, 3.0, 1e-9);
  EXPECT_NEAR(poses.back().pose.position.x, 15.0, 0.2);
  EXPECT_NEAR(poses.back().pose.position.y, 14.0, 0.2);
  // Sampled at path_resolution (0.05 m).
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double ds = std::hypot(
      poses[i].pose.position.x - poses[i - 1].pose.position.x, poses[i].pose.position.y - poses[i - 1].pose.position.y);
    EXPECT_LE(ds, 0.05 + 1e-9);
  }
  EXPECT_GT(result.result->cost, 0.0F);
  for (int i = 0; i < 40 && plans_received_ == 0; ++i) {
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GT(plans_received_, 0);
}

TEST_F(PlannerNodeTest, StartsFromTheCurrentPose)
{
  publish_map_and_wait();
  tf2_msgs::msg::TFMessage tf;
  geometry_msgs::msg::TransformStamped map_odom;
  map_odom.header.frame_id = "map";
  map_odom.child_frame_id = "odom";
  map_odom.transform.translation.x = 1.0;
  map_odom.transform.rotation.w = 1.0;
  geometry_msgs::msg::TransformStamped odom_base;
  odom_base.header.frame_id = "odom";
  odom_base.child_frame_id = "base_link";
  odom_base.transform.translation.x = 3.0;
  odom_base.transform.translation.y = 5.0;
  odom_base.transform.rotation.z = std::sin(0.25);
  odom_base.transform.rotation.w = std::cos(0.25);
  tf.transforms = {map_odom, odom_base};
  for (int i = 0; i < 5; ++i) {
    tf_publisher_->publish(tf);
    std::this_thread::sleep_for(20ms);
  }
  PlanPath::Goal goal;  // empty start frame_id: from where the robot is
  goal.goal = pose("map", 14.0, 12.0, 0.0);
  const auto result = plan(goal);
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  const auto & first = result.result->path.poses.front().pose;
  EXPECT_NEAR(first.position.x, 4.0, 1e-9);
  EXPECT_NEAR(first.position.y, 5.0, 1e-9);
  EXPECT_NEAR(2.0 * std::atan2(first.orientation.z, first.orientation.w), 0.5, 1e-9);
}

TEST_F(PlannerNodeTest, RejectsGoalsInOtherFrames)
{
  publish_map_and_wait();
  PlanPath::Goal goal;
  goal.start = pose("map", 3.0, 3.0, 0.0);
  goal.goal = pose("odom", 10.0, 10.0, 0.0);
  const auto result = plan(goal);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, PlanPath::Result::ERROR_INVALID_GOAL);
}
