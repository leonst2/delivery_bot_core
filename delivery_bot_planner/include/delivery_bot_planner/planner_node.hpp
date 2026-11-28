// The planner node (delivery_bot_core#17): PlanPath action server around the Hybrid-A* search.

#ifndef DELIVERY_BOT_PLANNER_PLANNER_NODE_HPP_
#define DELIVERY_BOT_PLANNER_PLANNER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <delivery_bot_interfaces/action/plan_path.hpp>
#include <delivery_bot_interfaces/msg/heartbeat.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "delivery_bot_collision/collision_checker.hpp"
#include "delivery_bot_collision/footprint.hpp"
#include "delivery_bot_geometry/path.hpp"
#include "delivery_bot_geometry/se2.hpp"
#include "delivery_bot_planner/hybrid_astar.hpp"
#include "delivery_bot_planner/planar_tf.hpp"
#include "delivery_bot_planner/planner_params.hpp"

namespace delivery_bot_planner
{

class PlannerNode : public rclcpp::Node
{
public:
  using PlanPath = delivery_bot_interfaces::action::PlanPath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanPath>;

  explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_map(const nav_msgs::msg::OccupancyGrid & map);
  void on_tf(const tf2_msgs::msg::TFMessage & message);
  void publish_heartbeat();
  // What a request needs from the shared state, copied under the lock.
  struct Snapshot
  {
    std::shared_ptr<HybridAStar> planner;
    std::optional<delivery_bot_geometry::Pose2> start;
    std::optional<delivery_bot_geometry::Path> previous_path;
    delivery_bot_geometry::Pose2 previous_goal;
  };

  [[nodiscard]] Snapshot snapshot(const PlanPath::Goal & goal);
  [[nodiscard]] std::uint8_t check_request(const Snapshot & snap, const PlanPath::Goal & goal) const;
  [[nodiscard]] PlanResult search(
    const Snapshot & snap, const delivery_bot_geometry::Pose2 & target, const std::shared_ptr<GoalHandle> & goal_handle,
    std::chrono::steady_clock::time_point started) const;
  void execute(const std::shared_ptr<GoalHandle> & goal_handle);
  [[nodiscard]] nav_msgs::msg::Path to_message(const delivery_bot_geometry::Path & path) const;

  // Parameters.
  std::string map_frame_;
  std::string base_frame_;
  double default_time_limit_{1.0};
  double path_resolution_{0.05};
  PlannerParams planner_params_;
  delivery_bot_collision::FootprintParams footprint_params_;
  delivery_bot_collision::CheckerParams checker_params_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<delivery_bot_interfaces::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp_action::Server<PlanPath>::SharedPtr action_server_;

  std::mutex state_mutex_;                // guards everything below
  std::shared_ptr<HybridAStar> planner_;  // rebuilt for every new map
  std::string map_status_{"no map received yet"};
  bool map_problem_{false};
  PlanarTf tf_;
  // The last successful plan, for fast re-plans to the same goal.
  std::optional<delivery_bot_geometry::Path> last_path_;
  delivery_bot_geometry::Pose2 last_goal_;

  std::mutex plan_mutex_;  // one search at a time
};

}  // namespace delivery_bot_planner

#endif  // DELIVERY_BOT_PLANNER_PLANNER_NODE_HPP_
