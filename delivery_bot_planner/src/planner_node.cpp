#include "delivery_bot_planner/planner_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "delivery_bot_collision/grid.hpp"
#include "delivery_bot_planner/replan.hpp"

namespace delivery_bot_planner
{

namespace dbg = delivery_bot_geometry;
namespace dbc = delivery_bot_collision;
using delivery_bot_interfaces::msg::Heartbeat;

namespace
{

constexpr double kPlanarYawTolerance = 1e-6;  // the map frame must not be rotated against the grid

dbg::Pose2 to_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return {pose.position.x, pose.position.y, PlanarTf::yaw_from_quaternion(q.x, q.y, q.z, q.w)};
}

}  // namespace

PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: Node("planner", options)
{
  const std::string map_topic = declare_parameter("map_topic", std::string("/costmap/global"));
  map_frame_ = declare_parameter("map_frame", std::string("map"));
  base_frame_ = declare_parameter("base_frame", std::string("base_link"));
  default_time_limit_ = declare_parameter("default_time_limit", 1.0);
  path_resolution_ = declare_parameter("path_resolution", 0.05);
  const double heartbeat_period = declare_parameter("heartbeat_period", 0.25);

  footprint_params_.x_min = declare_parameter("footprint.x_min", footprint_params_.x_min);
  footprint_params_.x_max = declare_parameter("footprint.x_max", footprint_params_.x_max);
  footprint_params_.y_min = declare_parameter("footprint.y_min", footprint_params_.y_min);
  footprint_params_.y_max = declare_parameter("footprint.y_max", footprint_params_.y_max);
  footprint_params_.padding = declare_parameter("footprint.padding", footprint_params_.padding);
  checker_params_.rule.occupied_threshold =
    static_cast<int>(declare_parameter("occupied_threshold", static_cast<std::int64_t>(65)));
  checker_params_.rule.unknown =
    declare_parameter("unknown_is_lethal", true) ? dbc::UnknownPolicy::kLethal : dbc::UnknownPolicy::kFree;
  checker_params_.sweep_step = declare_parameter("sweep_step", checker_params_.sweep_step);

  PlannerParams & p = planner_params_;
  p.xy_resolution = declare_parameter("xy_resolution", p.xy_resolution);
  p.yaw_bins = static_cast<int>(declare_parameter("yaw_bins", static_cast<std::int64_t>(p.yaw_bins)));
  p.motion.wheelbase = declare_parameter("wheelbase", p.motion.wheelbase);
  p.motion.min_turning_radius = declare_parameter("min_turning_radius", p.motion.min_turning_radius);
  p.motion.arc_length = declare_parameter("primitive_length", p.motion.arc_length);
  p.motion.steering_count =
    static_cast<int>(declare_parameter("steering_count", static_cast<std::int64_t>(p.motion.steering_count)));
  p.motion.allow_reverse = declare_parameter("allow_reverse", p.motion.allow_reverse);
  p.reverse_penalty = declare_parameter("reverse_penalty", p.reverse_penalty);
  p.direction_change_penalty = declare_parameter("direction_change_penalty", p.direction_change_penalty);
  p.steering_penalty = declare_parameter("steering_penalty", p.steering_penalty);
  p.steering_change_penalty = declare_parameter("steering_change_penalty", p.steering_change_penalty);
  p.cost_weight = declare_parameter("cost_weight", p.cost_weight);
  p.heuristic_weight = declare_parameter("heuristic_weight", p.heuristic_weight);
  p.goal_xy_tolerance = declare_parameter("goal_xy_tolerance", p.goal_xy_tolerance);
  p.goal_yaw_tolerance = declare_parameter("goal_yaw_tolerance", p.goal_yaw_tolerance);

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, latched, [this](const nav_msgs::msg::OccupancyGrid & map) { on_map(map); });
  tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100), [this](const tf2_msgs::msg::TFMessage & message) { on_tf(message); });
  tf_static_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local(),
    [this](const tf2_msgs::msg::TFMessage & message) { on_tf(message); });
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("/plan", latched);
  heartbeat_publisher_ = create_publisher<Heartbeat>("/heartbeat", 10);
  // A wall timer runs on the steady clock: with use_sim_time, ROS timers stop without /clock.
  heartbeat_timer_ =
    create_wall_timer(std::chrono::duration<double>(heartbeat_period), [this] { publish_heartbeat(); });

  action_server_ = rclcpp_action::create_server<PlanPath>(
    this, "/plan_path",
    [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const PlanPath::Goal> &) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle> &) { return rclcpp_action::CancelResponse::ACCEPT; },
    [this](const std::shared_ptr<GoalHandle> & goal_handle) {
      // Searches take up to the time limit: off the executor thread, so heartbeats and maps keep flowing.
      std::thread([this, goal_handle] { execute(goal_handle); }).detach();
    });
}

void PlannerNode::on_map(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & info = map.info;
  const double yaw = to_pose(info.origin).yaw;
  const std::lock_guard<std::mutex> lock(state_mutex_);
  if (std::abs(yaw) > kPlanarYawTolerance) {
    map_status_ = "map rejected: origin yaw must be 0";
    map_problem_ = true;
    RCLCPP_WARN(get_logger(), "%s", map_status_.c_str());
    return;
  }
  try {
    dbc::Grid grid(
      static_cast<int>(info.width), static_cast<int>(info.height), info.resolution, info.origin.position.x,
      info.origin.position.y, std::vector<std::int8_t>(map.data.begin(), map.data.end()));
    auto checker =
      std::make_shared<const dbc::CollisionChecker>(std::move(grid), footprint_params_.make(), checker_params_);
    planner_ = std::make_shared<HybridAStar>(checker, planner_params_);
    last_path_.reset();
    map_problem_ = false;
    map_status_ = "map " + std::to_string(info.width) + " x " + std::to_string(info.height) + " at " +
                  std::to_string(info.resolution) + " m";
    RCLCPP_INFO(get_logger(), "%s", map_status_.c_str());
  } catch (const std::exception & e) {
    map_status_ = std::string("map rejected: ") + e.what();
    map_problem_ = true;
    RCLCPP_WARN(get_logger(), "%s", map_status_.c_str());
  }
}

void PlannerNode::on_tf(const tf2_msgs::msg::TFMessage & message)
{
  const std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto & t : message.transforms) {
    const auto & q = t.transform.rotation;
    tf_.set(
      t.header.frame_id, t.child_frame_id,
      {t.transform.translation.x, t.transform.translation.y, PlanarTf::yaw_from_quaternion(q.x, q.y, q.z, q.w)});
  }
}

void PlannerNode::publish_heartbeat()
{
  Heartbeat message;
  message.header.stamp = now();
  message.node = get_name();
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    // OK without a map: the stack also runs (in CI) before a map server or costmap publishes one.
    message.status = map_problem_ ? Heartbeat::WARN : Heartbeat::OK;
    message.detail = map_status_;
  }
  heartbeat_publisher_->publish(message);
}

nav_msgs::msg::Path PlannerNode::to_message(const dbg::Path & path) const
{
  nav_msgs::msg::Path message;
  message.header.frame_id = map_frame_;
  message.header.stamp = now();
  for (const dbg::PathSample & sample : path.sample(path_resolution_)) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message.header;
    pose.pose.position.x = sample.pose.x;
    pose.pose.position.y = sample.pose.y;
    pose.pose.orientation.z = std::sin(sample.pose.yaw / 2.0);
    pose.pose.orientation.w = std::cos(sample.pose.yaw / 2.0);
    message.poses.push_back(pose);
  }
  return message;
}

PlannerNode::Snapshot PlannerNode::snapshot(const PlanPath::Goal & goal)
{
  const std::lock_guard<std::mutex> lock(state_mutex_);
  Snapshot snap;
  snap.planner = planner_;
  snap.previous_path = last_path_;
  snap.previous_goal = last_goal_;
  if (goal.start.header.frame_id.empty()) {
    snap.start = tf_.lookup(map_frame_, base_frame_);
  } else if (goal.start.header.frame_id == map_frame_) {
    snap.start = to_pose(goal.start.pose);
  }
  return snap;
}

std::uint8_t PlannerNode::check_request(const Snapshot & snap, const PlanPath::Goal & goal) const
{
  if (!snap.planner) {
    RCLCPP_WARN(get_logger(), "PlanPath: no map received yet");
    return PlanPath::Result::ERROR_NO_PATH;
  }
  if (!snap.start) {
    RCLCPP_WARN(
      get_logger(), "PlanPath: no start (an empty frame_id needs TF %s -> %s)", map_frame_.c_str(),
      base_frame_.c_str());
    return PlanPath::Result::ERROR_INVALID_START;
  }
  if (goal.goal.header.frame_id != map_frame_) {
    RCLCPP_WARN(get_logger(), "PlanPath: the goal must be in frame %s", map_frame_.c_str());
    return PlanPath::Result::ERROR_INVALID_GOAL;
  }
  return PlanPath::Result::ERROR_NONE;
}

PlanResult PlannerNode::search(
  const Snapshot & snap, const dbg::Pose2 & target, const std::shared_ptr<GoalHandle> & goal_handle,
  std::chrono::steady_clock::time_point started) const
{
  const dbg::Pose2 start = snap.start.value_or(dbg::Pose2{});
  // Fast path: the previous plan to the same goal still fits.
  if (snap.previous_path) {
    if (auto reused = reuse_previous(*snap.planner, *snap.previous_path, snap.previous_goal, start, target)) {
      PlanResult result;
      result.error = PlanError::kNone;
      result.cost = snap.planner->path_cost(*reused);
      result.path = std::move(*reused);
      return result;
    }
  }
  const auto goal = goal_handle->get_goal();
  const double time_limit = goal->time_limit > 0.0F ? goal->time_limit : default_time_limit_;
  PlanLimits limits;
  limits.deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(time_limit));
  limits.cancelled = [&goal_handle] { return goal_handle->is_canceling(); };
  limits.progress = [&goal_handle](std::size_t expanded) {
    auto feedback = std::make_shared<PlanPath::Feedback>();
    feedback->expanded_nodes = static_cast<std::uint32_t>(expanded);
    goal_handle->publish_feedback(feedback);
  };
  return snap.planner->plan(start, target, limits);
}

void PlannerNode::execute(const std::shared_ptr<GoalHandle> & goal_handle)
{
  const std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  const auto started = std::chrono::steady_clock::now();
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PlanPath::Result>();
  const auto finish = [&](std::uint8_t error) {
    result->error_code = error;
    result->planning_time =
      static_cast<float>(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    if (error == PlanPath::Result::ERROR_NONE) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  };

  const Snapshot snap = snapshot(*goal);
  if (const std::uint8_t error = check_request(snap, *goal); error != PlanPath::Result::ERROR_NONE) {
    finish(error);
    return;
  }
  const dbg::Pose2 target = to_pose(goal->goal.pose);
  const PlanResult planned = search(snap, target, goal_handle, started);
  if (planned.error == PlanError::kCancelled) {
    result->error_code = PlanPath::Result::ERROR_NO_PATH;
    goal_handle->canceled(result);
    return;
  }
  if (planned.error != PlanError::kNone) {
    finish(static_cast<std::uint8_t>(planned.error));
    return;
  }
  result->path = to_message(planned.path);
  result->cost = static_cast<float>(planned.cost);
  plan_publisher_->publish(result->path);
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    if (planner_ == snap.planner) {  // still the same map
      last_path_ = planned.path;
      last_goal_ = target;
    }
  }
  finish(PlanPath::Result::ERROR_NONE);
}

}  // namespace delivery_bot_planner
