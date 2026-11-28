// Hybrid-A* path planner node (delivery_bot_core#17): PlanPath action on /plan_path, result also on /plan.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "delivery_bot_planner/planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<delivery_bot_planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
