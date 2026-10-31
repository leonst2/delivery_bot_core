#include <gtest/gtest.h>

#include <delivery_bot_interfaces/msg/grid_step_state.hpp>
#include <delivery_bot_interfaces/msg/tracked_obstacle_array.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

using delivery_bot_interfaces::msg::GridStepState;
using delivery_bot_interfaces::msg::TrackedObstacle;
using delivery_bot_interfaces::msg::TrackedObstacleArray;

template<typename MessageT>
MessageT round_trip(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  MessageT result;
  serialization.deserialize_message(&serialized, &result);
  return result;
}

TEST(Serialization, GridStepStateRoundTrip)
{
  GridStepState message;
  message.state = GridStepState::YIELD;
  message.state_name = "YIELD";
  message.step_index = 7;

  EXPECT_EQ(round_trip(message), message);
}

TEST(Serialization, NestedArrayRoundTrip)
{
  TrackedObstacle obstacle;
  obstacle.id = 3;
  obstacle.position_covariance = {0.1f, 0.0f, 0.0f, 0.2f};
  TrackedObstacleArray message;
  message.header.frame_id = "map";
  message.obstacles.push_back(obstacle);

  EXPECT_EQ(round_trip(message), message);
}
