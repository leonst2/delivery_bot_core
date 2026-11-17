// Cleans the LiDAR scans and publishes obstacle points in base_link.
// Stub (delivery_bot_core#9): only publishes heartbeats, so the stack and its health monitor run.

#include <chrono>
#include <memory>

#include <delivery_bot_interfaces/msg/heartbeat.hpp>
#include <rclcpp/rclcpp.hpp>

namespace delivery_bot_scan_filter
{

using delivery_bot_interfaces::msg::Heartbeat;

class ScanFilterNode : public rclcpp::Node
{
public:
  ScanFilterNode()
  : Node("scan_filter")
  {
    const double period = declare_parameter("heartbeat_period", 0.25);
    publisher_ = create_publisher<Heartbeat>("/heartbeat", 10);
    // A wall timer runs on the steady clock. With use_sim_time, ROS timers stop without /clock.
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] {publish_heartbeat();});
  }

private:
  void publish_heartbeat()
  {
    Heartbeat message;
    message.header.stamp = now();
    message.node = get_name();
    message.status = Heartbeat::OK;
    publisher_->publish(message);
  }

  rclcpp::Publisher<Heartbeat>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace delivery_bot_scan_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<delivery_bot_scan_filter::ScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
