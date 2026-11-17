//! Step controller (delivery_bot_core#19).
//!
//! Stub: only publishes heartbeats, so the stack and its health monitor run. The card replaces it.

use std::time::Duration;

use anyhow::Result;
use rclrs::*;
use ros_env::delivery_bot_interfaces::msg::Heartbeat;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("step_controller")?;

    let period: f64 = node
        .declare_parameter("heartbeat_period")
        .default(0.25)
        .mandatory()?
        .get();
    let publisher = node.create_publisher::<Heartbeat>("/heartbeat")?;
    let heartbeat_node = node.clone();
    // Steady clock on purpose: with use_sim_time, a node-clock timer stops without /clock.
    let _timer =
        node.create_timer_repeating(Duration::from_secs_f64(period).steady_time(), move || {
            let mut msg = Heartbeat {
                node: heartbeat_node.name(),
                status: Heartbeat::OK,
                ..Default::default()
            };
            if let Ok((sec, nanosec)) = heartbeat_node.get_clock().now().to_sec_nanosec() {
                msg.header.stamp.sec = sec;
                msg.header.stamp.nanosec = nanosec;
            }
            if let Err(err) = publisher.publish(&msg) {
                eprintln!("failed to publish heartbeat: {err}");
            }
        })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use rclrs::*;
    use ros_env::delivery_bot_interfaces::msg::AckermannCommand;
    use std::borrow::Cow;

    /// The Rust bindings of delivery_bot_interfaces convert to their RMW form and back.
    #[test]
    fn interfaces_convert_to_the_rmw_form_and_back() {
        let message = AckermannCommand {
            speed: 1.5,
            steering_angle: -0.25,
            ..Default::default()
        };
        let rmw = AckermannCommand::into_rmw_message(Cow::Borrowed(&message)).into_owned();
        assert_eq!(AckermannCommand::from_rmw_message(rmw), message);
    }
}
