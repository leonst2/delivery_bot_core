//! Rust toolchain spike (delivery_bot_core#2): proves that rclrs nodes build and run in our workspace.
//!
//! Subscribes `/clock` (rosgraph_msgs/Clock) and publishes a `Heartbeat` on `/heartbeat` at 2 Hz that reports
//! the latest sim time.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Result;
use rclrs::*;
use ros_env::delivery_bot_interfaces::msg::Heartbeat;
use ros_env::rosgraph_msgs::msg::Clock;

const NODE_NAME: &str = "rust_hello";
const HEARTBEAT_PERIOD: Duration = Duration::from_millis(500);

/// Status and detail text of a heartbeat, given the latest `/clock` time in nanoseconds (if any).
fn heartbeat_status(last_clock_ns: Option<i64>) -> (u8, String) {
    match last_clock_ns {
        Some(ns) => (Heartbeat::OK, format!("sim time {:.3} s", ns as f64 * 1e-9)),
        None => (Heartbeat::WARN, "no /clock received yet".to_owned()),
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;

    let last_clock_ns: Arc<Mutex<Option<i64>>> = Arc::new(Mutex::new(None));

    let clock_store = Arc::clone(&last_clock_ns);
    let _clock_sub = node.create_subscription::<Clock, _>("/clock", move |msg: Clock| {
        let ns = i64::from(msg.clock.sec) * 1_000_000_000 + i64::from(msg.clock.nanosec);
        *clock_store.lock().unwrap() = Some(ns);
    })?;

    let publisher = node.create_publisher::<Heartbeat>("/heartbeat")?;
    let clock_read = Arc::clone(&last_clock_ns);
    let _timer = node.create_timer_repeating(HEARTBEAT_PERIOD, move || {
        let last = *clock_read.lock().unwrap();
        let (status, detail) = heartbeat_status(last);
        let mut msg = Heartbeat {
            node: NODE_NAME.to_owned(),
            status,
            detail,
            ..Default::default()
        };
        if let Some(ns) = last {
            msg.header.stamp.sec = (ns / 1_000_000_000) as i32;
            msg.header.stamp.nanosec = (ns % 1_000_000_000) as u32;
        }
        if let Err(err) = publisher.publish(&msg) {
            eprintln!("failed to publish heartbeat: {err}");
        }
    })?;

    println!("{NODE_NAME}: publishing heartbeats on /heartbeat");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::delivery_bot_interfaces::msg::AckermannCommand;
    use std::borrow::Cow;

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

    #[test]
    fn warns_without_clock() {
        let (status, detail) = heartbeat_status(None);
        assert_eq!(status, Heartbeat::WARN);
        assert!(detail.contains("no /clock"));
    }

    #[test]
    fn reports_sim_time() {
        let (status, detail) = heartbeat_status(Some(12_345_000_000));
        assert_eq!(status, Heartbeat::OK);
        assert_eq!(detail, "sim time 12.345 s");
    }
}
